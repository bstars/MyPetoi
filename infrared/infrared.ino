#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include <BasicLinearAlgebra.h>
#include <IRremote.h>
#include "gaits.h"


#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define FREQUENCY 50


long float_map(float in, float low_in, float high_in, long low_out, long high_out){
    return low_out + (in - low_in) / (high_in - low_in) * (high_out - low_out);
}




class PetoiDog{
public:

    // Hardware settings
    Adafruit_PWMServoDriver pwm_driver;
    int alpha_maps[4] = {14, 1, 6, 9}; // Servo index mapping for alpha (shoulder)
    int beta_maps[4] = {15, 0, 7, 8}; //Servo index mapping for beta (knees)

    // Static parameters in cm(from the PDF)
    float a = 10.5; // body length
    float b = 10.;  // body width
    float c = 4.6;  // leg length
    float d = 4.6;  // foot length
    BLA::Matrix<4,1> ones_41;

    // Dynamic parameters
    float alphas[4] = {0., 0., 0., 0.}; // shoulder angles, in radius
    float betas[4] = {0., 0., 0., 0.}; // knee angles, in radius

    // Calibrations, in degrees
    float alpha_calibrate[4] = {1. , 6. , 9. , -7.}; 
    float beta_calibrate[4] = {3., 9., 9., -5.};

    // Parameters for inverse kinematics
    float gamma = 0.;           // current body angle
    float target_gamma = 0.;    // target body angle, use current angle and targe angle to enable a smooth transform
    float gamma_granularity = 2. / RAD_TO_DEG; // only increase/decrease angle 2 degree every time
    float gamma_max = 15. / RAD_TO_DEG; // Keep body angle in a range to keep servo safe
    float gamma_min = -20. / RAD_TO_DEG;


    float h = 0.;               // current body height
    float target_h = 0.;        // target body height, use current height and targe height to enable a smooth transform
    float h_granularity = 0.1;  // only increase/decrease height 0.1 cm every time
    float h_max = 1.; // Keep body height in a range to keep servo safe
    float h_min = -1.;

    BLA::Matrix<3,3> T01_front; // Transformation matrix for front legs
    BLA::Matrix<3,3> T01_back;  // Transformation matrix for rear legs
    BLA::Matrix<3,3> T01_front_inv, T01_back_inv; // inverse of transformation matrices
    

    void initialize(){
        pwm_driver.begin();
        pwm_driver.setPWMFreq(50);
        ones_41.Fill(1.);
        update_gamma_h();
    }

    

    void update_target_gamma(float gamma){ 
        target_gamma = gamma; 
        target_gamma = max(target_gamma, gamma_min);
        target_gamma = min(target_gamma, gamma_max);
    }
    void update_target_h(float h) { 
        target_h = h; 
        target_h = max(target_h, h_min);
        target_h = min(target_h, h_max);
    }

    void update_gamma_h(){
        /*
        Compute the smooth transform of body angle and height,
        most of the time this should be called every time you call write_degree
        */
        int sign;
        float change;

        // update gamma
        sign = this->target_gamma > this->gamma ? 1 : -1;
        change = min( abs(this->target_gamma - this->gamma), this->gamma_granularity );
        this->gamma += sign * change;

        float cg = cos(this->gamma);
        float sg = sin(this->gamma);
        this->T01_front = {
            cg,     sg, -this->a / 2 * cg,
            -sg,    cg, this->a / 2 * sg,
            0,      0,  1
        };
        this->T01_front_inv = BLA::Inverse(this->T01_front);

        this->T01_back = {
            cg,     sg, this->a / 2 * cg,
            -sg,    cg, -this->a / 2 * sg,
            0,      0,  1
        };
        this->T01_back_inv = BLA::Inverse(this->T01_back);

        // update h
        sign = this->target_h > this->h ? 1 : -1;
        change = min( abs(this->target_h - this->h), this->h_granularity );
        this->h += sign * change;
    }

    void write_degree(float* alpha_vals=nullptr, float* beta_vals=nullptr){
        /*
        Write angle to servo motors
        the parameter angles are in radius
        */
        long pulsewidth = 0;
        alpha_vals = alpha_vals ? alpha_vals : alphas;
        beta_vals = beta_vals ? beta_vals : betas;

        // write alpha (shoulder) degrees
        // note that for the right side of the body, need to rotate the servo on the other direction
        for (int i=0; i<=3; i++){
            int flip = i >= 2 ? -1 : 1;
            pulsewidth = float_map(flip * (alpha_vals[i] * RAD_TO_DEG + alpha_calibrate[i]), -135, 135, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
            pulsewidth = min(MAX_PULSE_WIDTH, max(pulsewidth, MIN_PULSE_WIDTH));
            pwm_driver.writeMicroseconds(alpha_maps[i], pulsewidth);

            pulsewidth = float_map(flip * (beta_vals[i] * RAD_TO_DEG + beta_calibrate[i]), -135, 135, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
            pulsewidth = min(MAX_PULSE_WIDTH, max(pulsewidth, MIN_PULSE_WIDTH));
            pwm_driver.writeMicroseconds(beta_maps[i], pulsewidth);
        }
    } 

    void leg_ik(BLA::Matrix<4,2> xz){
        /*
        Compute the inverse kinematics for each leg and store the angles to this->alphas and this->betas, 
        :param xz: shape [4,2], each row containing the xz coordinate of the end-effector
        */
        BLA::Matrix <4,3> xz0 = xz || ones_41;
        for(size_t i=0; i<=3; i++){
            xz0(i,1) -= this->h;
        }

        BLA::Matrix<3,3> T01_inv;
        BLA::Matrix<3,1> xz1;
        BLA::Matrix<3,1> temp;
        float x1, z1, L, alpha1_tilde, alpha2_tilde, beta_tilde;
        // Compute the inverse kinematics
        for(size_t i=0; i<=3; i++){
            T01_inv = (i==0 || i==3) ? this->T01_front_inv : this->T01_back_inv;
            temp = {xz0(i,0), xz0(i,1), xz0(i,2)};
            xz1 = T01_inv * temp;
            x1 = xz1(0,0);
            z1 = xz1(1,0);

            L = sqrt(x1*x1 + z1*z1);
            alpha1_tilde = acos(
                (c*c + L*L - d*d) / (2 * c * L)
            );
            alpha2_tilde = asin(x1/L);
            (this->alphas)[i] = alpha1_tilde + alpha2_tilde;

            beta_tilde = acos( 
                (c*c + d*d - L*L) / (2 * c * d) 
            );
            (this->betas)[i] = PI/2 - beta_tilde;
        }


    }


};


PetoiDog robot;
Gait current_gait = standing_gait;


#define IR_PIN 4
IRrecv irrecv(IR_PIN);
decode_results results;



void setup(){

    Serial.begin(9600);
    robot.initialize();
    irrecv.enableIRIn();
}

void loop(){

    if(irrecv.decode(&results)){
        // if (results.value != 0xFFFFFFFF){
        //     Serial.println(results.value, HEX);
        //     Serial.println(results.decode_type);
            
        // }

        // Serial.println(results.value, HEX);
        switch (results.value) {
            case 0xFF629D:
                robot.update_target_h(
                    robot.h + 0.3
                );
                break;
            case 0xFFA857:
                robot.update_target_h(
                    robot.h - 0.3
                );
                break;
            case 0xFF22DD:
                robot.update_target_gamma(
                    robot.gamma + 3 / RAD_TO_DEG
                );
                break;

            case 0xFFC23D:
                robot.update_target_gamma(
                    robot.gamma - 3 / RAD_TO_DEG
                );
                break;

            case 0xFF02FD:
                current_gait.reset();
                current_gait = standing_gait;
                break;

            case 0xFF6897:
                current_gait.reset();
                current_gait = stepping_gait;
                break;

            case 0xFF30CF:
                current_gait.reset();
                current_gait = walking_gait;
                break;;
        }
        irrecv.resume();
        
    }

    robot.leg_ik(
        current_gait.next_frame()
    );

    robot.write_degree();
    robot.update_gamma_h();

    // Serial.print(robot.alphas[0] * RAD_TO_DEG); Serial.print(' '); Serial.print(robot.betas[0] * RAD_TO_DEG); Serial.println();
    // Serial.print(robot.alphas[1] * RAD_TO_DEG); Serial.print(' '); Serial.print(robot.betas[1] * RAD_TO_DEG); Serial.println();
    // Serial.print(robot.alphas[2] * RAD_TO_DEG); Serial.print(' '); Serial.print(robot.betas[2] * RAD_TO_DEG); Serial.println();
    // Serial.print(robot.alphas[3] * RAD_TO_DEG); Serial.print(' '); Serial.print(robot.betas[3] * RAD_TO_DEG); Serial.println();
    // Serial.println("--------------------");
    delay(70);
}