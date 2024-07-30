#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"


#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define FREQUENCY 50


long float_map(float in, float low_in, float high_in, long low_out, long high_out){
    return low_out + (in - low_in) / (high_in - low_in) * (high_out - low_out);
}

class PetoiDog{
public:
    int alpha_maps[4] = {14, 1, 6, 9}; // Servo index mapping for alpha (shoulder)
    int beta_maps[4] = {15, 0, 7, 8}; //Servo index mapping for beta (knees)
    float alphas[4] = {0., 0., 0., 0.}; // shoulder angles, in radius
    float betas[4] = {0., 0., 0., 0.}; // knee angles, in radius

    // Calibrations, in degrees
    float alpha_calibrate[4] = {0. , 6. , 8. , -8.}; 
    float beta_calibrate[4] = {3., 8., 10., -4.}; 


    Adafruit_PWMServoDriver pwm_driver;

    void write_degree(float* alpha_vals=nullptr, float* beta_vals=nullptr){
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

    void initialize(){
        pwm_driver.begin();
        pwm_driver.setPWMFreq(50);
    }

};

void parse_float_array(float* arr, String s, size_t n){
    float f = 0.;
    for(size_t i=0; i<n; i++){
        f = s.toFloat();
        // Serial.println(f);
        arr[i] = f;
        s = s.substring(s.indexOf(' ') + 1);
    }
}

PetoiDog dog;





float alphas_betas[8];

void setup(){
    Serial.begin(9600);
    Serial.setTimeout(20);
    dog.initialize();
    dog.write_degree();

    // pinMode(13, OUTPUT);
    // String s = "1.0 2.2 3.3 4.45";
    // parse_float_array(alphas, s, 4);
}

void loop(){

    if (Serial.available() > 0){
        String msg = Serial.readString();
        parse_float_array(alphas_betas, msg, 8);
        dog.write_degree(alphas_betas, alphas_betas + 4);
        Serial.println("done");
    }

}



