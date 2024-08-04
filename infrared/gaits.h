#ifndef gaits_h
#define gaits_h

#include <BasicLinearAlgebra.h>

class Gait{
public:
    BLA::Matrix<4,2>*  gait;
    size_t length, idx;


    Gait(BLA::Matrix<4,2>* gait_array, size_t length){
        this->gait = gait_array;
        this->length = length;
        this->idx = 0;
    }

    const BLA::Matrix<4,2>& next_frame(){
        size_t cidx = this->idx;
        this->idx = (this->idx + 1) % length;
        return this->gait[cidx];
    }

    void reset(){
        idx = 0;
    }
};


BLA::Matrix<4,2> standing_gait_frames[1] = {
    {
        -5, -5,
        5, -5,
        5, -5,
        -5, -5
    }
};

BLA::Matrix<4,2> stepping_gait_frame[4] = {
    {
        -5, -5,
        5, -6,
        5, -5,
        -5, -6
    },
    {
        -5, -5.5, 
        5, -5.5,
        5, -5.5,
        -5, -5.5
    },
    {
        -5, -6,
        5, -5,
        5, -6,
        -5, -5
    },
    {
        -5, -5.5,
        5, -5.5,
        5, -5.5,
        -5, -5.5
    }
};

BLA::Matrix<4,2> walking_gait_frame[6] = {
    {
        -5, -5,
        6, -5.75,
        5, -5,
        -4, -5.75
    },
    {
        -6, -5.25,
        7, -6,
        4, -5.25,
        -3, -6
    },
    {
        -5, -5.5,
        6, -5,
        5, -5.5,
        -4, -5
    },
    {
        -4, -5.75,
        5, -5,
        6, -5.75,
        -5, -5
    },
    {
        -3, -6,
        4, -5.25,
        7, -6,
        -6, -5.25
    },
    {
        -4, -5,
        5, -5.5,
        6, -5,
        -5, -5.5
    }
};

Gait standing_gait(standing_gait_frames,1);
Gait stepping_gait(stepping_gait_frame, 4);
Gait walking_gait(walking_gait_frame, 6);

#endif

