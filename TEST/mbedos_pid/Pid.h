#ifndef PID_H_
#define PID_H_

#include <Arduino.h>


class Pid
{
private:

    /* Controller "memory" */
    float integrator;
    float prevError;			/* Required for integrator */
    float differentiator;
    float prevMeasurement;		/* Required for differentiator */


    /* timestamp */
    uint64_t update_timestamp; // micro sec

    /* data */
public:

    /* Controller output */
    float out;

    /* Controller gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant */
    float tau;

    /* Output limits */
    float limMin;
    float limMax;

    /* Sample time (in seconds) */
    float T;

    enum PID_STATUS{ NO_UPDATE , UPDATED };

    Pid(float KP, float KI, float KD, float sT,  float TAU, float LIMMIN = -255, float LIMMAX = 255);
    void Begin();
    PID_STATUS Update(float setpoint, float measurement);
};

#endif