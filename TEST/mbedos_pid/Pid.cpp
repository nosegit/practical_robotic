#include "Pid.h"

Pid::Pid(float KP, float KI, float KD, float sT,  float TAU, float LIMMIN , float LIMMAX){

    Kp = KP;
    Ki = KI;
    Kd = KD;

    T = sT;
    tau = TAU;

    limMin = LIMMIN;
    limMax = LIMMAX;
}

void Pid::Begin(){
    
    /* Clear controller variables */
	integrator = 0.0f;
	prevError  = 0.0f;

	differentiator  = 0.0f;
	prevMeasurement = 0.0f;

	out = 0.0f;
}

Pid::PID_STATUS Pid::Update(float setpoint, float measurement){

    /* update loop at constant sampling time*/
    if(micros() - update_timestamp > T*1e6){
        update_timestamp = micros();

        float error = setpoint - measurement;

        /*
        * Proportional
        */
        float proportional = Kp * error;


        /*
        * Integral
        */
        integrator = integrator + 0.5f * Ki * T * (error + prevError);

        /* Anti-wind-up via integrator clamping */
        float limMinInt, limMaxInt;

        /* compute integrator limit */
        if(limMax > proportional){
            limMaxInt = limMax - proportional;
        }
        else{
            limMaxInt = 0.0f;
        }

        if(limMin < proportional){
            limMinInt = limMin - proportional;
        }
        else{
            limMinInt = 0.0f;
        }

        /* Clamp integrator */
        if (integrator > limMaxInt) {

            integrator = limMaxInt;

        } else if (integrator < limMinInt) {

            integrator = limMinInt;

        }

        /*
        * Derivative (band-limited differentiator)
        */
            
        differentiator = -(2.0f * Kd * (measurement - prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                            + (2.0f * tau - T) * differentiator)
                            / (2.0f * tau + T);


        /*
        * Compute output and apply limits
        */
        out = proportional + integrator + differentiator;

        if (out > limMax) {

            out = limMax;

        } else if (out < limMin) {

            out = limMin;

        }

        /* Store error and measurement for later use */
        prevError       = error;
        prevMeasurement = measurement;

        /* Return controller output */
        return Pid::PID_STATUS::UPDATED;
    }

    return Pid::PID_STATUS::NO_UPDATE;

}
