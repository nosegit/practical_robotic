#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "ARB.h"
#include "measurement.h"
#include "DigitalOut.h"
#include <Arduino.h>
#include <mbed.h>

#define ENABLE_MEASUREMENT 1

class ULTRASONIC{

  private:
    PinName             _PIN;
    mbed::InterruptIn*  _PIN_INTERRUPT  = nullptr;
    mbed::DigitalOut*   _PIN_OUTPUT     = nullptr;
    mbed::Timer         _TIMER;
    float               _DISTANCE       = 0;

    void Enable_interrupt_mode();
    void Enable_digital_mode();

    void Falling_edge();
    void Rising_edge();

  public:

    MEASUREMENT         _MEASURE;
    uint8_t             _FINISH_READING = 0;

    ULTRASONIC(PinName pin) : _PIN(pin){
      _MEASURE = (int)pin;
    }

    void Begin();
    float Get_distance();
};

#endif