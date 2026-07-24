#ifndef _MOTOR_H
#define _MOTOR_H

#include <mbed.h>
#include <chrono>
#include "Timer.h"
#include "Callback.h"
using namespace std::chrono;

class MOTOR{

  private:
  /* MOTOR A */
  mbed::DigitalOut    _DIRA;
  mbed::PwmOut        _PWMA;
  mbed::InterruptIn   _ENCODERA;
  uint32_t            _PWMA_FREQ = 100e3;
  uint32_t            _PWMA_PERIOD_US;
  mbed::Timer         _ENCODERA_TIMER;
  float               _ENCODERA_SPEED;
  
  // /* MOTOR B */
  mbed::DigitalOut    _DIRB;
  mbed::PwmOut        _PWMB;
  mbed::InterruptIn   _ENCODERB;
  uint32_t            _PWMB_FREQ = 100e3;
  uint32_t            _PWMB_PERIOD_US;
  mbed::Timer         _ENCODERB_TIMER;
  float               _ENCODERB_SPEED;

  void EncoderA_trigger();
  void EncoderB_trigger();


  public:

  MOTOR(PinName DIRA,PinName PWMA,PinName ENCODERA,PinName DIRB,PinName PWMB,PinName ENCODERB );

  void Begin();
  void Set_dir(int motor, int dir);
  void Set_pwm(int motor, int pwm);
  void Set_frquency(int motor, int frequency);
  float Get_speed(int motor){ return motor ? _ENCODERB_SPEED : _ENCODERA_SPEED; }

};



#endif