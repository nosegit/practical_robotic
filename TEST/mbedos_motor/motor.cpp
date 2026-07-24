#include <chrono>
#include "motor.h"

MOTOR::MOTOR(PinName DIRA,PinName PWMA,PinName ENCODERA,PinName DIRB,PinName PWMB,PinName ENCODERB ):
    _DIRA(DIRA), _PWMA(PWMA), _ENCODERA(ENCODERA),
    _DIRB(DIRB), _PWMB(PWMB), _ENCODERB(ENCODERB)
{
  _PWMA_PERIOD_US = 1e6/_PWMA_FREQ; 
  _PWMB_PERIOD_US = 1e6/_PWMB_FREQ; 

  _ENCODERA.rise(mbed::callback(this, &MOTOR::EncoderA_trigger));
  _ENCODERA.fall(mbed::callback(this, &MOTOR::EncoderA_trigger));

  _ENCODERB.rise(mbed::callback(this, &MOTOR::EncoderB_trigger));
  _ENCODERB.fall(mbed::callback(this, &MOTOR::EncoderB_trigger));
}

void MOTOR::Begin(){
  Set_frquency(0, _PWMA_FREQ);
  Set_frquency(1, _PWMB_FREQ);

  Set_dir(0, 1);
  Set_dir(1, 1);

  Set_pwm(0, 0);
  Set_pwm(1, 0);

  _ENCODERA_TIMER.reset();
  _ENCODERB_TIMER.reset();

  _ENCODERA_TIMER.start();
  _ENCODERB_TIMER.start();


} 

void MOTOR::Set_dir(int motor, int dir){
  switch (motor){
    case 0: _DIRA = dir ? 1:0; break;
    case 1: _DIRB = dir ? 0:1; break;
  }
}

void MOTOR::Set_pwm(int motor, int pwm){
  switch (motor){
    case 0: _PWMA.write(pwm < 100 ? (float)pwm/100:1); break;
    case 1: _PWMB.write(pwm < 100 ? (float)pwm/100:1); break;
  }
}

void MOTOR::Set_frquency(int motor, int frequency){
  switch (motor){
    case 0: _PWMA_FREQ = frequency; _PWMA_PERIOD_US = 1e6/_PWMA_FREQ; _PWMA.period_us(_PWMA_PERIOD_US); break;
    case 1: _PWMB_FREQ = frequency; _PWMB_PERIOD_US = 1e6/_PWMB_FREQ; _PWMB.period_us(_PWMB_PERIOD_US); break;
  }
}

void MOTOR::EncoderA_trigger(){
  _ENCODERA_TIMER.stop();

  float diff_t = duration_cast<microseconds>(_ENCODERA_TIMER.elapsed_time()).count();
  _ENCODERA_SPEED = 1e6/diff_t;

  _ENCODERA_TIMER.reset();
  _ENCODERA_TIMER.start();
}

void MOTOR::EncoderB_trigger(){
  _ENCODERB_TIMER.stop();
  float diff_t = duration_cast<microseconds>(_ENCODERB_TIMER.elapsed_time()).count();
  _ENCODERB_SPEED = 1e6/diff_t;

  _ENCODERB_TIMER.reset();
  _ENCODERB_TIMER.start();
}