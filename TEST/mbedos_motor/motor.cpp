#include <sys/_types.h>
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

  Set_dir(0, 0);
  Set_dir(1, 0);

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

float MOTOR::Get_speed(int motor){

  // float diff_t;
  unsigned long diff_t;


  switch(motor){
    case 0: 
      // _ENCODERA_TIMER.stop();

      // diff_t = duration_cast<microseconds>(_ENCODERA_TIMER.elapsed_time()).count();
      diff_t = micros()-_ENCODERA_TIMESTAMP_US;
      _ENCODERA_SPEED = (float)(_ENCODERA_CNT-_ENCODERA_CNT_PREV)*1e6/diff_t;

      // _ENCODERA_TIMER.reset();
      // _ENCODERA_TIMER.start();

      _ENCODERA_CNT_PREV = _ENCODERA_CNT;
      _ENCODERA_TIMESTAMP_US = micros();

      return _ENCODERA_SPEED;
      // break;

    case 1: 
      // _ENCODERB_TIMER.stop();
      // diff_t = duration_cast<microseconds>(_ENCODERB_TIMER.elapsed_time()).count();
      diff_t = micros()-_ENCODERB_TIMESTAMP_US;
      _ENCODERB_SPEED = (_ENCODERB_CNT-_ENCODERB_CNT_PREV)*1e6/diff_t;
      // _ENCODERB_SPEED = (_ENCODERB_CNT-_ENCODERB_CNT_PREV);


      // _ENCODERB_TIMER.reset();
      // _ENCODERB_TIMER.start();
      
      _ENCODERB_CNT_PREV = _ENCODERB_CNT;
      _ENCODERB_TIMESTAMP_US = micros();

      return _ENCODERB_SPEED;
      // break;
  }
}

void MOTOR::EncoderA_trigger(){

  _ENCODERA_CNT += _DIRA ? -1:1;

}

void MOTOR::EncoderB_trigger(){

  _ENCODERB_CNT += _DIRB ? 1:-1;

}

void MOTOR::Reset_encoder(){
  _ENCODERA_CNT = _ENCODERB_CNT = 0;
}


void MOTOR::Print_info(){
  if(Serial){
    for(int i = 0 ; i < 2; i++){
      Serial.print(i);
      Serial.print("encoder:");
      Serial.println(Get_encoder_cnt(i));
      Serial.print(i);
      Serial.print("speed:");
      Serial.println(i ? _ENCODERB_SPEED, _ENCODERA_SPEED);
    }
  }
}

void MOTOR::Print_info_CSV(){
  if(Serial){
      for(int i = 0 ; i < 2; i++){
        Serial.print(i);
        Serial.print(",");
        Serial.print(Get_encoder_cnt(i));
        Serial.print(",");
        Serial.print(Get_speed(i));
        Serial.print(",");
    }
    Serial.println(" ");
  }
}