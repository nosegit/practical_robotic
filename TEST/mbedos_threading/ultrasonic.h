#include "PinNames.h"
#ifndef _ULTRASONIC_H
#define _ULTRASONIC_H

#include "ARB.h"
#include "measurement.h"
#include "DigitalOut.h"
#include <Arduino.h>
#include <mbed.h>

#define ENABLE_MEASUREMENT 1

#define ULTRASONIC_PIN1 P0_23
#define ULTRASONIC_PIN2 P1_14

class ULTRASONIC{

  private:
    PinName             _PIN;
    mbed::InterruptIn*  _PIN_INTERRUPT  = nullptr;
    mbed::DigitalOut*   _PIN_OUTPUT     = nullptr;
    mbed::Timer         _TIMER;
    float               _DISTANCE       = 0;
    uint8_t             _AUTO_RESTART   = 0;

    void Enable_interrupt_mode(){
      // Serial.println("ultra interrupt");

      if(_PIN_OUTPUT){
        delete _PIN_OUTPUT;
        _PIN_OUTPUT = nullptr;
      }

      _PIN_INTERRUPT = new mbed::InterruptIn(_PIN);
      _PIN_INTERRUPT->mode(PullNone);

      _PIN_INTERRUPT->rise( mbed::callback(this,&ULTRASONIC::Rising_edge)  );
      // _PIN_INTERRUPT->fall( mbed::callback(this,&ULTRASONIC::Falling_edge) );
      
    }

    void Enable_digital_mode(){
      // Serial.println("ultra digital");
      if(_PIN_INTERRUPT){
        delete _PIN_INTERRUPT;
        _PIN_INTERRUPT = nullptr;
      }

      _PIN_OUTPUT = new mbed::DigitalOut(_PIN);
    }




  public:
#if ENABLE_MEASUREMENT
    MEASUREMENT         _MEASURE;
#endif
    uint8_t             _FINISH_READING = 0;

    void Falling_edge(){
      // Serial.println("falling");
      _TIMER.stop();
      // _DISTANCE = uSecToCM( std::chrono::duration_cast<std::chrono::microseconds>(_TIMER.elapsed_time()).count() );
      float current_dis = (float) std::chrono::duration_cast<std::chrono::microseconds>(_TIMER.elapsed_time()).count()*0.01715;
      _DISTANCE = 0.9*_DISTANCE + 0.1*current_dis;
      _FINISH_READING = 1;

#if ENABLE_MEASUREMENT
      _MEASURE.record_value(_DISTANCE);
#endif
    }

    void Rising_edge(){
      // Serial.println("rising");
      _TIMER.reset();
      _TIMER.start();

      _PIN_INTERRUPT->fall( mbed::callback(this,&ULTRASONIC::Falling_edge) );
    }


#if ENABLE_MEASUREMENT
    ULTRASONIC(PinName pin) : _PIN(pin){
      _MEASURE = (int)pin;
    }
#else
    ULTRASONIC(PinName pin) : _PIN(pin){}
#endif

    void Enable_auto_restart(){
      _AUTO_RESTART = 1;
    }

    void Disable_auto_restart(){
      _AUTO_RESTART = 0;
    }

    void Begin(){
      // Serial.println("ultra begin");
      _FINISH_READING = 0;
      Enable_digital_mode();
      wait_us(1);
      // raise TRIGGER for 10 us.
      *_PIN_OUTPUT = 0;
      wait_us(1);
      *_PIN_OUTPUT = 1;
      wait_us(15);
      *_PIN_OUTPUT = 0;
      wait_us(1);

      Enable_interrupt_mode();
    }

    float Get_distance(){
      return _DISTANCE;
    }
};

ULTRASONIC Ultrasonic[2] {{ULTRASONIC_PIN1},{ULTRASONIC_PIN2}};
void Ultrasonic_setup(){

  // Serial.println("ultra setup");

  for(int i=0 ; i<2 ; i++){
    // Ultrasonic[i].Enable_auto_restart();
    // Serial.println("in1");
    Ultrasonic[i].Begin();
  }

}

unsigned long ultrasonic_timestamp[2] = {0,0};
void Ultrasonic_loop(){

    for(int i=0 ; i<2 ; i++){
    // Ultrasonic[i].Enable_auto_restart();
    // Serial.println(Ultrasonic[i]._FINISH_READING);
      if(Ultrasonic[i]._FINISH_READING || (millis() - ultrasonic_timestamp[i] > 40)){
        // Serial.println("in2");
        Ultrasonic[i].Begin();
        ultrasonic_timestamp[i] = millis();
      }
    }


}

#endif