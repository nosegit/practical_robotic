#include "PRBot.h"


PRBOT::PRBOT() :
  _myi2c( I2C_SDA,I2C_SCL ),
  _infrared( &_myi2c ),
  _ultrasonic{ {ULTRASONIC_PIN1}, {ULTRASONIC_PIN2} }
{
}

void PRBOT::setup(){

  _thread_sensor.start(mbed::callback(this,&PRBOT::Sensor_loop));

}

void PRBOT::Sensor_loop(){
    for (;;) {
    _infrared.Begin();

    for (int i = 0; i < 2; i++) {
      _ultrasonic[i].Begin();
    }

    rtos::ThisThread::sleep_for(100);  // 10Hz
  }
}