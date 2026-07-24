#ifndef _PRBOT_H
#define _PRBOT_H

#include <mbed.h>
#include "Callback.h"
#include "ultrasonic.h"
#include "infrared.h"

#define I2C_SDA P0_31
#define I2C_SCL P0_2
#define ULTRASONIC_PIN1 P0_23
#define ULTRASONIC_PIN2 P1_14

class PRBOT{

public:
  PRBOT();
  void setup();

private:

  /* Sensor variables */
  ULTRASONIC    _ultrasonic[2];
  mbed::I2C     _myi2c;
  INFRARED      _infrared;
  rtos::Thread  _thread_sensor;
  void Sensor_loop();


};


#endif