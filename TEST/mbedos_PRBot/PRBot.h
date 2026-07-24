#ifndef _PRBOT_H
#define _PRBOT_H

#include <mbed.h>
#include "ultrasonic.h"
#include "infrared.h"
#include "communication.h"
#include "odometry.h"
#include "motor.h"
#include "robotstate.h"

#define I2C_SDA P0_31
#define I2C_SCL P0_2
#define ULTRASONIC_PIN1 P0_23
#define ULTRASONIC_PIN2 P1_14

#define MOTORDIRA P0_4
#define MOTORDIRB P0_5
#define MOTORPWMA P0_27
#define MOTORPWMB P1_2
#define MOTORENCODERA P1_11
#define MOTORENCODERB P1_12

#define WHEELRADIUS 0.024       // 2.4cm
#define WHEELSEPARATION 0.1975  // 19.75 cm
#define TICKPERREVOLUTION 1788  // 298*6

#define ENABLEUSBSERIAL 0
#define LEDPIN P0_13

class PRBOT{

public:
  PRBOT();
  void setup();

private:

  mbed::DigitalOut _myled;

  /* Driver */
  void Driver_loop();
  rtos::Thread _thread_driver;

  /* Sensor variables */
  ULTRASONIC    _ultrasonic[2];
  mbed::I2C     _myi2c;
  INFRARED      _infrared;
  rtos::Thread  _thread_sensor;
  void Sensor_loop();

  /* Communication variables */
  COMMUNICATION _communication;

  /* Odometry variables */
  ODOMETRY _odometry;

  /* Motor variables */
  MOTOR _motor;

  /* RobotState variables */
  ROBOTSTATE _robotstate;



};


#endif