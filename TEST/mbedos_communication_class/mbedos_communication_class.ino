#include "motor.h"
#include "odometry.h"
#include "robotstate.h"
#include "communication.h"


#define MOTORDIRA P0_4
#define MOTORDIRB P0_5
#define MOTORPWMA P0_27
#define MOTORPWMB P1_2
#define MOTORENCODERA P1_11
#define MOTORENCODERB P1_12

MOTOR Motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);

#define WHEELRADIUS 0.024       // 2.4cm
#define WHEELSEPARATION 0.1975  // 19.75 cm
#define TICKPERREVOLUTION 1788  // 298*6
ODOMETRY Odometry(WHEELRADIUS, WHEELSEPARATION, TICKPERREVOLUTION);

ROBOTSTATE State;
COMMUNICATION Communication;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  while (!Serial)
    ;

  Motor.Begin();
  Odometry.Begin(&Motor, &State);
  Communication.begin(&State, &Serial);
}

void loop() {
  // put your main code here, to run repeatedly:

  float v, w;
  unsigned long last_time;
  State.getTargets(v, w, last_time);
  WheelSpeeds speed = Odometry.calculateWheelSpeeds(v, w);
  Motor.Set_speed(Odometry.meterToTick(speed.left), Odometry.meterToTick(speed.right));

  delay(50);
}
