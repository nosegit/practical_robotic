#include "motor.h"
#include "odometry.h"


#define MOTORDIRA       P0_4
#define MOTORDIRB       P0_5
#define MOTORPWMA       P0_27
#define MOTORPWMB       P1_2
#define MOTORENCODERA   P1_11
#define MOTORENCODERB   P1_12

MOTOR Motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);

#define WHEELRADIUS       0.024   // 2.4cm
#define WHEELSEPARATION   0.1975  // 19.75 cm
#define TICKPERREVOLUTION 1788    // 298*6
ODOMETRY Odometry(WHEELRADIUS, WHEELSEPARATION, TICKPERREVOLUTION);


void setup() {
  // put your setup code here, to run once:

  while(!Serial);
  Serial.begin(9600);

  Motor.Begin();
  Odometry.Begin(&Motor);

  test_90degree();

  // test_10cm();


}

void loop() {
  // put your main code here, to run repeatedly:

}

void test_10cm(){

  WheelSpeeds speed = Odometry.calculateWheelSpeeds(0.025, 0);
  Motor.Set_speed( Odometry.meterToTick(speed.left), Odometry.meterToTick(speed.right) );

  while(1){

    Odometry.printPose();

    Pose current_pose = Odometry.getPose();
    if( current_pose.x >= 0.10 ) { break; }


  }
  Motor.Set_speed(0,0);
  Serial.println("Done");
}

void test_90degree(){
  WheelSpeeds speed = Odometry.calculateWheelSpeeds(0, 0.25);
  Motor.Set_speed( Odometry.meterToTick(speed.left), Odometry.meterToTick(speed.right) );

  while(1){

    Odometry.printPose();

    Pose current_pose = Odometry.getPose();
    if( current_pose.theta >= M_PI/2 ) { break; }


  }
  Motor.Set_speed(0,0);
  Serial.println("Done");
}
