#include <mbed.h>
#include "motor.h"
#include "Pid.h"

#define MOTORDIRA       P0_4
#define MOTORDIRB       P0_5
#define MOTORPWMA       P0_27
#define MOTORPWMB       P1_2
#define MOTORENCODERA   P1_11
#define MOTORENCODERB   P1_12

MOTOR Motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);
long long MotorA_speed, MotorB_speed;
unsigned long motor_speed_timestamp_us;
//  pid   Kp  Ki  Kd  sT    tau   min  max
Pid pidL( 5,  10,  0,  1000, 0.1,  -100,100);
// Pid pidR( 100,  10,  0,  0.01, 0.1,  -100,100);
Pid pidR( 5,  0,  0,  0.05, 0.1,  -100,100);
// Pid pidR( 10, 12,  0.1,  0.001, 0.1,  -100,100);



#define GEAR_RATIOA 298
#define WHEEL_REV_ENCA 2*3*GEAR_RATIOA
#define WHEEL_DIAMETER 4.7
#define WHEEL_CIRCUMFERENCE WHEEL_DIAMETER*3.14
#define WHEEL_ENC_CM_RATIO 2*60.548389609  // WHEEL_REV_ENCA/WHEEL_CIRCUMFERENCE -> 3*298/(4.7*3.1415)  cnt/cm

// float myfloat = WHEEL_CIRCUMFERENCE;
float myfloat = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Motor.Begin();
  // Motor.Set_frquency(0,100);  // 100 Hz

  pidL.Begin();
  pidR.Begin();
  
  Motor.Set_dir(1, 0);
 
}

void loop() {
  // put your main code here, to run repeatedly:

  serialfloatin();
  //  Motor.Set_pwm(1, 10);
   
  //  Motor.Print_info();
  motor_speed_update();
  pid_update(myfloat,myfloat);

}

void pid_update(float speedL, float speedR){
  

  Pid::PID_STATUS statusL = pidL.Update(speedL, MotorA_speed/(WHEEL_ENC_CM_RATIO)); // cm/s
  Pid::PID_STATUS statusR = pidR.Update(speedR, MotorB_speed/(WHEEL_ENC_CM_RATIO)); // cm/s
  
  if(statusL == Pid::PID_STATUS::UPDATED){
    Motor.Set_dir(0, pidL.out > 0 ? 0:1);
    Motor.Set_pwm(0, abs(pidL.out));
    Serial.print("0pid:");
    Serial.println(pidL.out);
  }

  if(statusR == Pid::PID_STATUS::UPDATED){
    Motor.Set_dir(1, pidR.out > 0 ? 0:1);
    Motor.Set_pwm(1, abs(pidR.out));
    // Motor.Print_info();
  }
}

void motor_speed_update(){
  if(micros() - motor_speed_timestamp_us > 50e3){ // 20hz
    MotorA_speed = 0.2 * Motor.Get_speed(0) + 0.8*MotorA_speed;
    MotorB_speed = 0.2 * Motor.Get_speed(1) + 0.8*MotorB_speed;
    motor_speed_timestamp_us = micros();
    Serial.print("1speed:");
    Serial.println(MotorB_speed/(WHEEL_ENC_CM_RATIO));
    Serial.print("1setpoint:");
    Serial.println(myfloat);
    Serial.print("1pid:");
    Serial.println(pidR.out);
  }
}

void serialfloatin(){
  if (Serial.available() > 0) {
    
  // Read all characters until a newline ('\n') is received
  String serialDataIn = Serial.readStringUntil('\n');

  // Convert the received String to a float
  myfloat = serialDataIn.toFloat();

  // Print the float back to confirm it was stored correctly
  Serial.print("You entered: ");
  Serial.println(myfloat);
  }
}
