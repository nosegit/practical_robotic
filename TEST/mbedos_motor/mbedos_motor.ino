#include "motor.h"
#include <mbed.h>

#define MOTORDIRA       P0_4
#define MOTORDIRB       P0_5
#define MOTORPWMA       P0_27
#define MOTORPWMB       P1_2
#define MOTORENCODERA   P1_11
#define MOTORENCODERB   P1_12

MOTOR Motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB);

void setup() {

  // while(!Serial);
  Serial.begin(9600);

  Motor.Begin();
  // Motor.Set_frquency(0,100);  // 100 Hz

  // for(int i = 0 ; i < 4 ; i++){
    // motor_test_basic();
  //   delay(500);
  // }

  motor_test_basic2();

  // motor_test_10_cm();
  // for(int i = 0 ; i < 4; i++){
  //   motor_test_90_degree();
  //   delay(500);
  // }
  // motor_test_single_revolution();

}

void loop() {
  // put your main code here, to run repeatedly:

  // Motor.Set_pwm(1, 50);
  // Motor.Print_info();

  // motor_frequency_test();

}

unsigned long motor_frequency_timestamp_us;
void motor_frequency_test(){

  if(micros() - motor_frequency_timestamp_us > 1e5){ // 10hz
    Motor.Set_pwm(1, 20);
    Motor.Set_pwm(0, 20);
    Motor.Print_info();
    motor_frequency_timestamp_us = micros();
  }
}


void motor_test_basic(){
    Motor.Set_dir(0, 0);
    Motor.Set_dir(1, 0);

    for(int i = 0; i < 100; i++){
      Serial.print(i);
      Serial.print(",");
      Motor.Set_pwm(0, i);
      Motor.Set_pwm(1, i);

      // Motor.Print_info();
      Motor.Print_info_CSV();
      delay(100);

    }
    Motor.Set_pwm(0, 0);
    Motor.Set_pwm(1, 0);
    delay(500);

    Motor.Set_dir(0, 1);
    Motor.Set_dir(1, 1);

    for(int i = 0; i < 100; i++){
      Serial.print(-i);
      Serial.print(",");
      Motor.Set_pwm(0, i);
      Motor.Set_pwm(1, i);

      // Motor.Print_info();
      Motor.Print_info_CSV();
      delay(100);
    }
    // delay(1000);

    Motor.Set_pwm(0, 0);
    Motor.Set_pwm(1, 0);

    // Motor.Print_info();
}


#define GEAR_RATIOA 298
#define WHEEL_REV_ENCA 2*3*GEAR_RATIOA
#define WHEEL_DIAMETER 4.7
#define WHEEL_CIRCUMFERENCE WHEEL_DIAMETER*3.14
#define WHEEL_ENC_CM_RATIO 2*60.548389609  // WHEEL_REV_ENCA/WHEEL_CIRCUMFERENCE -> 3*298/(4.7*3.1415) 

void motor_test_10_cm(){

  long long enc_start_a = Motor.Get_encoder_cnt(0);
  long long enc_start_b = Motor.Get_encoder_cnt(1);

  Motor.Set_dir(0, 0);
  Motor.Set_dir(1, 0);

  Motor.Set_pwm(0, 60);
  Motor.Set_pwm(1, 50);

  long long target = WHEEL_ENC_CM_RATIO*10;

  move_to_target(target, target);
  Motor.Set_pwm(0, 0);
  Motor.Set_pwm(1, 0);

}

void motor_test_90_degree(){

  long long enc_start_a = Motor.Get_encoder_cnt(0);
  long long enc_start_b = Motor.Get_encoder_cnt(1);

  Motor.Set_dir(0, 0);
  Motor.Set_dir(1, 0);

  Motor.Set_pwm(0, 35);
  Motor.Set_pwm(1, 35);

  long long target = WHEEL_ENC_CM_RATIO* 3.1415*19.75/4;

  move_to_target(enc_start_a-target, enc_start_b+target);
  Motor.Set_pwm(0, 0);
  Motor.Set_pwm(1, 0);

}

void move_to_target(long long targeta, long long targetb){

  long long enc_start_a = Motor.Get_encoder_cnt(0);
  long long enc_start_b = Motor.Get_encoder_cnt(1);

  Motor.Set_dir(0, targeta > enc_start_a ? 0 : 1);
  Motor.Set_dir(1, targetb > enc_start_b ? 0 : 1);

  Motor.Set_pwm(0, 50);
  Motor.Set_pwm(1, 50);

  while ( target_compare(Motor.Get_encoder_cnt(0), targeta)==0 && target_compare(Motor.Get_encoder_cnt(1), targetb)==0){
      Motor.Print_info();
      
      if(target_compare(Motor.Get_encoder_cnt(0), targeta)==1){
        Motor.Set_pwm(0,0);
      }

      if(target_compare(Motor.Get_encoder_cnt(1), targetb)==1){
        Motor.Set_pwm(1,0);
      }
    }
    Motor.Set_pwm(0, 0);
    Motor.Set_pwm(1, 0);
}

uint8_t target_compare(long long input, long long target){ // 0: not reach, 1: reach
  if(target >= 0){
    return input < target ? 0 : 1;
  }
  else{
    return input < target ? 1 : 0;
  }
}

void motor_test_single_revolution(){

  Motor.Set_dir(0, 0);
  Motor.Set_dir(1, 0);

  while(Motor.Get_encoder_cnt(0) <= 3*298 ){
    Motor.Set_pwm(0, 50);
    Motor.Print_info();
  }
  Motor.Set_pwm(0,0);
  Motor.Print_info();

  while(Motor.Get_encoder_cnt(1) <= 3*298 ){
    Motor.Set_pwm(1, 50);
    Motor.Print_info();
  }
  Motor.Set_pwm(1,0);
  Motor.Print_info();

}


