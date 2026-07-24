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

  while(!Serial);
  Serial.begin(9600);

  Motor.Begin();
  Motor.Set_frquency(0,100);  // 100 Hz

  Motor.Set_dir(0, 0);
  Motor.Set_dir(1, 0);

  for(int i = 0; i < 100; i++){
    Motor.Set_pwm(0, i);
    Motor.Set_pwm(1, i);

    Serial.print(0);
    Serial.print(":");
    Serial.println(Motor.Get_speed(0));

    Serial.print(1);
    Serial.print(":");
    Serial.println(Motor.Get_speed(1));
    delay(50);

  }
  delay(1000);

  Motor.Set_dir(0, 1);
  Motor.Set_dir(1, 1);

  for(int i = 0; i < 100; i++){
    Motor.Set_pwm(0, i);
    Motor.Set_pwm(1, i);

    Serial.print(0);
    Serial.print(":");
    Serial.println(Motor.Get_speed(0));

    Serial.print(1);
    Serial.print(":");
    Serial.println(Motor.Get_speed(1));
    delay(50);
  }
  delay(1000);

  Motor.Set_pwm(0, 0);
  Motor.Set_pwm(1, 0);

}

void loop() {
  // put your main code here, to run repeatedly:

}
