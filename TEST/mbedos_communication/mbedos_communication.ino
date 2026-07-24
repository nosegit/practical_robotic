#include "ultrasonic.h"
#include "infrared.h"


#define I2C_SDA P0_31
#define I2C_SCL P0_2
mbed::I2C myi2c(I2C_SDA, I2C_SCL);
INFRARED Infrared(&myi2c);

#define ULTRASONIC_PIN1 P0_23
#define ULTRASONIC_PIN2 P1_14
ULTRASONIC Ultrasonic[2]{ { ULTRASONIC_PIN1 }, { ULTRASONIC_PIN2 } };



rtos::Thread thread_sensor;

void setup() {
  // put your setup code here, to run once:

  while (!Serial);
  Serial.begin(9600);
  // Serial1.begin(9600);

  thread_sensor.start(Sensor_read);
}

void loop() {
  // put your main code here, to run repeatedly:

  // Send_sensor_data();

  Serial.print("Infrared :");
  Serial.print(Infrared.Get_distance());
  Serial.print(", Ultrasonic0 :");
  Serial.print((int)Ultrasonic[0].Get_distance());
  Serial.print(", Ultrasonic1 :");
  Serial.println((int)Ultrasonic[1].Get_distance());

  // delay(100);
}


void Send_sensor_data(){

  float infra_dis = Infrared.Get_distance();
  float ultra_dis[2] = {Ultrasonic[0].Get_distance(), Ultrasonic[1].Get_distance()}; 

  byte message[5];
  message[0] = 's';
  message[1] = infra_dis < 256 ? uint8_t(infra_dis):255;
  message[2] = ultra_dis[0] < 256 ? uint8_t(ultra_dis[0]):255;
  message[3] = ultra_dis[1] < 256 ? uint8_t(ultra_dis[1]):255;
  message[4] = '#';

  Serial1.write(message,5);

}


void Sensor_read() {

  for (;;) {
    Infrared.Begin();

    for (int i = 0; i < 2; i++) {
      Ultrasonic[i].Begin();
    }

    rtos::ThisThread::sleep_for(100);  // 10Hz
  }
}