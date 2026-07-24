#include "ultrasonic.h"

#define ULTRASONIC_PIN1 P0_23
#define ULTRASONIC_PIN2 P1_14

ULTRASONIC Ultrasonic[2]{ { ULTRASONIC_PIN1 }, { ULTRASONIC_PIN2 } };
unsigned long ultrasonic_timestamp[2] = { 0, 0 };
unsigned long timestamp_ms = 0;

void setup() {
  // put your setup code here, to run once:
  while(!Serial);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  for (int i = 0; i < 2; i++) {
    // Ultrasonic[i].Enable_auto_restart();
    // Serial.println(Ultrasonic[i]._FINISH_READING);
    if (Ultrasonic[i]._FINISH_READING || (millis() - ultrasonic_timestamp[i] > 1000)) {
      // Serial.println("in2");
      Ultrasonic[i].Begin();
      ultrasonic_timestamp[i] = millis();
    }
  }

  if(millis() - timestamp_ms > 1000){
    Ultrasonic[0]._MEASURE.display_avg();
    Ultrasonic[1]._MEASURE.display_avg();
    timestamp_ms = millis();
  }
}
