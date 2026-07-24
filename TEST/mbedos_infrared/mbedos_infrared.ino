#include <mbed.h>
#include "infrared.h"

#define I2C_SDA P0_31
#define I2C_SCL P0_2
mbed::I2C myi2c(I2C_SDA,I2C_SCL);
INFRARED Infrared(&myi2c);

unsigned long timestamp_ms = 0;
void setup() {
  // put your setup code here, to run once:
  while(!Serial);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  Infrared.Begin();

  if(millis() - timestamp_ms > 1000){
    Infrared._MEASURE.display_avg();
    Infrared._MEASURE.display_freq();
    timestamp_ms = millis();
  }

}
