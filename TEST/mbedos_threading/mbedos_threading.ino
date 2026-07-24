#include <mbed.h>
#include <ARB.h>
#include "infrared.h"
#include "led.h"
#include "ultrasonic.h"



rtos::Thread  thread_led(osPriorityNormal), 
              thread_infrared(osPriorityNormal),
              thread_display(osPriorityNormal);

void setup() {
  // put your setup code here, to run once:
  while(!Serial)
  Serial.begin(9600);
  // while(!Serial.available());
  Serial.println("setup");

  Ultrasonic_setup();

  // thread_led.start(Led_thread);
  // thread_infrared.start(Infrared_thread);
  // thread_ultrasonic.start(Ultrasonic_thread);

  // thread_display.start(Display_thread);

}

void loop() {
  // put your main code here, to run repeatedly:
  Ultrasonic_loop();
  // Serial.println("test");
  Display();
}

unsigned long display_timestamp_ms = 0;
void Display(){

    // infraed_measurement.display_freq();
    // infraed_measurement.display_sd();
    // infraed_measurement.display_avg();

  if(millis() - display_timestamp_ms > 10){
    for(int i = 0; i<2; i++){
      // Ultrasonic[i]._MEASURE.display_freq();
      // Ultrasonic[i]._MEASURE.display_sd();
      // Ultrasonic[i]._MEASURE.display_avg();
      Serial.print(i);
      Serial.print(":");
      Serial.println(Ultrasonic[i].Get_distance());
    }
    display_timestamp_ms = millis();
  }

    
}












