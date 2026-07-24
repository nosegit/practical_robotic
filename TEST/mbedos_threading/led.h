#ifndef _LED_H
#define _LED_H

#define LED_PIN P0_13
mbed::DigitalOut myled(LED_PIN);

void Led_thread(){
  
  while(1){
    myled = !myled;
    rtos::ThisThread::sleep_for(1000);
  }
}

#endif