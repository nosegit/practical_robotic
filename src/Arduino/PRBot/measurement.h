#ifndef _MEASUREMENT_H
#define _MEASUREMENT_H
#include <Arduino.h>

class MEASUREMENT{

  private:
    uint32_t timestamp;
    int value[10];
    int value_ind = 0;
    int avg = 0;
    float freq = 0;
    float sd = 0;
    int _ID;

  public:
    MEASUREMENT(int ID) : timestamp(0),value_ind(0),freq(0),_ID(ID){}
    MEASUREMENT() : timestamp(0),value_ind(0),freq(0){}

  void set_ID(int id){
    _ID = id;
  }

  void record_value(int input_value){

    value[value_ind++] = input_value;

    value_ind %= 10;
    
    freq = (float)1e6/(float)(micros()-timestamp);
    timestamp = micros();

    cal_sd();

  }

  void display_freq(){
    Serial.print(_ID);
    Serial.print("FREQ:");
    Serial.println((int)freq);    
  }

  void cal_avg(){
    
    for(int i = 0; i < 10; i++){
      avg += value[i];
    }
    avg /= 10;
  }

  void cal_sd(){

    cal_avg();

    sd = 0;
    for(int i = 0; i < 10; i++){
      int diff = value[i]-avg;
      sd += diff*diff/10;
    }
    sd = sqrt(sd); 

  }

  void display_sd(){
    Serial.print(_ID);
    Serial.print("SD:");
    Serial.println(sd); 
  }

  void display_avg(){
    Serial.print(_ID);
    Serial.print("Avg:");
    Serial.println(avg); 
  }
};

#endif