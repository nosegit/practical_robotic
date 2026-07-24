#include "ultrasonic.h"

void ULTRASONIC::Enable_interrupt_mode() {
  if (_PIN_OUTPUT) {
    delete _PIN_OUTPUT;
    _PIN_OUTPUT = nullptr;
  }

  _PIN_INTERRUPT = new mbed::InterruptIn(_PIN);
  _PIN_INTERRUPT->mode(PullNone);

  _PIN_INTERRUPT->rise(mbed::callback(this, &ULTRASONIC::Rising_edge));
}

void ULTRASONIC::Enable_digital_mode() {

  if (_PIN_INTERRUPT) {
    delete _PIN_INTERRUPT;
    _PIN_INTERRUPT = nullptr;
  }

  _PIN_OUTPUT = new mbed::DigitalOut(_PIN);
}

void ULTRASONIC::Falling_edge() {
  _TIMER.stop();
  // _DISTANCE = uSecToCM( std::chrono::duration_cast<std::chrono::microseconds>(_TIMER.elapsed_time()).count() );
  float current_dis = (float)std::chrono::duration_cast<std::chrono::microseconds>(_TIMER.elapsed_time()).count() * 0.01715;
  _DISTANCE = 0.9 * _DISTANCE + 0.1 * current_dis;
  _FINISH_READING = 1;

#if ENABLE_MEASUREMENT
  _MEASURE.record_value(_DISTANCE);
#endif
}

void ULTRASONIC::Rising_edge() {
  // Serial.println("rising");
  _TIMER.reset();
  _TIMER.start();

  _PIN_INTERRUPT->fall(mbed::callback(this, &ULTRASONIC::Falling_edge));
}

void ULTRASONIC::Begin() {
  _FINISH_READING = 0;
  ULTRASONIC::Enable_digital_mode();
  wait_us(1);
  // raise TRIGGER for 10 us.
  *_PIN_OUTPUT = 0;
  wait_us(1);
  *_PIN_OUTPUT = 1;
  wait_us(15);
  *_PIN_OUTPUT = 0;
  wait_us(1);

  ULTRASONIC::Enable_interrupt_mode();
}

float ULTRASONIC::Get_distance() {
  return _DISTANCE;
}