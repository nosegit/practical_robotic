#include "PRBot.h"


PRBOT::PRBOT() :
  _myi2c( I2C_SDA,I2C_SCL ),
  _infrared( &_myi2c ),
  _ultrasonic{ {ULTRASONIC_PIN1}, {ULTRASONIC_PIN2} },
  _motor(MOTORDIRA, MOTORPWMA, MOTORENCODERA, MOTORDIRB, MOTORPWMB, MOTORENCODERB),
  _odometry(WHEELRADIUS, WHEELSEPARATION, TICKPERREVOLUTION),
  _myled(LEDPIN)
{
}

void PRBOT::setup(){

#if ENABLEUSBSERIAL
  Stream& _mySerial = Serial;
  Serial.begin(115200);
#else
  Stream& _mySerial = Serial1;
  Serial1.begin(115200);
#endif

  _motor.Begin();
  _odometry.Begin(&_motor, &_robotstate);
  _communication.begin(&_robotstate, &_mySerial);

  _thread_sensor.start(mbed::callback(this,&PRBOT::Sensor_loop));
  _thread_driver.start(mbed::callback(this,&PRBOT::Driver_loop));


  _myled = 1;

}

void PRBOT::Sensor_loop(){
    for (;;) {
    _infrared.Begin();

    for (int i = 0; i < 2; i++) {
      _ultrasonic[i].Begin();
    }

    _robotstate.setSensors( _ultrasonic[0].Get_distance(), 
                            _ultrasonic[1].Get_distance(), 
                            _infrared.Get_distance() );

    rtos::ThisThread::sleep_for(100);  // 10Hz
  }
}

void PRBOT::Driver_loop(){
  for(;;){
    float v, w;
    unsigned long last_time;
    _robotstate.getTargets(v, w, last_time);
    WheelSpeeds speed = _odometry.calculateWheelSpeeds(v, w);
    _motor.Set_speed(_odometry.meterToTick(speed.left), _odometry.meterToTick(speed.right));

    rtos::ThisThread::sleep_for(33);  // 30Hz
  }

}