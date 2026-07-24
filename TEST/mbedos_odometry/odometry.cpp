#include "odometry.h"

ODOMETRY::ODOMETRY(float wheel_radius, float wheel_separation, float ticks_per_revolution):
  _wheel_radius(wheel_radius),
  _wheel_separation(wheel_separation),
  _ticks_per_revolution(ticks_per_revolution),
  _THREAD(osPriorityHigh)
{
  _wheel_circumference = 2 * PI * _wheel_radius;
  _meters_per_tick = _wheel_circumference / _ticks_per_revolution;
}

WheelSpeeds ODOMETRY::calculateWheelSpeeds(float linear_vel, float angular_vel){

  WheelSpeeds Speed;
  // --- Step 1: Inverse Kinematics ---
  // Calculate the required linear speed (m/s) for each wheel
  Speed.left  = linear_vel - (angular_vel * _wheel_separation) / 2.0;
  Speed.right = linear_vel + (angular_vel * _wheel_separation) / 2.0;

  return Speed;
}

float ODOMETRY::meterToTick(float input){
  return (input/_wheel_circumference)*_ticks_per_revolution;
}

void ODOMETRY::updatePose(long total_ticks_left, long total_ticks_right){


  // calculate delta
  long distance_tick_l = total_ticks_left - _last_ticks_left;
  long distance_tick_r = total_ticks_right - _last_ticks_right;

  // save previous encoder value
  _last_ticks_left = total_ticks_left;
  _last_ticks_right = total_ticks_right;

  // convert to distance in meters
  float distance_meter_l = distance_tick_l * _meters_per_tick;
  float distance_meter_r = distance_tick_r * _meters_per_tick;

  // calculate center motion
  float delta_distance_center = (distance_meter_r + distance_meter_l)/2.0;
  float delta_theta = (distance_meter_r - distance_meter_l)/ _wheel_separation;

  // Serial.print(distance_tick_l);
  // Serial.print(",");
  // Serial.print(distance_tick_r);
  // Serial.print(",");
  // Serial.print(distance_meter_l);
  // Serial.print(",");
  // Serial.print(distance_meter_r);
  // Serial.print(",");
  // Serial.print(delta_distance_center);
  // Serial.print(",");
  // Serial.println(delta_theta);


  // update global pose (x, y , thta) 
  // using Midpoint Intergration for better accuracy on curves
  _pose.x += delta_distance_center * cos(_pose.theta + (delta_theta/2.0));
  _pose.y += delta_distance_center * sin(_pose.theta + (delta_theta/2.0));
  _pose.theta += delta_theta;

  // normalize Theta to -PI to PI
  if( _pose.theta > M_PI ) { _pose.theta -= 2*M_PI; }
  if( _pose.theta < -M_PI ) { _pose.theta += 2*M_PI; }

}

void ODOMETRY::Begin(MOTOR* motor_ptr){

  _MOTOR = motor_ptr;

  _THREAD.start(mbed::callback(this, &ODOMETRY::loop));

}

void ODOMETRY::loop(){

  for(;;){

    // calculate POSE
    updatePose(_MOTOR->Get_encoder_cnt(0), _MOTOR->Get_encoder_cnt(1));

    // sleep for 10 ms
    rtos::ThisThread::sleep_for(TARGET_LOOP_PERIOD);

  }

}

void ODOMETRY::printPose(){

  Serial.print("[X: ");
  Serial.print(_pose.x, 4); // The '4' specifies 4 decimal places
  Serial.print(", Y: ");
  Serial.print(_pose.y, 4); 
  Serial.print(", Theta: ");
  Serial.print(_pose.theta, 4);
  Serial.println("]");

}




