#include "odometry.h"

ODOMETRY::ODOMETRY(float wheel_radius, float wheel_separation, float ticks_per_revolution):
  _wheel_radius(wheel_radius),
  _wheel_separation(wheel_separation),
  _ticks_per_revolution(ticks_per_revolution)
{
  _wheel_circumference = 2 * PI * _wheel_radius;
  _distance_per_tick = _wheel_circumference / _ticks_per_revolution;
}

WheelSpeeds ODOMETRY::calculateWheelSpeeds(float linear_vel, float angular_vel){

  WheelSpeeds Speed;
  // --- Step 1: Inverse Kinematics ---
  // Calculate the required linear speed (m/s) for each wheel
  Speed.left  = linear_vel - (angular_vel * _wheel_separation) / 2.0;
  Speed.right = linear_vel + (angular_vel * _wheel_separation) / 2.0;
}

float ODOMETRY::convertMetersPerSecToTicksPerSec(float input){
  return (input/_wheel_circumference)*_ticks_per_revolution;
}

void ODOMETRY::updatePose(long total_ticks_left, long total_ticks_right){

}




