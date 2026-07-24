#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include <mbed.h>
#include <Arduino_LSM9DS1.h>
#include "motor.h"
#include "robotstate.h"
using namespace std::chrono;

// A simple structure to return two values
struct WheelSpeeds {
    float left;  // target speed in m/s
    float right; // target speed in m/s
};

// A simple structure to hold the robot's pose
struct Pose {
    float x;     // m
    float y;     // m
    float theta; // radians
};

class ODOMETRY{
public:
  // --- Constructor ---
  // Pass in your robot's physical constants
  ODOMETRY(float wheel_radius, float wheel_separation, float ticks_per_revolution);

  // --- 1. Inverse Kinematics ---
  // Takes your 'v' (linear) and 'w' (angular) commands...
  // ...and returns the target speeds for each wheel (in m/s).
  WheelSpeeds calculateWheelSpeeds(float linear_vel, float angular_vel);

  // --- Helpers ---
  Pose getPose() { return _pose; }; // Get the current pose
  void reset() { _pose = {0,0,0}; };     // Reset the pose (x, y, theta) to zero
  void set_theta(float theta) { _pose.theta = theta; } // force set theta.
  float meterToTick(float input);  // convert m/s to ticks/s
  void printPose();

  void Begin(MOTOR* motor_ptr, ROBOTSTATE* state_prt);




private:

  ROBOTSTATE* _STATE;

  // --- Robot Constants ---
  float _wheel_radius;
  float _wheel_separation;
  float _ticks_per_revolution;

  // --- Pre-calculated Values ---
  float _meters_per_tick; // (2 * PI * R) / Ticks
  float _wheel_circumference; // 2*pi*r

  // --- Internal State ---
  Pose _pose; // The robot's current pose
  long _last_ticks_left;
  long _last_ticks_right;
  float _linear_velocity_m;
  float _angular_velocity_m;


  // --- Thread ---
  const milliseconds TARGET_LOOP_PERIOD = 10ms;
  rtos::Thread _THREAD;
  MOTOR* _MOTOR;
  void loop();

  // --- IMU ---
  float _gyro_bias;
  const float GYRO_DEADBAND_RADS = 2 * ( PI / 180.0 );
  const float GYRO_SCALE = 1.11875; // 360/316

  // --- 2. Forward Kinematics (Odometry) ---
  // Takes the *total* accumulated encoder ticks...
  // ...and updates the robot's internal X, Y, and Theta pose.
  void updatePose(long total_ticks_left, long total_ticks_right, float gyro_z_rads, float dt);
  unsigned long _prev_micros;
  float _last_gyro_z;


};

#endif