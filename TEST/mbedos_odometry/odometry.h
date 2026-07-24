#ifndef _ODOMETRY_H
#define _ODOMETRY_H

#include <mbed.h>

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

  // --- 2. Forward Kinematics (Odometry) ---
  // Takes the *total* accumulated encoder ticks...
  // ...and updates the robot's internal X, Y, and Theta pose.
  void updatePose(long total_ticks_left, long total_ticks_right);

  // --- Helpers ---
  Pose getPose(); // Get the current pose
  void reset();     // Reset the pose (x, y, theta) to zero
  float convertMetersPerSecToTicksPerSec(float input);  // convert m/s to ticks/s

private:
  // --- Robot Constants ---
  float _wheel_radius;
  float _wheel_separation;
  float _ticks_per_revolution;

  // --- Pre-calculated Values ---
  float _distance_per_tick; // (2 * PI * R) / Ticks
  float _wheel_circumference; // 2*pi*r

  // --- Internal State ---
  Pose _pose; // The robot's current pose
  long _last_ticks_left;
  long _last_ticks_right;


};

#endif