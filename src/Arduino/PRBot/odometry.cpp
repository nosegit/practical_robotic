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

// UPDATE: Added gyro_z (in Radians/sec) and dt (in Seconds)
void ODOMETRY::updatePose(long total_ticks_left, long total_ticks_right, float gyro_z_rads, float dt){

  // --- 1. ENCODER CALCULATIONS ---
  long distance_tick_l = total_ticks_left - _last_ticks_left;
  long distance_tick_r = total_ticks_right - _last_ticks_right;

  // Save previous encoder value
  _last_ticks_left = total_ticks_left;
  _last_ticks_right = total_ticks_right;

  // Convert to distance in meters
  float distance_meter_l = distance_tick_l * _meters_per_tick;
  float distance_meter_r = distance_tick_r * _meters_per_tick;

  // Calculate Center Distance (Linear Motion)
  float delta_distance_center = (distance_meter_r + distance_meter_l) / 2.0;

  // Calculate Encoder Delta Theta (The "Noisy" but "Drift-Free" Rotation)
  float delta_theta_enc = (distance_meter_r - distance_meter_l) / _wheel_separation;


  // --- 2. GYRO CALCULATIONS (Trapezoidal Integration) ---
  // Average the previous gyro rate and current gyro rate to handle acceleration
  float avg_gyro_rate = (_last_gyro_z + gyro_z_rads) / 2.0;
  
  // Calculate Gyro Delta Theta (The "Smooth" but "Drifting" Rotation)
  float delta_theta_gyro = avg_gyro_rate * dt;

  // Save current gyro for next loop's "previous"
  _last_gyro_z = gyro_z_rads;


  // --- 3. SENSOR FUSION (Complementary Filter) ---
  // We fuse the CHANGES in angle (Deltas), not the absolute angle.
  // Alpha is usually 0.95 to 0.98 (Trust Gyro for fast turns, Encoder for slow drift)
  // const float ALPHA = 0.99; 
  const float ALPHA = 0.0; 
  // const float ALPHA = 1.0; 


  float delta_theta_fused = (ALPHA * delta_theta_gyro) + ((1.0 - ALPHA) * delta_theta_enc);

  // --- 4. UPDATE GLOBAL POSE ---
  // Use the FUSED theta for the heading update
  // Use Midpoint Integration for X/Y (using half the fused change)
  _pose.x += delta_distance_center * cos(_pose.theta + (delta_theta_fused / 2.0));
  _pose.y += delta_distance_center * sin(_pose.theta + (delta_theta_fused / 2.0));

  // update linear and angular velocity;
  _linear_velocity_m = delta_distance_center;
  _angular_velocity_m = delta_theta_fused;
  
  // Update Heading
  _pose.theta += delta_theta_fused;

  // --- 5. NORMALIZE THETA ---
  if( _pose.theta > M_PI ) { _pose.theta -= 2*M_PI; }
  if( _pose.theta < -M_PI ) { _pose.theta += 2*M_PI; }
}

void ODOMETRY::Begin(MOTOR* motor_ptr, ROBOTSTATE* state_prt){

  _MOTOR = motor_ptr;
  _STATE = state_prt;

  // 1. CALIBRATION ROUTINE
  if (!IMU.begin()) { /* Handle Error */ }
  
  // // Take 1000 samples to find the "Zero" offset
  // float sum = 0;
  // for(int i=0; i<1000; i++) {
  //   while (!IMU.gyroscopeAvailable()); // Wait for data
  //   float x, y, z;
  //   IMU.readGyroscope(x, y, z);
  //   sum += z;
  // }
  // _gyro_bias = sum / 1000.0;

  _THREAD.start(mbed::callback(this, &ODOMETRY::loop));

}

void ODOMETRY::loop(){

  for(;;){

    // --- PHASE 1: HARDWARE ---
    // A. Read Motors (Fast, Atomic)
    long ticks_L, ticks_R;
    ticks_L = _MOTOR->Get_encoder_cnt(0);
    ticks_R = _MOTOR->Get_encoder_cnt(1);

    // B. Read Gyro (Slow, I2C)
    float gx, gy, gz;
    
    IMU.readGyroscope(gx, gy, gz);

    // --- PHASE 2: PRE-PROCESSING ---
    // Convert units here, keep the math function clean
    float gz_rads = (gz - _gyro_bias) * (PI / 180.0);
    gz_rads *= GYRO_SCALE;

    // APPLY DEADBAND and Zero Velocity Clamp
    // Deadband to filter noise while moving straight,
    // Zero Velocity Clamp to lock the heading while standing still
    // If the rotation is extremely slow or encoder not moving, force it to exactly 0.0
    long diff_L = ticks_L - _last_ticks_left;
    long diff_R = ticks_R - _last_ticks_right;
    // if ( (abs(gz_rads) < GYRO_DEADBAND_RADS) || ((diff_L == 0) && (diff_R == 0))  ) {
    //     gz_rads = 0.0;
    // }

    // if ( (abs(gz_rads) < GYRO_DEADBAND_RADS)) {
    //     gz_rads = 0.0;
    // }

    if ( ((diff_L == 0) && (diff_R == 0))  ) {
        gz_rads = 0.0;
    }


    // Calculate precise DT
    unsigned long current_micros = micros();
    float dt = (current_micros - _prev_micros) / 1000000.0;
    _prev_micros = current_micros;

    // --- PHASE 3: ALGORITHM (The Math) ---
    // This function knows NOTHING about I2C or Libraries.
    // It only knows numbers.
    updatePose(ticks_L, ticks_R, gz_rads, dt);

    if(_STATE){
      _STATE->setPose(_pose.x, _pose.y, _pose.theta);
      _STATE->setVelocity(_linear_velocity_m, _angular_velocity_m);
    }

    // --- PHASE 4: TIMING ---
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




