# PRBot - Practical Robotic Bot

## Project Overview
PRBot is an Arduino-based robotic platform with sensor fusion, motor control, and communication capabilities. The system runs on mbed OS with multi-threaded sensor and driver loops.

## Project Structure

### Core Files
- **PRBot.ino** - Main entry point; initializes the robot and runs the main loop
- **PRBot.h** - Header file with class definition and pin configurations
- **PRBot.cpp** - Implementation of PRBOT class with setup and control loops
- **communication.h** - Serial communication protocol and telemetry system
- **motor.h/cpp** - Dual motor control with PID feedback and encoder management
- **odometry.h/cpp** - Pose estimation using encoder and gyro sensor fusion

## Hardware Configuration

### Pin Assignments
| Component | Pin | Type |
|-----------|-----|------|
| I2C SDA | P0_31 | Communication |
| I2C SCL | P0_2 | Communication |
| Ultrasonic 1 | P0_23 | Sensor |
| Ultrasonic 2 | P1_14 | Sensor |
| Motor A Direction | P0_4 | Motor |
| Motor A PWM | P0_27 | Motor |
| Motor A Encoder | P1_11 | Motor |
| Motor B Direction | P0_5 | Motor |
| Motor B PWM | P1_2 | Motor |
| Motor B Encoder | P1_12 | Motor |
| LED | P0_13 | Indicator |

### Robot Specifications
- **Wheel Radius**: 2.4 cm (0.024 m)
- **Wheel Separation**: 20 cm (0.20 m)
- **Encoder Ticks per Revolution**: 1788 ticks

## Key Components

### PRBOT Class
The main robot controller class that manages:
- **Motor Control** - Dual motor system with PWM and encoder feedback
- **Odometry** - Wheel-based position and heading estimation
- **Sensors** - 2x Ultrasonic sensors + IR sensor via I2C
- **Communication** - Serial interface (USB or Serial1 selectable)
- **Robot State** - Centralized state management

### Motor Class
Manages dual motor control with integrated PID feedback:
- **PWM Control** - Each motor runs at 100 kHz frequency
- **Encoder Feedback** - Real-time speed measurement from rising/falling edges
- **PID Loop** - Corrects motor speed every 50ms to match target velocities
- **Speed Filtering** - Complementary filter (60% previous + 40% current) for smooth readings
- **Bidirectional Control** - Direction control via GPIO; speed via PWM duty cycle

### Odometry Class
Estimates robot pose using sensor fusion:
- **Inverse Kinematics** - Converts linear (v) and angular (w) velocity commands to per-wheel speeds
- **Encoder Odometry** - Calculates position/heading from accumulated encoder ticks
- **Gyro Fusion** - Complements encoder angle with IMU gyroscope (LSM9DS1) for drift reduction
- **Pose Update** - 100 Hz loop updates X, Y, and Theta using midpoint integration
- **IMU Integration** - Gyro bias calibration; zero-velocity clamping when stationary

### Threading
- **Sensor Loop** (30 Hz) - Reads ultrasonic and IR sensors, updates robot state
- **Driver Loop** (30 Hz) - Processes velocity/angular velocity commands, controls motors
- **Communication Loop** (30 Hz) - Handles serial I/O and telemetry transmission
- **Motor PID Loop** (20 Hz) - Corrects individual wheel speeds via PID controller
- **Odometry Loop** (100 Hz) - Updates pose estimation continuously

## Communication Interface

### Serial Configuration
- **Baud Rate**: 115200 bps
- **Port Selection**: 
  - Set `ENABLEUSBSERIAL 1` for USB Serial
  - Set `ENABLEUSBSERIAL 0` for Serial1 (default)
- **Update Rate**: 30 Hz (33 ms interval)
- **Protocol**: Binary packet-based with checksums

### 1. Telemetry Packet (Robot → Controller)
**Outgoing data from robot to host controller**

| Field | Type | Size | Unit | Description |
|-------|------|------|------|-------------|
| Start Byte | uint8_t | 1 | - | Sync header: 0xA5 |
| Timestamp | uint32_t | 4 | ms | System milliseconds |
| X Position | int16_t | 2 | mm | X coordinate (odometry) |
| Y Position | int16_t | 2 | mm | Y coordinate (odometry) |
| Theta | int16_t | 2 | rad×10000 | Robot heading angle |
| Linear Velocity | int16_t | 2 | mm/s | Actual velocity |
| Angular Velocity | int16_t | 2 | rad/s×1000 | Actual angular velocity |
| Ultrasonic Left | uint16_t | 2 | cm | Left sensor distance |
| Ultrasonic Right | uint16_t | 2 |cm | Right sensor distance |
| IR Front | uint16_t | 2 | cm | Front IR sensor distance |
| Checksum | uint8_t | 1 | - | XOR of timestamp through IR |

**Total Size**: 22 bytes  
**Checksum**: XOR of all bytes from timestamp through IR (indices 1-20)

### 2. Command Packet (Controller → Robot)
**Incoming commands from host controller to robot**

| Field | Type | Size | Unit | Description |
|-------|------|------|------|-------------|
| Start Byte | uint8_t | 1 | - | Sync header: 0x5A |
| V Target | int16_t | 2 | mm/s | Target linear velocity |
| W Target | int16_t | 2 | rad/s×1000 | Target angular velocity |
| Checksum | uint8_t | 1 | - | XOR of V_target and W_target |

**Total Size**: 6 bytes  
**Checksum**: XOR of v_target and w_target bytes

### Special Commands

#### Set Theta (Heading)
```
v_target = -31000 (mm/s)
w_target = theta_value (rad/s × 1000)
```
Sets the robot's heading angle in odometry without moving.

#### Reset Pose
```
v_target = -30000 (mm/s)
w_target = 0
```
Resets robot position to (0, 0, 0).

### Data Scaling Reference
- **Distance**: mm (millimeters)
- **Velocity**: mm/s (millimeters per second)
- **Angle**: radians × 10000 (theta) or radians × 1000 (angular velocity)
- **Conversion**: 1 m/s = 1000 mm/s

### Communication Flow

1. **Robot sends Telemetry** (30 Hz)
   - Packages current pose, velocity, and sensor readings
   - Calculates checksum
   - Transmits 22-byte packet

2. **Controller sends Command** (variable rate)
   - Specifies target linear and angular velocities
   - Calculates checksum
   - Transmits 6-byte packet

3. **Robot receives Command**
   - State machine parses incoming bytes
   - Validates checksum
   - Updates motor targets via odometry module
   - Executes motion at next driver loop cycle

### Error Handling
- **Invalid Checksum**: Packet is discarded
- **Sync Loss**: FSM resets to WAIT_SYNC state
- **Buffer Overflow**: Automatic reset if packet exceeds expected size
- **Fragmented Packets**: FSM handles multi-byte reception correctly

## Dependencies
- mbed.h - mbed OS framework
- Arduino_LSM9DS1.h - IMU gyroscope support
- Custom modules: ultrasonic, infrared, communication, odometry, motor, robotstate
