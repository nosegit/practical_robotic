#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <mbed.h>
#include <rtos.h>

struct TelemetryData {
    // Pose
    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
    
    // Velocity
    float v_actual = 0.0;
    float w_actual = 0.0;

    // Sensors
    float us_left = 0.0;
    float us_right = 0.0;
    float ir_front = 0.0;
    float batt_voltage = 0.0;
    
    // Status
    int system_status = 0;
};

class ROBOTSTATE {
private:
    TelemetryData _data;
    rtos::Mutex _lock;

public:
    // SETTERS (Used by Subsystems)
    void setPose(float x, float y, float t) {
        _lock.lock();
        _data.x = x;
        _data.y = y;
        _data.theta = t;
        _lock.unlock();
    }

    void setVelocity(float v, float w){
      _lock.lock();
      _data.v_actual = v;
      _data.w_actual = w;
      _lock.unlock();
    }

    void setSensors(float left, float right, float front) {
        _lock.lock();
        _data.us_left = left;
        _data.us_right = right;
        _data.ir_front = front;
        _lock.unlock();
    }

    // GETTER (Used by Communication)
    TelemetryData getSnapshot() {
        _lock.lock();
        TelemetryData copy = _data; // Copy safely
        _lock.unlock();
        return copy;
    }
};

#endif