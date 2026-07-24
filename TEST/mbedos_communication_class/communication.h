#include <mbed.h>
#include "robotstate.h"

// Forward declarations to avoid include loops
class Robot; 

class COMMUNICATION {
private:
    rtos::Thread thread;
    ROBOTSTATE* _state;
    Stream* _serialPort = nullptr;

    void loop() {
        while (true) {
            // --- 1. RECEIVE FROM PI (Non-Blocking) ---
            if (_serialPort->available() > 0) {
                String cmd = _serialPort->readStringUntil('>');
                // Remove the start '<' if present
                int start_idx = cmd.indexOf('<');
                if (start_idx != -1) {
                    parseCommand(cmd.substring(start_idx + 1));
                }
            }

            // --- 2. SEND TO PI (Telemetry) ---
            sendTelemetry();

            // --- 3. TIMING (30Hz) ---
            rtos::ThisThread::sleep_for(33ms); 
        }
    }

    void sendTelemetry() {
      // 1. Get a safe copy of ALL data instantly
      TelemetryData data = _state->getSnapshot();

      // Format: <Time, x, y, theta, v, w, us_l, us_r, ir>
      _serialPort->print("<");
      _serialPort->print(millis()); _serialPort->print(",");
      _serialPort->print(data.x, 3); _serialPort->print(",");
      _serialPort->print(data.y, 3); _serialPort->print(",");
      _serialPort->print(data.theta, 3); _serialPort->print(",");
      _serialPort->print(data.v_actual, 4); _serialPort->print(",");
      _serialPort->print(data.w_actual, 4); _serialPort->print(",");
      _serialPort->print(data.us_left, 1); _serialPort->print(",");
      _serialPort->print(data.us_right, 1); _serialPort->print(",");
      _serialPort->print(data.ir_front, 1); _serialPort->print(",");
      _serialPort->println(">");

    }

    void parseCommand(String data) {
        // Simple comma parser
        // Expecting: "v,w,cmd"
        int first_comma = data.indexOf(',');
        int second_comma = data.indexOf(',', first_comma + 1);

        if (first_comma > 0 && second_comma > 0) {
            float v = data.substring(0, first_comma).toFloat();
            float w = data.substring(first_comma + 1, second_comma).toFloat();
            int mode = data.substring(second_comma + 1).toInt();

            // set target state
            _state->setTargetVelocity(v, w);
        }
    }

public:
    // Priority: Above Normal (Keep the stream smooth)
    COMMUNICATION() : thread(osPriorityAboveNormal) {}

    void begin(ROBOTSTATE* state_ptr, Stream* serial_ptr) {
        _state = state_ptr;
        _serialPort = serial_ptr;

        thread.start(mbed::callback(this, &COMMUNICATION::loop));
    }
};