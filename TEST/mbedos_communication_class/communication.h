#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <mbed.h>
#include <rtos.h>
#include "robotstate.h"

// --- PROTOCOL DEFINITIONS ---

// 1. OUTGOING: Telemetry (Robot -> Pi)
// Size: 23 Bytes (Start + Timestamp + 9*2 + Checksum)
struct __attribute__((packed)) TelemetryPacket {
    uint8_t startByte = 0xA5;    // Sync Header
    uint32_t timestamp;          // 4 bytes (millis)
    int16_t x;                   // 2 bytes (mm)
    int16_t y;                   // 2 bytes (mm)
    int16_t theta;               // 2 bytes (rad * 10000)
    int16_t v;                   // 2 bytes (mm/s)
    int16_t w;                   // 2 bytes (rad/s * 1000)
    uint16_t us_l;               // 2 bytes (mm)
    uint16_t us_r;               // 2 bytes (mm)
    uint16_t ir;                 // 2 bytes (mm)
    // Add dummy padding if alignment issues occur, currently 23 bytes
    uint8_t checksum;            // 1 byte (XOR of timestamp through ir)
};

// 2. INCOMING: Commands (Pi -> Robot)
// Size: 6 Bytes
struct __attribute__((packed)) CommandPacket {
    uint8_t startByte = 0x5A;    // Different Sync Header for safety
    int16_t v_target;            // mm/s
    int16_t w_target;            // rad/s * 1000
    uint8_t checksum;            // XOR of v_target and w_target
};

class COMMUNICATION {
private:
    rtos::Thread thread;
    ROBOTSTATE* _state;
    Stream* _serialPort;
    bool _running;

    // Parsing State Machine
    enum ParseState {
        WAIT_SYNC,
        READ_DATA,
        CHECK_CS
    };
    ParseState _pState = WAIT_SYNC;
    uint8_t _rxBuffer[sizeof(CommandPacket)];
    uint8_t _rxIndex = 0;

    void loop() {
        while (true) {
            if (_serialPort != nullptr && _state != nullptr) {
                // 1. Handle Incoming Data (Non-blocking)
                while (_serialPort->available() > 0) {
                    processIncomingByte(_serialPort->read());
                }

                // 2. Send Telemetry at ~30Hz
                sendTelemetry();
            }
            
            // Yield to other RTOS threads
            rtos::ThisThread::sleep_for(33ms); 
        }
    }

    // FSM Parser to handle fragmentation
    void processIncomingByte(uint8_t byte) {
        switch (_pState) {
            case WAIT_SYNC:
                if (byte == 0x5A) { // Command Sync Byte
                    _rxIndex = 0;
                    _rxBuffer[_rxIndex++] = byte;
                    _pState = READ_DATA;
                }
                break;

            case READ_DATA:
                _rxBuffer[_rxIndex++] = byte;
                if (_rxIndex >= sizeof(CommandPacket)) {
                    _pState = CHECK_CS;
                    // Fallthrough immediately to check checksum
                    processIncomingByte(0); // Dummy call to trigger next state check logic if needed, 
                                            // or just handle it here:
                    if (validatePacket()) {
                        applyCommand();
                    }
                    _pState = WAIT_SYNC;
                }
                break;
                
            case CHECK_CS:
                 // Handled inside READ_DATA logic for efficiency in loop
                 _pState = WAIT_SYNC;
                 break;
        }
    }

    bool validatePacket() {
        CommandPacket* pkt = (CommandPacket*)_rxBuffer;
        uint8_t calc_cs = 0;
        uint8_t* ptr = (uint8_t*)pkt;
        
        // XOR payload (skip start byte (idx 0) and checksum (last byte))
        // Structure: [Start, V_L, V_H, W_L, W_H, CS]
        for (size_t i = 1; i < sizeof(CommandPacket) - 1; i++) {
            calc_cs ^= ptr[i];
        }
        
        return (calc_cs == pkt->checksum);
    }

    void applyCommand() {
        CommandPacket* pkt = (CommandPacket*)_rxBuffer;
        
        // Convert integer scaling back to float for the robot physics model
        float v = (float)pkt->v_target / 1000.0f; // mm/s -> m/s
        float w = (float)pkt->w_target / 1000.0f; // mrad/s -> rad/s
        
        // Thread-safe write to Blackboard
        _state->setTargetVelocity(v, w);
    }

    void sendTelemetry() {
        TelemetryData data = _state->getSnapshot();
        
        TelemetryPacket pkt;
        pkt.timestamp = millis();
        pkt.x = (int16_t)(data.x * 1000.0f);
        pkt.y = (int16_t)(data.y * 1000.0f);
        pkt.theta = (int16_t)(data.theta * 10000.0f);
        pkt.v = (int16_t)(data.v_actual * 1000.0f);
        pkt.w = (int16_t)(data.w_actual * 1000.0f);
        pkt.us_l = (uint16_t)data.us_left;
        pkt.us_r = (uint16_t)data.us_right;
        pkt.ir = (uint16_t)data.ir_front;

        pkt.checksum = 0;
        uint8_t* ptr = (uint8_t*)&pkt;
        for (size_t i = 1; i < sizeof(TelemetryPacket) - 1; i++) {
            pkt.checksum ^= ptr[i];
        }

        _serialPort->write((uint8_t*)&pkt, sizeof(TelemetryPacket));
    }

public:
    COMMUNICATION() : thread(osPriorityAboveNormal, 4096), _running(false) {}

    void begin(ROBOTSTATE* state_ptr, Stream* serial_ptr) {
        _state = state_ptr;
        _serialPort = serial_ptr;
        _running = true;
        thread.start(mbed::callback(this, &COMMUNICATION::loop));
    }
};

#endif