# Robot Communication Nodes

This set of scripts implements a **Hybrid ZeroMQ Architecture** to control a robot over a network. It separates the hardware driver from the control logic using a Service/Client model.

## 📁 File Structure

| File | Role | Description |
| --- | --- | --- |
| robot_service.py | Server | Runs in the background. Connects to the Arduino via Serial. Publishes telemetry (Port 5555) and listens for commands (Port 5556). |
| lib/robot_driver.py | Driver | Handles the low-level serial communication, checksum validation, and binary packet parsing. |
| lib/robot_client.py | SDK | A library for user scripts. Abstracts ZMQ sockets into simple methods like set_velocity() and get_state(). |
| test/simple_client.py | User Script | A sample application that connects to the service and runs a predefined movement pattern. |

## 🚀 How to Run

1.  Start the Service (Background Process)
    
    This must run continuously to bridge the hardware to the network.
    
        python robot_service.py
        
    
2.  Run the Client (Control Script)
    
    Open a new terminal to run your logic.
    
        python simple_client.py
        
    

## 📡 Network Architecture

*   **Telemetry (Port 5555 - PUB/SUB):** The robot broadcasts state data at ~50Hz. Multiple clients (logger, dashboard, navigation) can listen simultaneously.
    
*   **Commands (Port 5556 - REQ/REP):** The client sends a command and waits for an `ACK`. This ensures reliable control synchronization.