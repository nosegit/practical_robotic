import serial
import time

# --- Configuration ---
SERIAL_PORT = "/dev/ttyUSB0"  # Change this to your Pi's port
BAUD_RATE = 9600
TERMINATOR = b'#'           # Use a 'bytes' object for the terminator
# --- End Configuration ---


# --- Opcode Handler Functions ---
# Each function receives the 'values' as a bytes object

def handle_get_sensor(values):

    print(f"[Handling Opcode 's' - get_sensor ]")
    if len(values) == 3:
        # 'big' or 'little' depends on your device's byte order
        # speed = int.from_bytes(values, 'big')
        infrared = values[0]
        ultrasonic0 = values[1]
        ultrasonic1 = values[2]
        print(f"  > Success: {infrared}, {ultrasonic0}, {ultrasonic1}")
    else:
        print(f"  > Error: Expected 3 value bytes, got {len(values)}")


# --- Main Application ---

# This "dispatch table" maps the opcode (as an int) to the function
# that should handle it.
OPCODE_DISPATCH = {
    ord('s'): handle_get_sensor,
    # Add all your other opcodes here
}

def main_loop():
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        # Use a 'with' block for clean open/close
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2.0) as ser:
            print(f"Connected. Waiting for packets ending in {TERMINATOR}...")
            
            while True:
                # 1. Read one full "packet" from the serial buffer
                # This blocks until it finds TERMINATOR or times out
                packet_buffer = ser.read_until(TERMINATOR)
                
                # 2. Validate the packet
                if not packet_buffer.endswith(TERMINATOR):
                    # This was a timeout. No terminator was found.
                    if len(packet_buffer) > 0:
                        print(f"Timeout! Discarding partial data: {packet_buffer}")
                    continue # Go back to waiting for a packet
                
                # 3. Clean and parse the packet
                # Get all bytes *before* the terminator
                packet = packet_buffer[:-1] 
                
                if not packet:
                    # We received an empty packet (just '#'), ignore it
                    continue
                    
                # 4. Extract opcode and values
                opcode = packet[0]   # The first byte is the opcode (as an int)
                values = packet[1:]  # The rest are the values (as bytes)
                
                # 5. Dispatch to the correct handler
                if opcode in OPCODE_DISPATCH:
                    # Find the correct function in our dictionary
                    handler_function = OPCODE_DISPATCH[opcode]
                    
                    # Call the function, passing it the values
                    handler_function(values)
                else:
                    print(f"Error: Unknown opcode {hex(opcode)} received.")

    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")

# Run the main loop
if __name__ == "__main__":
    main_loop()