# Robot Communication Protocol

Version: 1.0

Type: Binary, Fixed-Point, Little Endian

Baud Rate: 115200

Framing: Sync Byte + Payload + Checksum

## 1\. Pi $\\to$ Arduino (Commands)

The Raspberry Pi sends velocity commands to the robot.

*   Total Packet Size: 6 Bytes
    
*   Sync Byte: 0x5A
    
*   Frequency: On Demand (typically 10-20Hz)
    

| Offset | Field | Type | Unit | Scale Factor | Description |
| --- | --- | --- | --- | --- | --- |
| 0 | START | uint8_t | - | - | Fixed 0x5A |
| 1 | v_target | int16_t | mm/s | 1000 | Linear Velocity |
| 3 | w_target | int16_t | rad/s | 1000 | Angular Velocity |
| 5 | CHECKSUM | uint8_t | - | XOR | XOR of bytes 1-4 |

### Example Encoding

To send $v = 0.5$ m/s and $w = -1.2$ rad/s:

1.  Convert:
    

*   $v = 0.5 \\times 1000 = 500$ (0x01F4)
    
*   $w = -1.2 \\times 1000 = -1200$ (0xFB50)
    

2.  Payload: F4 01 50 FB (Little Endian)
    
3.  Checksum: 0xF4 ^ 0x01 ^ 0x50 ^ 0xFB = 0x5E
    
4.  Final Packet: 5A F4 01 50 FB 5E
    

## 2\. Arduino $\\to$ Pi (Telemetry)

The Robot sends state data back to the Pi.

*   Total Packet Size: 22 Bytes
    
*   Sync Byte: 0xA5
    
*   Frequency: Continuous (~30Hz)
    

| Offset | Field | Type | Unit | Scale Factor | Description |
| --- | --- | --- | --- | --- | --- |
| 0 | START | uint8_t | - | - | Fixed 0xA5 |
| 1 | timestamp | uint32_t | ms | 1 | Time since boot |
| 5 | x | int16_t | mm | 1000 | Global X Position |
| 7 | y | int16_t | mm | 1000 | Global Y Position |
| 9 | theta | int16_t | rad | 10000 | Global Heading |
| 11 | v | int16_t | mm/s | 1000 | Current Linear Vel |
| 13 | w | int16_t | rad/s | 1000 | Current Angular Vel |
| 15 | us_l | uint16_t | cm | 1 | Ultrasonic Left |
| 17 | us_r | uint16_t | cm | 1 | Ultrasonic Right |
| 19 | ir | uint16_t | cm | 1 | Infrared Center |
| 21 | CHECKSUM | uint8_t | - | XOR | XOR of bytes 1-20 |

### Python Unpack Format

``` python
\# 'I' = uint32, 'h' = int16, 'H' = uint16  
\# < = Little Endian  
format\_string = '<I5h3H'  
packet\_size = 22  
```

## 3\. Implementation Notes

### Byte Alignment

*   C++: Use \_\_attribute\_\_((packed)) on the struct definition to prevent the compiler from adding padding bytes (which would shift the offsets).
    
*   Python: Use struct.pack('<...') to enforce Little Endian byte order, regardless of the host machine architecture.
    

### Checksum Calculation

The checksum is an 8-bit XOR of the payload only. It does not include the Start Byte or the Checksum byte itself.

``` c++
// C++ Example  
uint8\_t calc = 0;  
for (int i = 1; i < sizeof(packet)-1; i++) {  
    calc ^= packet\_buffer\[i\];  
}
```