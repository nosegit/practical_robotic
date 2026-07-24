#ifndef _INFRARED_H
#define _INFRARED_H

#include "measurement.h"

#define I2C_SDA P0_31
#define I2C_SCL P0_2
mbed::I2C myi2c(I2C_SDA,I2C_SCL);

MEASUREMENT infraed_measurement(3);
int infrared_distance = 0;
void Infrared_thread() {

  while (1) {
    const char mux_cmd = 0x01;
    const char mux_addr = 0xEE;
    myi2c.write(mux_addr, &mux_cmd, 1);
    
    char cmd[2];
    cmd[0] = 0x5E;
    cmd[1] = 0x00;
    myi2c.write(0x80, cmd, 1);
    myi2c.read(0x80, cmd, 2);
    infrared_distance = (cmd[0] * 16 + cmd[1])/16/(int)pow(2,0x02) ;

    infraed_measurement.record_value(infrared_distance);
  }
}

#endif