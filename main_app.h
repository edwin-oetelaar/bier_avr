#ifndef RC_INTERFACE_FIRMWARE_H
#define RC_INTERFACE_FIRMWARE_H

struct Controller_values
{
    volatile uint8_t throttle;
    volatile uint8_t steer;
    volatile uint8_t IO_stuff;
};

#define STEER    0
#define THROTTLE 1

#define RC_I2C_STEER_ADDRESS    0x58
#define RC_I2C_THROTTLE_ADDRESS 0x28
#define RC_I2C_IO_ADDRESS       0x32

#define IO_AUX_SWITCH_MASK      0x01
#define IO_EXTRA_SWITCH_MASK    0x02

#endif