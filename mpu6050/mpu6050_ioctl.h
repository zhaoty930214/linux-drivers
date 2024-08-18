#ifndef __MPU6050_IOCTL_H
#define __MPU6050_IOCTL_H

#include <asm/ioctl.h>
// #include <linux/types.h>

#define MPU6050_IOCTL_MAGIC              'W'

struct i2c_read_reg{
    // uint8_t reg_addr;   /*read from which addr of mpu chip*/
    char *buff;      /*where to store the read back data*/
};


#define MPU6050_CAPTURE_DATA    _IOW(MPU6050_IOCTL_MAGIC, 0, \
                                    struct i2c_read_reg)

#endif