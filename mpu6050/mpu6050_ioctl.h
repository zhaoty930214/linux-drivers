#ifndef __MPU6050_IOCTL_H
#define __MPU6050_IOCTL_H

#include <asm/ioctl.h>
#include <linux/types.h>

#define MPU6050_IOCTL_MAGIC              'W'

struct i2c_read_reg{
    uint8_t reg_addr;   /*read from which addr of mpu chip*/
    uint8_t *buff;      /*where to store the read back data*/
    int length;         /*how many byte will be read*/
};

struct i2c_write_reg{
    uint8_t reg_addr;
    uint8_t *buff;
    int length;
};

#define MPU6050_READ_REG    _IOW(MPU6050_IOCTL_MAGIC, 0, \
                                    struct i2c_read_reg)


#define MPU6050_WRITE_REG   _IOW(MPU6050_IOCTL_MAGIC, 1, \
                                    struct i2c_write_reg)

#endif