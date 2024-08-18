#ifndef __MPU6050_IOCTL_H
#define __MPU6050_IOCTL_H

#include <asm/ioctl.h>
// #include <linux/types.h>

#define MPU6050_IOCTL_MAGIC              'W'

struct mpu6050_read_reg{
    uint8_t reg_addr;   /*read from which addr of mpu chip*/
    uint8_t length;     /*how many bytes of data want to read*/
    char *buff;      /*where to store the read back data*/
};

struct mpu6050_write_reg{
    uint8_t reg_addr;
    uint8_t length;
    char *buff;
};

#define MPU6050_READ_DATA    _IOW(MPU6050_IOCTL_MAGIC, 0, \
                                    struct mpu6050_read_reg)


#define MPU6050_WRITE_DATA      _IOW(MPU6050_IOCTL_MAGIC, 1, \
                                    struct mpu6050_write_reg)


#endif