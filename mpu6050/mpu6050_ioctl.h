#include <asm/ioctl.h>
#include <linux/types.h>

#define MPU6050_IOCTL_MAGIC              'W'

struct i2c_read_reg{
    uint8_t reg_addr;   /*read from which addr of mpu chip*/
    uint8_t *buff;      /*where to store the read back data*/
    int length;         /*how many byte will be read*/
};


#define MPU6050_READ_REG    _IOW(MPU6050_IOCTL_MAGIC, 0, \
                                    struct i2c_read_reg)