#ifndef __MPU6050_LIB_H
#define __MPU6050_LIB_H
#include "mpu6050_types.h"

#define CLIENT_I2C_NAME     "I2C_MPU6050"

// Truncates the full __FILE__ path, only displaying the basename
#define __FILENAME__ \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define mpu6050_err(fmt, ...) \
    printk(KERN_ERR CLIENT_I2C_NAME ": %s: %s: %d: " fmt, __FILENAME__, __func__, \
           __LINE__, ## __VA_ARGS__)

/*iic interface function*/
int iic_write_reg8(struct IIC_IO iic_io, uint8_t addr , uint8_t *data, int len);
int iic_write_byte(struct IIC_IO iic_io, uint8_t addr , uint8_t data);
uint8_t iic_read_reg8(struct IIC_IO iic_io, uint8_t *buff, uint8_t addr, int len);

/*mpu6050 config interface*/
void mpu6050_soft_reset(struct mpu6050 *mpu);
// uint8_t atk_ms6050_set_gyro_fsr(uint8_t fsr, struct IIC_IO *iic_io);
// uint8_t atk_ms6050_set_accel_fsr(uint8_t fsr, struct IIC_IO *iic_io);
uint8_t atk_ms6050_set_lpf(uint16_t lpf, struct IIC_IO *iic_io);
// uint8_t atk_ms6050_set_rate(uint16_t rate, struct IIC_IO *iic_io);
uint8_t atk_ms6050_get_accelerometer(int16_t *ax, int16_t *ay, int16_t *az, struct IIC_IO *iic_io);
// uint8_t atk_ms6050_get_temperature(int16_t *temp, struct IIC_IO *iic_io);
uint8_t atk_ms6050_get_gyroscope(int16_t *gx, int16_t *gy, int16_t *gz, struct IIC_IO *iic_io);



void mpu6050_init(struct mpu6050 *mpu);


/*chrdev function*/
int mpu6050_chrdev_init(struct platform_device *pdev);
int mpu6050_chrdev_exit(struct platform_device *pdev);


extern float asin_array[];
#endif
