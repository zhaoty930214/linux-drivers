#ifndef __MPU6050_TYPES_H
#define __MPU6050_TYPES_H
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>

struct mpu6050{
    struct i2c_client *client;
    struct cdev cdev;
    dev_t dev_id;
    struct class *class;
    struct device *device;
    int gpio_SCL;
    int gpio_SDA;
    struct timer_list timer;
};

#define MPU6050_ADDR    (0x68<<1)


#define    MPU6050_SMPLRT_DIV        0x19
#define    MPU6050_CONFIG            0x1A
#define    MPU6050_GYRO_CONFIG        0x1B
#define    MPU6050_ACCEL_CONFIG    0x1C
 
#define    MPU6050_ACCEL_XOUT_H    0x3B
#define    MPU6050_ACCEL_XOUT_L    0x3C
#define    MPU6050_ACCEL_YOUT_H    0x3D
#define    MPU6050_ACCEL_YOUT_L    0x3E
#define    MPU6050_ACCEL_ZOUT_H    0x3F
#define    MPU6050_ACCEL_ZOUT_L    0x40
#define    MPU6050_TEMP_OUT_H        0x41
#define    MPU6050_TEMP_OUT_L        0x42
#define    MPU6050_GYRO_XOUT_H        0x43
#define    MPU6050_GYRO_XOUT_L        0x44
#define    MPU6050_GYRO_YOUT_H        0x45
#define    MPU6050_GYRO_YOUT_L        0x46
#define    MPU6050_GYRO_ZOUT_H        0x47
#define    MPU6050_GYRO_ZOUT_L        0x48
 
#define    MPU6050_PWR_MGMT_1        0x6B
#define    MPU6050_PWR_MGMT_2        0x6C
#define    MPU6050_WHO_AM_I        0x75
 
#endif