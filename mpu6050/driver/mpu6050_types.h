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

struct IIC_IO{
    int gpio_SCL;
    int gpio_SDA;
};

//typdef int (*fptr_iic_read)(char *, uint8_t, uint8_t) iic_read;

struct mpu6050{
    struct i2c_client *client;
    struct cdev cdev;
    dev_t dev_id;
    struct class *class;
    struct device *device;
    struct IIC_IO iic_io;
    struct timer_list timer;
    //fptr_iic_read IIC_read;
};




#endif