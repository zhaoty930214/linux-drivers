#ifndef __MPU6050_LIB_H
#define __MPU6050_LIB_H


void iic_write_reg8(int gpio_SCL, int gpio_SDA, uint8_t addr , uint16_t data);
uint8_t iic_read_reg8(int gpio_SCL, int gpio_SDA, uint8_t addr);

#endif