#ifndef __ATK_MS6050_H
#define __ATK_MS6050_H
#include <linux/types.h>
#include "../mpu6050_lib.h"

// #define atk_ms6050_read(i2c_addr, reg_addr, len, buff)   iic_read_reg8(iic, buff, reg_addr, len)

int atk_ms6050_write(uint8_t i2c_addr, uint8_t reg_addr, int len, uint8_t *buff);
int atk_ms6050_read(uint8_t i2c_addr, uint8_t reg_addr, int len, uint8_t *buff);

void atk_set_iic(struct IIC_IO *iic_io);

#endif