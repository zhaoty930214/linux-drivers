#ifndef __ATK_MS6050_H
#define __ATK_MS6050_H
// #include <linux/types.h>
// #include "../driver/mpu6050_lib.h"

// #define atk_ms6050_read(i2c_addr, reg_addr, len, buff)   iic_read_reg8(iic, buff, reg_addr, len)

#include <stdint.h>

int atk_ms6050_write(uint8_t i2c_addr, uint8_t reg_addr, int len, uint8_t *buff);
int atk_ms6050_read(uint8_t i2c_addr, uint8_t reg_addr, int len, uint8_t *buff);

// void atk_set_iic(struct IIC_IO *iic_io);
void atk_ms6050_sw_reset(void);
uint8_t atk_ms6050_set_gyro_fsr(uint8_t fsr);
uint8_t atk_ms6050_set_accel_fsr(uint8_t fsr);
uint8_t atk_ms6050_set_lpf(uint16_t lpf);
uint8_t atk_ms6050_set_rate(uint16_t rate);
uint8_t atk_ms6050_get_temperature(int16_t *temp);
uint8_t atk_ms6050_get_gyroscope(int16_t *gx, int16_t *gy, int16_t *gz);
uint8_t atk_ms6050_get_accelerometer(int16_t *ax, int16_t *ay, int16_t *az);
uint8_t atk_ms6050_write_byte(uint8_t addr, uint8_t reg, uint8_t dat);
uint8_t atk_ms6050_read_byte(uint8_t addr, uint8_t reg, uint8_t *dat);


#endif