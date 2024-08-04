#include "atk_ms6050.h"
#include "../mpu6050_lib.h"

struct IIC_IO *atk_iic_io;

void atk_set_iic(struct IIC_IO *iic_io)
{
    atk_iic_io = iic_io;
}

int atk_ms6050_write(uint8_t i2c_addr, uint8_t reg_addr, int len, uint8_t *buff)
{
    iic_write_reg8(*atk_iic_io, reg_addr, buff, len);
    return 0;
}

int atk_ms6050_read(uint8_t i2c_addr, uint8_t reg_addr, int len, uint8_t *buff)
{
    iic_read_reg8(*atk_iic_io, buff, reg_addr, len);
    return 0;
}