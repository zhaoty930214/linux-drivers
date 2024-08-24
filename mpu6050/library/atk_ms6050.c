#include "atk_ms6050.h"
#include "../driver/mpu6050_ioctl.h"
#include "../driver/mpu6050_constants.h"    /*Register Map*/
// #include "../mpu6050_lib.h"
#include <stdio.h>
#include <fcntl.h>              /*flags for open*/
#include <unistd.h>             /*Close() system call */
#include <sys/ioctl.h>
#include <time.h>


/**
 * @brief       ATK-MS6050软件复位
 * @param       无
 * @retval      无
 */
void atk_ms6050_sw_reset(void)
{
    struct timespec ts;

    atk_ms6050_write_byte(ATK_MS6050_IIC_ADDR, MPU_PWR_MGMT1_REG, 0x80);
    ts.tv_sec = 0;
    ts.tv_nsec = 100*1000*1000;

    nanosleep(&ts, NULL);
    
    atk_ms6050_write_byte(ATK_MS6050_IIC_ADDR, MPU_PWR_MGMT1_REG, 0x00);
}

/**
 * @brief       ATK-MS6050设置陀螺仪传感器量程范围
 * @param       frs: 0 --> ±250dps
 *                   1 --> ±500dps
 *                   2 --> ±1000dps
 *                   3 --> ±2000dps
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_set_gyro_fsr(uint8_t fsr)
{
    return atk_ms6050_write_byte(ATK_MS6050_IIC_ADDR, MPU_GYRO_CFG_REG, fsr << 3);
}

/**
 * @brief       ATK-MS6050设置加速度传感器量程范围
 * @param       frs: 0 --> ±2g
 *                   1 --> ±4g
 *                   2 --> ±8g
 *                   3 --> ±16g
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_set_accel_fsr(uint8_t fsr)
{
    return atk_ms6050_write_byte(ATK_MS6050_IIC_ADDR, MPU_ACCEL_CFG_REG, fsr << 3);
}

/**
 * @brief       ATK-MS6050设置数字低通滤波器频率
 * @param       lpf: 数字低通滤波器的频率（Hz）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_set_lpf(uint16_t lpf)
{
    uint8_t dat;
    
    if (lpf >= 188)
    {
        dat = 1;
    }
    else if (lpf >= 98)
    {
        dat = 2;
    }
    else if (lpf >= 42)
    {
        dat = 3;
    }
    else if (lpf >= 20)
    {
        dat = 4;
    }
    else if (lpf >= 10)
    {
        dat = 5;
    }
    else
    {
        dat = 6;
    }
    
    return atk_ms6050_write_byte(ATK_MS6050_IIC_ADDR, MPU_CFG_REG, dat);
}

/**
 * @brief       ATK-MS6050设置采样率
 * @param       rate: 采样率（4~1000Hz）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_set_rate(uint16_t rate)
{
    uint8_t ret;
    uint8_t dat;
    
    if (rate > 1000)
    {
        rate = 1000;
    }
    
    if (rate < 4)
    {
        rate = 4;
    }
    
    dat = 1000 / rate - 1;
    ret = atk_ms6050_write_byte(ATK_MS6050_IIC_ADDR, MPU_SAMPLE_RATE_REG, dat);
    if (ret != ATK_MS6050_EOK)
    {
        return ret;
    }
    
    ret = atk_ms6050_set_lpf(rate >> 1);
    if (ret != ATK_MS6050_EOK)
    {
        return ret;
    }
    
    return ATK_MS6050_EOK;
}

/**
 * @brief       ATK-MS6050获取温度值
 * @param       temperature: 获取到的温度值（扩大了100倍）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_get_temperature(int16_t *temp)
{
    uint8_t dat[2];
    uint8_t ret;
    int16_t raw = 0;
    
    ret = atk_ms6050_read(ATK_MS6050_IIC_ADDR, MPU_TEMP_OUTH_REG, 2, dat);
    if (ret == ATK_MS6050_EOK)
    {
        raw = ((uint16_t)dat[0] << 8) | dat[1];
        *temp = (int16_t)((36.53f + ((float)raw / 340)) * 100);
    }
    
    return ret;
}

/**
 * @brief       ATK-MS6050获取陀螺仪值
 * @param       gx，gy，gz: 陀螺仪x、y、z轴的原始度数（带符号）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_get_gyroscope(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t dat[6];
    uint8_t ret;
    
    ret =  atk_ms6050_read(ATK_MS6050_IIC_ADDR, MPU_GYRO_XOUTH_REG, 6, dat);
    if (ret == ATK_MS6050_EOK)
    {
        *gx = ((uint16_t)dat[0] << 8) | dat[1];
        *gy = ((uint16_t)dat[2] << 8) | dat[3];
        *gz = ((uint16_t)dat[4] << 8) | dat[5];
    }
    
    return ret;
}

/**
 * @brief       ATK-MS6050获取加速度值
 * @param       ax，ay，az: 加速度x、y、z轴的原始度数（带符号）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_get_accelerometer(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t dat[6];
    uint8_t ret;
    
    ret =  atk_ms6050_read(ATK_MS6050_IIC_ADDR, MPU_ACCEL_XOUTH_REG, 6, dat);
    if (ret == ATK_MS6050_EOK)
    {
        *ax = ((uint16_t)dat[0] << 8) | dat[1];
        *ay = ((uint16_t)dat[2] << 8) | dat[3];
        *az = ((uint16_t)dat[4] << 8) | dat[5];
    }
    
    return ret;
}

/**
 * @brief       往ATK-MS6050的指定寄存器写入一字节数据
 * @param       addr: ATK-MS6050的IIC通讯地址
 *              reg : ATK-MS6050寄存器地址
 *              dat : 写入的数据
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_write_byte(uint8_t addr, uint8_t reg, uint8_t dat)
{
    return atk_ms6050_write(addr, reg, 1, &dat);
}

/**
 * @brief       读取ATK-MS6050指定寄存器的值
 * @param       addr: ATK-MS6050的IIC通讯地址
 *              reg : ATK-MS6050寄存器地址
 *              dat: 读取到的寄存器的值
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_read_byte(uint8_t addr, uint8_t reg, uint8_t *dat)
{
    return atk_ms6050_read(addr, reg, 1, dat);
}


int atk_ms6050_write(uint8_t i2c_addr, uint8_t reg_addr, int len, uint8_t *buff)
{
    int ret;
    // iic_write_reg8(*atk_iic_io, reg_addr, buff, len);
    int fd = open("/dev/I2C_MPU6050", O_RDWR);
    if(fd < 0)
    {
        printf("Can not open device\n");
    }
    struct mpu6050_write_reg wrReg;
    wrReg.reg_addr = reg_addr;
    wrReg.length = len;
    wrReg.buff = (char *) buff;
    ret = ioctl(fd, MPU6050_WRITE_DATA, &wrReg);

    close(fd);
    return ret;
}

int atk_ms6050_read(uint8_t i2c_addr, uint8_t reg_addr, int len, uint8_t *buff)
{
    int ret;
    // iic_read_reg8(*atk_iic_io, buff, reg_addr, len);
    int fd = open("/dev/I2C_MPU6050", O_RDWR);
    if(fd < 0)
    {
        printf("Can not open device\n");
    }

    struct mpu6050_read_reg rdReg;
    rdReg.reg_addr = reg_addr;
    rdReg.length = len;
    rdReg.buff = (char *) buff;
    ret = ioctl(fd, MPU6050_READ_DATA, &rdReg);

    close(fd);

    return ret;
}