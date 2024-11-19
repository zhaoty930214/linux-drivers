#include "mpu6050_types.h"
#include "mpu6050_lib.h"
#include "mpu6050_constants.h"
// #include "atk_ms6050/atk_ms6050.h"

#define iic_write_byte_t(a, b, c)     mpu6050_write_byte(a, b, c, 1)

bool mpu6050_self_check(struct mpu6050 *mpu)
{
    bool ret;
    uint8_t  devid;
    mpu6050_read_reg(mpu->client, MPU_DEVICE_ID_REG, &devid, 1);

    printk("MPU6050 Self check result: 0x%02x\n", devid);

    return ret = (devid == 0x68);
}

void mpu6050_soft_reset(struct mpu6050 *mpu)
{
    iic_write_byte_t(mpu->client, MPU_PWR_MGMT1_REG, 0x80);
    msleep(100);
    iic_write_byte_t(mpu->client, MPU_PWR_MGMT1_REG, 0x01);
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
uint8_t atk_ms6050_set_gyro_fsr(struct mpu6050 *mpu, uint8_t fsr, struct IIC_IO *iic_io)
{
    return iic_write_byte_t(mpu->client, MPU_GYRO_CFG_REG, fsr << 3);
}


uint8_t atk_ms6050_set_accel_fsr(struct mpu6050 *mpu, uint8_t fsr, struct IIC_IO *iic_io)
{
    return iic_write_byte_t(mpu->client, MPU_ACCEL_CFG_REG, fsr << 3);
}


/**
 * @brief       ATK-MS6050设置数字低通滤波器频率
 * @param       lpf: 数字低通滤波器的频率（Hz）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_set_lpf(struct mpu6050 *mpu, uint16_t lpf, struct IIC_IO *iic_io)
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
    
    return iic_write_byte_t(mpu->client, MPU_CFG_REG, dat);
}


/**
 * @brief       ATK-MS6050设置采样率
 * @param       rate: 采样率（4~1000Hz）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t atk_ms6050_set_rate(struct mpu6050 *mpu, uint16_t rate, struct IIC_IO *iic_io)
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
    
    dat = 19;
    ret = iic_write_byte_t(mpu->client, MPU_SAMPLE_RATE_REG, dat);
    if (ret != ATK_MS6050_EOK)
    {
        return ret;
    }
    
    ret = atk_ms6050_set_lpf(mpu, rate >> 1, iic_io);
    if (ret != ATK_MS6050_EOK)
    {
        return ret;
    }
    
    return ATK_MS6050_EOK;
}

/**
 * @brief       ATK-MS6050获取加速度值
 * @param       ax，ay，az: 加速度x、y、z轴的原始度数（带符号）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
// uint8_t atk_ms6050_get_accelerometer(int16_t *ax, int16_t *ay, int16_t *az, struct IIC_IO *iic_io)
// {
//     uint8_t dat[6];
//     uint8_t ret;
    
//     ret =  atk_ms6050_read(0, MPU_ACCEL_XOUTH_REG, 6, dat);
//     if (ret == ATK_MS6050_EOK)
//     {
//         *ax = ((uint16_t)dat[0] << 8) | dat[1];
//         *ay = ((uint16_t)dat[2] << 8) | dat[3];
//         *az = ((uint16_t)dat[4] << 8) | dat[5];
//     }
    
//     return ret;
// }


/**
 * @brief       ATK-MS6050获取温度值
 * @param       temperature: 获取到的温度值（扩大了100倍）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
// uint8_t atk_ms6050_get_temperature(int16_t *temp, struct IIC_IO *iic_io)
// {
//     uint8_t dat[2];
//     uint8_t ret;
//     int16_t raw = 0;
    
//     ret = atk_ms6050_read(0, MPU_TEMP_OUTH_REG, 2, dat);
//     if (ret == ATK_MS6050_EOK)
//     {
//         raw = ((uint16_t)dat[0] << 8) | dat[1];
//         *temp = (int16_t)((36.53f + ((float)raw / 340)) * 100);
//     }
    
//     return ret;
// }

/**
 * @brief       ATK-MS6050获取陀螺仪值
 * @param       gx，gy，gz: 陀螺仪x、y、z轴的原始度数（带符号）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
// uint8_t atk_ms6050_get_gyroscope(int16_t *gx, int16_t *gy, int16_t *gz, struct IIC_IO *iic_io)
// {
//     uint8_t dat[6];
//     uint8_t ret;
    
//     ret =  atk_ms6050_read(0, MPU_GYRO_XOUTH_REG, 6, dat);
//     if (ret == ATK_MS6050_EOK)
//     {
//         *gx = ((uint16_t)dat[0] << 8) | dat[1];
//         *gy = ((uint16_t)dat[2] << 8) | dat[3];
//         *gz = ((uint16_t)dat[4] << 8) | dat[5];
//     }
    
//     return ret;
// }




void mpu6050_init(struct mpu6050 *mpu)
{
    mpu6050_self_check(mpu);                                   /* self check*/

    mpu6050_soft_reset(mpu);                                   /* soft reset */

    iic_write_byte_t(mpu->client, MPU_INT_EN_REG,    0X00);    /* disable interrupts */

    iic_write_byte_t(mpu->client, MPU_ACCEL_CFG_REG, 0X08);    /* ±4g */

    iic_write_byte_t(mpu->client, MPU_SAMPLE_RATE_REG, 0x00);  /* sample rate*/

    iic_write_byte_t(mpu->client, MPU_INTBP_CFG_REG, 0x02);    /* direct accessing sub I2C*/

    iic_write_byte_t(mpu->client, MPU_CFG_REG, 0x04);          /* output 1kHz, DLPF=20HZ*/

    iic_write_byte_t(mpu->client, MPU_GYRO_CFG_REG, 0x18);     /* full scale +-2000 d/s*/

    iic_write_byte_t(mpu->client, MPU_ACCEL_CFG_REG, 0x08);    /* accel full scale +-4 g*/

    // atk_ms6050_set_gyro_fsr(mpu, 3, &mpu->iic_io);             /* 陀螺仪传感器，±2000dps */   
    // atk_ms6050_set_accel_fsr(mpu, 0, &mpu->iic_io);            /* 加速度传感器，±2g */
    // atk_ms6050_set_rate(mpu, 50, &mpu->iic_io);                /* 采样率，50Hz */
    
    // iic_write_byte_t(mpu->client, MPU_USER_CTRL_REG, 0X00);    /* 关闭IIC主模式 */

    // iic_write_byte_t(mpu->client, MPU_FIFO_EN_REG,   0X00);    /* 关闭FIFO */
    // iic_write_byte_t(mpu->client, MPU_INTBP_CFG_REG, 0X80);    /* INT引脚低电平有效 */
    // // iic_write_byte_t(mpu->client, MPU_PWR_MGMT1_REG, 0x01);    /* 设置CLKSEL，PLL X轴为参考 */
    // iic_write_byte_t(mpu->client, MPU_PWR_MGMT2_REG, 0x00);    /* 加速度与陀螺仪都工作 */
    // atk_ms6050_set_rate(mpu, 40, &mpu->iic_io);                   /* 采样率，50Hz */

}