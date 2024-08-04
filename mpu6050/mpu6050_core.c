#include "mpu6050_lib.h"
#include "eMPL/inv_mpu.h"

void mpu6050_run(int *arg)
{
    uint8_t ret;
    uint8_t key;
    uint8_t niming_report = 0;
    float pit, rol, yaw;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    int16_t temp;
    
    struct mpu6050 *mpu = (struct mpu6050 *) (arg);

    // set_iic(&mpu->iic_io);
    /* 初始化ATK-MS6050 */
    mpu6050_init(mpu);
    
    /* 初始化ATK-MS6050 DMP */
    // ret = atk_ms6050_dmp_init();


    /* 获取ATK-MS6050 DMP处理后的数据 */
    // ret  = atk_ms6050_dmp_get_data(&pit, &rol, &yaw);
    // /* 获取ATK-MS6050加速度值 */
    // ret += atk_ms6050_get_accelerometer(&acc_x, &acc_y, &acc_z, &mpu->iic_io);
    // /* 获取ATK-MS6050陀螺仪值 */
    // ret += atk_ms6050_get_gyroscope(&gyr_x, &gyr_y, &gyr_z, &mpu->iic_io);
    // /* 获取ATK-MS6050温度值 */
    // ret += atk_ms6050_get_temperature(&temp, &mpu->iic_io);
    if (ret == 0)
    {
        if (niming_report == 0)
        {
            /* 上传相关数据信息至串口调试助手 */
            // printk("pit: %.2f, rol: %.2f, yaw: %.2f, ", pit, rol, yaw);
            printk("acc_x: %d, acc_y: %d, acc_z: %d, ", acc_x, acc_y, acc_z);
            printk("gyr_x: %d, gyr_y: %d, gyr_z: %d, ", gyr_x, gyr_y, gyr_z);
            printk("temp: %d\r\n", temp);
        }
        // else
        // {
        //     /* 上传状态帧和传感器帧至匿名地面站V4 */
        //     demo_niming_report_status((int16_t)(rol * 100), (int16_t)((pit) * 100), (int16_t)(yaw * 100), 0, 0, 0);
        //     demo_niming_report_senser(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, 0, 0, 0);
        // }
    }
    
}
