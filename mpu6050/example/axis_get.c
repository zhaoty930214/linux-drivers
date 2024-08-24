#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
// #include <linux/uaccess.h>
#include <stdint.h>             // uint8_t type
#include <sys/ioctl.h>          // IOCTL system call
#include <stdbool.h>            // bool type true and false
#include <time.h>               /* nanosleep*/

#include "atk_ms6050.h"
#include "inv_mpu.h"

#include "../driver/mpu6050_ioctl.h"


int main(int argc, char *argv[])
{
    int rc;
    uint8_t ret;
    uint8_t key;
    uint8_t niming_report = 0;
    float pit, rol, yaw;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    int16_t temp;

    printf("Got %d arguments\r\n", argc);
    char buff[6];

    // rc = atk_ms6050_read(0, 0x3B, 6, buff);

    // if(rc != 0)
    // {
    //     printf("Call Ioctl failed\n");
    //     return -1;
    // }
    // for(int i=0; i<6; i++)
    // {
    //     printf("Read out %d=%x", i, buff[i]);
    // }

        /* 初始化ATK-MS6050 DMP */
    ret = atk_ms6050_dmp_init();

    // char data = 0x00;
    // atk_ms6050_write(0, 0x38, 1, &data);

    // struct timespec ts;
    // ts.tv_sec = 0;
    // ts.tv_nsec = 50*1000*1000;
    while(true)
    {
        /* 获取ATK-MS6050 DMP处理后的数据 */
        ret  = atk_ms6050_dmp_get_data(&pit, &rol, &yaw);
        printf("ret= %x \r\n", ret);

        /* 获取ATK-MS6050加速度值 */
        ret += atk_ms6050_get_accelerometer(&acc_x, &acc_y, &acc_z) << 1;
        /* 获取ATK-MS6050陀螺仪值 */
        ret += atk_ms6050_get_gyroscope(&gyr_x, &gyr_y, &gyr_z) << 2;
        /* 获取ATK-MS6050温度值 */
        ret += atk_ms6050_get_temperature(&temp) << 3;

        printf("pit: %f, rol: %f, yaw: %f;", pit, rol, yaw);
        printf("acc_x: %d, acc_y: %d, acc_z: %d, ", acc_x, acc_y, acc_z);
        printf("gyr_x: %d, gyr_y: %d, gyr_z: %d, ", gyr_x, gyr_y, gyr_z);
        printf("temp: %d. ret= %x \r\n", temp, ret);

        //nanosleep(&ts, NULL);
    }

    return 0;
}

// arm-linux-gnueabihf-gcc main.c atk_ms6050.c ../eMPL/inv_mpu.c ../eMPL/inv_mpu_dmp_motion_driver.c -o main -lm -std=gnu11