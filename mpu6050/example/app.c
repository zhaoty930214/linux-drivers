// #include <stdio.h>  
// #include <math.h>  
  
// #define TABLE_SIZE 500 // 查找表的大小  
// //#define RANGE_RAD (M_PI*2) // 我们关注的弧度范围，例如0到π/2  
// #define QUANTIZATION (2.0f/TABLE_SIZE) // 量化步长  
  
// double atan_table[TABLE_SIZE];  
  
// void generate_atan_table() {  
//     for (int i= 0; i <= TABLE_SIZE; i++) {      
// 	double rad = i * QUANTIZATION -1.0f; // 计算当前索引对应的弧度  
//         //double deg = rad * (180.0 / M_PI); // 将弧度转换为度  
//         atan_table[i] = asin(rad); // 存储转换后的hudu值

// 	atan_table[i] *= (180.0f/M_PI);  
// 	//printf("%f, %f, ", rad, atan_table[i]);
//     }  
// }  
  
// int main() {  
//     generate_atan_table();  
//     printf("rad step=%f\r\n", QUANTIZATION);    

//     printf("// 反正切函数查找表（弧度索引，以度为单位），从atan(0)到atan(π/2)的近似值\n");  
//     printf("// 注意：这只是一个示例，实际使用中可能需要根据需要调整量化步长和查找表大小\n");  
	
//     for (int i = 0; i <= TABLE_SIZE; i++) {  
//         if(i%20==0)
// 	{
// 	    printf("\r\n");
// 	}

//         if (i < TABLE_SIZE - 1) {  
//             printf("%.2f, ", atan_table[i]);  
//         } else {  
//             printf("%.2f\n", atan_table[i]);  
//         }  
//     }  

//     fflush(stdout);
//     return 0;  
// }

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
// #include <linux/uaccess.h>
#include <stdint.h>             // uint8_t type
#include <sys/ioctl.h>          // IOCTL system call
#include <stdbool.h>            // bool type true and false
#include <time.h>               /* nanosleep*/

#include "atk_ms6050.h"
#include "../driver/mpu6050_ioctl.h"
#include "../eMPL/inv_mpu.h"


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

    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 50*1000*1000;
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