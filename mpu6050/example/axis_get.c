// #include <stdio.h>
// #include <fcntl.h>
// #include <stdlib.h>
// // #include <linux/uaccess.h>
// #include <stdint.h>             // uint8_t type
// #include <sys/ioctl.h>          // IOCTL system call
// #include <stdbool.h>            // bool type true and false
// #include <time.h>               /* nanosleep*/
// #include <unistd.h>             /* usleep */

// #include "../driver/mpu6050_constants.h"
// #include "atk_ms6050.h"
// #include "inv_mpu.h"

// #include "../driver/mpu6050_ioctl.h"




// static int mpu6050_self_test()
// {
//     /* Set self test value
//        1. unused
//        2. reg_addr
//        3. len
//        4. buffer pointer*/
//     uint8_t data = 0;
//     float valid_FT = 0;
//     float FTX, FTY, FTZ;
//     int16_t acc_x, acc_y, acc_z;
//     data = 0;
//     atk_ms6050_write(0, MPU_SELF_TESTX_REG, 1, &data);
//     atk_ms6050_write(0, MPU_SELF_TESTY_REG, 1, &data);
//     atk_ms6050_write(0, MPU_SELF_TESTZ_REG, 1, &data);
//     data = 0x15;
//     atk_ms6050_write(0, MPU_SELF_TESTA_REG, 1, &data);

//     valid_FT = 4096*0.34*0.92/0.34;
//     FTX = valid_FT;
//     FTY = -valid_FT;
//     FTZ = valid_FT;

//     /* Enable self test*/
//     data = 0xE0;
//     atk_ms6050_write(0, MPU_ACCEL_CFG_REG, 1, &data);


//     /* get output data with selt-test on */
//     printf("With self-test enabled: \n");
//     atk_ms6050_get_accelerometer(&acc_x, &acc_y, &acc_z);


//     /* Disable self test*/
//     data = 0x00;
//     atk_ms6050_write(0, MPU_ACCEL_CFG_REG, 1, &data);
//     printf("With self-test disabled: \n");
//     atk_ms6050_get_accelerometer(&acc_x, &acc_y, &acc_z);

//     return 0;
// }


// int main(int argc, char *argv[])
// {
//     int rc;
//     uint8_t ret = 0;
//     uint8_t key;
//     uint8_t niming_report = 0;
//     float pit, rol, yaw;
//     int16_t acc_x, acc_y, acc_z;
//     int16_t gyr_x, gyr_y, gyr_z;
//     int16_t temp;

//     printf("Got %d arguments\r\n", argc);
//     char buff[6];

//     // rc = atk_ms6050_read(0, 0x3B, 6, buff);

//     // if(rc != 0)
//     // {
//     //     printf("Call Ioctl failed\n");
//     //     return -1;
//     // }
//     // for(int i=0; i<6; i++)
//     // {
//     //     printf("Read out %d=%x", i, buff[i]);
//     // }

//         /* 初始化ATK-MS6050 DMP */
//     // ret = atk_ms6050_dmp_init();

//     // char data = 0x00;
//     // atk_ms6050_write(0, 0x38, 1, &data);
//     uint8_t ID;

//     struct timespec ts;
//     ts.tv_sec = 0;
//     ts.tv_nsec = 1000*1000*1000;
//     while(true)
//     {
//         /* 获取ATK-MS6050 DMP处理后的数据 */
//         ret  = atk_ms6050_dmp_get_data(&pit, &rol, &yaw);
//         printf("ret= %x \r\n", ret);

//         /* 获取ATK-MS6050加速度值 */
//         ret += atk_ms6050_get_accelerometer(&acc_x, &acc_y, &acc_z) << 1;
//         /* 获取ATK-MS6050陀螺仪值 */
//         ret += atk_ms6050_get_gyroscope(&gyr_x, &gyr_y, &gyr_z) << 2;
//         /* 获取ATK-MS6050温度值 */
//         ret += atk_ms6050_get_temperature(&temp) << 3;
//         atk_ms6050_read(0, MPU_DEVICE_ID_REG, 1, &ID);

//         printf("ID=%x", ID);
//         printf("pit: %f, rol: %f, yaw: %f;", pit, rol, yaw);
//         printf("acc_x: %d, acc_y: %d, acc_z: %d, ", acc_x, acc_y, acc_z);
//         printf("gyr_x: %d, gyr_y: %d, gyr_z: %d, ", gyr_x, gyr_y, gyr_z);
//         printf("temp: %d. ret= %x \r\n", temp, ret);

//         // mpu6050_self_test();
//         usleep(500*1000);
//     }

//     return 0;
// }

// // arm-linux-gnueabihf-gcc main.c atk_ms6050.c ../eMPL/inv_mpu.c ../eMPL/inv_mpu_dmp_motion_driver.c -o main -lm -std=gnu11


#include <stdio.h>
#include <stdint.h>             /*type for uint8_t*/

#include "../driver/mpu6050_ioctl.h"
#include "../driver/mpu6050_constants.h"    /*Register Map*/
// #include "../mpu6050_lib.h"
#include <stdio.h>
#include <fcntl.h>              /*flags for open*/
#include <unistd.h>             /*Close() system call */
#include <sys/ioctl.h>
#include <time.h>
#include <math.h>
#include <errno.h>
#include <string.h>

//#define uint8_t  u_int8_t
#define Acc_Gain  	0.0001220f	  //加速度变成G (初始化加速度满量程-+4g LSBa = 2*4/65535.0)
#define G			9.80665f		      // m/s^2	
#define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)
#define N   20
#define Gyro_Gr	    0.0010641f			  //角速度变成弧度(3.1415/180 * LSBg)       

typedef struct float_xyz{
    float x;
    float y;
    float z;
} float_data;

typedef struct short_xyz{
    short x;
    short y;
    short z;
} short_data;

typedef struct float_Angle{
    float yaw;
    float pit;
    float rol;
} float_angle;

float_data Gyr_rad, Gyr_rad_old;
float_data Accel_filt_old;

void accel_convert(float_data *data)
{
    data->x = data->x * Acc_Gain * G;
    data->y = data->y * Acc_Gain * G;
    data->z = data->z * Acc_Gain * G;
}



/*******************************************************************************
*函  数 ：float FindPos(float*a,int low,int high)
*功  能 ：确定一个元素位序
*参  数 ：a  数组首地址
*         low数组最小下标
*         high数组最大下标
*返回值 ：返回元素的位序low
*备  注 : 无
*******************************************************************************/
float FindPos(float*a,int low,int high)
{
    float val = a[low];                      //选定一个要确定值val确定位置
    while(low<high)
    {
        while(low<high && a[high]>=val)
             high--;                       //如果右边的数大于VAL下标往前移
             a[low] = a[high];             //当右边的值小于VAL则复值给A[low]

        while(low<high && a[low]<=val)
             low++;                        //如果左边的数小于VAL下标往后移
             a[high] = a[low];             //当左边的值大于VAL则复值给右边a[high]
    }
    a[low] = val;//
    return low;
}

/*******************************************************************************
*函  数 ：void QuiteSort(float* a,int low,int high)
*功  能 ：快速排序
*参  数 ：a  数组首地址
*         low数组最小下标
*         high数组最大下标
*返回值 ：无
*备  注 : 无
*******************************************************************************/
 void QuiteSort(float* a,int low,int high)
 {
     int pos;
     if(low<high)
     {
         pos = FindPos(a,low,high); //排序一个位置
         QuiteSort(a,low,pos-1);    //递归调用
         QuiteSort(a,pos+1,high);
     }
 }


/*******************************************************************************
*函  数 ：void  SortAver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n)
*功  能 ：去最值平均值滤波三组数据
*参  数 ：*acc 要滤波数据地址
*         *Acc_filt 滤波后数据地址
*返回值 ：返回滤波后的数据
*备  注 : 无
*******************************************************************************/
void SortAver_FilterXYZ(short_data *acc, float_data *Acc_filt, uint8_t n)
{
	static float bufx[N],bufy[N],bufz[N];
	static uint8_t cnt =0,flag = 1;
	float temp1=0,temp2=0,temp3=0;
	uint8_t i;
	bufx[cnt] = acc->x;
	bufy[cnt] = acc->y;
	bufz[cnt] = acc->z;
	cnt++;      //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
	if(cnt<n && flag) 
		return;   //数组填不满不计算
	else
		flag = 0;
	
    QuiteSort(bufx,0,n-1);
	QuiteSort(bufy,0,n-1);
	QuiteSort(bufz,0,n-1);
	for(i=1;i<n-1;i++)
	 {
		temp1 += bufx[i];
		temp2 += bufy[i];
		temp3 += bufz[i];
	 }

	 if(cnt>=n) cnt = 0;
	 Acc_filt->x  = temp1/(n-2);
	 Acc_filt->y  = temp2/(n-2);
	 Acc_filt->z  = temp3/(n-2);
}


/**************************实现函数*********************************************************************
函  数：static float invSqrt(float x) 
功　能: 快速计算 1/Sqrt(x) 	
参  数：要计算的值
返回值：结果
备  注：比普通Sqrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*********************************************************************************************************/
static float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*********************************************************************************************************
*函  数：void IMUupdate(FLOAT_XYZ *Gyr_rad,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle)
*功　能：获取姿态角
*参  数：Gyr_rad 指向角速度的指针（注意单位必须是弧度）
*        Acc_filt 指向加速度的指针
*        Att_Angle 指向姿态角的指针
*返回值：无
*备  注：求解四元数和欧拉角都在此函数中完成
**********************************************************************************************************/	
//kp=ki=0 就是完全相信陀螺仪
#define Kp 1.50f                         // proportional gain governs rate of convergence to accelerometer/magnetometer
                                         //比例增益控制加速度计，磁力计的收敛速率
#define Ki 0.005f                        // integral gain governs rate of convergence of gyroscope biases  
                                         //积分增益控制陀螺偏差的收敛速度
#define halfT 0.005f                     // half the sample period 采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

void IMUupdate(float_data *Gyr_rad, float_data *Acc_filt, float_angle *Att_Angle)
{
	uint8_t i;
	float matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };//初始化矩阵
  float ax = Acc_filt->x,ay = Acc_filt->y,az = Acc_filt->z;
  float gx = Gyr_rad->x,gy = Gyr_rad->y,gz = Gyr_rad->z;
  float vx, vy, vz;
  float ex, ey, ez;
  float norm;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
  if(ax*ay*az==0)
 		return;
	
  //加速度计测量的重力向量(机体坐标系) 
  norm = invSqrt(ax*ax + ay*ay + az*az); 
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);
 
	//陀螺仪积分估计重力向量(机体坐标系) 
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;
 // printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz); 
	
	//测量的重力向量与估算的重力向量差积求出向量间的误差 
  ex = (ay*vz - az*vy); //+ (my*wz - mz*wy);                     
  ey = (az*vx - ax*vz); //+ (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx); //+ (mx*wy - my*wx);

  //用上面求出误差进行积分
  exInt = exInt + ex * Ki;								 
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  //将误差PI后补偿到陀螺仪
  gx = gx + Kp*ex + exInt;					   		  	
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

  //四元素的微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  //单位化四元数 
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;  
  q3 = q3 * norm;
	
	//矩阵R 将惯性坐标系(n)转换到机体坐标系(b) 
	matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11(前列后行)
	matrix[1] = 2.f * (q1q2 + q0q3);	    // 12
	matrix[2] = 2.f * (q1q3 - q0q2);	    // 13
	matrix[3] = 2.f * (q1q2 - q0q3);	    // 21
	matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
	matrix[5] = 2.f * (q2q3 + q0q1);	    // 23
	matrix[6] = 2.f * (q1q3 + q0q2);	    // 31
	matrix[7] = 2.f * (q2q3 - q0q1);	    // 32
	matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33
	 
  //四元数转换成欧拉角(Z->Y->X) 
  Att_Angle->yaw += Gyr_rad->z *RadtoDeg*0.01f;     
//	Att_Angle->yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f; // yaw
  Att_Angle->pit = -asin(2.f * (q1q3 - q0q2))* 57.3f;                                 // pitch(负号要注意) 
  Att_Angle->rol = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f ; // roll
//   for(i=0;i<9;i++)
//   {
//     *(&(DCMgb[0][0])+i) = matrix[i];
//   }

	//   for(i=0;i<9;i++)
//   {
//     *(&(DCMgb[0][0])+i) = matrix[i];
//   }
	//失控保护 (调试时可注释掉)
//	Safety_Check(); 
}


int main(int argc, char *argv[])
{
    int fd, rc;

    if(argc != 2)
    {
        printf("Error usage: ./%s <path_of_device>\n", argv[0]);
        return -1;
    }

    fd = open(argv[1], O_RDWR);

    if(fd < 0)
    {
        printf("Error. Unable to open the device %s\n", argv[1]);
        return -2;
    }

    char mpu6050_data[14];

    static struct mpu6050_read_reg rdReg;



    for(;;)
    {
        rdReg.length = 14;
        rdReg.reg_addr = MPU_ACCEL_XOUTH_REG;
        rdReg.buff = mpu6050_data;

        rc = ioctl(fd, MPU6050_READ_DATA, &rdReg);
        if(rc != 0)
        {
            printf("Error. Unable to call ioctl of MPU6050. %s\n", strerror(fd));
            continue;
        }

        short gyro_x, gyro_y, gyro_z;
        #define GYRO_GAIN_DEGREE_PER_COUNT  0.0609756f
        gyro_x = (mpu6050_data[8] << 8)  | mpu6050_data[9];
        gyro_y = (mpu6050_data[10]<< 10) | mpu6050_data[11];
        gyro_z = (mpu6050_data[12]<< 12) | mpu6050_data[13];


        printf("Gryo x:%d, y:%d, z:%d.\n", gyro_x, 
                                        gyro_y, 
                                        gyro_z);

        printf("Gryo x:%f, y:%f, z:%f.\n", GYRO_GAIN_DEGREE_PER_COUNT*gyro_x, 
                                        GYRO_GAIN_DEGREE_PER_COUNT*gyro_y, 
                                        GYRO_GAIN_DEGREE_PER_COUNT*gyro_z);

        float_data fdata;
        short_data sdata;

        sdata.x = (mpu6050_data[0] << 8)  | mpu6050_data[1];
        sdata.y = (mpu6050_data[2] << 8)  | mpu6050_data[3];
        sdata.z = (mpu6050_data[4] << 8)  | mpu6050_data[5];

        SortAver_FilterXYZ(&sdata, &fdata, 12);

        //加速度AD值 转换成 米/平方秒 
        fdata.x = (float)fdata.x * Acc_Gain * G;
        fdata.y = (float)fdata.y * Acc_Gain * G;
        fdata.z = (float)fdata.z * Acc_Gain * G;
    //printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);

        //陀螺仪AD值 转换成 弧度/秒    
        Gyr_rad.x = (float) gyro_x * Gyro_Gr;  
        Gyr_rad.y = (float) gyro_y * Gyro_Gr;
        Gyr_rad.z = (float) gyro_z * Gyro_Gr;

        float_data Gyr_rad = {Gyr_rad.x, Gyr_rad.y, Gyr_rad.z};
        float_angle angle;
        IMUupdate(&Gyr_rad, &fdata, &angle);

        printf("Yaw=%f, rol=%f, pit=%f. \n", angle.yaw, angle.rol, angle.pit);

        usleep(10*1000);
    }

    close(rc);
    return 0;
    
}