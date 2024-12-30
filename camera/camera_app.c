#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include "linux/ioctl.h"
#include "stdlib.h"
#include <dirent.h>

#define CLOSE_CMD   _IO(0XEF,  1)
#define OPEN_CMD    _IO(0xEF,  2)
#define PERIOD_CMD	_IO(0xEF,  3)

#define Y_RES	480
#define X_RES	752
#define DSIZE   (Y_RES*X_RES)

#define uint8_t u_int8_t

static  uint8_t bmp_head[54] = {
0x42,0x4d,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x36,0x0,0x0,0x0,0x28,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x1,0x0,0x18,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0xc4,0xe,0x0,0x0,0xc4,0x0e,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0 };


int get_file_count(void)
{
    int cnt;
    DIR *dir;
    struct dirent *ptr;
    dir = opendir("/home/root");
    while( (ptr = readdir(dir))!= NULL){
        cnt++;
        printf("file name %s\r\n", ptr->d_name);
    }
    closedir(dir);
    return cnt;
}

int main(int argc, char *argv[])
{
    int fd_dma, ret;
    char *filename, led_value = 0;
    unsigned int cmd;
    unsigned int arg;
    unsigned char str[100];

    /* 验证输入参数个数 */
    if(argc != 2)
    {
        printf("Error Usage\r\n");
        return -1;
    }

    /* 打开输入的设备文件, 获取文件句柄 */
    filename = argv[1];
    fd_dma = open(filename, O_RDWR, 0666);
    if(fd_dma < 0)
    {
        /* 打开文件失败 */
        printf("file %s open failed\r\n", argv[1]);
        return -1;
    }
    else
    {
        printf("Open dma:%s success!\r\n", filename);
    }
    

    char *usr_buffer;
    ssize_t rd_size=0;
    usr_buffer = malloc(DSIZE*3);

    if(usr_buffer == NULL)
        printf("Malloc memory for user buffer failed!\r\n");
    else
    {
        rd_size = read(fd_dma, usr_buffer, DSIZE);
        
        if(rd_size == -1)
        {
            printf("Read data from vdma error");
            exit(-1);
        }
        else
        {
            printf("Read %d bytes from vdma driver\r\n", rd_size);
        }
    }
    
    int fd2 = 0, cnt = 0;

    char *s_name = malloc(30);
    int i = get_file_count();
    if(s_name != NULL)
    {
        snprintf(s_name, 30, "/home/root/test%d.bmp", i);
    }
    else
    {
        return -1;
    }
    
    fd2 = open(s_name, O_CREAT | O_TRUNC | O_RDWR, 0666);
    free(s_name);
    if(fd2 <0)
    {
        printf("Create bmp file error!\r\n");
    }


    unsigned int *bf_size    = (unsigned int *)(bmp_head + 0x2);
    unsigned int *bmp_width  = (unsigned int *)(bmp_head + 0x12);
    unsigned int *bmp_height = (unsigned int *)(bmp_head + 0x16);
    unsigned int *bmp_size   = (unsigned int *)(bmp_head + 0x22);

    
    *bmp_size   = DSIZE*3;
    *bf_size    = *bmp_size + 54;
    *bmp_width  = X_RES;
    *bmp_height = Y_RES;

    cnt = write(fd2, bmp_head, sizeof(bmp_head));
    printf("Wrote %d bytes bmp_head to file!\r\n", cnt);

    cnt = write(fd2, usr_buffer, DSIZE*3);
    printf("Wrote %d bytes bmp_data to file!\r\n", cnt);

    close(fd2);

    free(usr_buffer);
    close(fd_dma);


    return 0;
}
























