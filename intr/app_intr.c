#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

static int fd;

static void sigio_signal_func(int signum)
{
    int flag, ret, i;
    printf("App got notification from irq\r\n");

    ret = read(fd, &flag, sizeof(int));
    if(ret < 0)
    {
        printf("read error\r\n");
    }
    else
    {
        printf("app, ret= %d, flag=%d\r\n", ret, flag);
        for(i=0; i<32; i++)
        {
            if( 0x1<<i & flag)
            {
                printf("Bit %d was captured\r\n", i);
            }
        }
    }
}

int main(int argc, char *argv[])
{
    int flags = 0;

    if( 2 != argc)
    {
        printf("Usage error\r\n");
        return -1;
    }

    fd = open(argv[1], O_RDWR | O_NONBLOCK);
    if( 0 >fd)
    {
        printf("Error:%s file open failed fd = %d\r\n", argv[1], fd);
        return -1;
    }
    
    signal(SIGIO, sigio_signal_func);

    fcntl(fd, F_SETOWN, getpid());
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | FASYNC);

    for(;;)
    {
        sleep(1);
    }
    
    close(fd);

    return 0;
}