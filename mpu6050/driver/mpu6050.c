#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>

#include "mpu6050_types.h"
#include "mpu6050_lib.h"

#include <linux/timer.h>


static int client_i2c_probe(struct platform_device *pdev)
{
    int ret;

    ret = mpu6050_chrdev_init(pdev);
    
    return 0;
}


static int client_i2c_remove(struct platform_device *client)
{
    mpu6050_chrdev_exit(client);

    return 0;
}


static const struct of_device_id client_i2c_dt_match[]=
{
    {.compatible = "zty,mpu-6050"},
    {/*Setinel*/}
};

static struct platform_driver client_i2c_driver = {
    .probe = client_i2c_probe,
    .remove = client_i2c_remove,
    // .shutdown = client_i2c_shutdown,
    .driver = {
        .name = "mpu6050",
        .of_match_table = client_i2c_dt_match,
    },
};




module_platform_driver(client_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("tianyu.zhao@mevion.com");