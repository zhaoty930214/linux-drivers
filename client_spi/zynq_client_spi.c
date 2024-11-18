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

#include "client_spi_types.h"
#include "client_spi.h"

static int client_spi_probe(struct spi_device *spi)
{
    /*申请spi设备内存*/
    struct client_spi *lspi = kmalloc(sizeof(struct client_spi), GFP_KERNEL);

    /*设置platform_dev_data指针*/
    spi_set_drvdata(spi, lspi);

    client_spi_chrdev_init(spi);
    return 0;
}


static int client_spi_remove(struct spi_device *spi)
{
    struct client_spi *lspi = spi_get_drvdata(spi);

    client_spi_chrdev_exit(spi);

    kfree(lspi);

    return 0;
}

static void client_spi_shutdown(struct spi_device *spi)
{

}

static const struct of_device_id client_spi_dt_match[]=
{
    {.compatible = "zty,mpu-6050"},
    {/*Setinel*/}
};

static struct spi_driver client_spi_driver = {
    .probe = client_spi_probe,
    .remove = client_spi_remove,
    .shutdown = client_spi_shutdown,
    .driver = {
        .name = "mpu6050",
        .of_match_table = client_spi_dt_match,
    },
};


// static int __init client_spi_init(void)
// {
//     platform_driver_register(&client_spi_driver);
//     return 0;
// }

// static void __exit client_spi_exit(void)
// {
//     platform_driver_unregister(&client_spi_driver);
// }


module_spi_driver(client_spi_driver);

// module_init(client_spi_init);
// module_exit(client_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhao_ty@qq.com");
