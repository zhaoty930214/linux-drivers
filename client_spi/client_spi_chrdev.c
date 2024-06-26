#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>

#include "client_spi_types.h"


static struct file_operations client_spi_fops = {
    .open = NULL,
    .release = NULL,
    .read = NULL,
    .write = NULL,
};

int client_spi_chrdev_init(struct spi_device *spi)
{
    int ret;
    struct client_spi *cspi = spi_get_drvdata(spi);
    if(IS_ERR(cspi))
    {
        printk(KERN_ERR"error drvdata\r\n");
        return -ENXIO;
    }
    //p1: output of first assigned number
    //p2: first of the requested range of minor numbers
    //p3: the number of minor numbers required
    //p4: name of device
    ret = alloc_chrdev_region(&cspi->dev_id, 0, 1, CLIENT_SPI_NAME);
    if(ret)
    {
        printk(KERN_ERR"alloc dev_id failed\r\n");
    }
    //cdev_init
    cdev_init(&cspi->cdev, &client_spi_fops);

    //cdev_add
    ret = cdev_add(&cspi->cdev, cspi->dev_id, 1);
    if(ret)
    {
        
    }
    //class_create

    //device_create


    return 0;
}

int client_spi_chrdev_exit(struct spi_device *spi)
{
    struct client_spi *cspi = spi_get_drvdata(spi);

    cdev_del(&cspi->cdev);

    return 0;
}

