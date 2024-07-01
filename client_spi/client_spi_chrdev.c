#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>

#include "client_spi_types.h"


static int client_spi_open(struct inode *inode, struct file *filp)
{
    struct client_spi *myspi = container_of(inode->i_cdev, struct client_spi, cdev);
    filp->private_data = myspi;
    return 0;
}

static int client_spi_close(struct inode *inode, struct file *filp)
{
    filp->private_data = NULL;
    return 0;
}

static ssize_t client_spi_read(struct file *filp, char __user *buf,
                         size_t cnt, loff_t *offt)
{
    ssize_t ret;
    // struct client_spi *myspi = filp->private_data;
    // ssize_t len;
    // struct spi_message msg;

    // struct spi_transfer t = {
    //     .tx_buf = buf,
    //     .bits_per_word = 8,
    //     .len = 2,
    // };

    // spi_message_init(&msg);
    // spi_message_add_tail(&t, &msg);
    // ret = spi_sync(myspi->dspi, &msg);
    // if(ret)
    //     return ret;

     return ret;

}


static ssize_t client_spi_write(struct file *filp, const char __user *buf,
                         size_t cnt, loff_t *offt)
{
    struct client_spi *myspi = filp->private_data;
    ssize_t len, ret;
    struct spi_message msg;

    struct spi_transfer t = {
        .tx_buf = buf,
        .bits_per_word = 8,
        .len = 2,
    };

    spi_message_init(&msg);
    spi_message_add_tail(&t, &msg);
    ret = spi_sync(myspi->dspi, &msg);
    if(ret)
        return ret;

    return ret;
}

EXPORT_SYMBOL(client_spi_write);



static struct file_operations client_spi_fops = {
    .open = client_spi_open,
    .release = client_spi_close,
    .read = client_spi_read,
    .write = client_spi_write,
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
        return ret;
    }
    //cdev_init
    cdev_init(&cspi->cdev, &client_spi_fops);

    //cdev_add
    ret = cdev_add(&cspi->cdev, cspi->dev_id, 1);
    if(ret)
    {
        goto fail1;
    }

    //class_create
    cspi->class = class_create(THIS_MODULE, CLIENT_SPI_NAME);
    if(IS_ERR(cspi->class))
    {
        ret = PTR_ERR(cspi->class);
        goto fail2;
    }

    //device_create
    cspi->device = device_create(cspi->class, &spi->dev, 
                                 cspi->dev_id, NULL, CLIENT_SPI_NAME);

    if(IS_ERR(cspi->device))
    {
        ret = PTR_ERR(&cspi->device);
        goto fail3;
    }                             

    return 0;
fail3:
    class_destroy(cspi->class);
fail2:
    cdev_del(&cspi->cdev);
fail1:
    unregister_chrdev_region(cspi->dev_id, 1);
    return ret;
}

int client_spi_chrdev_exit(struct spi_device *spi)
{
    struct client_spi *cspi = spi_get_drvdata(spi);

    device_destroy(cspi->class, cspi->dev_id);

    class_destroy(cspi->class);

    cdev_del(&cspi->cdev);

    unregister_chrdev_region(cspi->dev_id, 1);

    return 0;
}

