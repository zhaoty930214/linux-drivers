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

struct mpu6050 *mpu;

#define CLIENT_I2C_NAME     "I2C_MPU6050"


static const struct file_operations mpu6050_ops= {
    .owner = THIS_MODULE,
    .open = NULL,
    .read = NULL,
    .write = NULL,
    .release = NULL,
};


static int of_parse_tree(struct device_node *nd)
{
    int ret;
    struct device_node *np = nd;
    if(NULL == np)
    {
        printk(KERN_ERR "vdma node can not found!\r\n");
        return -EINVAL;
    }

    mpu->gpio_SCL = of_get_named_gpio(nd, "gpio-SCL", 0);
    if(!gpio_is_valid(mpu->gpio_SCL))
    {
        printk(KERN_WARNING"Can't get gpio-SCL\r\n");
        return -1;
    }
    else
    {
        printk(KERN_INFO "gpio-SCL=%d\r\n", mpu->gpio_SCL);
        ret = gpio_request(mpu->gpio_SCL, "SCCB-SCL-GPIO");
        printk(KERN_INFO"SCCB GPIO SCL:%d\r\n", mpu->gpio_SCL);
        if(ret)
        {
            printk(KERN_ERR "mpu6050: Failed to request sccbSCL-gpio\n");
            return ret;
        }
        gpio_direction_output(mpu->gpio_SCL, 1);
    }

    mpu->gpio_SDA = of_get_named_gpio(nd, "gpio-SDA", 0);
    if(!gpio_is_valid(mpu->gpio_SDA))
    {
        printk(KERN_WARNING"Can't get gpio-SDA\r\n");
        return -1;
    }
    else
    {
        printk(KERN_INFO "gpio-SDA=%d\r\n", mpu->gpio_SDA);
        ret = gpio_request(mpu->gpio_SDA, "SCCB-SDA-GPIO");
        printk(KERN_INFO"SCCB GPIO SDA:%d\r\n", mpu->gpio_SDA);
        if(ret)
        {
            printk(KERN_ERR "mpu6050: Failed to request sccbSCL-gpio\n");
            return ret;
        }
        gpio_direction_output(mpu->gpio_SDA, 1);

    }

    gpio_set_value(mpu->gpio_SCL, 1);
    gpio_set_value(mpu->gpio_SDA, 1);

    return 0;
}

static void mpu_timer_function(unsigned long arg)
{
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3B);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3C);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3D);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3E);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3F);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x40);

    mod_timer(&mpu->timer, jiffies + msecs_to_jiffies(1000));

}

static int client_i2c_probe(struct platform_device *pdev)
{
    int ret;

    mpu = kmalloc(sizeof(struct mpu6050), GFP_KERNEL);

    ret = of_parse_tree(pdev->dev.of_node);

    if(ret)
    {
        dev_err(&pdev->dev, "%s parse tree failed", __func__);
    }
    
    /*申请spi设备内存*/
    // struct client_spi *lspi = kmalloc(sizeof(struct client_spi), GFP_KERNEL);

    /*request GPIO*/
    ret = alloc_chrdev_region(&mpu->dev_id, 0, 1, CLIENT_I2C_NAME);
    if(ret)
    {
        printk(KERN_ERR"alloc dev_id failed\r\n");
        return ret;
    }
    //cdev_init
    cdev_init(&mpu->cdev, &mpu6050_ops);

    //cdev_add
    ret = cdev_add(&mpu->cdev, mpu->dev_id, 1);
    if(ret)
    {
        goto fail1;
    }

    //class_create
    mpu->class = class_create(THIS_MODULE, CLIENT_I2C_NAME);
    if(IS_ERR(mpu->class))
    {
        ret = PTR_ERR(mpu->class);
        goto fail2;
    }

    //device_create
    mpu->device = device_create(mpu->class, &pdev->dev, 
                                mpu->dev_id, NULL, CLIENT_I2C_NAME);

    if(IS_ERR(mpu->device))
    {
        ret = PTR_ERR(&mpu->device);
        goto fail3;
    }                             

    platform_set_drvdata(pdev, mpu);


    iic_write_reg8(mpu->gpio_SCL, mpu->gpio_SDA, MPU6050_PWR_MGMT_1, 0x00);
    //iic_write_reg8(mpu->gpio_SCL, mpu->gpio_SDA, MPU6050_PWR_MGMT_2, 0x00);
    iic_write_reg8(mpu->gpio_SCL, mpu->gpio_SDA, MPU6050_SMPLRT_DIV, 0x07);
    iic_write_reg8(mpu->gpio_SCL, mpu->gpio_SDA, MPU6050_CONFIG,     0x06);
    iic_write_reg8(mpu->gpio_SCL, mpu->gpio_SDA, MPU6050_GYRO_CONFIG, 0x18);
    iic_write_reg8(mpu->gpio_SCL, mpu->gpio_SDA, MPU6050_ACCEL_CONFIG, 0x01);

    //iic_write_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x6b, 0x80);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x6b);

    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3B);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3C);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3D);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3E);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x3F);
    iic_read_reg8(mpu->gpio_SCL, mpu->gpio_SDA, 0x40);

    init_timer(&mpu->timer);
    mpu->timer.function = mpu_timer_function;
    mod_timer(&mpu->timer, jiffies + msecs_to_jiffies(1000));
    //mpu->timer.expires = jffies + msecs_to_jiffies(1000);
    return 0;
fail3:
    class_destroy(mpu->class);
fail2:
    cdev_del(&mpu->cdev);
fail1:
    unregister_chrdev_region(mpu->dev_id, 1);
    return ret;
    /*设置platform_dev_data指针*/

    return 0;
}


static int client_i2c_remove(struct platform_device *client)
{
    device_destroy(mpu->class, mpu->dev_id);

    class_destroy(mpu->class);

    cdev_del(&mpu->cdev);

    unregister_chrdev_region(mpu->dev_id, 1);

    gpio_free(mpu->gpio_SCL);
    gpio_free(mpu->gpio_SDA);

    del_timer(&mpu->timer);
    kfree(mpu);
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


// static int __init client_spi_init(void)
// {
//     platform_driver_register(&client_i2c_driver);
//     return 0;
// }

// static void __exit client_spi_exit(void)
// {
//     platform_driver_unregister(&client_i2c_driver);
// }

module_platform_driver(client_i2c_driver);

// module_init(client_spi_init);
// module_exit(client_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("tianyu.zhao@mevion.com");