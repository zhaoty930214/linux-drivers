#include <linux/init.h>
#include <linux/platform_device.h>
// #include <linux/pm_runtime.h>
// #include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/of_gpio.h>
#include <linux/uaccess.h>
#include <linux/timer.h>

#include "mpu6050_lib.h"
#include "mpu6050_ioctl.h"


int mpu6050_open(struct inode *nd, struct file *filep)
{
    struct mpu6050 *mpu = container_of(nd->i_cdev, struct mpu6050, cdev);

    filep->private_data = mpu;

    return 0;
}


static int mpu6050_release(struct inode *nd, struct file *filep)
{
    filep->private_data = NULL;

    return 0;
}

static ssize_t mpu6050_read(struct file *filep, char __user *buff,  size_t len, loff_t *off_t)
{
    int ret;
    char data[len];

    struct mpu6050 *mpu = filep->private_data;

    if(IS_ERR(mpu))
    {
        printk("Read err, mpu is NULL\r\n");
    }

    iic_read_reg8(mpu->iic_io, data, 0x08, len);

    ret = copy_to_user(buff, (const void *) data, len);

    return ret;
}

static int mpu6050_get_external(struct mpu6050 *mpu, struct i2c_read_reg *rd_reg)
{
    int ret;
    uint8_t *pbuffer;
    if(rd_reg->length > 0)
    {
        pbuffer = kmalloc(rd_reg->length, GFP_KERNEL);
        if(IS_ERR(pbuffer))
        {
            return -ENOMEM;
        }

        iic_read_reg8(mpu->iic_io, pbuffer, rd_reg->reg_addr, rd_reg->length);

        ret = copy_to_user(rd_reg->buff, pbuffer, rd_reg->length);

        kfree(pbuffer);
    }
    else
    {
        return -EINVAL;
    }

    return 0;
}

static int mpu6050_set_external(struct mpu6050 *mpu, struct i2c_write_reg *wr_reg)
{
    int ret;
    uint8_t *pbuffer;
    if(wr_reg->length > 0)
    {
        pbuffer = kmalloc(wr_reg->length, GFP_KERNEL);
        if(IS_ERR(pbuffer))
        {
            return -ENOMEM;
        }

        copy_from_user(pbuffer, wr_reg->buff, wr_reg->length);
        ret = iic_write_reg8(mpu->iic_io, wr_reg->reg_addr, pbuffer, wr_reg->length);
        
        kfree(pbuffer);
    }

    return 0;
}

static long mpu6050_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    long rc;
    struct mpu6050 *mpu = filep->private_data;

    uint8_t *pbuf;
    struct i2c_read_reg read_reg;
    struct i2c_write_reg write_reg;

    void * __user arg_ptr = (void __user *) arg;
    switch(cmd){
        case MPU6050_READ_REG:
        {
            copy_from_user(&read_reg, arg_ptr, sizeof(read_reg));
            
            rc = mpu6050_get_external(mpu, &read_reg);
        }
        break;

        case MPU6050_WRITE_REG:
        {
            copy_from_user(&write_reg, arg_ptr, sizeof(write_reg));

            rc = mpu6050_set_external(mpu, &write_reg);
        }
        break;


        default:
        break;
    }

    return rc;
}


static const struct file_operations mpu6050_ops= {
    .owner = THIS_MODULE,
    .open = mpu6050_open,
    .read = mpu6050_read,
    .write = NULL,
    .release = mpu6050_release,
    .unlocked_ioctl = mpu6050_ioctl,
};



static int of_parse_tree(struct mpu6050 *mpu, struct device_node *nd)
{
    int ret;
    struct device_node *np = nd;

    // struct mpu6050 *mpu = nd->
    if(NULL == np)
    {
        printk(KERN_ERR "vdma node can not found!\r\n");
        return -EINVAL;
    }

    mpu->iic_io.gpio_SCL = of_get_named_gpio(nd, "gpio-SCL", 0);
    if(!gpio_is_valid(mpu->iic_io.gpio_SCL))
    {
        printk(KERN_WARNING"Can't get gpio-SCL\r\n");
        return -1;
    }
    else
    {
        printk(KERN_INFO "gpio-SCL=%d\r\n", mpu->iic_io.gpio_SCL);
        ret = gpio_request(mpu->iic_io.gpio_SCL, "SCCB-SCL-GPIO");
        printk(KERN_INFO"SCCB GPIO SCL:%d\r\n", mpu->iic_io.gpio_SCL);
        if(ret)
        {
            printk(KERN_ERR "mpu6050: Failed to request sccbSCL-gpio\n");
            return ret;
        }
        gpio_direction_output(mpu->iic_io.gpio_SCL, 1);
    }

    mpu->iic_io.gpio_SDA = of_get_named_gpio(nd, "gpio-SDA", 0);
    if(!gpio_is_valid(mpu->iic_io.gpio_SDA))
    {
        printk(KERN_WARNING"Can't get gpio-SDA\r\n");
        return -1;
    }
    else
    {
        printk(KERN_INFO "gpio-SDA=%d\r\n", mpu->iic_io.gpio_SDA);
        ret = gpio_request(mpu->iic_io.gpio_SDA, "SCCB-SDA-GPIO");
        printk(KERN_INFO"SCCB GPIO SDA:%d\r\n", mpu->iic_io.gpio_SDA);
        if(ret)
        {
            printk(KERN_ERR "mpu6050: Failed to request sccbSCL-gpio\n");
            return ret;
        }
        gpio_direction_output(mpu->iic_io.gpio_SDA, 1);

    }

    gpio_set_value(mpu->iic_io.gpio_SCL, 1);
    gpio_set_value(mpu->iic_io.gpio_SDA, 1);

    return 0;
}


static void mpu_timer_function(unsigned long arg)
{
    uint8_t data[6];

    struct mpu6050 *mpu = (struct mpu6050 *) arg;

    iic_read_reg8(mpu->iic_io, data, 0x3B, 6);
    // iic_read_reg8(mpu->iic_io, &data, 0x3C, 1);
    // iic_read_reg8(mpu->iic_io, &data, 0x3D, 1);
    // iic_read_reg8(mpu->iic_io, &data, 0x3E, 1);
    // iic_read_reg8(mpu->iic_io, &data, 0x3F, 1);
    // iic_read_reg8(mpu->iic_io, &data, 0x40, 1);

    mod_timer(&mpu->timer, jiffies + msecs_to_jiffies(1000));

}

static void mpu_timer_function1(struct timer_list *t)
{
    // uint8_t data[6];

    // struct mpu6050 *mpu = (struct mpu6050 *) arg;

    // iic_read_reg8(mpu->iic_io, data, 0x3B, 6);
    // // iic_read_reg8(mpu->iic_io, &data, 0x3C, 1);
    // // iic_read_reg8(mpu->iic_io, &data, 0x3D, 1);
    // // iic_read_reg8(mpu->iic_io, &data, 0x3E, 1);
    // // iic_read_reg8(mpu->iic_io, &data, 0x3F, 1);
    // // iic_read_reg8(mpu->iic_io, &data, 0x40, 1);

    // mod_timer(&mpu->timer, jiffies + msecs_to_jiffies(1000));

}

int mpu6050_chrdev_init(struct platform_device *pdev)
{
    int ret;

    struct mpu6050 *mpu = kmalloc(sizeof(struct mpu6050), GFP_KERNEL);

    if(IS_ERR(mpu))
    {
        ret = -ENOMEM;
        goto fail1_nomem;
    }

    ret = of_parse_tree(mpu, pdev->dev.of_node);

    if(ret)
    {
        mpu6050_err("parse tree failed");
    }

    ret = alloc_chrdev_region(&mpu->dev_id, 0, 1, CLIENT_I2C_NAME);
    if(ret)
    {
        mpu6050_err("alloc chrdev failed");
        goto fail2_alloc_err;
    }

    cdev_init(&mpu->cdev, &mpu6050_ops);

    ret = cdev_add(&mpu->cdev, mpu->dev_id, 1);
    if(ret)
    {
        mpu6050_err("cdev_add failed");
        goto fail3_cdev_add_err;
    }


    //class_create
    mpu->class = class_create(THIS_MODULE, CLIENT_I2C_NAME);
    if(IS_ERR(mpu->class))
    {
        ret = PTR_ERR(mpu->class);
        goto fail4_class_err;
    }

    //device_create
    mpu->device = device_create(mpu->class, &pdev->dev, 
                                mpu->dev_id, NULL, CLIENT_I2C_NAME);

    if(IS_ERR(mpu->device))
    {
        ret = PTR_ERR(&mpu->device);
        goto fail5_device_err;
    }                             

    platform_set_drvdata(pdev, mpu);

    mpu6050_init(mpu);
    //setup_timer(&mpu->timer, mpu_timer_function, (unsigned long) mpu);
    // init_timers(&mpu->timer);
    timer_setup(&mpu->timer, mpu_timer_function1, 0);
    mod_timer(&mpu->timer, jiffies + msecs_to_jiffies(1000));

    return 0;

fail5_device_err:
    class_destroy(mpu->class);
fail4_class_err:
    cdev_del(&mpu->cdev);
fail3_cdev_add_err:
    unregister_chrdev_region(mpu->dev_id, 1);
fail2_alloc_err:
    kfree(mpu);
fail1_nomem:
    return ret;
}

int mpu6050_chrdev_exit(struct platform_device *pdev)
{

    struct mpu6050 *mpu = (struct mpu6050 *)platform_get_drvdata(pdev);
    
    del_timer(&mpu->timer);

    device_destroy(mpu->class, mpu->dev_id);

    class_destroy(mpu->class);

    cdev_del(&mpu->cdev);

    unregister_chrdev_region(mpu->dev_id, 1);

    gpio_free(mpu->iic_io.gpio_SCL);
    gpio_free(mpu->iic_io.gpio_SDA);

    // del_timer(&mpu->timer);
    kfree(mpu);
    return 0;
}