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

#define USE_SIMU_I2C  0

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

    iic_read_reg8(mpu, data, 0x08, len);

    ret = copy_to_user(buff, (const void *) data, len);

    return ret;
}

// static int mpu6050_get_external(struct mpu6050 *mpu, struct mpu6050_read_reg *rd_reg)
// {
//     int ret;
//     uint8_t *pbuffer;
//     if(rd_reg->length > 0)
//     {
//         pbuffer = kmalloc(rd_reg->length, GFP_KERNEL);
//         if(IS_ERR(pbuffer))
//         {
//             return -ENOMEM;
//         }

//         iic_read_reg8(mpu->iic_io, pbuffer, rd_reg->reg_addr, rd_reg->length);

//         ret = copy_to_user(rd_reg->buff, pbuffer, rd_reg->length);

//         kfree(pbuffer);
//     }
//     else
//     {
//         return -EINVAL;
//     }

//     return 0;
// }

// static int mpu6050_set_external(struct mpu6050 *mpu, struct mpu6050_write_reg *wr_reg)
// {
//     int ret;
//     uint8_t *pbuffer;
//     if(wr_reg->length > 0)
//     {
//         pbuffer = kmalloc(wr_reg->length, GFP_KERNEL);
//         if(IS_ERR(pbuffer))
//         {
//             return -ENOMEM;
//         }

//         copy_from_user(pbuffer, wr_reg->buff, wr_reg->length);
//         ret = iic_write_reg8(mpu->iic_io, wr_reg->reg_addr, pbuffer, wr_reg->length);
        
//         kfree(pbuffer);
//     }

//     return 0;
// }

static long mpu6050_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    long rc;
    struct mpu6050 *mpu = filep->private_data;
    struct mpu6050_read_reg rd_reg;
    struct mpu6050_write_reg wr_reg;
    char *buff;
    // struct mpu6050_read_reg *ptrRdReg;
    // struct mpu6050_write_reg * __user wrRegPtr;
    void * __user arg_ptr = (void __user *) arg;
    
    switch(cmd){
        case MPU6050_READ_DATA:
        {
            // /*copy struct from user space*/
            rc = copy_from_user(&rd_reg, arg_ptr, sizeof(rd_reg));
            if(rc != 0)
            {
                printk("Unable to copy channel buffer address from \
                            userspace for MPU6050_READ_DATA.\n");
            }
            // printk("Addr=%x\n", rd_reg.reg_addr);
            // printk("Len=%d\n", rd_reg.length);

            buff = kzalloc(rd_reg.length, GFP_KERNEL);

            #if USE_SIMU_I2C
                iic_read_reg8(mpu, buff, rd_reg.reg_addr, rd_reg.length);
            #else
                mpu6050_read_reg(mpu->client, rd_reg.reg_addr, buff, rd_reg.length);
            #endif
            // /*on succes 0 returned, on fail non-0 returned which 
            // * represent how many bytes failed to copy*/
            rc = copy_to_user(rd_reg.buff, buff, rd_reg.length);

            kfree(buff);
        }
        break;

        case MPU6050_WRITE_DATA:
        {
            rc = copy_from_user(&wr_reg, arg_ptr, sizeof(wr_reg));
            if(rc != 0)
            {
                printk("Unable to copy channel buffer address from \
                            userspace for MPU6050_WRITE_DATA.\n");
            }

            /*prepare buffer for datas from userspace*/
            // printk("Addr=%x\n", wr_reg.reg_addr);
            // printk("Len=%d\n", wr_reg.length);
            // printk("buff start at=%x\n", wr_reg.buff);

            buff= kzalloc(wr_reg.length, GFP_KERNEL);
            
            /*BUG*/
            //wr_reg.buff = kzalloc(wr_reg.length, GFP_KERNEL);
            rc = copy_from_user(buff, wr_reg.buff, wr_reg.length);

            if(rc !=0 )
            {
                printk("Unable to copy buff data from userspace for MPU6050_WRITE_DATA\n");
            }

            #if USE_SIMU_I2C
            iic_write_reg8(mpu, wr_reg.reg_addr, buff, wr_reg.length);
            #else
            mpu6050_write_reg(mpu->client, wr_reg.reg_addr, buff, wr_reg.length);
            #endif

            kfree(buff);
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



int mpu6050_parse_tree(struct mpu6050 *mpu, struct device_node *nd)
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

    iic_read_reg8(mpu, data, 0x3B, 6);
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

    ret = mpu6050_parse_tree(mpu, pdev->dev.of_node);

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
    // setup_timer(&mpu->timer, mpu_timer_function, (unsigned long) mpu);
    // mod_timer(&mpu->timer, jiffies + msecs_to_jiffies(1000));

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