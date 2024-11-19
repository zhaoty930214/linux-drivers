#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include "mpu6050_types.h"
#include "mpu6050_ioctl.h"
#include "mpu6050_lib.h"

#define CLIENT_I2C_NAME     "I2C_MPU6050"

int mpu6050_open (struct inode *inode, struct file *filp)
{
    filp->private_data = container_of(inode->i_cdev, struct mpu6050, cdev);

    return 0;
}

int mpu6050_release(struct inode *inode, struct file *filp)
{
    filp->private_data = NULL;

    return 0;
}


/*
 * @description			: 从pcf8563设备中读取多个连续的寄存器数据
 * @param – dev			: pcf8563设备
 * @param – reg			: 要读取的寄存器首地址
 * @param – buf			: 数据存放缓存区地址
 * @param – len			: 读取的字节长度
 * @return				: 成功返回0，失败返回一个负数
 */
// static int pcf8563_read_reg(struct pcf8563_dev *dev, u8 reg, u8 *buf, u8 len)
// {
// 	struct i2c_client *client = dev->client;
// 	struct i2c_msg msg[2];
// 	int ret;

// 	/* msg[0]: 发送消息 */
// 	msg[0].addr = client->addr;		// pcf8563从机地址
// 	msg[0].flags = client->flags;	// 标记为写数据
// 	msg[0].buf = &reg;				// 要写入的数据缓冲区
// 	msg[0].len = 1;					// 要写入的数据长度

// 	/* msg[1]: 接收消息 */
// 	msg[1].addr = client->addr;		// pcf8563从机地址
// 	msg[1].flags = client->flags | I2C_M_RD;// 标记为读数据
// 	msg[1].buf = buf;				// 存放读数据的缓冲区
// 	msg[1].len = len;				// 读取的字节长度

// 	ret = i2c_transfer(client->adapter, msg, 2);
// 	if (2 != ret) {
// 		dev_err(&client->dev, "%s: error: reg=0x%x, len=0x%x\n",
// 					__func__, reg, len);
// 		return -EIO;
// 	}

// 	return 0;
// }

static long mpu6050_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
    int rc;
    struct mpu6050 *mpu = filep->private_data;
    struct i2c_client *client = mpu->client;
    struct mpu6050_read_reg rdReg;
    struct mpu6050_write_reg wrReg;
    char *buffer;

    void * __user argPtr = (void __user *) arg;
    struct i2c_msg msg[2];
    struct i2c_msg msg1;

    switch(cmd){
        case MPU6050_READ_DATA:
        {
            /*get read structure from user space*/
            rc = copy_from_user(&rdReg, argPtr, sizeof(struct mpu6050_read_reg));
            if( 0 != rc)
            {
                printk("Unable to copy channel buffer struct from \
                        userspace for MPU6050_READ_DATA.\n");
            }

            /*alloc buffer for data incoming*/
            buffer = kzalloc(rdReg.length, GFP_KERNEL);

            // dev_info(&client->dev, "%s: client->addr=%x \n.",
            //                 __func__, client->addr);
            /*Preparing to read*/
           
            msg[0].addr = client->addr;
            msg[0].buf = &rdReg.reg_addr;
            msg[0].flags = client->flags;
            msg[0].len = 1;

            msg[1].addr = client->addr;
            msg[1].buf = buffer;
            msg[1].flags = client->flags | I2C_M_RD;
            msg[1].len = rdReg.length;

            rc = i2c_transfer(client->adapter, msg, 2);
            if(2 != rc) {
                dev_err(&client->dev, "%s: error: reg=0x%x, len=0x%x\n", 
                    __func__, rdReg.reg_addr, rdReg.length);
            }
            else
            {
                // dev_info(&client->dev, "%s read success: reg=0x%x, len=0x%x\n",
                //     __func__, rdReg.reg_addr, rdReg.length);
                // for( rc = 0; rc<rdReg.length; rc++)
                // {
                //     printk("\rreadback val:%d=0x%x", rc, buffer[rc]);
                // }
    
            }

            rc = copy_to_user(rdReg.buff, buffer, rdReg.length);
            if( 0!= rc)
            {
                printk("Unable to copy channel buffer address to \
                        userspace for MPU6050_READ_DATA.\n");
            }
            kfree(buffer);
        }
        break;

        case MPU6050_WRITE_DATA:
        {
            rc = copy_from_user(&wrReg, argPtr, sizeof(struct mpu6050_write_reg));
            if( 0 != rc)
            {
                printk("Unable to copy channel buffer struct from \
                        userspace for MPU6050_WRITE_DATA.\n");
            }

            buffer = kzalloc(wrReg.length, GFP_KERNEL);

            /*copy data from user space*/
            rc= copy_from_user(buffer, wrReg.buff, wrReg.length);
            if( 0 != rc)
            {
                printk("Unable to copy channel buffer address from \
                        userspace for MPU6050_WRITE_DATA.\n");
            }


            msg1.addr = client->addr;
            msg1.flags = client->flags;
            msg1.buf = buffer;
            msg1.len = wrReg.length;

            rc = i2c_transfer(client->adapter, &msg1, 1);
            if(1 != rc) {
                dev_err(&client->dev, "%s: error: reg=0x%x, len=0x%x\n", 
                    __func__, wrReg.reg_addr, wrReg.length);
            }
            else
            {
                // dev_info(&client->dev, "%s read success: reg=0x%x, len=0x%x\n",
                //     __func__, wrReg.reg_addr, wrReg.length);

            }
            kfree(buffer);
        }
        break;
        default:
            return -EINVAL;
        break;
    }

    return 0;
}

static struct file_operations mpu6050_ops={
    .open = mpu6050_open,
    .release = mpu6050_release,
    .read = NULL,
    .write = NULL,
    .unlocked_ioctl = mpu6050_ioctl,
};



static int client_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;

    printk(KERN_INFO"This is mpu6050 driver probe\n");

    struct mpu6050 *mpu = kmalloc(sizeof(struct mpu6050), GFP_KERNEL);

    if(IS_ERR(mpu))
    {
        ret = -ENOMEM;
        goto fail1_nomem;
    }

    mpu->client = client;
    // ret = mpu6050_parse_tree(mpu, pdev->dev.of_node);

    if(ret)
    {
        printk("parse tree failed\r\n");
    }

    ret = alloc_chrdev_region(&mpu->dev_id, 0, 1, CLIENT_I2C_NAME);
    if(ret)
    {
        printk("alloc chrdev failed\r\n");
        goto fail2_alloc_err;
    }

    mpu->cdev.owner = THIS_MODULE;

    cdev_init(&mpu->cdev, &mpu6050_ops);

    ret = cdev_add(&mpu->cdev, mpu->dev_id, 1);
    if(ret)
    {
        printk("cdev_add failed\r\n");
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
    mpu->device = device_create(mpu->class, &client->dev, 
                                mpu->dev_id, NULL, CLIENT_I2C_NAME);

    if(IS_ERR(mpu->device))
    {
        ret = PTR_ERR(&mpu->device);
        goto fail5_device_err;
    }                             

    i2c_set_clientdata(client, mpu);

    mpu6050_init(mpu);

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

static int client_i2c_remove(struct i2c_client *i2c)
{
    struct mpu6050 *mpu = (struct mpu6050 *)i2c_get_clientdata(i2c);
    
    //del_timer(&mpu->timer);

    device_destroy(mpu->class, mpu->dev_id);

    class_destroy(mpu->class);

    cdev_del(&mpu->cdev);

    unregister_chrdev_region(mpu->dev_id, 1);

    // del_timer(&mpu->timer);
    kfree(mpu);

    //task_struct *t;
    return 0;
}



static struct of_device_id clien_i2c_dt_match[] = 
{
    {.compatible = "zty,mpu-6050"},
    {/*Setinel*/}
};

static struct i2c_driver client_i2c_driver = {
    .probe = client_i2c_probe,
    .remove = client_i2c_remove,
    .driver = {
        .name = "mpu6050",
        .of_match_table = clien_i2c_dt_match,
    },
};



module_i2c_driver(client_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("zhao_ty@qq.com");
