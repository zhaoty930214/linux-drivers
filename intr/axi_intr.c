#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/irq.h>

struct axi_intr{
    dev_t   devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd1;
    struct device_node *nd2;
    int key_gpio;
    int irq_num1;
    int irq_num2;
    struct fasync_struct *async_queue;
};

static struct axi_intr axi_intr_t;

static irqreturn_t axi_gpio_handler(int irq, void *dev_id)
{
    kill_fasync(&axi_intr_t.async_queue, SIGIO, POLL_IN);
    printk(KERN_INFO "IRQ triggered\n");
    return IRQ_HANDLED;
}

static int axi_intr_open(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "open was called\r\n");
    filp->private_data = &axi_intr_t;
    return 0;
}

static ssize_t axi_intr_read(struct file *filp, char __user *buf,
                         size_t cnt, loff_t *offt)
{
    return 0;
}

static ssize_t axi_intr_write(struct file *filp, const char __user *buf,
                         size_t cnt, loff_t *offt)
{
    return 0;
}

static int axi_intr_release(struct inode *inode, struct file *filp)
{
    filp->private_data = NULL;
    return 0;
}


static int axi_intr_fasync(int fd, struct file *filp, int on)
{
    return fasync_helper(fd, filp, on, &axi_intr_t.async_queue);
}



static struct file_operations axi_intr_fops={
    .owner = THIS_MODULE,
    .open = axi_intr_open,
    .release = axi_intr_release,
    .read = axi_intr_read,
    .write = axi_intr_write,
    .fasync = axi_intr_fasync,
};


static int key_parse_dt(void)
{
    struct device_node *nd;
    //const char *str;
    int ret;

    // nd = of_find_node_by_path("/fpga-debug");
    // if(NULL == nd)
    // {
    //     printk(KERN_ERR "Failed to get fpga-debug node\r\n");
    // }
    // {
    //     printk(KERN_INFO "\r\nSuccess to get fpga-debug node\r\n");
    // }


    axi_intr_t.nd1 = of_find_node_by_path("/pl_key1");
    if(NULL == axi_intr_t.nd1)
    {
        printk(KERN_ERR "axi_intr: Failed to get axi_intr node\r\n");
    }

    axi_intr_t.irq_num1 = irq_of_parse_and_map(axi_intr_t.nd1, 0);
    if(!axi_intr_t.irq_num1)
    {
        printk(KERN_ERR "get irq_num failed. ret= %d\r\n", axi_intr_t.irq_num1);
        return -EINVAL;
    }

    ret = request_irq(axi_intr_t.irq_num1, axi_gpio_handler, 0, "PL_AXI_GPIO0", NULL);
    if(ret)
    {
        printk(KERN_INFO "request irq failed\r\n");
    }


/*--------------------------------------------*/
    axi_intr_t.nd2 = of_find_node_by_path("/pl_key2");
    if(NULL == axi_intr_t.nd2)
    {
        printk(KERN_ERR "axi_intr: Failed to get axi_intr node\r\n");
    }

    axi_intr_t.irq_num2 = irq_of_parse_and_map(axi_intr_t.nd2, 0);
    if(!axi_intr_t.irq_num2)
    {
        printk(KERN_ERR "get irq_num failed. ret= %d\r\n", axi_intr_t.irq_num2);
        return -EINVAL;
    }

    ret = request_irq(axi_intr_t.irq_num2, axi_gpio_handler, 0, "PL_AXI_GPIO1", NULL);
    if(ret)
    {
        printk(KERN_INFO "request irq failed\r\n");
    }


    return 0;
}

#define IRQ_NAME    "irq"

static int axi_intr_chrdev_init(void )
{
    int ret;
    /*step1: Register dev_id*/
    ret = alloc_chrdev_region(&axi_intr_t.devid, 0, 1, IRQ_NAME);
    if(ret < 0)
    {
        printk(KERN_ERR "allocate dev_id failded\r\n");
    }
    /*step2: cdev init*/
    axi_intr_t.cdev.owner = THIS_MODULE;
    cdev_init(&axi_intr_t.cdev, &axi_intr_fops);

    /*step3: cdev add*/
    ret = cdev_add(&axi_intr_t.cdev, axi_intr_t.devid, 1);
    if(ret)
        goto fail_1;

    /*step4: class create*/
    axi_intr_t.class = class_create(THIS_MODULE, IRQ_NAME);
    if(IS_ERR(axi_intr_t.class))
    {
        ret = PTR_ERR(axi_intr_t.class);
        goto fail_2;
    }


    /*step5: device create*/
    axi_intr_t.device = device_create(axi_intr_t.class, NULL,
                                      axi_intr_t.devid, NULL,
                                      IRQ_NAME);
    if(IS_ERR(axi_intr_t.device))
    {
        ret = PTR_ERR(axi_intr_t.device);
        goto fail_3;
    }
    return 0;
fail_3:
    class_destroy(axi_intr_t.class);
fail_2:
    cdev_del(&axi_intr_t.cdev);
fail_1:
    unregister_chrdev_region(axi_intr_t.devid, 1);

    return ret;
}

// EXPORT(axi_intr_chrdev_init);

static int __init axi_intr_init(void)
{
    int ret;
    printk(KERN_ERR "tag1\r\n");

    ret = key_parse_dt();
    printk(KERN_ERR "tag2. ret = %d\r\n", ret);

    axi_intr_chrdev_init();

    if(NULL == axi_intr_t.nd1)
    {
        printk(KERN_ERR "axi: failed to get pl-key node\r\n");
        return -EINVAL;
    }
    else {
        printk(KERN_INFO "key: find node success\r\n");
    }

    return 0;
}

static void __exit axi_intr_exit(void)
{
    device_destroy(axi_intr_t.class, axi_intr_t.devid);

    class_destroy(axi_intr_t.class);

    cdev_del(&axi_intr_t.cdev);

    unregister_chrdev_region(axi_intr_t.devid, 1);

    free_irq(axi_intr_t.irq_num1 , NULL);
    free_irq(axi_intr_t.irq_num2 , NULL);
}

module_init(axi_intr_init);
module_exit(axi_intr_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("tianyu.zhao@mevion.com");
