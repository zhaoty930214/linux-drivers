#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/fs.h>  
#include <linux/init.h>  
#include <linux/ide.h>  
#include <linux/types.h>  
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/mach/map.h>
#include <asm/io.h>
#include <linux/dma/xilinx_dma.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_dma.h>
#include <linux/random.h>
#include <linux/delay.h>

#define Y_RES 480
#define X_RES 752
// #define MT9V034_DEV_ID    0x90   //MT9V034 器件地址
#define MT9V034_DEV_ID    0x90   //MT9V034 器件地址

#define DEV_COUNT   1
#define DEV_NAME    "my-axi-vdma"
/* 自定义结构体用于描述我们的LCD设备 */
struct xilinx_vdma_camera_dev {
	struct platform_device *pdev;	// platform平台设备
	struct dma_chan *vdma;			// VDMA通道
    // void *buf;                      // 存储数据的缓冲区
    void *p_virt;
    void *p_virt_ori;
    dma_addr_t fb_phys;		// 显存物理地址
	int major;
	int minor;
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct device_node *nd;
    int gpio_SCL;
    int gpio_SDA;

    atomic_t b_finish;
};


/*字符设备打开，关闭，读取，写入*/

static int vdma_open(struct inode *inode, struct file *file)
{
    struct xilinx_vdma_camera_dev *data;

    data = container_of(inode->i_cdev, struct xilinx_vdma_camera_dev, cdev);

    file->private_data = data;

    return 0;
}

static int vdma_release(struct inode *inode, struct file *file)
{
    // if(file->private_data != NULL)
    //     kfree(file->private_data);
    file->private_data = NULL;
    return 0;
}


static void xilinx_vdmatest_slave_rx_callback(void *completion)
{
    struct xilinx_vdma_camera_dev *dev = (struct xilinx_vdma_camera_dev *) completion;
    atomic_set(&dev->b_finish, 1);
	printk("VDMA run in callback\r\n");
}



static void launch_vdma(struct xilinx_vdma_camera_dev *camdev)
{
    struct dma_async_tx_descriptor *tx_desc;

    // dmaengine_terminate_all(camdev->vdma);

	/* This barrier ensures 'thread' is initialized and
	 * we get valid DMA channels
	 */
	smp_rmb();

    struct dma_interleaved_template dma_template = {0};
    /* 初始化VDMA通道 */
    dma_template.dir            =       DMA_DEV_TO_MEM;
    dma_template.dst_start      =       camdev->fb_phys;	// 物理地址
    
    dma_template.numf           =       Y_RES;
	dma_template.sgl[0].size    =       X_RES*3;
	dma_template.sgl[0].icg     =       0;
    dma_template.frame_size     =       1;
    /*TO DO*/
	dma_template.dst_inc        =       1;
	dma_template.dst_sgl        =       1;
    dma_template.src_sgl        =       0;
    dma_template.src_inc        =       0;

    tx_desc = dmaengine_prep_interleaved_dma(camdev->vdma, &dma_template, 
                DMA_CTRL_ACK|DMA_PREP_INTERRUPT);
	if (!tx_desc) {
		printk(KERN_INFO"Failed to prepare DMA descriptor\n");
		dma_release_channel(camdev->vdma);
		return;
	}
    printk("Module driver VDMA TAG3\r\n");

    tx_desc->callback = xilinx_vdmatest_slave_rx_callback;
    tx_desc->callback_param = camdev;
    dmaengine_submit(tx_desc);

    dma_async_issue_pending(camdev->vdma);

    printk("Module driver VDMA TAG4\r\n");
}


static ssize_t vdma_read(struct file *file, char __user *buf, size_t size, loff_t *off)
{
    struct xilinx_vdma_camera_dev *data = file->private_data;
    // int i = 0;
    int ret = -1;
    int i;

    if(size != (Y_RES*X_RES*3))
    {
        printk("Error read size from user space!\r\n");
        return 0;
    }
    else
    {
        data->p_virt = data->p_virt_ori;
        // for(i=0; i<Y_RES*X_RES; i++)
        // {
        //     *((uint8_t *)data->p_virt + i) = (uint8_t) i%256;
        // }

        launch_vdma(data);
        while(atomic_read(&data->b_finish)== 0);
        printk("Module driver VDMA TAG5\r\n");

        ret = copy_to_user(buf, data->p_virt, size);
        if(ret == 0)
            printk(KERN_INFO"Copy to user success\r\n");
        else
        {
            printk(KERN_INFO"Copy to user failed. Remain %d bytes\r\n", ret);
        }
    }
    return ret;
}

static const struct file_operations vdma_fops = {
    .owner = THIS_MODULE,
    .open  = vdma_open,
    .read  = vdma_read,
    .release = vdma_release
};


static int local_register_chrdev(struct xilinx_vdma_camera_dev *camdev)
{
	if(camdev->major){
		camdev->devid = MKDEV(camdev->major, 0);
		register_chrdev_region(camdev->devid, DEV_COUNT, DEV_NAME);
	} else {
		alloc_chrdev_region(&camdev->devid, camdev->minor, DEV_COUNT, DEV_NAME);
		camdev->major = MAJOR(camdev->devid);
		camdev->minor = MINOR(camdev->devid);
	}

	cdev_init(&camdev->cdev, &vdma_fops);

	cdev_add(&camdev->cdev, camdev->devid, DEV_COUNT);

	camdev->class = class_create(THIS_MODULE, DEV_NAME);
	if( IS_ERR(camdev->class)) {
		return PTR_ERR(camdev->class);
	}

	camdev->device = device_create(camdev->class, NULL,  
			      camdev->devid, NULL,
				  DEV_NAME);
	if( IS_ERR(camdev->device)){
		return PTR_ERR(camdev->device);
	}

    return 0;
}

void iic_start(struct xilinx_vdma_camera_dev *camdev)
{
    gpio_set_value(camdev->gpio_SCL, 1);
    gpio_set_value(camdev->gpio_SDA, 1);
    
    udelay(4);

    gpio_set_value(camdev->gpio_SDA, 0);

    udelay(4);

    gpio_set_value(camdev->gpio_SCL, 0);
}


void iic_stop(struct xilinx_vdma_camera_dev *camdev)
{
    gpio_set_value(camdev->gpio_SCL, 0);
    gpio_set_value(camdev->gpio_SDA, 0);
    
    udelay(4);

    gpio_set_value(camdev->gpio_SCL, 1);

    udelay(4);

    gpio_set_value(camdev->gpio_SDA, 1);
}

void iic_send_byte(struct xilinx_vdma_camera_dev *camdev, uint8_t txd)
{
    uint8_t t;
    gpio_set_value(camdev->gpio_SCL, 0);

    for(t=0; t<8; t++)
    {
        gpio_set_value(camdev->gpio_SDA, (txd&0x80)>>7);
        txd <<= 1;

        udelay(4);

        gpio_set_value(camdev->gpio_SCL, 1);

        udelay(4);

        gpio_set_value(camdev->gpio_SCL, 0);

        udelay(4);
    }
}


uint8_t iic_rece_byte(struct xilinx_vdma_camera_dev *camdev)
{
    uint8_t i=0, rxd=0;

    gpio_direction_input(camdev->gpio_SDA);

    gpio_set_value(camdev->gpio_SCL, 0);

    udelay(4);

    for(i=0; i<8; i++)
    {
        gpio_set_value(camdev->gpio_SCL, 1);
        udelay(2);

        rxd <<= 1;
        if(gpio_get_value(camdev->gpio_SDA))
        {
            rxd |= 0x01;
        }
        udelay(2);

        gpio_set_value(camdev->gpio_SCL, 0);

        udelay(4);
    }

    gpio_direction_output(camdev->gpio_SDA, 0);

    return rxd;
}



//产生ACK应答
void iic_ack(struct xilinx_vdma_camera_dev *camdev)
{
    gpio_set_value(camdev->gpio_SCL, 0);
    gpio_set_value(camdev->gpio_SDA, 0);
    udelay(4);

    gpio_set_value(camdev->gpio_SCL, 1);
    udelay(4);

    gpio_set_value(camdev->gpio_SCL, 0);
    udelay(4);;
}

//两线式接口写寄存器
void iic_write_reg8(struct xilinx_vdma_camera_dev *camdev, uint8_t addr , uint16_t data)
{

	iic_start(camdev);

	iic_send_byte(camdev, MT9V034_DEV_ID);
	iic_ack(camdev);

	iic_send_byte(camdev, addr);
	iic_ack(camdev);

	iic_send_byte(camdev, data>>8 & 0X00FF);
	iic_ack(camdev);
    
	iic_send_byte(camdev, data & 0x00FF);
	iic_ack(camdev);
    
  	iic_stop(camdev);
}


//两线式接口读寄存器
uint16_t iic_read_reg8(struct xilinx_vdma_camera_dev *camdev, uint8_t addr)
{
	uint16_t rxd;

	iic_start(camdev);

	iic_send_byte(camdev, MT9V034_DEV_ID);
	iic_ack(camdev);

	iic_send_byte(camdev, addr);
	iic_ack(camdev);

  	iic_start(camdev);

	iic_send_byte(camdev, MT9V034_DEV_ID | 0x01);
	iic_ack(camdev);

	rxd = iic_rece_byte(camdev) << 8;
	iic_ack(camdev);
    
	rxd |= iic_rece_byte(camdev);
	iic_ack(camdev);    

  	iic_stop(camdev);

  	return  rxd ;
}


static int init_gpio(struct platform_device *pdev, struct xilinx_vdma_camera_dev *camdev)
{
    int ret;
    struct device_node *nd = camdev->nd;
    /*init MT9V034*/
    uint16_t cam_id = 0;
    uint16_t hor_blank = 94;             //Horizontal Blanking,默认值94

    nd = pdev->dev.of_node;
    if(NULL == nd)
    {
        printk(KERN_ERR"vdma node can not found!\r\n");
        return -EINVAL;
    }
    
    /*SCL*/
    camdev->gpio_SCL = of_get_named_gpio(nd, "sccbSCL-gpio", 0);
    if(!gpio_is_valid(camdev->gpio_SCL))
    {
        printk(KERN_ERR"camVdma:Failed to get sccbSCL-gpio\n");
        return -EINVAL;
    }
    printk(KERN_INFO"camVdma: SCCB SCL gpio num = %d\r\n", camdev->gpio_SCL);
    
    ret = gpio_request(camdev->gpio_SCL, "SCCB-SCL-GPIO");
    printk(KERN_INFO"SCCB GPIO SCL:%d", camdev->gpio_SCL);
    if(ret)
    {
		printk(KERN_ERR "camVdma: Failed to request sccbSCL-gpio\n");
		return ret;
    }

    gpio_direction_output(camdev->gpio_SCL, 1);


    /*SDA*/
    camdev->gpio_SDA = of_get_named_gpio(nd, "sccbSDA-gpio", 0);
    if(!gpio_is_valid(camdev->gpio_SDA))
    {
        printk(KERN_ERR"camVdma:Failed to get sccbSDA-gpio\n");
        return -EINVAL;
    }
    printk(KERN_INFO"camVdma: SCCB SDA gpio num = %d\r\n", camdev->gpio_SDA);
    
    ret = gpio_request(camdev->gpio_SDA, "SCCB-SDA-GPIO");
    printk(KERN_INFO"SCCB GPIO SDA:%d", camdev->gpio_SDA);
    if(ret)
    {
		printk(KERN_ERR "camVdma: Failed to request sccbSDA-gpio\n");
		return ret;
    }

    gpio_direction_output(camdev->gpio_SDA, 1);


    gpio_set_value(camdev->gpio_SCL, 1);
    gpio_set_value(camdev->gpio_SDA, 1);


    //读MT9V034摄像头ID
    cam_id = iic_read_reg8(camdev, 0x00);
    
    if(cam_id != 0x1324)            //如果没有获取到正确的MT9V034 ID
    {
        printk(KERN_ERR"11get mt9v034 id error. ID=0x%d.\r\n", cam_id);
        return -EINVAL;
    }
    else
    {
        printk(KERN_INFO"get mt9v034 id success:%d\r\n", cam_id);
    	//复位寄存器
		iic_write_reg8(camdev, 0x0c,0x01); // BIT[0]-Reset all the Reg
		udelay(1000);

    	iic_write_reg8 (camdev, 0x04, X_RES) ;        //设置水平方向分辨率(Width)
    	iic_write_reg8 (camdev, 0x05, hor_blank) ;  //设置水平消隐像素数(Horizontal Blanking)
		iic_write_reg8 (camdev, 0x03, Y_RES) ;        //设置垂直方向分辨率(Height)



        printk(KERN_INFO"Start to read iic reg\r\n");
        printk(KERN_INFO"0x04: %x\r\n", iic_read_reg8(camdev, 0x04));
        printk(KERN_INFO"0x04: %x\r\n", iic_read_reg8(camdev, 0x04));

        printk(KERN_INFO"0x05: %x\r\n", iic_read_reg8(camdev, 0x05));
        printk(KERN_INFO"0x05: %x\r\n", iic_read_reg8(camdev, 0x05));
        printk(KERN_INFO"0x03: %x\r\n", iic_read_reg8(camdev, 0x03));
        printk(KERN_INFO"0x03: %x\r\n", iic_read_reg8(camdev, 0x03));
    }

    return 0;
}



static int vdmafb_probe(struct platform_device *pdev)
{
    int i=0;
    struct xilinx_vdma_camera_dev  *camdev;
    // void *m_virt;
    dma_addr_t fb_phys;		// 显存物理地址
    struct device *dev = &pdev->dev;
    
    struct xilinx_vdma_config vdma_config = {0};
    
    printk("Probing vdma client driver\r\n");

    camdev = devm_kzalloc(&pdev->dev, sizeof(*camdev), GFP_KERNEL);
    atomic_set(&camdev->b_finish, 0);

    // camdev->b_finish = ATOMIC_INIT(0);

    if(camdev == NULL)
    {
        pr_err("devm_kzalloc failed!\n");
        return -ENOMEM;
    }
    platform_set_drvdata(pdev, camdev);

    if( init_gpio(pdev, camdev) != 0)
    {
        return -EINVAL;
    }

    if(0!=local_register_chrdev(camdev))
    {
        printk("Register chrdev failed!\r\n");
        return -1;
    }
    printk("Register vdma device success\r\n");

    camdev->vdma = of_dma_request_slave_channel(dev->of_node, "vdma1");
    vdma_config.park = 1;
    xilinx_vdma_channel_set_config(camdev->vdma, &vdma_config);

    if(IS_ERR(camdev->vdma))
    {
        dev_err(dev, "Failed to request vdma channel");
        return PTR_ERR(camdev->vdma);
    }
    printk("Module driver VDMA TAG1\r\n");


    camdev->p_virt_ori = dma_alloc_wc(dev, PAGE_ALIGN(Y_RES*X_RES*3), &fb_phys, GFP_KERNEL); 
    if (!camdev->p_virt_ori)
        return -ENOMEM;
    //camdev->buf = kzalloc(Y_RES*X_RES,GFP_KERNEL);
    camdev->fb_phys = fb_phys;
    camdev->p_virt = camdev->p_virt_ori;
    //camdev->buf = camdev->p_virt;

    /*Init screen*/
    for(i=0; i<Y_RES*X_RES; i++)
    {
        *((uint8_t *)camdev->p_virt + i) = (uint8_t) i%256;
    }

    if(camdev->p_virt == NULL)
    {
        dev_err(dev, "Failed to request memory for vdma buffer");
        return -ENOMEM;
    }
    printk("Module driver VDMA TAG2\r\n");
    return 0;
}


static int vdmafb_remove(struct platform_device *pdev)
{
	struct xilinx_vdma_camera_dev *fbdev = platform_get_drvdata(pdev);

    gpio_free(fbdev->gpio_SCL);
    gpio_free(fbdev->gpio_SDA);

    cdev_del(&fbdev->cdev);

	unregister_chrdev_region(fbdev->devid, DEV_COUNT);
	
	device_destroy(fbdev->class, fbdev->devid);
	
	class_destroy(fbdev->class);


	dmaengine_terminate_all(fbdev->vdma);

	dma_release_channel(fbdev->vdma);

    dma_free_wc(&pdev->dev, Y_RES*X_RES*3,
            fbdev->p_virt_ori, fbdev->fb_phys);


    //if(file->private_data != NULL)
    devm_kfree(&pdev->dev, fbdev);

	return 0;
}


static const struct of_device_id vdmafb_of_match_table[] = {
	{ .compatible = "xlnx,vdmaCamera", },
	{ /* end of table */ },
};

MODULE_DEVICE_TABLE(of, vdmafb_of_match_table);


static struct platform_driver xilinx_vdma_camera = {
	.probe    = vdmafb_probe,
	.remove   = vdmafb_remove,
	.driver = {
		.name           = "xilinx-vdmaCamera",
		.of_match_table = vdmafb_of_match_table,
	},
};


module_platform_driver(xilinx_vdma_camera);

MODULE_DESCRIPTION("Camera data driver based on Xilinx VDMA IP Core.");
MODULE_AUTHOR("Zhaoty");
MODULE_LICENSE("GPL");