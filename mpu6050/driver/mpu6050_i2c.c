#include "mpu6050_types.h"
#include "mpu6050_lib.h"
#include "mpu6050_constants.h"

static void iic_start(struct IIC_IO iio_io)
{
    gpio_set_value(iio_io.gpio_SCL, 1);
    gpio_set_value(iio_io.gpio_SDA, 1);
    
    udelay(4);

    gpio_set_value(iio_io.gpio_SDA, 0);

    udelay(4);

    gpio_set_value(iio_io.gpio_SCL, 0);
}


static void iic_stop(struct IIC_IO iio_io)
{
    gpio_set_value(iio_io.gpio_SCL, 0);
    gpio_set_value(iio_io.gpio_SDA, 0);
    
    udelay(4);

    gpio_set_value(iio_io.gpio_SCL, 1);

    udelay(4);

    gpio_set_value(iio_io.gpio_SDA, 1);
}

static void iic_send_byte(struct IIC_IO iio_io, uint8_t txd)
{
    uint8_t t;
    gpio_set_value(iio_io.gpio_SCL, 0);

    for(t=0; t<8; t++)
    {
        gpio_set_value(iio_io.gpio_SDA, (txd&0x80)>>7);
        txd <<= 1;

        udelay(4);

        gpio_set_value(iio_io.gpio_SCL, 1);

        udelay(4);

        gpio_set_value(iio_io.gpio_SCL, 0);

        udelay(4);
    }
}


static uint8_t iic_rece_byte(struct IIC_IO iio_io)
{
    uint8_t i=0, rxd=0;

    gpio_direction_input(iio_io.gpio_SDA);

    gpio_set_value(iio_io.gpio_SCL, 0);

    udelay(4);

    for(i=0; i<8; i++)
    {
        gpio_set_value(iio_io.gpio_SCL, 1);
        udelay(2);

        rxd <<= 1;
        if(gpio_get_value(iio_io.gpio_SDA))
        {
            rxd |= 0x01;
        }
        udelay(2);

        gpio_set_value(iio_io.gpio_SCL, 0);

        udelay(4);
    }

    gpio_direction_output(iio_io.gpio_SDA, 0);

    return rxd;
}



//产生ACK应答
static void iic_ack(struct IIC_IO iio_io)
{
    gpio_set_value(iio_io.gpio_SCL, 0);
    gpio_set_value(iio_io.gpio_SDA, 0);
    udelay(4);

    gpio_set_value(iio_io.gpio_SCL, 1);
    udelay(4);

    gpio_set_value(iio_io.gpio_SCL, 0);
    udelay(4);;
}

//两线式接口写寄存器
int iic_write_reg8(struct mpu6050 *mpu, uint8_t addr , uint8_t *data, int len)
{
    int i;
	iic_start(mpu->iic_io);

	iic_send_byte(mpu->iic_io, MPU6050_ADDR | 0x0);
	iic_ack(mpu->iic_io);

	iic_send_byte(mpu->iic_io, addr);
	iic_ack(mpu->iic_io);

    for(i=0; i< len; i++)
    {
        iic_send_byte(mpu->iic_io, data[i]);
        iic_ack(mpu->iic_io);
    }
    
  	iic_stop(mpu->iic_io);

    return 0;
}

int iic_write_byte(struct mpu6050 *mpu, uint8_t addr , uint8_t data)
{
	iic_start(mpu->iic_io);

	iic_send_byte(mpu->iic_io, MPU6050_ADDR | 0x0);
	iic_ack(mpu->iic_io);

	iic_send_byte(mpu->iic_io, addr);
	iic_ack(mpu->iic_io);

    iic_send_byte(mpu->iic_io, data);
    iic_ack(mpu->iic_io);
    
  	iic_stop(mpu->iic_io);

    return 0;
}



//两线式接口读寄存器
uint8_t iic_read_reg8(struct mpu6050 *mpu, uint8_t *buff, uint8_t addr, int len)
{   
    int i;

    if(len < 0)
    {
        mpu6050_err("iic read len err");
    }

	iic_start(mpu->iic_io);

	iic_send_byte(mpu->iic_io, MPU6050_ADDR | 0x0);
	iic_ack(mpu->iic_io);

	iic_send_byte(mpu->iic_io, addr);
	iic_ack(mpu->iic_io);

  	iic_start(mpu->iic_io);

	iic_send_byte(mpu->iic_io, MPU6050_ADDR | 0x01);
	iic_ack(mpu->iic_io);
    
    for(i=0; i<len; i++)
    {
        buff[i] = iic_rece_byte(mpu->iic_io);
        iic_ack(mpu->iic_io);    
    }

  	iic_stop(mpu->iic_io);
    // for(i=0; i<len; i++)
    // {
    //     printk(KERN_INFO "Read out from *%x=%x\r\n", addr+i, buff[i]);
    // }
  	return  buff[0];
}

int mpu6050_write_reg(struct i2c_client *pclient, u8 reg, u8 *buf, u8 len) 
{
	struct i2c_client *client = pclient;
	struct i2c_msg msg;
	int ret;
    char *send_buf;

	if (16 < len) {
		dev_err(&client->dev, "%s: error: Invalid transfer byte length %d\n",
					__func__, len);
		return -EINVAL;
	}

    send_buf = kzalloc(len+1, GFP_KERNEL);

	memcpy(&send_buf[1], buf, len);	// 将要写入的数据存放到数组send_buf后面

    send_buf[0] = reg;
	msg.addr = client->addr;		// pcf8563从机地址
	msg.flags = client->flags;		// 标记为写数据
	msg.buf = send_buf;				// 要写入的数据缓冲区
	msg.len = len + 1;				// 要写入的数据长度

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (1 != ret) {
        kfree(send_buf);
		dev_err(&client->dev, "%s: error: reg=0x%x, len=0x%x\n",
					__func__, 0, len);
		return -EIO;
	}

    kfree(send_buf);

    return 0;
}

int mpu6050_write_byte(struct i2c_client *pclient, u8 reg, u8 value, u8 len) 
{
	struct i2c_client *client = pclient;
	struct i2c_msg msg;
	int ret;
    char buf[2];

	if (16 < len) {
		dev_err(&client->dev, "%s: error: Invalid transfer byte length %d\n",
					__func__, len);
		return -EINVAL;
	}

    buf[0] = reg;
    buf[1] = value;

	msg.addr = client->addr;		// pcf8563从机地址
	msg.flags = client->flags;		// 标记为写数据
	msg.buf = buf;				// 要写入的数据缓冲区
	msg.len = 2;				// 要写入的数据长度

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (1 != ret) {
		dev_err(&client->dev, "%s: error: reg=0x%x, len=0x%x\n",
					__func__, 0, len);
		return -EIO;
	}
    else
    {
        dev_info(&client->dev, "%s success: reg=0x%x, val=0x%x\n",
                    __func__, reg, value);
    }

    return 0;
}


int mpu6050_read_reg(struct i2c_client *pclient, u8 reg, u8 *buf, u8 len)
{
    int ret;
    struct i2c_client *client = pclient;
    struct i2c_msg msg[2];

    msg[0].addr = client->addr;
    msg[0].flags = client->flags;
    msg[0].buf = &reg;
    msg[0].len = 1;

    msg[1].addr = client->addr;
    msg[1].flags = client->flags | I2C_M_RD;
    msg[1].buf = buf;
    msg[1].len = len;

    ret = i2c_transfer(client->adapter, msg, 2);
    if(2 != ret) {
        dev_err(&client->dev, "%s: error: reg=0x%x, len=0x%x\n", 
                    __func__, reg, len);
    }
    // else
    // {
    //     dev_info(&client->dev, "%s read success: reg=0x%x, len=0x%x\n",
    //                 __func__, reg, len);
    // }
    return 0;
}


