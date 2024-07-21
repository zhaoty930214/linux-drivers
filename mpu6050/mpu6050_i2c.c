#include "mpu6050_types.h"


static void iic_start(int gpio_SCL, int gpio_SDA)
{
    gpio_set_value(gpio_SCL, 1);
    gpio_set_value(gpio_SDA, 1);
    
    udelay(4);

    gpio_set_value(gpio_SDA, 0);

    udelay(4);

    gpio_set_value(gpio_SCL, 0);
}


static void iic_stop(int gpio_SCL, int gpio_SDA)
{
    gpio_set_value(gpio_SCL, 0);
    gpio_set_value(gpio_SDA, 0);
    
    udelay(4);

    gpio_set_value(gpio_SCL, 1);

    udelay(4);

    gpio_set_value(gpio_SDA, 1);
}

static void iic_send_byte(int gpio_SCL, int gpio_SDA, uint8_t txd)
{
    uint8_t t;
    gpio_set_value(gpio_SCL, 0);

    for(t=0; t<8; t++)
    {
        gpio_set_value(gpio_SDA, (txd&0x80)>>7);
        txd <<= 1;

        udelay(4);

        gpio_set_value(gpio_SCL, 1);

        udelay(4);

        gpio_set_value(gpio_SCL, 0);

        udelay(4);
    }
}


static uint8_t iic_rece_byte(int gpio_SCL, int gpio_SDA)
{
    uint8_t i=0, rxd=0;

    gpio_direction_input(gpio_SDA);

    gpio_set_value(gpio_SCL, 0);

    udelay(4);

    for(i=0; i<8; i++)
    {
        gpio_set_value(gpio_SCL, 1);
        udelay(2);

        rxd <<= 1;
        if(gpio_get_value(gpio_SDA))
        {
            rxd |= 0x01;
        }
        udelay(2);

        gpio_set_value(gpio_SCL, 0);

        udelay(4);
    }

    gpio_direction_output(gpio_SDA, 0);

    return rxd;
}



//产生ACK应答
static void iic_ack(int gpio_SCL, int gpio_SDA)
{
    gpio_set_value(gpio_SCL, 0);
    gpio_set_value(gpio_SDA, 0);
    udelay(4);

    gpio_set_value(gpio_SCL, 1);
    udelay(4);

    gpio_set_value(gpio_SCL, 0);
    udelay(4);;
}

//两线式接口写寄存器
void iic_write_reg8(int gpio_SCL, int gpio_SDA, uint8_t addr , uint16_t data)
{

	iic_start(gpio_SCL, gpio_SDA);

	iic_send_byte(gpio_SCL, gpio_SDA, MPU6050_ADDR | 0x0);
	iic_ack(gpio_SCL, gpio_SDA);

	iic_send_byte(gpio_SCL, gpio_SDA, addr);
	iic_ack(gpio_SCL, gpio_SDA);

	iic_send_byte(gpio_SCL, gpio_SDA, data>>8 & 0X00FF);
	iic_ack(gpio_SCL, gpio_SDA);
    
	iic_send_byte(gpio_SCL, gpio_SDA, data & 0x00FF);
	iic_ack(gpio_SCL, gpio_SDA);
    
  	iic_stop(gpio_SCL, gpio_SDA);
}


//两线式接口读寄存器
uint8_t iic_read_reg8(int gpio_SCL, int gpio_SDA, uint8_t addr)
{
	uint8_t rxd=0;

	iic_start(gpio_SCL, gpio_SDA);

	iic_send_byte(gpio_SCL, gpio_SDA, MPU6050_ADDR | 0x0);
	iic_ack(gpio_SCL, gpio_SDA);

	iic_send_byte(gpio_SCL, gpio_SDA, addr);
	iic_ack(gpio_SCL, gpio_SDA);

  	iic_start(gpio_SCL, gpio_SDA);

	iic_send_byte(gpio_SCL, gpio_SDA, MPU6050_ADDR | 0x01);
	iic_ack(gpio_SCL, gpio_SDA);
    
	rxd |= iic_rece_byte(gpio_SCL, gpio_SDA);
	iic_ack(gpio_SCL, gpio_SDA);    

  	iic_stop(gpio_SCL, gpio_SDA);

    printk(KERN_INFO "Read out from *%x=%x\r\n", addr, rxd);
  	return  rxd ;
}

static int mpu6050_read_reg(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
    struct i2c_msg msg[2];
    int ret;

    msg[0].addr = client->addr;
    msg[0].flags = client->flags;
    msg[0].buf = &reg;
    msg[0].len = 1;

    msg[1].addr = client->addr;
    msg[1].flags = client->flags | I2C_M_RD;
    msg[1].buf = buf;
    msg[1].len = len;

    ret = i2c_transfer(client->adapter, msg, 2);
    if( 2!= ret)
    {
        dev_err(&client->dev, "%s:error:reg=0x%02x,len=0x%x\n",
                                __func__, reg, len);
        return -EIO;
    }
    return 0;
}