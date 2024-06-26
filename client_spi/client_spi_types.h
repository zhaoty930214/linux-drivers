#ifndef __CLIENT_SPI_TYPES_H
#define __CLIENT_SPI_TYPES_H
#include <linux/module.h>

#define CLIENT_SPI_NAME "client_spi"


struct client_spi{
    struct spi_device *dspi;
    dev_t   dev_id;
    struct cdev cdev;
    struct class *class;
    struct device *device;
};

#endif