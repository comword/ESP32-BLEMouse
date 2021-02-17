#ifndef APP_SPI_H_
#define APP_SPI_H_

#include "driver/spi_master.h"

void spi_init(spi_device_handle_t *handle);
int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);

#endif
