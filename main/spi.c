#include <string.h>

#include "spi.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5
#define SPI_DEBUG_TAG "SPI"

#define READ_BIT_MASK			0x80
#define WRITE_BIT_MASK			0x7F

/* Forward declaration */
static int spi_master_transfer_tx(spi_device_handle_t *handle, uint8_t register_addr, const uint8_t * value, uint32_t len);
static int spi_master_transfer_rx(spi_device_handle_t *handle, uint8_t register_addr, uint8_t * value, uint32_t len);

void spi_init(spi_device_handle_t *handle)
{
	esp_err_t ret;
	spi_bus_config_t buscfg={
		.mosi_io_num=GPIO_MOSI,
		.miso_io_num=GPIO_MISO,
		.sclk_io_num=GPIO_SCLK
	};

	spi_device_interface_config_t devcfg={
		.command_bits = 0,
		.address_bits = 8,
		.dummy_bits = 0,
		.clock_speed_hz=8*1000*1000,    //Clock out at 8 MHz
		.mode=0,
		.spics_io_num=GPIO_CS,
		.queue_size=1,
		.flags=0
	};

	gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);
	ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
	assert(ret==ESP_OK);
	ret=spi_bus_add_device(VSPI_HOST, &devcfg, handle);
	ESP_LOGI(SPI_DEBUG_TAG, "%s initialize SPI successfully\n", __func__);
}

static int spi_master_transfer_tx(spi_device_handle_t *handle, uint8_t register_addr, const uint8_t * value, uint32_t len)
{
	int ret = 0;
	uint8_t reg	= register_addr;
//	const uint8_t *p_rbuf = value;
//	uint32_t rsize = len;
//	uint32_t i;
//	uint8_t uc_pcs = 0;
//	uint16_t data = 0;
//
	reg &= WRITE_BIT_MASK;
//
//	spi_write(SPI_MASTER_BASE, reg, 0, 0); /* write cmd/reg-addr */
//	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done */
//	spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* dummy read */
//
//	for (i = 0; i < rsize; i++) {
//		spi_write(SPI_MASTER_BASE, p_rbuf[i], 0, 0); /* dummy write to generate clock */
//		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done. */
//		spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* read actual register data */
//	}

	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.addr = reg;
	t.length = len*8;
	t.rxlength = 0;
	t.tx_buffer = value;
	t.rx_buffer = NULL;
	ret = spi_device_transmit(*handle, &t);
	assert(ret==ESP_OK);
	return 0;
}

static int spi_master_transfer_rx(spi_device_handle_t *handle, uint8_t register_addr, uint8_t * value, uint32_t len)
{
	int ret = 0;
	uint8_t reg	= register_addr;
//	uint8_t *p_rbuf	= value;
//	uint32_t rsize	= len;
//	uint32_t i;
//	uint8_t uc_pcs = 0;//SPI_CHIP_PCS;
//	uint16_t data = 0;
//
	reg |= READ_BIT_MASK;
//
//	spi_write(SPI_MASTER_BASE, reg, 0, 0); /* write cmd/reg-addr */
//	while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done */
//	spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* dummy read */
//
//	for (i = 0; i < rsize; i++) {
//		spi_write(SPI_MASTER_BASE, 0x0, 0, 0); /* dummy write to generate clock */
//		while ((spi_read_status(SPI_MASTER_BASE) & SPI_SR_TDRE) == 0); /* Wait transfer data reg done. */
//		spi_read(SPI_MASTER_BASE, &data, &uc_pcs); /* read actual register data */
//		p_rbuf[i] = (uint8_t)(data & 0xFF);
//	}
	int len_in_bits = len*8;
	char *dummy_send = malloc(len);
	memset(dummy_send, 0, len);
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.addr = reg;
	t.length = len_in_bits;
	t.rxlength = len_in_bits;
	t.tx_buffer = dummy_send;
	t.rx_buffer = value;
	ret = spi_device_transmit(*handle, &t);
	assert(ret==ESP_OK);
	free(dummy_send);
	return 0;
}

int idd_io_hal_read_reg(void * context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	(void)context;
	return spi_master_transfer_rx(context, reg, rbuffer, rlen);
}

int idd_io_hal_write_reg(void * context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	(void)context;
	return spi_master_transfer_tx(context, reg, wbuffer, wlen);
}
