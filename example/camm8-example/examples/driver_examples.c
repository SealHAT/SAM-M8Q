/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

/**
 * Example of using analog_in to generate waveform.
 */
void analog_in_example(void)
{
	uint8_t buffer[2];

	adc_sync_enable_channel(&analog_in, 0);

	while (1) {
		adc_sync_read_channel(&analog_in, 0, buffer, 2);
	}
}

/* CRC Data in flash */
COMPILER_ALIGNED(4)
static const uint32_t crc_datas[] = {0x00000000,
                                     0x11111111,
                                     0x22222222,
                                     0x33333333,
                                     0x44444444,
                                     0x55555555,
                                     0x66666666,
                                     0x77777777,
                                     0x88888888,
                                     0x99999999};

/**
 * Example of using hash_chk to Calculate CRC32 for a buffer.
 */
void hash_chk_example(void)
{
	/* The initial value used for the CRC32 calculation usually be 0xFFFFFFFF,
	 * but can be, for example, the result of a previous CRC32 calculation if
	 * generating a common CRC32 of separate memory blocks.
	 */
	uint32_t crc = 0xFFFFFFFF;
	uint32_t crc2;
	uint32_t ind;

	crc_sync_enable(&hash_chk);
	crc_sync_crc32(&hash_chk, (uint32_t *)crc_datas, 10, &crc);

	/* The read value must be complemented to match standard CRC32
	 * implementations or kept non-inverted if used as starting point for
	 * subsequent CRC32 calculations.
	 */
	crc ^= 0xFFFFFFFF;

	/* Calculate the same data with subsequent CRC32 calculations, the result
	 * should be same as previous way.
	 */
	crc2 = 0xFFFFFFFF;
	for (ind = 0; ind < 10; ind++) {
		crc_sync_crc32(&hash_chk, (uint32_t *)&crc_datas[ind], 1, &crc2);
	}
	crc2 ^= 0xFFFFFFFF;

	/* The calculate result should be same. */
	while (crc != crc2)
		;
}

void wire_example(void)
{
	struct io_descriptor *wire_io;

	i2c_m_sync_get_io_descriptor(&wire, &wire_io);
	i2c_m_sync_enable(&wire);
	i2c_m_sync_set_slaveaddr(&wire, 0x12, I2C_M_SEVEN);
	io_write(wire_io, (uint8_t *)"Hello World!", 12);
}

/**
 * Example of using spi_dev to write "Hello World" using the IO abstraction.
 */
static uint8_t example_spi_dev[12] = "Hello World!";

void spi_dev_example(void)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&spi_dev, &io);

	spi_m_sync_enable(&spi_dev);
	io_write(io, example_spi_dev, 12);
}

void delay_example(void)
{
	delay_ms(5000);
}
