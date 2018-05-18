#include <atmel_start.h>
#include "usb_start.h"
#include "Utilities/seal_UTIL.h"
#include "gps.h"

static gps_log_t log[20];

int main(void)
{
	uint8_t retval = 0;
    UBXMsgBuffer buf;
    UBXMsg *msg;
	uint16_t blink = 5000;
	/* Initializes MCU, drivers and middleware */
	i2c_unblock_bus(SDA,SCL);
	atmel_start_init();
	gpio_set_pin_level(TX_RDY, true);

	/* set up the device for DDC (i2c) communication */
    if (GPS_SUCCESS == gps_init_i2c(&I2C_DEV)) {
        blink = 1000;
		gpio_set_pin_level(LED_BUILTIN, true);
    } else {
		blink = 100;
		gpio_set_pin_level(LED_BUILTIN, false);
	}

	/* pol the device for hardware configuration */
	buf = getMON_HW_POLL();
	if (GPS_SUCCESS == gps_write_i2c((const uint8_t*)buf.data, buf.size)) {
		/* read back the hardware configuration */
		buf = getMON_HW();
		retval = gps_read_i2c_poll((uint8_t*)buf.data, buf.size);
		msg = (UBXMsg*)buf.data;
	} else {
		delay_ms(1000);
	}
	
	/* poll the device for port configuration */
    buf = getCFG_PRT_POLL_OPT(UBXPRTDDC);
	if (GPS_SUCCESS == gps_write_i2c((const uint8_t*)buf.data, buf.size)) {
		buf = getCFG_PRT();
		retval = gps_read_i2c_search((uint8_t*)buf.data, buf.size);
		msg = (UBXMsg*)buf.data;
	} else {
		delay_ms(1000);
	}
	
	if ((msg->payload.CFG_PRT.txReady.en    == 1U			) &&
		(msg->payload.CFG_PRT.txReady.pol   == M8Q_TXR_POL	) &&
		(msg->payload.CFG_PRT.txReady.pin   == M8Q_TXR_PIO	) && 
		(msg->payload.CFG_PRT.txReady.thres ==  (M8Q_TXR_CNT >> 3))) {
			gpio_toggle_pin_level(LED_BUILTIN);
			delay_ms(500);
			gpio_toggle_pin_level(LED_BUILTIN);
			delay_ms(500);
			gpio_toggle_pin_level(LED_BUILTIN);
			delay_ms(500);
			gpio_toggle_pin_level(LED_BUILTIN);
			delay_ms(500);
	} else {
		gpio_set_pin_level(LED_BUILTIN, false);
	}
	
	for (;;)
	{
		if (gpio_get_pin_level(TX_RDY) == false) {
			gps_readfifo();
			gpio_toggle_pin_level(LED_BUILTIN);
			retval = gps_parsefifo(GPS_FIFO, log, 20);
			gpio_toggle_pin_level(LED_BUILTIN);
		}
	}
}
