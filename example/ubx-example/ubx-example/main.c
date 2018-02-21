#include <atmel_start.h>
#include <string.h>
#include "ubxproto/ubx.h"

#define SPI_SIZE (128)

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	delay_init(SysTick);

	UBXMsgBuffer ubxmsgbuff;
	ubxmsgbuff = getMON_VER_POLL();
	
	/* spi buffer init code */
	char   spi_clear[SPI_SIZE];
	char   spi_ibuff[SPI_SIZE];
	char   spi_obuff[SPI_SIZE];
	struct spi_xfer spi_buff;
		
	/* initialize read/write buffers */
	for(int i = 0; i < SPI_SIZE; i++) {
		spi_clear[i] = 0xff;
		spi_obuff[i] = 0xff;
		spi_ibuff[i] = 0xff;
	}
		
	/* associate flash buffers with spi device */
	spi_buff.size = SPI_SIZE;
	spi_buff.rxbuf = (uint8_t*) spi_ibuff;
	spi_buff.txbuf = (uint8_t*) spi_obuff;
		
	/* set clock mode and enable spi */
	spi_m_sync_set_mode(&SPI_0, SPI_MODE_0);
	spi_m_sync_enable(&SPI_0);

	/* disable all periodic nmea messages */
	//memcpy(spi_obuff, NMEA_NOMSGS, strlen(NMEA_NOMSGS));
	//gpio_set_pin_level(GPIO_PIN(SPI_SS),false);
	//spi_m_sync_transfer(&SPI_0, &spi_buff);
	//gpio_set_pin_level(GPIO_PIN(SPI_SS),true);
	
	/* poll for device information */
	memcpy(spi_obuff, ubxmsgbuff.data, ubxmsgbuff.size);
	gpio_set_pin_level(GPIO_PIN(SPI_SS),false);
	spi_m_sync_transfer(&SPI_0, &spi_buff);
	gpio_set_pin_level(GPIO_PIN(SPI_SS),true);

	/* clear buffer */
	memcpy(spi_obuff, spi_clear, SPI_SIZE);
	gpio_set_pin_level(GPIO_PIN(SPI_SS),false);
	spi_m_sync_transfer(&SPI_0, &spi_buff);
	gpio_set_pin_level(GPIO_PIN(SPI_SS),true);
	
    /* Setting SPI Mode confs */
    //TODO Look at ubxmessages.h 1215, make structs and pack
    UBXCFG_PRT cfgprt;
    cfgprt.txReady.en = 0;
    //cfgprt.mode.UBX_DDC = 0;
    //cfgprt.mode.UBX_UART = 0;
    //cfgprt.mode.UBX_USB = 0;
    cfgprt.mode.UBX_SPI.spiMode = UBXPRTSPIMode0;
    cfgprt.mode.UBX_SPI.flowControl = 0;
    cfgprt.mode.UBX_SPI.ffCnt = 4;
    cfgprt.inProtoMask = 1;
    cfgprt.outProtoMask = 1;
    cfgprt.flags = UBXPRTExtendedTxTimeout;

    ubxmsgbuff = setCFG_PRT_SPI(cfgprt);
    memcpy(spi_obuff, ubxmsgbuff.data, ubxmsgbuff.size);
    gpio_set_pin_level(GPIO_PIN(SPI_SS),false);
    spi_m_sync_transfer(&SPI_0, &spi_buff);
    gpio_set_pin_level(GPIO_PIN(SPI_SS),true);

    /* clear buffer */
    //memcpy(spi_obuff, spi_clear, SPI_SIZE);
    //gpio_set_pin_level(GPIO_PIN(SPI_SS),false);
    //spi_m_sync_transfer(&SPI_0, &spi_buff);
    //gpio_set_pin_level(GPIO_PIN(SPI_SS),true);

	while (1) {
		gpio_set_pin_level(GPIO_PIN(SPI_SS),false);
		spi_m_sync_transfer(&SPI_0, &spi_buff);
		gpio_set_pin_level(GPIO_PIN(SPI_SS),true);
		delay_ms(10);

	}
}
