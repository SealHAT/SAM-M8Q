#include <atmel_start.h>
#include <string.h>
#include "ubx.h"


#define SPI_SIZE	(128)
#define UBX_FFCNT	(16)

void alignbuffer(char *spibuf, UBXMsgBuffer* ubxbuf);


int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
    //delay_init(SysTick);

    /* SPI init code */
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

    /* UBX init code */
    UBXMsgBuffer ubx_ibuff;
    UBXMsgBuffer ubx_obuff;
    UBXMsgs      ubxmsgs;
	UBXMsg*		 ubxmsg;
	
    /* UBX Config Port SPI */
    ubxmsgs.CFG_PRT.portID = UBXPRTSPI;
    ubxmsgs.CFG_PRT.txReady.en = 0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.spiMode = UBXPRTSPIMode0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.flowControl = 0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.ffCnt = UBX_FFCNT;
    ubxmsgs.CFG_PRT.option.OtherReserved = 0;
    ubxmsgs.CFG_PRT.inProtoMask = UBXPRTInProtoInUBX;
    ubxmsgs.CFG_PRT.outProtoMask = UBXPRTOutProtoOutUBX;
    ubxmsgs.CFG_PRT.flags = UBXPRTExtendedTxTimeout;

    delay_ms(1000);


    /* Associate ubx and spi buffers */

	ubx_obuff = setCFG_PRT(ubxmsgs.CFG_PRT); /* BROKEN */
	
	memcpy(spi_obuff, ubx_obuff.data, ubx_obuff.size);
    spi_buff.size = SPI_SIZE;
	delay_ms(100);
	
	gpio_set_pin_level(SPI_SS, false);
	spi_m_sync_transfer(&SPI_0, &spi_buff);
	gpio_set_pin_level(SPI_SS, true);
	
	memcpy(spi_obuff, spi_clear, SPI_SIZE);
	
	gpio_set_pin_level(SPI_SS, false);
	spi_m_sync_transfer(&SPI_0, &spi_buff);
	gpio_set_pin_level(SPI_SS, true);

    ubx_ibuff.data = spi_ibuff;
	
//	alignbuffer(spi_obuff, &ubx_ibuff);
	/* Replace with your application code */
//    ubx_obuff.size = ubx_buffptr->size;
    //memcpy(ubx_ibuff.data, spi_ibuff);
	delay_ms(100);
	
	ubx_obuff = getNAV_PVT_POLL();	
	memcpy(spi_obuff, ubx_obuff.data, ubx_obuff.size);
	spi_buff.size = SPI_SIZE;
	delay_ms(100);
	
	gpio_set_pin_level(SPI_SS, false);
	spi_m_sync_transfer(&SPI_0, &spi_buff);
	gpio_set_pin_level(SPI_SS, true);
	
	delay_ms(100);
	
	while (1) {
	

		gpio_set_pin_level(SPI_SS, false);
		spi_m_sync_transfer(&SPI_0, &spi_buff);
		gpio_set_pin_level(SPI_SS, true);
		
		delay_ms(100);
		ubx_ibuff.data = spi_ibuff;
		ubxmsg = (UBXMsg*)ubx_ibuff.data;

	}
}

//void getpositionaldata()
//{
	//
//}

void alignbuffer(char *spibuf, UBXMsgBuffer* ubxbuf)
{
	int i;
	i = 0;
	
	while(i < UBX_FFCNT && spibuf[i] == 0xff) {
		i++;
	}
	
	if(i == UBX_FFCNT) {
		ubxbuf->data = &spibuf[i];
	}
}