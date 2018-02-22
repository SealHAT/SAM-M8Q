#include <atmel_start.h>
#include "ubx.h"

#define SPI_SIZE (128)

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
    delay_init(SysTick);

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

    /* UBX Config Port SPI */
    ubxmsgs.CFG_PRT.portID = UBXPRTSPI;
    ubxmsgs.CFG_PRT.txReady.en = 0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.spiMode = UBXPRTSPIMode0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.flowControl = 0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.ffCnt = 4;
    ubxmsgs.CFG_PRT.inProtoMask = UBXPRTInProtoInUBX;
    ubxmsgs.CFG_PRT.outProtoMask = UBXPRTOutProtoOutUBX;
    ubxmsgs.CFG_PRT.flags = UBXPRTExtendedTxTimeout;

    ubx_obuff = setCFG_PRT(ubxmsgs.CFG_PRT);

    /* Associate ubx and spi buffers */
    //TODO point a ubxbufferstruct to the spi buffer, probz aligned to 0xffffffff0x??

	/* Replace with your application code */
	while (1) {

	}
}
