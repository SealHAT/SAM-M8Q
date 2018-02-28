#include <atmel_start.h>
#include <string.h>
#include "ubx.h"

#define SPI_SIZE	(256)
#define UBX_FFCNT	(64)

static uint8_t MOSI[SPI_SIZE];
static uint8_t MISO[SPI_SIZE];

/* move to uc-interfa */
uint8_t	cfgUBXoverSPI(struct spi_m_sync_descriptor *spi, uint8_t ffCnt);
void	clearSPIbuffers();


int main(void)
{
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();

    /* set clock mode and enable spi */
    spi_m_sync_set_mode(&SPI_0, SPI_MODE_0);
    spi_m_sync_enable(&SPI_0);

    /* SPI init code */
    UBXMsgBuffer ubx_obuff;
    UBXMsg *ubxmsg;
    struct spi_xfer spi_buff;
    uint8_t cfgerr;
    	
    /* initialize read/write buffers */
    clearSPIbuffers();
	
	/* Do a Poll message to get info/check if GPS is ready */

    //ubx_obuff = getCFG_PRT_POLL_OPT(UBXPRTSPI);    
	ubx_obuff = getCFG_RXM_POLL();
	
	/* Init SO buffer and copy data */
    spi_buff.size  = ubx_obuff.size;
	spi_buff.size  = SPI_SIZE;
    memcpy(MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);
	
	/* Associate SPI buffers */
    spi_buff.txbuf = MOSI;
    spi_buff.rxbuf = MISO;
	
	/* Do single spi transfer */
    gpio_set_pin_level(SPI_SS, false);
    spi_m_sync_transfer(&SPI_0, &spi_buff);
    gpio_set_pin_level(SPI_SS, true);
	
	/* align message buffer to input for easy member access */
    ubxmsg = (UBXMsg*)&MISO[ubx_obuff.size + 2];


    //delay_ms(100);
//
    ///* Get a nav position/velocity/time reading */
    //ubx_obuff = getNAV_PVT_POLL(); 
    //spi_buff.size  = SPI_SIZE;
    //memcpy(MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    //clearUBXMsgBuffer(&ubx_obuff);
    //spi_buff.txbuf = MOSI;
    //spi_buff.rxbuf = MISO;
    //gpio_set_pin_level(SPI_SS, false);
    //spi_m_sync_transfer(&SPI_0, &spi_buff);
    //gpio_set_pin_level(SPI_SS, true);
    //ubxmsg = (UBXMsg*)&MISO[ubx_obuff.size+UBX_FFCNT];
    ///* End default CFG_PRT settings */
    
    delay_ms(100);
    cfgerr = cfgUBXoverSPI(&SPI_0, UBX_FFCNT);
	
	
	
    while (1) {
        delay_ms(100);
		
        gpio_set_pin_level(SPI_SS, false);
	    spi_m_sync_transfer(&SPI_0, &spi_buff);
	    gpio_set_pin_level(SPI_SS, true);
	}
}


uint8_t cfgUBXoverSPI(struct spi_m_sync_descriptor *spi, uint8_t ffCnt)
{
    #define ACK_SIZE (10) /* ACK_* message size in bytes */
    struct spi_xfer spi_buff;
    UBXMsgBuffer    ubx_obuff;
    UBXMsgs         ubxmsgs;
    UBXMsg          *ubxmsg;
    uint32_t        msgsize;
    uint8_t         ack;

    /* Populate CFG_PRT variables to enable UBX only over SPI */
    ubxmsgs.CFG_PRT.portID = UBXPRTSPI;
    ubxmsgs.CFG_PRT.txReady.en = 0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.spiMode = UBXPRTSPIMode0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.flowControl = 0;
    ubxmsgs.CFG_PRT.mode.UBX_SPI.ffCnt = ffCnt;
    ubxmsgs.CFG_PRT.option.OtherReserved = 0;
    ubxmsgs.CFG_PRT.inProtoMask = UBXPRTInProtoInUBX;
    ubxmsgs.CFG_PRT.outProtoMask = UBXPRTOutProtoOutUBX;
    ubxmsgs.CFG_PRT.flags = UBXPRTExtendedTxTimeout;

    /* Build MOSI configuration message, and MISO buffer */
    ubx_obuff = setCFG_PRT(ubxmsgs.CFG_PRT);
    msgsize   = ubx_obuff.size + ffCnt + ACK_SIZE;

    /* Construct/Associate SPI tx/rx buffers */
    spi_buff.size  = ubx_obuff.size;
    memcpy(MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);

    spi_buff.txbuf = MOSI;
    spi_buff.rxbuf = MISO;

    /* Send CFG_PRT message, Receive ACK_??? message */
   	gpio_set_pin_level(SPI_SS, false);
	spi_m_sync_transfer(spi, &spi_buff);
	gpio_set_pin_level(SPI_SS, true);

	/* TODO: Timing to limit buffsize/numxfers */
	clearSPIbuffers();

	/* May need a second xfer to receive ACK */	
    gpio_set_pin_level(SPI_SS, false);
	spi_m_sync_transfer(spi, &spi_buff);
	gpio_set_pin_level(SPI_SS, true);
	
    /* Check rxbuffer for ACK/NAK */
	/* TODO: verify aligment and sizes here */
    ubxmsg = (UBXMsg*)&MISO[msgsize - ffCnt - ACK_SIZE];
    if(ubxmsg->hdr.msgClass == UBXMsgClassACK &&
       ubxmsg->hdr.msgId == UBXMsgClassACK)
    {
        ack = true;
    }
    else
    {
        ack = false;
    }

    #undef ACK_SIZE
    return ack;
}

void clearSPIbuffers()
{    
    for(int i = 0; i < SPI_SIZE; i++) {
        MOSI[i] = 0xff;
        MISO[i] = 0x00;
    }
}
