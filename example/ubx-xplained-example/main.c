#include <atmel_start.h>
#include <string.h>
#include "ubx.h"

/* move to uc-interfa */
#define SPI_SIZE	(256)
#define UBX_FFCNT	(32)

static uint8_t MOSI[SPI_SIZE];
static uint8_t MISO[SPI_SIZE];

int32_t spi_transfer(struct spi_m_sync_descriptor *spi_des, const struct spi_xfer *p_xfer);
uint8_t	cfgUBXoverSPI(struct spi_m_sync_descriptor *spi_des, uint8_t ffCnt);
void	clearSPIbuffers();
uint8_t cfgPSMOO(struct spi_m_sync_descriptor *spi_des, uint8_t period);
/* end uc-interfa */

int main(void)
{
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();

    /* set clock mode and enable spi */
    spi_m_sync_set_mode(&SPI_0, SPI_MODE_0);
    spi_m_sync_enable(&SPI_0);

    /* SPI init code */
    UBXMsgBuffer    ubx_obuff;
    UBXMsg          *ubxmsg;
    struct spi_xfer spi_buff;
    uint8_t         cfgerr;
    	
    /* initialize read/write buffers */
    clearSPIbuffers();
    spi_buff.size  = SPI_SIZE;
    spi_buff.txbuf = MOSI;
    spi_buff.rxbuf = MISO;
    ubxmsg         = NULL;
    delay_ms(500); /* allow startup for device */
    spi_transfer(&SPI_0, &spi_buff); /* empty communication */
	
	/* Do a Poll message to get info/check if GPS is ready */
    //ubx_obuff = getCFG_RST(0,0);
    //ubx_obuff = getCFG_PRT_POLL_OPT(UBXPRTSPI);    
	//ubx_obuff = getCFG_RXM_POLL();
    //ubx_obuff = getCFG_PM2_POLL();
    //ubx_obuff = getCFG_GNSS_POLL();
    //ubx_obuff = getCFG_MSG_RATE();
    ubx_obuff = getRXM_PMREQ(10000,2);
    
	/* Init SO buffer and copy data */
    memcpy(MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);
    
    delay_ms(100);
    spi_transfer(&SPI_0, &spi_buff);
    alignUBXmessage(ubxmsg, MISO, SPI_SIZE);

    ubx_obuff = getCFG_RXM_POLL();
    memcpy(MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);

    delay_ms(100);
    spi_transfer(&SPI_0, &spi_buff);
   // alignUBXmessage(ubxmsg, MISO, SPI_SIZE);
    
    delay_ms(100);
//    cfgerr = cfgUBXoverSPI(&SPI_0, UBX_FFCNT);
//    delay_ms(100);
    cfgerr = cfgPSMOO(&SPI_0,0);
    alignUBXmessage(ubxmsg, MISO, SPI_SIZE);
    delay_ms(100);

	ubx_obuff = getNAV_PVT_POLL();
    memcpy(MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);

    while (1) 
    {
        delay_ms(100);	
        spi_transfer(&SPI_0, &spi_buff);
	}
}

int32_t spi_transfer(struct spi_m_sync_descriptor *spi_des, const struct spi_xfer *p_xfer)
{
    int32_t retval;

    gpio_set_pin_level(SPI_SS, false);
    retval = spi_m_sync_transfer(spi_des, p_xfer);
    gpio_set_pin_level(SPI_SS, true);

    return retval;
}


uint8_t cfgUBXoverSPI(struct spi_m_sync_descriptor *spi_des, uint8_t ffCnt)
{
    struct spi_xfer spi_buff;
    UBXMsgBuffer    ubx_obuff;
    UBXMsgs         ubxmsgs;
    UBXMsg          *ubxmsg;
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
    ubxmsg    = NULL;

    /* Construct/Associate SPI tx/rx buffers */
    spi_buff.size  = ubx_obuff.size;
    memcpy(MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);

    spi_buff.txbuf = MOSI;
    spi_buff.rxbuf = MISO;

    /* Send CFG_PRT message, Receive ACK_??? message */
    spi_transfer(spi_des, &spi_buff);

	/* TODO: Timing to limit buffsize/numxfers */
	clearSPIbuffers();

	/* May need a second xfer to receive ACK */	
    spi_transfer(spi_des, &spi_buff);
	
    /* Check rxbuffer for ACK/NAK */
    alignUBXmessage(ubxmsg, MISO, SPI_SIZE);

    if(ubxmsg != NULL &&
       ubxmsg->hdr.msgClass == UBXMsgClassACK &&
       ubxmsg->hdr.msgId == UBXMsgClassACK )
    {
        ack = true;
    }
    else
    {
        ack = false;
    }

    return ack;
}

void clearSPIbuffers()
{    
    for(int i = 0; i < SPI_SIZE; i++) {
        MOSI[i] = 0xff;
        MISO[i] = 0x00;
    }
}

uint8_t cfgPSMOO(struct spi_m_sync_descriptor *spi_des, uint8_t period)
{
    struct spi_xfer spi_buff;
    UBXMsgBuffer    ubx_obuff;
    UBXMsgs         ubxmsgs;
    UBXMsg          *ubxmsg;
    uint8_t         ack;

    ubxmsg    = NULL;

    /* Message version (protocol 15) */
    ubxmsgs.CFG_PM2.version = 1;

    /* Set Flags */
    /* Disable External Interrupt */
    ubxmsgs.CFG_PM2.flags.extIntSelect = 0;
    ubxmsgs.CFG_PM2.flags.extIntWake = 0;
    ubxmsgs.CFG_PM2.flags.extIntBackup = 0;

    /* Misc. Flags */
    ubxmsgs.CFG_PM2.flags.limitPeakCurr = UBXPM2LimitCurrentEnabled;
    ubxmsgs.CFG_PM2.flags.waitTimeFix = 0;
    ubxmsgs.CFG_PM2.flags.updateRTC = 0;
    ubxmsgs.CFG_PM2.flags.updateEPH = 0;
    ubxmsgs.CFG_PM2.flags.doNotEnterOff = 0;

    /* Power Saving Mode */
    ubxmsgs.CFG_PM2.flags.mode = UBXPM2OnOffOperation;
    /* End Flags */

    /* Position update period (ms) */
    ubxmsgs.CFG_PM2.updatePeriod = 0;
    /* Position search period (ms) */
    ubxmsgs.CFG_PM2.searchPeriod = 0;

    //TODO : Will this help?
    ubxmsgs.CFG_PM2.gridOffset = 0;

    /* Time to stay in tracking mode (s) */
    ubxmsgs.CFG_PM2.onTime = 0;

    /* Minimal search time (s) */
    ubxmsgs.CFG_PM2.minAcqTime = 0;

    /* Build MOSI configuration message, and MISO buffer */
    ubx_obuff = getCFG_PM2( ubxmsgs.CFG_PM2.flags, 
                            ubxmsgs.CFG_PM2.updatePeriod, 
                            ubxmsgs.CFG_PM2.searchPeriod,
                            ubxmsgs.CFG_PM2.gridOffset, 
                            ubxmsgs.CFG_PM2.onTime, 
                            ubxmsgs.CFG_PM2.minAcqTime);
    

    /* Construct/Associate SPI tx/rx buffers */
    spi_buff.size  = ubx_obuff.size;
    memcpy(MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);

    spi_buff.txbuf = MOSI;
    spi_buff.rxbuf = MISO;

    /* Send CFG_PRT message, Receive ACK_??? message */
    spi_transfer(spi_des, &spi_buff);

    clearSPIbuffers();

    /* May need a second xfer to receive ACK */
    spi_transfer(spi_des, &spi_buff);
    	
    /* Check rxbuffer for ACK/NAK */
    alignUBXmessage(ubxmsg, MISO, SPI_SIZE);

    if(ubxmsg != NULL &&
       ubxmsg->hdr.msgClass == UBXMsgClassACK &&
       ubxmsg->hdr.msgId == UBXMsgClassACK )
    {
        ack = true;
    }
    else
    {
        ack = false;
    }

    return ack;
}
