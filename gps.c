#include "gps.h"

static uint8_t GPS_MISO[GPS_BUFFSIZE];
static uint8_t GPS_MOSI[GPS_BUFFSIZE];
static struct io_descriptor *gps_desc;
static struct spi_xfer               gps_buff;


static void	    gps_clearbuffers();
static int32_t  gps_transfer();



uint8_t gps_init_spi(struct spi_m_sync_descriptor *spi_desc) 
{
    UBXMsgBuffer    ubx_obuff;

    gps_desc       = spi_desc;
    gps_buff.size  = GPS_BUFFSIZE;
    gps_buff.txbuf = GPS_MOSI;
    gps_buff.rxbuf = GPS_MISO;

    spi_m_sync_set_mode(spi_desc, SPI_MODE_0);
    spi_m_sync_enable(spi_desc);
    
    /* Clear GPS buffers and allow time for device to boot */
    gps_clearbuffers();
    delay_ms(500);

	// TODO implement initial config
    /* OPTIONAL - Reset GPS to device defaults */
	/* ALTERNATIVELY - load/verify saved cfg */
    //ubx_obuff = getCFG_RST(0,0);    
    //memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    //clearUBXMsgBuffer(&ubx_obuff);
    //gps_transfer();
	
	/* Set default msg rate over SPI */
	ubx_obuff = getCFG_MSG_RATE(UBXMsgClassNAV,UBXMsgIdNAV_PVT,1);
	memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
	clearUBXMsgBuffer(&ubx_obuff);
	gps_transfer();
	gps_clearbuffers();
	
	/* Up sampling speed to 10Hz */
	ubx_obuff = getCFG_RATE(1000,1,0);
	memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
	clearUBXMsgBuffer(&ubx_obuff);
	gps_transfer();
	gps_clearbuffers();
	
	
	cfgUBXoverSPI(UBX_FFTCNT);
	

    //TODO get and return device status
    return 1;
}


uint8_t gps_init_i2c(struct io_descriptor *i2c_desc) 
{
	struct io_descriptor *I2C_GPS_io;
    struct _i2c_m_msg msg;
    uint8_t buf[2];


    UBXMsgBuffer    ubx_obuff;

    msg.addr = GPS_SLAVEADDR;
    msg.len  = GPS_BUFFSIZE;
    msg.flags = I2C_M_STOP;
    msg.buffer = gps_buff;

     i2c_m_sync_enable(&I2C_GPS);
     i2c_m_sync_set_slaveaddr(&I2C_GPS, GPS_SLAVEADDR, I2C_M_SEVEN);

     /* Clear GPS buffers and allow time for device to boot */
     gps_clearbuffers();
     delay_ms(500);

     // TODO implement initial config
     /* OPTIONAL - Reset GPS to device defaults */
     /* ALTERNATIVELY - load/verify saved cfg */
     //ubx_obuff = getCFG_RST(0,0);
     //memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
     //clearUBXMsgBuffer(&ubx_obuff);
     //gps_transfer();
     
     /* Set default msg rate over SPI */
     ubx_obuff = getCFG_MSG_RATE(UBXMsgClassNAV,UBXMsgIdNAV_PVT,1);
     memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
     clearUBXMsgBuffer(&ubx_obuff);
     gps_transfer();
     gps_clearbuffers();
     
     /* Up sampling speed to 10Hz */
     ubx_obuff = getCFG_RATE(1000,1,0);
     memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
     clearUBXMsgBuffer(&ubx_obuff);
     gps_transfer();
     gps_clearbuffers();
     
     
     cfgUBXoverSPI(UBX_FFTCNT);

     return 1;
}

uint8_t gps_getfix(location_t *fix, UBXNAV_PVT *soln)
{
    UBXMsgBuffer    ubx_obuff;
    UBXMsg          *msg;
    uint8_t         result;

    msg    = NULL;
	
    ubx_obuff = getNAV_PVT_POLL();
    memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);

    result = gps_transfer();
	//delay_ms(500);
    //gps_transfer();
	
    /* error check result of gps_transfer */
    //TODO Verify successful fix
    
    /* retrieve ubx message */
    alignUBXmessage(&msg, GPS_MISO, GPS_BUFFSIZE);
	
	if (msg->hdr.msgId == UBXMsgIdNAV_PVT)
	{
        *soln = msg->payload.NAV_PVT;
		fix->latitude = msg->payload.NAV_PVT.lat;
		fix->longitude = msg->payload.NAV_PVT.lon;
		
		result = 1;
	}
	else
	{
        fix->latitude  = 999999999;
        fix->longitude = 999999999;
		result = 0;
	}

    //fix->altitude = msg->payload.NAV_PVT.height;
    //fix->climb
    
    

    return result;
}

uint8_t gps_gettime(utc_time_t *time)
{

}

bool gps_setrate(const uint32_t period)
{

}

bool gps_sleep()
{

}

bool gps_wake()
{

}

bool gps_setprofile(const GPS_PROFILE profile)
{

}

void gps_clearbuffers()
{
    int i;
    for(i = 0; i < GPS_BUFFSIZE; i++) {
        GPS_MOSI[i] = 0xff;
        GPS_MISO[i] = 0x00;
    }
}

int32_t gps_transfer()
{
    int32_t retval;

    gpio_set_pin_level(SPI_SS, false);
    retval = spi_m_sync_transfer(gps_desc, &gps_buff);
    gpio_set_pin_level(SPI_SS, true);

    return retval;
}

//TODO encapsulate these helper functions


uint8_t cfgUBXoverSPI(uint8_t ffCnt)
{
    UBXMsgBuffer    ubx_obuff;
    UBXMsgs         ubxmsgs;
    UBXMsg          *ubxmsg;
    uint8_t         ack;

    /* Populate CFG_PRT variables to enable UBX only over SPI */
    ubxmsgs.CFG_PRT.portID = UBXPRTSPI;
    //ubxmsgs.CFG_PRT.txReady.en = 1; /* enable tx ready pin */
	//ubxmsgs.CFG_PRT.txReady.pol = 1;/* active low */
	//ubxmsgs.CFG_PRT.txReady.pin
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
    memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);


    /* Send CFG_PRT message, Receive ACK_??? message */
    gps_transfer();

    /* TODO: Timing to limit buffsize/numxfers */
    gps_clearbuffers();

    /* May need a second xfer to receive ACK */
    gps_transfer();

    /* Check rxbuffer for ACK/NAK */
    alignUBXmessage(&ubxmsg, GPS_MISO, GPS_BUFFSIZE);

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



uint8_t cfgPSMOO(uint8_t period)
{
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
    memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);


    /* Send CFG_PRT message, Receive ACK_??? message */
    gps_transfer();

    gps_clearbuffers();

    /* May need a second xfer to receive ACK */
    gps_transfer();

    /* Check rxbuffer for ACK/NAK */
    alignUBXmessage(&ubxmsg, GPS_MISO, GPS_BUFFSIZE);

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

uint8_t gps_selftest()
{
    UBXMsgBuffer    ubx_obuff;
    UBXMsg          *msg;
    GPS_ERROR       ack;
    
	msg = NULL;
    //ubx_obuff = getCFG_RST(0,0);
    //ubx_obuff = getCFG_PRT_POLL_OPT(UBXPRTSPI);
    //ubx_obuff = getCFG_RXM_POLL();
    //ubx_obuff = getCFG_PM2_POLL();
    //ubx_obuff = getCFG_GNSS_POLL();
    //ubx_obuff = getCFG_RATE_POLL();
    //ubx_obuff = getRXM_PMREQ(10000,2);
	ubx_obuff = getCFG_MSG_POLL(UBXMsgClassNAV, UBXMsgIdNAV_PVT);
    //ubx_obuff = getNAV_SAT_POLL();

    memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);

    gps_transfer();
    alignUBXmessage(&msg, GPS_MISO, GPS_BUFFSIZE);
    
	if (msg->hdr.msgId == UBXMsgIdACK_ACK)
	{
		ack = GPS_SUCCESS;
	} 
	else
	{
		ack = GPS_FAILURE;
	}
	
    gps_clearbuffers();

    ubx_obuff = getCFG_PRT_POLL_OPT(UBXPRTSPI);
    memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
    clearUBXMsgBuffer(&ubx_obuff);
    gps_transfer();
    alignUBXmessage(&msg, GPS_MISO, GPS_BUFFSIZE);
    gps_clearbuffers();
    
    return (uint8_t)ack;
}

