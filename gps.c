#include "gps.h"

static uint8_t	GPS_SDABUF[GPS_BUFFSIZE];
uint8_t			GPS_FIFO[GPS_FIFOSIZE];

static struct i2c_m_sync_desc	gps_i2c_desc;
static struct _i2c_m_msg		gps_i2c_msg;

//static void	    gps_clearbuffers();
//static int32_t  gps_transfer();
static UBXMsg   ubx_msg;
/*static UBXMsgs	ubx_msgs;*/

// uint8_t gps_init_spi(struct spi_m_sync_descriptor *spi_desc) 
// {
//     UBXMsgBuffer    ubx_obuff;
// 
//     gps_desc       = spi_desc;
//     gps_buff.size  = GPS_BUFFSIZE;
//     gps_buff.txbuf = GPS_MOSI;
//     gps_buff.rxbuf = GPS_MISO;
// 
//     spi_m_sync_set_mode(spi_desc, SPI_MODE_0);
//     spi_m_sync_enable(spi_desc);
//     
//     /* Clear GPS buffers and allow time for device to boot */
//     gps_clearbuffers();
//     delay_ms(500);
// 
// 	// TODO implement initial config
//     /* OPTIONAL - Reset GPS to device defaults */
// 	/* ALTERNATIVELY - load/verify saved cfg */
//     //ubx_obuff = getCFG_RST(0,0);    
//     //memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
//     //clearUBXMsgBuffer(&ubx_obuff);
//     //gps_transfer();
// 	
// 	/* Set default msg rate over SPI */
// 	ubx_obuff = getCFG_MSG_RATE(UBXMsgClassNAV,UBXMsgIdNAV_PVT,1);
// 	memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
// 	clearUBXMsgBuffer(&ubx_obuff);
// 	gps_transfer();
// 	gps_clearbuffers();
// 	
// 	/* Up sampling speed to 10Hz */
// 	ubx_obuff = getCFG_RATE(1000,1,0);
// 	memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
// 	clearUBXMsgBuffer(&ubx_obuff);
// 	gps_transfer();
// 	gps_clearbuffers();
// 	
// 	
// 	cfgUBXoverSPI(UBX_FFTCNT);
// 	
// 
//     //TODO get and return device status
//     return 1;
// }


uint8_t gps_init_i2c(struct i2c_m_sync_desc* const I2C_DESC) 
{
	uint8_t retval = 0;
	gps_i2c_desc = *I2C_DESC;
	
	i2c_m_sync_enable(&gps_i2c_desc);
	i2c_m_sync_set_slaveaddr(&gps_i2c_desc, M8Q_SLAVE_ADDR, I2C_M_SEVEN);
	
	/* disable other ports (Default: UART,USB,DDC enabled; SPI disabled) */
	ubx_msg.payload.CFG_PRT.portID			        = UBXPRTUART;
    ubx_msg.payload.CFG_PRT.reserved0               = 1U;
    ubx_msg.payload.CFG_PRT.mode.UBX_UART.charLen   = UBXPRTMode8BitCharLen;
    ubx_msg.payload.CFG_PRT.mode.UBX_UART.nStopBits = UBXPRTMode1StopBit;
    ubx_msg.payload.CFG_PRT.mode.UBX_UART.parity    = UBXPRTModeNoParity;
    ubx_msg.payload.CFG_PRT.inProtoMask		        = UBXPRTInProtoInUBX|UBXPRTInProtoInNMEA;
	ubx_msg.payload.CFG_PRT.outProtoMask	        = 0U;
    ubx_msg.payload.CFG_PRT.option.UARTbaudRate     = 9600U;
	if (!gps_cfgprt(ubx_msg)) {
		return 0;
	}
	
	ubx_msg.payload.CFG_PRT.portID			= UBXPRTUSB;
	ubx_msg.payload.CFG_PRT.inProtoMask		= 0U;
	ubx_msg.payload.CFG_PRT.outProtoMask	= 0U;
	if (!gps_cfgprt(ubx_msg)) {
		return 0;
	}
	
    /* configure the device to communicate over DDC (i2c) with a FIFO interrupt */
	ubx_msg.payload.CFG_PRT.portID					= UBXPRTDDC;
	ubx_msg.payload.CFG_PRT.txReady.en				= 1U;					/* 0 - disabled, 1 - enabled			*/
	ubx_msg.payload.CFG_PRT.txReady.pol			    = M8Q_TXR_POL;			/* 0 - High-active, 1 - Low-active		*/
	ubx_msg.payload.CFG_PRT.txReady.pin			    = M8Q_TXR_PIO;			/* PIO06 is TxD on the SAM-M8Q			*/
	ubx_msg.payload.CFG_PRT.txReady.thres			= GPS_FIFOSIZE / 8;		/* Given value is multiplied by 8 bytes */
	ubx_msg.payload.CFG_PRT.mode.UBX_DDC.slaveAddr  = M8Q_SLAVE_ADDR;
	ubx_msg.payload.CFG_PRT.inProtoMask			    = UBXPRTInProtoInUBX|UBXPRTInProtoInNMEA;
	ubx_msg.payload.CFG_PRT.outProtoMask			= UBXPRTOutProtoOutUBX;
	ubx_msg.payload.CFG_PRT.flags					= UBXPRTExtendedTxTimeout;
	retval = gps_cfgprt(ubx_msg);
	
	return retval;
}

uint8_t gps_cfgprt(const UBXMsg MSG)
{
	UBXMsg          *ack_msg;
	UBXMsgBuffer    ubx_buf;
	uint8_t         result  = 0;
	uint16_t        timeout = 0;
	
	do {
		/* send message to configure the DDC serial port and verify */
		ubx_buf = setCFG_PRT(MSG.payload.CFG_PRT);
		if (gps_write_i2c((const uint8_t*)ubx_buf.data, ubx_buf.size)) {
			    
			/* get acknowledge message and verify */
			ubx_buf = getACK_ACK();
			if (gps_read_i2c((uint8_t*)ubx_buf.data, ubx_buf.size)) {
				    
				/* check that the receiver configured successfully */
				ack_msg = (UBXMsg*)ubx_buf.data;
				if (ack_msg->payload.ACK_ACK.msgId == UBXMsgIdACK_ACK) {
					result = 1;
				}
			}
		}
		/* repeat until timeout or successful acknowledge from device */
	} while (result == 0 && timeout++ < CFG_TIMEOUT);
	
	return result;
}

uint8_t gps_readfifo()
{
	return gps_read_i2c(GPS_FIFO, GPS_FIFOSIZE);
}
// 
// uint8_t gps_getfix(location_t *fix, UBXNAV_PVT *soln)
// {
//     UBXMsgBuffer    ubx_obuff;
//     UBXMsg          *msg;
//     uint8_t         result;
// 
//     msg    = NULL;
// 	
//     ubx_obuff = getNAV_PVT_POLL();
//     memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
//     clearUBXMsgBuffer(&ubx_obuff);
// 
//     result = gps_transfer();
// 	//delay_ms(500);
//     //gps_transfer();
// 	
//     /* error check result of gps_transfer */
//     //TODO Verify successful fix
//     
//     /* retrieve ubx message */
//     alignUBXmessage(&msg, GPS_MISO, GPS_BUFFSIZE);
// 	
// 	if (msg->hdr.msgId == UBXMsgIdNAV_PVT)
// 	{
//         *soln = msg->payload.NAV_PVT;
// 		fix->latitude = msg->payload.NAV_PVT.lat;
// 		fix->longitude = msg->payload.NAV_PVT.lon;
// 		
// 		result = 1;
// 	}
// 	else
// 	{
//         fix->latitude  = 999999999;
//         fix->longitude = 999999999;
// 		result = 0;
// 	}
// 
//     //fix->altitude = msg->payload.NAV_PVT.height;
//     //fix->climb
//     
//     
// 
//     return result;
// }

uint8_t gps_gettime(utc_time_t *time)
{
	return 0;
}

bool gps_setrate(const uint32_t period)
{
	return true;
}

bool gps_sleep()
{
	return true;
}

bool gps_wake()
{
	return true;
}

bool gps_setprofile(const GPS_PROFILE profile)
{
	return true;
}

// int32_t gps_transfer()
// {
//     int32_t retval = 0;
// 
// //     gpio_set_pin_level(SPI_SS, false);
// // /*    retval = spi_m_sync_transfer(gps_desc, &gps_buff);*/
// //     gpio_set_pin_level(SPI_SS, true);
// 
//     return retval;
// }
// 
// //TODO encapsulate these helper functions
// uint8_t cfgPSMOO(uint8_t period)
// {
//     UBXMsgBuffer    ubx_obuff;
//     UBXMsgs         ubxmsgs;
//     UBXMsg          *ubxmsg;
//     uint8_t         ack;
// 
//     ubxmsg    = NULL;
// 
//     /* Message version (protocol 15) */
//     ubxmsgs.CFG_PM2.version = 1;
// 
//     /* Set Flags */
//     /* Disable External Interrupt */
//     ubxmsgs.CFG_PM2.flags.extIntSelect = 0;
//     ubxmsgs.CFG_PM2.flags.extIntWake = 0;
//     ubxmsgs.CFG_PM2.flags.extIntBackup = 0;
// 
//     /* Misc. Flags */
//     ubxmsgs.CFG_PM2.flags.limitPeakCurr = UBXPM2LimitCurrentEnabled;
//     ubxmsgs.CFG_PM2.flags.waitTimeFix = 0;
//     ubxmsgs.CFG_PM2.flags.updateRTC = 0;
//     ubxmsgs.CFG_PM2.flags.updateEPH = 0;
//     ubxmsgs.CFG_PM2.flags.doNotEnterOff = 0;
// 
//     /* Power Saving Mode */
//     ubxmsgs.CFG_PM2.flags.mode = UBXPM2OnOffOperation;
//     /* End Flags */
// 
//     /* Position update period (ms) */
//     ubxmsgs.CFG_PM2.updatePeriod = 0;
// 	
//     /* Position search period (ms) */
//     ubxmsgs.CFG_PM2.searchPeriod = 0;
// 
//     //TODO : Will this help?
//     ubxmsgs.CFG_PM2.gridOffset = 0;
// 
//     /* Time to stay in tracking mode (s) */
//     ubxmsgs.CFG_PM2.onTime = 0;
// 
//     /* Minimal search time (s) */
//     ubxmsgs.CFG_PM2.minAcqTime = 0;
// 
//     /* Build MOSI configuration message, and MISO buffer */
//     ubx_obuff = getCFG_PM2( ubxmsgs.CFG_PM2.flags,
//     ubxmsgs.CFG_PM2.updatePeriod,
//     ubxmsgs.CFG_PM2.searchPeriod,
//     ubxmsgs.CFG_PM2.gridOffset,
//     ubxmsgs.CFG_PM2.onTime,
//     ubxmsgs.CFG_PM2.minAcqTime);
// 
// 
//     /* Construct/Associate SPI tx/rx buffers */
//     memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
//     clearUBXMsgBuffer(&ubx_obuff);
// 
// 
//     /* Send CFG_PRT message, Receive ACK_??? message */
//     gps_transfer();
// 
//     gps_clearbuffers();
// 
//     /* May need a second xfer to receive ACK */
//     gps_transfer();
// 
//     /* Check rxbuffer for ACK/NAK */
//     alignUBXmessage(&ubxmsg, GPS_MISO, GPS_BUFFSIZE);
// 
//     if(ubxmsg != NULL &&
//        ubxmsg->hdr.msgClass == UBXMsgClassACK &&
//        ubxmsg->hdr.msgId == UBXMsgClassACK )
//     {
//         ack = true;
//     }
//     else
//     {
//         ack = false;
//     }
// 
//     return ack;
// }
// 
// uint8_t gps_selftest()
// {
//     UBXMsgBuffer    ubx_obuff;
//     UBXMsg          *msg;
//     GPS_ERROR       ack;
//     
// 	msg = NULL;
//     //ubx_obuff = getCFG_RST(0,0);
//     //ubx_obuff = getCFG_PRT_POLL_OPT(UBXPRTSPI);
//     //ubx_obuff = getCFG_RXM_POLL();
//     //ubx_obuff = getCFG_PM2_POLL();
//     //ubx_obuff = getCFG_GNSS_POLL();
//     //ubx_obuff = getCFG_RATE_POLL();
//     //ubx_obuff = getRXM_PMREQ(10000,2);
// 	ubx_obuff = getCFG_MSG_POLL(UBXMsgClassNAV, UBXMsgIdNAV_PVT);
//     //ubx_obuff = getNAV_SAT_POLL();
// 
//     memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
//     clearUBXMsgBuffer(&ubx_obuff);
// 
//     gps_transfer();
//     alignUBXmessage(&msg, GPS_MISO, GPS_BUFFSIZE);
//     
// 	if (msg->hdr.msgId == UBXMsgIdACK_ACK)
// 	{
// 		ack = GPS_SUCCESS;
// 	} 
// 	else
// 	{
// 		ack = GPS_FAILURE;
// 	}
// 	
//     gps_clearbuffers();
// 
//     ubx_obuff = getCFG_PRT_POLL_OPT(UBXPRTSPI);
//     memcpy(GPS_MOSI, (uint8_t*)ubx_obuff.data, ubx_obuff.size);
//     clearUBXMsgBuffer(&ubx_obuff);
//     gps_transfer();
//     alignUBXmessage(&msg, GPS_MISO, GPS_BUFFSIZE);
//     gps_clearbuffers();
//     
//     return (uint8_t)ack;
// }

uint8_t gps_write_i2c(const uint8_t *DATA, const uint16_t SIZE)
{
	uint16_t timeout = 0;
	
    /* add the address and message to the buffer */
	memcpy(&GPS_SDABUF[0], DATA, SIZE);
	
    /* set up the i2c packet */
	gps_i2c_msg.addr	= M8Q_SLAVE_ADDR;
	gps_i2c_msg.len		= SIZE;
	gps_i2c_msg.flags	= I2C_M_STOP;
	gps_i2c_msg.buffer	= GPS_SDABUF;
	
    /* send, repeat until successful or timeout */
	while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
		if (timeout++ == I2C_TIMEOUT) {
			return 0;
		}
	}
	
	return 1;
}

uint8_t gps_read_i2c(uint8_t *data, const uint16_t SIZE)
{
	uint16_t timeout = 0;

    /* add the address and empty the buffer */
    memset(GPS_SDABUF, 0x00, SIZE);

    /* set up the i2c packet */
    gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
    gps_i2c_msg.len     = SIZE;
    gps_i2c_msg.flags   = I2C_M_STOP | I2C_M_RD;
    gps_i2c_msg.buffer  = GPS_SDABUF;

    /* send, repeat until successful or timeout */
    while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
        if (timeout++ == I2C_TIMEOUT) {
            return 0;
        }
    }

    return 1;
}