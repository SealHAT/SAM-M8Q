#include "gps.h"

uint8_t	GPS_FIFO[GPS_FIFOSIZE];

static struct   i2c_m_sync_desc	gps_i2c_desc;
static struct   _i2c_m_msg		gps_i2c_msg;
static          UBXMsg          ubx_msg;
static          UBXMsgBuffer    ubx_buf;
int32_t	                        gps_err;

uint8_t gps_init_i2c(struct i2c_m_sync_desc* const I2C_DESC) 
{
	gps_i2c_desc = *I2C_DESC;
	
    /* enable and set up the i2c device     */
	gps_err = i2c_m_sync_enable(&gps_i2c_desc);
	gps_err = i2c_m_sync_set_slaveaddr(&gps_i2c_desc, M8Q_SLAVE_ADDR, I2C_M_SEVEN);	
	
    /* reset the device to default settings */
	ubx_buf = getCFG_RST(0,0);
	gps_err = gps_write_i2c((const uint8_t*)ubx_buf.data, ubx_buf.size);
	delay_ms(500);
	
	if (GPS_FAILURE == gps_disable_nmea()) {
		return GPS_FAILURE;
	}

	/* disable other ports (Default: UART,USB,DDC enabled; SPI disabled) */
	ubx_msg.payload.CFG_PRT.portID			        = UBXPRTUART;
    ubx_msg.payload.CFG_PRT.reserved0               = 0;
	ubx_msg.payload.CFG_PRT.txReady.en				= 0;
	ubx_msg.payload.CFG_PRT.txReady.pin				= 8;
	ubx_msg.payload.CFG_PRT.txReady.thres			= 0;
	ubx_msg.payload.CFG_PRT.txReady.pol				= 0;
    ubx_msg.payload.CFG_PRT.mode.UBX_UART.charLen   = UBXPRTMode8BitCharLen;
    ubx_msg.payload.CFG_PRT.mode.UBX_UART.nStopBits = UBXPRTMode1StopBit;
    ubx_msg.payload.CFG_PRT.mode.UBX_UART.parity    = UBXPRTModeOddParity;
	ubx_msg.payload.CFG_PRT.mode.UBX_UART.reserved0 = 0;
    ubx_msg.payload.CFG_PRT.inProtoMask		        = 0U;
	ubx_msg.payload.CFG_PRT.outProtoMask	        = 0U;
    ubx_msg.payload.CFG_PRT.option.UARTbaudRate     = 0x2580;
	if (GPS_FAILURE == gps_cfgprt(ubx_msg)) {
		return GPS_FAILURE;
	}
	
	ubx_msg.payload.CFG_PRT.portID			= UBXPRTUSB;
	ubx_msg.payload.CFG_PRT.inProtoMask		= 0U;
	ubx_msg.payload.CFG_PRT.outProtoMask	= 0U;
	if (GPS_FAILURE == gps_cfgprt(ubx_msg)) {
		return GPS_FAILURE;
	}
	
    /* configure the device to communicate over DDC (i2c) with a FIFO interrupt */
	ubx_msg.payload.CFG_PRT.portID					= UBXPRTDDC;
	ubx_msg.payload.CFG_PRT.txReady.en				= 1U;					/* 0 - disabled, 1 - enabled			*/
	ubx_msg.payload.CFG_PRT.txReady.pol			    = M8Q_TXR_POL;			/* 0 - High-active, 1 - Low-active		*/
	ubx_msg.payload.CFG_PRT.txReady.pin			    = M8Q_TXR_PIO;			/* PIO06 is TxD on the SAM-M8Q			*/
	ubx_msg.payload.CFG_PRT.txReady.thres			= M8Q_TXR_CNT >> 3;	/* Given value is multiplied by 8 bytes */
	ubx_msg.payload.CFG_PRT.mode.UBX_DDC.slaveAddr  = M8Q_SLAVE_ADDR;
	ubx_msg.payload.CFG_PRT.inProtoMask			    = UBXPRTInProtoInUBX;
	ubx_msg.payload.CFG_PRT.outProtoMask			= UBXPRTOutProtoOutUBX|UBXPRTOutProtoOutNMEA;
	ubx_msg.payload.CFG_PRT.flags					= UBXPRTExtendedTxTimeout;
	if (GPS_FAILURE == gps_cfgprt(ubx_msg)) {
		return GPS_FAILURE;
	}
	
	if (GPS_FAILURE == gps_init_msgs()) {
		return GPS_FAILURE;
	}
	
	
	return GPS_SUCCESS;
}

uint8_t gps_disable_nmea() 
{
	int i;
	UBXMsg          *ack_msg;
	uint8_t         result;
	uint16_t        timeout;
	uint8_t rates[6] = {0,0,0,0,0,0};
	
	/* disable default (NMEA) messages */
	for(i = 0; i < 6; i++) {
		
		timeout = 0;
		result = GPS_FAILURE;
		
		do {
			/* send message disable particular message on all IO ports */
			ubx_buf = getCFG_MSG_RATES(UBXMsgClassNMEA, (UBXMessageId)i, rates);
			if (GPS_SUCCESS == gps_write_i2c((const uint8_t*)ubx_buf.data, ubx_buf.size)) {
				
				/* get acknowledge message and verify */
				ubx_buf = getACK_ACK();
				if (GPS_SUCCESS == gps_read_i2c_poll((uint8_t*)ubx_buf.data, ubx_buf.size)) {
					
					/* check that the receiver configured successfully */
					ack_msg = (UBXMsg*)ubx_buf.data;
					if (ack_msg->hdr.msgId == UBXMsgIdACK_ACK) {
						result = GPS_SUCCESS;
					}
				}
			}
			/* repeat until timeout or successful acknowledge from device */
		} while (result == GPS_FAILURE && timeout++ < CFG_TIMEOUT);
	}
	
	return result;
}

uint8_t gps_init_msgs()
{
	UBXMsg          *ack_msg;
	uint8_t         result  = GPS_FAILURE;
	uint16_t        timeout = 0;
	
	/* set default 1Hz rate */
	do {
		/* send message disable particular message on all IO ports */
		ubx_buf = getCFG_RATE(1000,1,0);
		if (GPS_SUCCESS == gps_write_i2c((const uint8_t*)ubx_buf.data, ubx_buf.size)) {
				
			/* get acknowledge message and verify */
			ubx_buf = getACK_ACK();
			if (GPS_SUCCESS == gps_read_i2c_poll((uint8_t*)ubx_buf.data, ubx_buf.size)) {
					
				/* check that the receiver configured successfully */
				ack_msg = (UBXMsg*)ubx_buf.data;
				if (ack_msg->hdr.msgId == UBXMsgIdACK_ACK) {
					result = GPS_SUCCESS;
				}
			}
		}
		/* repeat until timeout or successful acknowledge from device */
	} while (result == GPS_FAILURE && timeout++ < CFG_TIMEOUT);
	
	if (GPS_FAILURE == result) {
		return GPS_FAILURE;
	}
	result = GPS_FAILURE;
	timeout = 0;
	
	/* set up NAV messages */
	do {
		/* send message disable particular message on all IO ports */
		ubx_buf = getCFG_MSG_RATE(UBXMsgClassNAV,UBXMsgIdNAV_PVT,1);
		if (GPS_SUCCESS == gps_write_i2c((const uint8_t*)ubx_buf.data, ubx_buf.size)) {
			
			/* get acknowledge message and verify */
			ubx_buf = getACK_ACK();
			if (GPS_SUCCESS == gps_read_i2c_poll((uint8_t*)ubx_buf.data, ubx_buf.size)) {
				
				/* check that the receiver configured successfully */
				ack_msg = (UBXMsg*)ubx_buf.data;
				if (ack_msg->hdr.msgId == UBXMsgIdACK_ACK) {
					result = GPS_SUCCESS;
				}
			}
		}
		/* repeat until timeout or successful acknowledge from device */
	} while (result == GPS_FAILURE && timeout++ < CFG_TIMEOUT);
	
	return result;
}

uint8_t gps_cfgprt(const UBXMsg MSG)
{
	UBXMsg          *ack_msg;
	uint8_t         result  = GPS_FAILURE;
	uint16_t        timeout = 0;
	
	do {
		/* send message to configure the DDC serial port and verify */
		ubx_buf = setCFG_PRT(MSG.payload.CFG_PRT);
		if (GPS_SUCCESS == gps_write_i2c((const uint8_t*)ubx_buf.data, ubx_buf.size)) {
			    
			/* get acknowledge message and verify */
			ubx_buf = getACK_ACK();
			if (GPS_SUCCESS == gps_read_i2c_poll((uint8_t*)ubx_buf.data, ubx_buf.size)) {
				    
				/* check that the receiver configured successfully */
				ack_msg = (UBXMsg*)ubx_buf.data;
				if (ack_msg->hdr.msgId == UBXMsgIdACK_ACK &&
					ack_msg->payload.ACK_ACK.msgId == MSG.hdr.msgId) {
					result = GPS_SUCCESS;
				}
			}
		}
		/* repeat until timeout or successful acknowledge from device */
	} while (result == GPS_FAILURE && timeout++ < CFG_TIMEOUT);
	
	return result;
}

uint8_t gps_readfifo()
{
	uint16_t timeout = 0;

    /* set up the i2c packet */
    gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
    gps_i2c_msg.len     = GPS_FIFOSIZE;
    gps_i2c_msg.flags   = I2C_M_STOP | I2C_M_RD;
    gps_i2c_msg.buffer  = GPS_FIFO;

    /* send, repeat until successful or timeout */
    while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
        if (timeout++ == I2C_TIMEOUT*4) {
            return GPS_FAILURE;
        }
    }
	
    return GPS_SUCCESS;
}

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

uint8_t gps_write_i2c(const uint8_t *DATA, const uint16_t SIZE)
{
	uint16_t timeout = 0;
	
    /* set up the i2c packet */
	gps_i2c_msg.addr	= M8Q_SLAVE_ADDR;
	gps_i2c_msg.len		= SIZE;
	gps_i2c_msg.flags	= I2C_M_STOP;
	gps_i2c_msg.buffer	= (uint8_t*)DATA;
	
    /* send, repeat until successful or timeout */
	while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
		if (timeout++ == I2C_TIMEOUT) {
			return GPS_FAILURE;
		}
	}
	
	return GPS_SUCCESS;
}

uint8_t gps_read_i2c(uint8_t *data, const uint16_t SIZE)
{
	uint16_t timeout = 0;

    /* set up the i2c packet */
    gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
    gps_i2c_msg.len     = SIZE;
    gps_i2c_msg.flags   = I2C_M_STOP | I2C_M_RD;
    gps_i2c_msg.buffer  = data;

    /* send, repeat until successful or timeout */
    while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
        if (timeout++ == I2C_TIMEOUT) {
            return GPS_FAILURE;
        }
    }
    return GPS_SUCCESS;
}

uint8_t gps_read_i2c_poll(uint8_t *data, const uint16_t SIZE)
{
	uint16_t timeout = 0;
	
	/* set up the first i2c packet */
	gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
	gps_i2c_msg.len     = 1;
	gps_i2c_msg.flags   = I2C_M_RD;
	gps_i2c_msg.buffer  = data;
	
	do { /* attempt reads until the data returned is not 0xFF (no data) */
		while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
			if (timeout++ == I2C_TIMEOUT) {
				return GPS_FAILURE;
			}
		}
	} while (data[0] == 0xFF);

	if (data[0] != 0xB5) {
		_i2c_m_sync_send_stop(&gps_i2c_desc.device);
		return GPS_FAILURE;
	}
	/* set up the remaining i2c packet */
	gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
	gps_i2c_msg.len     = SIZE - 1;
	gps_i2c_msg.flags   |= I2C_M_STOP;
	gps_i2c_msg.buffer  = &data[1];

	/* send, repeat until successful or timeout */
	while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
		if (timeout++ == I2C_TIMEOUT) {
			return GPS_FAILURE;
		}
	}

	return GPS_SUCCESS;
}

uint8_t gps_read_i2c_search(uint8_t *data, const uint16_t SIZE)
{
	uint8_t cls,id;
	uint16_t timeout = 0;
	uint16_t buf_idx = 0;
	bool found = false;
	
	cls = data[2]; id = data[3];
	while(!found && buf_idx < SIZE) {
		/* set up the first i2c packet */
		gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
		gps_i2c_msg.len     = 1;
		gps_i2c_msg.flags   = I2C_M_RD;
		gps_i2c_msg.buffer  = data;
	
		do { /* attempt reads until the data returned is not 0xFF (no data) */
			while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
				if (timeout++ == I2C_TIMEOUT) {
					return GPS_FAILURE;
				}
			}
		} while (data[0] != 0xB5);
			
		/* set up the remaining i2c packet */
		gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
		gps_i2c_msg.len     = 3;
		gps_i2c_msg.flags   = I2C_M_RD;
		gps_i2c_msg.buffer  = &data[1];

		timeout = 0;
		
		/* send, repeat until successful or timeout */
		while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
			if (timeout++ == I2C_TIMEOUT) {
				return GPS_FAILURE;
			}
		}
		
		if(cls == data[2] && id == data[3]) {
			found = true;
		} else {
			buf_idx++;
		}
	}
	
	if (found) {
        timeout = 0;

		/* set up the remaining i2c packet */
		gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
		gps_i2c_msg.len     = SIZE - 4;
		gps_i2c_msg.flags   = I2C_M_STOP | I2C_M_RD;
		gps_i2c_msg.buffer  = &data[4];
		
		/* send, repeat until successful or timeout */
		while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
			if (timeout++ == I2C_TIMEOUT) {
				return GPS_FAILURE;
			}
		}
		
		return GPS_SUCCESS;
	}

	_i2c_m_sync_send_stop(&gps_i2c_desc.device);
	return GPS_FAILURE;
}

uint8_t gps_parsefifo(const uint8_t *FIFO, gps_log_t *log, const uint16_t LOG_SIZE)
{
	uint8_t i;			/* log entry index				*/
	uint16_t offset;	/* position to read from FIFO	*/
	UBXMsg *ubx_msg_p;	/* pointer to message in FIFO	*/
	
	i = 0;
	offset = 0;
	ubx_msg_p = NULL;
	
	/* parse messages until  the LOG array is full, or the FIFO is empty */
	while (i < LOG_SIZE && offset < GPS_FIFOSIZE) {
		/* find the first UBX message and update the position */
		offset += alignUBXmessage(&ubx_msg_p, &FIFO[offset], GPS_FIFOSIZE - offset);
	
		/* extract the data from the message according to the message type */
		if (ubx_msg_p != NULL) {
			switch (ubx_msg_p->hdr.msgId) {
				case UBXMsgIdNAV_PVT :
					log[i].position.fixOk	= (bool)ubx_msg_p->payload.NAV_PVT.flags.gnssFixOk;
					log[i].position.lat		= ubx_msg_p->payload.NAV_PVT.lat;
					log[i].position.lon		= ubx_msg_p->payload.NAV_PVT.lon;
                    #if (GPS_VERBOSE_LOG == 1)
                    log[i].position.fixType = ubx_msg_p->payload.NAV_PVT.fixType;
                    log[i].position.hAcc    = ubx_msg_p->payload.NAV_PVT.hAcc;
                    log[i].position.vAcc    = ubx_msg_p->payload.NAV_PVT.vAcc;
                    #endif
					log[i].time.year		= ubx_msg_p->payload.NAV_PVT.year;
					log[i].time.month		= ubx_msg_p->payload.NAV_PVT.month;
					log[i].time.day			= ubx_msg_p->payload.NAV_PVT.day;
					log[i].time.hour		= ubx_msg_p->payload.NAV_PVT.hour;
					log[i].time.minute		= ubx_msg_p->payload.NAV_PVT.min;
					log[i].time.second		= ubx_msg_p->payload.NAV_PVT.sec;
					log[i].time.nano		= ubx_msg_p->payload.NAV_PVT.nano;
					i++;
					break;
				case UBXMsgIdNAV_TIMEUTC :
					log[i].position.fixOk	= false;
					log[i].position.lat		= (UBXI4_t)GPS_INVALID_LAT;
					log[i].position.lon		= (UBXI4_t)GPS_INVALID_LON;
                    #if (GPS_VERBOSE_LOG == 1)
                    log[i].position.fixType = UBXGPSTimeOnlyFix;
                    log[i].position.hAcc    = 0;
                    log[i].position.vAcc    = 0;
                    #endif
					log[i].time.year		= ubx_msg_p->payload.NAV_TIMEUTC.year;
					log[i].time.month		= ubx_msg_p->payload.NAV_TIMEUTC.month;
					log[i].time.day			= ubx_msg_p->payload.NAV_TIMEUTC.day;
					log[i].time.hour		= ubx_msg_p->payload.NAV_TIMEUTC.hour;
					log[i].time.minute		= ubx_msg_p->payload.NAV_TIMEUTC.min;
					log[i].time.second		= ubx_msg_p->payload.NAV_TIMEUTC.sec;
					log[i].time.nano		= ubx_msg_p->payload.NAV_TIMEUTC.nano;
					i++;
					break;
				/* add cases to support other messages */
				default:
					/* discard */
					break;
			}
		}
		/* increment the index and update the offset */
		offset += ubx_msg_p->hdr.length;
		ubx_msg_p = NULL;
	}
	
	return i;
}

bool gps_selftest()
{
	GPS_ERROR   ack;
	UBXMsg		*msg;
	
	/* pol the device for hardware configuration */
	ubx_buf = getMON_HW_POLL();
	if (GPS_SUCCESS == gps_write_i2c((const uint8_t*)ubx_buf.data, ubx_buf.size)) {
		/* read back the hardware configuration */
		ubx_buf = getMON_HW();
		ack = gps_read_i2c_poll((uint8_t*)ubx_buf.data, ubx_buf.size);
		msg = (UBXMsg*)ubx_buf.data;
	} else {
		return GPS_FAILURE;
	}
	
	/* poll the device for port configuration */
	ubx_buf = getCFG_PRT_POLL_OPT(UBXPRTDDC);
	if (GPS_SUCCESS == gps_write_i2c((const uint8_t*)ubx_buf.data, ubx_buf.size)) {
		ubx_buf = getCFG_PRT();
		ack = gps_read_i2c_search((uint8_t*)ubx_buf.data, ubx_buf.size);
		msg = (UBXMsg*)ubx_buf.data;
	} else {
		return GPS_FAILURE;
	}
	
	/* verify the FIFO and watermark settings */
	if ((msg->payload.CFG_PRT.txReady.en    == 1U			) &&
		(msg->payload.CFG_PRT.txReady.pol   == M8Q_TXR_POL	) &&
		(msg->payload.CFG_PRT.txReady.pin   == M8Q_TXR_PIO	) &&
		(msg->payload.CFG_PRT.txReady.thres == (M8Q_TXR_CNT >> 3))) 
	{
		ack = GPS_SUCCESS;
	} else {
		ack = GPS_FAILURE;
	}
	
	return ack;
}

int32_t gps_checkfifo()
{
	uint8_t buf[2];
	uint16_t timeout = 0;
	int32_t	 retval = 0;
	
	buf[0] = M8Q_BYTES_HI_ADDR;

	/* set up the i2c packet */
	gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
	gps_i2c_msg.len     = 1;
	gps_i2c_msg.flags   = 0;
	gps_i2c_msg.buffer  = buf;

	/* send, repeat until successful or timeout */
	while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
		if (timeout++ == I2C_TIMEOUT) {
			return -GPS_FAILURE;
		}
	}
	
	/* set up the i2c packet */
	gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
	gps_i2c_msg.len     = 1;
	gps_i2c_msg.flags   = I2C_M_RD | I2C_M_STOP;
	gps_i2c_msg.buffer  = buf;
	
	/* send, repeat until successful or timeout */
	while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
		if (timeout++ == I2C_TIMEOUT) {
			return -GPS_FAILURE;
		}
	}
	
		buf[1] = M8Q_BYTES_LO_ADDR;

		/* set up the i2c packet */
		gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
		gps_i2c_msg.len     = 1;
		gps_i2c_msg.flags   = 0;
		gps_i2c_msg.buffer  = &buf[1];

		/* send, repeat until successful or timeout */
		while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
			if (timeout++ == I2C_TIMEOUT) {
				return -GPS_FAILURE;
			}
		}
		
		/* set up the i2c packet */
		gps_i2c_msg.addr    = M8Q_SLAVE_ADDR;
		gps_i2c_msg.len     = 1;
		gps_i2c_msg.flags   = I2C_M_RD | I2C_M_STOP;
		gps_i2c_msg.buffer  = &buf[1];
		
		/* send, repeat until successful or timeout */
		while (_i2c_m_sync_transfer(&gps_i2c_desc.device, &gps_i2c_msg)) {
			if (timeout++ == I2C_TIMEOUT) {
				return -GPS_FAILURE;
			}
		}
		
		retval = (buf[0] << 8) | (buf[1] << 0);
		
	return retval;
}