/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#include "usb_start.h"

#define USB_BUFFER_SIZE CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ		/* Define buffer size as endpoint size */
static uint8_t single_desc_bytes[] = { CDCD_ACM_DESCES_LS_FS };		/* Device descriptors and Configuration descriptors list. */
static struct usbd_descriptors single_desc[] = { {single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes) } };	/* Make a struct of the needed descriptors */

typedef struct {
	uint8_t buff[USB_BUFFER_SIZE];		// Buffer for receiving data from host
	volatile uint32_t head;				// Head index of the buffer
	volatile uint32_t tail;				// tail index of the buffer
	volatile bool outInProgress;        // indicates if there is data waiting to be read
	volatile enum usb_xfer_code lastCode;	// transfer code of the last completed transaction
} outData_t;

typedef struct {
	volatile uint32_t waiting;				// number of bytes waiting to be sent
	volatile enum usb_xfer_code lastCode;	// transfer code of the last completed transaction
} inData_t;

typedef struct {
	uint8_t buff[USB_BUFFER_SIZE];		// Buffer for USB control transactions
	volatile USB_State_t devState;		// tracks the USB device state
	volatile bool dtr;					// Flag to indicate status of DTR - Data Terminal Ready
	volatile bool rts;					// Flag to indicate status of RTS - Request to Send
} ctrlData_t;

static ctrlData_t ctrlBuf;				// CTRL endpoint buffer
static outData_t  outbuf;				// OUT endpoint buffer
static inData_t   inbuf;				// IN endpoint buffer

/**
 * \brief Callback for USB to simply set a flag that data has been received.
 */
static bool usb_out_complete(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	volatile hal_atomic_t __atomic;
	atomic_enter_critical(&__atomic);
	
	// only modify state if the transfer was on the BULK OUT endpoint	
	if(CONF_USB_CDCD_ACM_DATA_BULKOUT_EPADDR == ep) {	
        outbuf.head			 = 0;
        outbuf.tail			 = count;
        outbuf.lastCode		 = rc;
        outbuf.outInProgress = false;
	}
	
	atomic_leave_critical(&__atomic);
	
	return false;		// The example code returns false on success... ?
}

/**
 * \brief Callback for USB to simply set a flag that data has been sent successfully.
 */
static bool usb_in_complete(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	
	volatile hal_atomic_t __atomic;
	atomic_enter_critical(&__atomic);

	// only modify state if the transfer was on the BULK IN endpoint
	if(CONF_USB_CDCD_ACM_DATA_BULKIN_EPADDR == ep) {
		inbuf.waiting -= count;
		inbuf.lastCode = rc;
	}

	atomic_leave_critical(&__atomic);

	return false;		// The example code returns false on success... ?
}

/**
 * \brief Callback invoked when Line State Change
 *
 * This function is called when there is a change to the RTS and DTS control
 * lines on the serial connection. 
 *
 */
static bool usb_line_state_changed(usb_cdc_control_signal_t newState)
{
	static bool callbacks_registered = false;	// prevents callbacks from being registered twice
	
	ctrlBuf.dtr = newState.rs232.DTR;
	ctrlBuf.rts = newState.rs232.RTS;
			
	if (cdcdf_acm_is_enabled() && !callbacks_registered) {
		callbacks_registered = true;
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_out_complete);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_in_complete);
	}

	/* No error. */
	return false;
}

int32_t usb_start(void)
{
	int32_t err = ERR_NONE;
	
	// Initialize the static values to their defaults
	inbuf.waiting        = 0;
	outbuf.head          = 0;
	outbuf.tail		     = 0;
	outbuf.outInProgress = false;
	ctrlBuf.dtr          = false;
	ctrlBuf.rts          = false;
	ctrlBuf.devState     = USB_Detached;
	
	/* usb stack init */
	err = usbdc_init(ctrlBuf.buff);

	/* usbdc_register_funcion inside */
	err = cdcdf_acm_init();

	// start and attach the USB device
	err = usbdc_start(single_desc);
	usbdc_attach();

	err = cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_line_state_changed);
	
	return err;
}

int32_t usb_stop(void)
{	
	cdcdf_acm_stop_xfer();
	usbdc_detach();
	usbdc_stop();
	
	cdcdf_acm_deinit();
	usbdc_deinit();
	
	return ERR_NONE;	
}

// TODO - incorporate VBUS detection
USB_State_t usb_state(void)
{	
	ctrlBuf.devState = (USB_State_t)usbdc_get_state();
	return ctrlBuf.devState;
}

bool usb_dtr(void)
{
	return ctrlBuf.dtr;
}

bool usb_rts(void)
{
	return ctrlBuf.rts;
}

void usb_haltTraffic(void) {
    cdcdf_acm_stop_xfer();
    inbuf.waiting        = 0;
    outbuf.head          = 0;
    outbuf.tail		     = 0;
    outbuf.outInProgress = false;
}

/************************ TRANSMITTING DATA *************************************/
int32_t usb_write(void* outData, uint32_t BUFFER_SIZE) 
{
	int32_t err = ERR_BUSY;
	
	// This check IS needed. cdcdf_acm_write() will drop data if bus is busy and does
	// not appear to return an error message.
	if(0 == inbuf.waiting) {
		volatile hal_atomic_t __atomic;
		atomic_enter_critical(&__atomic);
		inbuf.waiting  = BUFFER_SIZE;
		atomic_leave_critical(&__atomic);
		
		err = cdcdf_acm_write((uint8_t*)outData, BUFFER_SIZE);
	}
	
	return  err;
}


/************************ RECEIVING DATA ****************************************/
int32_t usb_available()
{
    int32_t retval = ERR_NONE;       // return value, defaulted to 0

    // if the buffer is empty and a read is not in progress then request data
    if( (outbuf.tail - outbuf.head) == 0 && !outbuf.outInProgress ) {
        // request an OUT transfer. the head and tail will be adjusted in the callback function
        // this function will return negative if it fails for some reason
        retval = cdcdf_acm_read(outbuf.buff, USB_BUFFER_SIZE);
    }
    
    // If read happened without error (or didn't happen at all) return the buffer size
    if(retval >= 0) {
        retval = (outbuf.tail - outbuf.head);
    }
    return retval;
}

int32_t usb_get(void)
{
	int32_t retval = ERR_NONE;       // return value, defaulted to 0
	    
	retval = usb_available();
	if( retval > 0 ) {
        volatile hal_atomic_t __atomic;
        atomic_enter_critical(&__atomic);
		retval = outbuf.buff[outbuf.head++];
	    atomic_leave_critical(&__atomic);
    }
	else {
		// must return error if there are no bytes to read, 0 is a valid byte value.
		retval = ERR_FAILURE;
	}
	return retval;
}

int32_t usb_read(void* receiveBuffer, uint32_t BUFFER_SIZE)
{
	int32_t letter;         // value received from USB OUT buffer
    uint32_t i = 0;         // LCV and # of bytes received
    uint8_t* buff = (uint8_t*)receiveBuffer;    // receive buffer cast as bytes
	
	// Fill the provided buffer until there is is no more data or it is full
	while(i < BUFFER_SIZE) {
        letter = usb_get();
        if(letter >= 0) {
            buff[i++] = letter;
        }
		else {
			// Abort the loop if the usb OUT buffer is empty
			break;
		}
    }
		
	/* If there was an error, return error val. Else, return the number of bytes received. */
	return i;
}

int32_t usb_flushRx(void)
{
	outbuf.head = 0;
	outbuf.tail = 0;
	return ERR_NONE;
}