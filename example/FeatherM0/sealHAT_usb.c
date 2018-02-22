#include "atmel_start.h"
#include "sealHAT_usb.h"

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
	/* Device descriptors and Configuration descriptors list. */
CDCD_ACM_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
	/* Device descriptors and Configuration descriptors list. */
CDCD_ACM_HS_DESCES_HS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
static uint8_t single_desc_bytes[] = {
	/* Device descriptors and Configuration descriptors list. */
CDCD_ACM_DESCES_LS_FS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif

static struct usbd_descriptors single_desc[]
= {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
,
{single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

/* IN and OUT buffers for sending and receiving data respectivley */
static uint32_t usbd_cdc_rx_buffer[CDCD_ECHO_BUF_SIZ / 4];
static uint32_t usbd_cdc_tx_buffer[CDCD_ECHO_BUF_SIZ / 4];

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

/**
 * \brief Callback invoked when bulk OUT data received
 */
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	cdcdf_acm_write((uint8_t *)usbd_cdc_rx_buffer, count);
//	static const char str[] = "hello world!\n";
//	cdcdf_acm_write((uint8_t *)str, 13);

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when bulk IN data received
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	/* Echo data. */
	cdcdf_acm_read((uint8_t *)usbd_cdc_rx_buffer, sizeof(usbd_cdc_rx_buffer));

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (state.rs232.DTR) {
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_out);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
		/* Start Rx */
		cdcdf_acm_read((uint8_t *)usbd_cdc_rx_buffer, sizeof(usbd_cdc_rx_buffer));
	}

	/* No error. */
	return false;
}

/**
 * \brief CDC ACM Init
 */
void usb_initialize(void)
{
	/* usb stack init with the control endpoint buffer */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	/* start USB and set the descriptors */
	usbdc_start(single_desc);
    /* Attach device to the bus */
	usbdc_attach();
}

void usb_start(void) {
	/* wait cdc acm to be installed */
	while (!cdcdf_acm_is_enabled()) { };

	/* Set a callback to be used when the state changes to "connected" */
	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
}