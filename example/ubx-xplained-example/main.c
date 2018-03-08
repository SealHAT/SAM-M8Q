#include <atmel_start.h>
#include <string.h>
#include "gps.h"

int main(void)
{
    location_t fix;
    /* Initializes MCU, drivers and middleware */
    atmel_start_init();
    gps_init(&SPI_0);
	
	/* Do a Poll message to get info/check if GPS is ready */
    //ubx_obuff = getCFG_RST(0,0);
    //ubx_obuff = getCFG_PRT_POLL_OPT(UBXPRTSPI);    
	//ubx_obuff = getCFG_RXM_POLL();
    //ubx_obuff = getCFG_PM2_POLL();
    //ubx_obuff = getCFG_GNSS_POLL();
    //ubx_obuff = getCFG_MSG_RATE();
    //ubx_obuff = getRXM_PMREQ(10000,2);

    while (1) 
    {
        delay_ms(1000);
        gps_getfix(&fix);	
	}
}


