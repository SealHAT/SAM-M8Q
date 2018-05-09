#include "atmel_start.h"
#include <string.h>
#include "gps.h"

static struct timer_task sample;
static location_t g_gps_fix;
static UBXNAV_PVT g_gps_soln;
static location_t dummy_fix;
static uint8_t g_gps_test;
static char usb_out[49];

void gps_usb_writefix(location_t *fix, UBXNAV_PVT *soln);

void timer_cb(const struct timer_task *const timer_task)
{
    g_gps_test = gps_getfix(&g_gps_fix, &g_gps_soln);
    
    if( usb_dtr() )
    {
        if( g_gps_test == 1 ){
            gpio_set_pin_level(LED_DEBUG,false);
            gps_usb_writefix(&g_gps_fix, &g_gps_soln);
        } else {
            gps_usb_writefix(&dummy_fix, &g_gps_soln);
        }
        
    }
  //  timer_add_task(&TIMER_0, &sample);
}



int main(void)
{
	

	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	//gps_init(&SPI_0);
    gps_init_i2c(&I2C_GPS);
    gpio_set_pin_direction(LED_DEBUG, GPIO_DIRECTION_OUT);
    gpio_set_pin_level(LED_DEBUG, true);
    gpio_set_pin_function(LED_DEBUG,GPIO_PIN_FUNCTION_OFF);

    /* timer stuff */
    sample.cb = timer_cb;
    sample.interval = 1000;
    sample.mode = TIMER_TASK_REPEAT;
    
	
	/* Do a Poll message to get info/check if GPS is ready */
	g_gps_test = gps_selftest();
    dummy_fix.latitude  = 999999999;
    dummy_fix.longitude = 999999999;


    timer_add_task(&TIMER_0, &sample);
    timer_start(&TIMER_0);
	while (1)
	{
		delay_ms(100);

        gpio_set_pin_level(LED_DEBUG, true);
	}
}

void gps_usb_writefix(location_t *fix, UBXNAV_PVT *soln)
{
 //   char    usb_out[26];
    uint8_t usb_result;

    /* print latitude and longitude as signed floats */
    snprintf(usb_out, 49 ,"%d, %ld, %ld, %ld.%07d, %ld.%07d\n",
        soln->fixType,
        soln->vAcc,
        soln->hAcc,
        fix->latitude / 10000000,
        abs(fix->latitude % 10000000),
        fix->longitude / 10000000,
        abs(fix->longitude % 10000000));
        
    /* write to usb */
    do 
    {
        usb_result = usb_write((uint8_t*)usb_out, strlen(usb_out));
    } while (usb_result != 0);

}

