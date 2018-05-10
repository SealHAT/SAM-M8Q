#include "seal_UTIL.h"

void i2c_unblock_bus(const uint8_t SDA_PIN, const uint8_t SCL_PIN)
{
    uint32_t i, count;

    // Set pin SDA direction to input, pull off
    gpio_set_pin_direction(SDA_PIN, GPIO_DIRECTION_IN);
    gpio_set_pin_pull_mode(SDA_PIN, GPIO_PULL_OFF);
    gpio_set_pin_function(SDA_PIN, GPIO_PIN_FUNCTION_OFF);

    for(i = 0, count = 0; i < 50; i++) {
        count += gpio_get_pin_level(SDA_PIN);
    }

    if(count < 10) {
        // Set pin SCL direction to output, pull off
        gpio_set_pin_direction(SCL_PIN, GPIO_DIRECTION_OUT);
        gpio_set_pin_pull_mode(SCL_PIN, GPIO_PULL_OFF);
        gpio_set_pin_function(SCL_PIN, GPIO_PIN_FUNCTION_OFF);

        for(i = 0; i <= 32; i++) {
            gpio_toggle_pin_level(SCL_PIN);
        }
    }

}

//void set_lowPower_mode(void)
//{
    //// disable Brown Out Detector
    //SUPC->BOD33.reg &= ~SUPC_BOD33_ENABLE;
//
    ///* Select BUCK converter as the main voltage regulator in active mode */
    //SUPC->VREG.bit.SEL = SUPC_VREG_SEL_BUCK_Val;
    ///* Wait for the regulator switch to be completed */
    //while(!(SUPC->STATUS.reg & SUPC_STATUS_VREGRDY));
//
    ///* Set Voltage Regulator Low power Mode Efficiency */
    //SUPC->VREG.bit.LPEFF = 0x1;
//
    ///* Apply SAM L21 Erratum 15264 - CPU will freeze on exit from standby if BUCK is disabled */
    //SUPC->VREG.bit.RUNSTDBY = 0x1;
    //SUPC->VREG.bit.STDBYPL0 = 0x1;
//
    ///* Set Performance Level to PL0 as we run @12MHz */
    //_set_performance_level(PM_PLCFG_PLSEL_PL0_Val);
//}