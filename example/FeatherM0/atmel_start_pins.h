/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define A0 GPIO(GPIO_PORTA, 2)
#define VBATT GPIO(GPIO_PORTA, 7)
#define MISO GPIO(GPIO_PORTA, 12)
#define TX_RDY GPIO(GPIO_PORTA, 15)
#define LED_BUILTIN GPIO(GPIO_PORTA, 17)
#define DBG GPIO(GPIO_PORTA, 20)
#define SDA GPIO(GPIO_PORTA, 22)
#define SCL GPIO(GPIO_PORTA, 23)
#define USB_N GPIO(GPIO_PORTA, 24)
#define USB_P GPIO(GPIO_PORTA, 25)
#define MOSI GPIO(GPIO_PORTB, 10)
#define SCK GPIO(GPIO_PORTB, 11)

#endif // ATMEL_START_PINS_H_INCLUDED
