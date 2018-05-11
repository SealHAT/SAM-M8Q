/*
 * SerialPrint.h
 *
 * Created: 3/7/2018 2:39:48 PM
 *  Author: eslatter
 */ 


#ifndef SERIALPRINT_H_
#define SERIALPRINT_H_

#include <atmel_start.h>	/* where the IO functions live */
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * 
 */
size_t printFloat(double number, uint8_t digits);

size_t print(void* string);

size_t println(void* string);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* SERIALPRINT_H_ */