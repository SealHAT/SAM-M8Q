#include "SerialPrint.h"

size_t printFloat(double number, uint8_t digits)
{
	static const int OUTPUT_SIZE = 12;
	char output[OUTPUT_SIZE];
	uint8_t i;
	size_t n = 0;

	// Not a number, special floating point value
	if (isnan(number)) {
		n = snprintf(output, OUTPUT_SIZE, "nan");
	}
	// infinity, special floating point value
	else if (isinf(number)) {
		n = snprintf(output, OUTPUT_SIZE, "inf");
	}
	// constant determined empirically
	else if (number > 4294967040.0) {
		n = snprintf(output, OUTPUT_SIZE, "ovf");
	}
	// constant determined empirically
	else if (number <-4294967040.0) {
		n = snprintf(output, OUTPUT_SIZE, "ovf");
	}
	// A valid floating point value
	else {
		// Handle negative numbers
		if (number < 0.0) {
			output[n++] = '-';
			number = -number;
		}

		// Round correctly so that print(1.999, 2) prints as "2.00"
		double rounding = 0.5;
		for (i = 0; i < digits; i++) {
			rounding /= 10.0;
		}
		number += rounding;

		// Extract the integer part of the number and print it
		unsigned long int_part = (unsigned long)number;
		double remainder = number - (double)int_part;
		n += snprintf(&output[n], OUTPUT_SIZE-n, "%ld", int_part);

		// Print the decimal point, but only if there are digits beyond
		if (digits > 0) {
			output[n] = '.';
			output[++n] = '\0';
			
			// Extract digits from the remainder one at a time
			while (digits-- > 0) {
				// calculate the current digit
				remainder *= 10.0;
				unsigned int toPrint = (unsigned int)remainder;
				
				// overwrite the last null terminator with the current digit
				n += snprintf(&output[n], OUTPUT_SIZE-n, "%d", toPrint);
				
				// shift to the next digit
				remainder -= toPrint;
			}
		}
	}
	
	usb_write((uint8_t*)output, n);
	return n;
}