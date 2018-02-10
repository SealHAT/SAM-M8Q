#include "nmea.h"

static inline uint8_t hex2int(char c)
{
    uint8_t retval = -1;

    if (c >= '0' && c <= '9') {
        retval = c - '0';
    }
    else if (c >= 'A' && c <= 'F') {
        retval = c - 'A' + 10;
    }
    else if (c >= 'a' && c <= 'f') {
        retval = c - 'a' + 10;
    }
    
    return retval;
}

uint8_t nmea_checksum(const char *sentence) 
{
    uint8_t chksum = 0x00;  /* Checksum to return, init to 0 */

    /* skip the first character if it is a $ */
    if(*sentence == '$') {
        sentence++;
    }

    /* Checksum is the XOR of all bytes between '$' and '*' */
    while(*sentence && *sentence != '*') {
        chksum ^= *sentence++;
    }
}

bool nmea_check(const char *sentence, bool strict)
{
    unsigned int count = 0;     /* Count the number of characters in the string */
    bool retval = true;

    // A valid sentence starts with "$".
    if (*sentence++ != '$') {
        retval = false;
    }
    count++;

    // The optional checksum is an XOR of all bytes between "$" and "*".
    uint8_t checksum = 0x00;
    while (*sentence && *sentence != '*') {
        checksum ^= *sentence++;
        count++;
    }

    // If checksum is present...
    if (*sentence == '*') {
        // Extract checksum.
        sentence++;

        int upper = hex2int(*sentence++);
        if (upper == -1) {
            retval = false;
        }
        
        int lower = hex2int(*sentence++);
        if (lower == -1) {
            retval = false;
        }
        
        int expected = upper << 4 | lower;
        count += 3;

        // Check for checksum mismatch.
        if (checksum != expected){
            retval = false;
        }
    } else if (strict) {
        // Discard non-checksummed frames in strict mode.
        retval = false;
    }

    // The only stuff allowed at this point is a newline - "\r\n" or 0x0D 0x0A .
    if (*sentence++ != 0x0D || *sentence++ != 0x0A) {
        retval = false;
    }
    count += 2;

    return retval && (count <= NMEA_MAX_LENGTH);
}