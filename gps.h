
/**
 * gps_init
 *
 * Initializes and starts the GPS module with the default sampling and
 * messaging rates
 *
 * @return true if successful, false if initialization fails
 */
bool gps_init();

/**
 * gps_getfix
 *
 * Polls the GPS module for a single fix and the minimum recommended 
 * positional data
 *
 * @param fix reference to a gps fix data structure
 * @return 0 if successful, integer status code otherwise
 */
uint8_t gps_getfix(GpsData *fix );

/**
 * gps_gettime
 *
 * Polls the GPS module for a satellite provided time sample
 *
 * @param time reference to a time/date structure
 * @return 0 if successful, integer status code otherwise
 */
uint8_t gps_gettime(Time *time);

/**
 * gps_setrate
 *
 * Configure the GPS module to sample at a given rate
 *
 * @param period sampling rate of gps in seconds
 * @return true if succesful, false if sampling rate not set
 */
bool gps_setrate(const uint32_t period);

/**
 * gps_sleep
 *
 * Disable the GPS module by putting it in low-power (sleep) mode
 *
 * @return true if successful, false if device fails to sleep
 */
bool gps_sleep();

/**
 * gps_wake
 *
 * Enable the GPS module by waking it from low-power (sleep) mode, it will 
 * resume sampling according to last configuration
 *
 * @return true if successful, false if device fails to sleep
 */
bool gps_sleep();

/**
 * gps_setprofile
 *
 * Configure the GPS module with a predefined sampling/power scheme
 *
 * @param profile preconfigured sampling profile
 * @returns true if successful, false if profile not set
 */
bool gps_setprofile(const GpsProfile profile);
