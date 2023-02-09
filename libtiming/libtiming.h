#ifndef TIME_H
#define TIME_H

#include <sys/time.h>	// for struct timeval & gettimeofday()

#define UNIX_TIME_AT_00_00_00_01_01_2000 946684800
#define TIMESTAMP_CHARACTER_LENGTH 20

// stores the date and time in a format compatible with the OBC's RTC chip
struct Date_RTC
{
	unsigned char seconds; // seconds since start of most recent minute
	unsigned char minutes; // minutes since start of most recent hour
	unsigned char hours;   // hours since start of most recent day
	unsigned char days;    // days since start of most recent month
	unsigned char months;  // months since start of most recent year
	unsigned char years;   // years since the year 2000
};



/**
 * @brief Suspends the execution of the calling thread by ms milliseconds 
          using the function nanosleep()
 * @param ms Amount of milliseconds to sleep
*/
void delay_ms (int ms);

/**
 * @brief Puts the sum of the two timevals tmvl1, tmvl2 into tmvl_sum
 * @param tmvl1	First summand
 * @param tmvl2 Second summand
 * @param tmvl_sum The sum of the two summands, tmvl1 + tmvl2
*/
void timeadd(struct timeval* tmvl1, struct timeval* tmvl2, struct timeval* tmvl_sum);

/**
 * @brief Puts the difference between tmvl1 and tmvl2 into tmvl_diff
 * @param tmvl1	Minuend
 * @param tmvl2 Subtrahend
 * @param tmvl_diff The result (difference) of minued - subtrahend, tmvl1 - tmvl2
*/
void timediff(struct timeval* tmvl1, struct timeval* tmvl2, struct timeval* tmvl_diff);

/**
 * @brief Returns the difference in elapsed time in milliseconds between 
 *        two timeval structs tmvl1 - tmvl2
 * @param tmvl1	Minuend
 * @param tmvl2 Subtrahend
 * @return The difference between tmvl1 and tmvl2 (tmvl1 - tmvl2) in milliseconds
*/
int timediff_ms(struct timeval* tmvl1, struct timeval* tmvl2);



/**
 * @brief Converts the date given by struct Date_RTC* date_rtc into 
 *        unix time (seconds since 00:00:00 01-01-1970)
 * @param date_rtc Date to be converted, see definition of struct Date_RTC 
 *        for details on how to the date is stored
 * @param tmvl The calulated unix time will be placed in the struct timeval 
 *        variable to which tmvl points
*/
void date_to_unix_time(struct Date_RTC* date_rtc, struct timeval* tmvl);

/**
 * @brief Converts the unix time given by struct timeval* tmvl into a date of 
 *        the format hh:mm:ss DD-MM-20YY, stored in struct Date_RTC* date_rtc_out
 *        unix time (seconds since 00:00:00 01-01-1970)
 * @param tmvl The given unix time to be converted
 * @param date_rtc The resulting date that correspnds to the given unix time. See 
 *        definition of struct Date_RTC for details on how to the date is stored
*/
void unix_time_to_date(struct timeval* tmvl, struct Date_RTC* date_rtc_out);

/**
 * @brief Reads the current date from the RTC connected via I2C and puts it 
 *        into struct Date_RTC* date_rtc
 * @param fd_i2c File desciptor of the I2C device
 * @param date_rtc Pointer to a struct Date_RTC into which the date is put
*/
int get_RTC_date(int fd_i2c, struct Date_RTC* date_rtc);

/**
 * @brief Writes the data inside the date_rtc struct to the RTC connected via I2C
 * 
 * @param fd_i2c File descriptor to the I2C device
 * @param date_rtc Pointer to the struct Date_RTC which must contain valid data
 * 
 * @warning The function does not validate the data; it is the user's responsibility
 * 			that the data inside the struct indeed makes sense. You may use
 * 			print_RTC_date to check this.
 */
int set_RTC_date(int fd_i2c, struct Date_RTC* date_rtc);

/**
 * @brief Reads the current date from the RTC connected via I2C, converts it 
 *        to unix time and and puts it into struct timeval* tmvl
 * @param fd_i2c File desciptor of the I2C device
 * @param date_rtc Pointer to a struct timeval into which the unix time is put
*/
int get_RTC_unix_time(int fd_i2c, struct timeval* tmvl);

/**
 * @brief Converts the unix time given as a struct timeval into a c-string of 
 *        the format [DDDD hh:mm:ss.mss] 
 * @param tmvl Pointer to a struct timeval containing unix time
 * @param buffer Pointer to a c-string buffer of sufficient length (at least 21)
*/
void unix_time_to_buffer(struct timeval* tmvl, char* buffer);

/**
 * @brief Sets the RTC
 *  
 * @param unix_time Unix timestamp as big endian
*/
int set_RTC_unix(int fd_i2c, unsigned int unix_time);

/**
 * @brief Prints the unix time given as a struct timeval to the serial console
 * @param tmvl Pointer to a struct timeval containing unix time
*/
void print_unix_time(struct timeval* tmvl);

/**
 * @brief Prints the date time given as a struct Date_RTC to the serial console
 * @param date_rtc Pointer to a struct Date_RTC containing a date
*/
void print_RTC_date(struct Date_RTC* date_rtc);


// Old timestammping functions that are kept for no good reason
void printTime_old(unsigned int t);
int printTimeToBuffer_old(unsigned int t, char* buffer);



#endif