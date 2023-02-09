#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
// #include <macros.h> // Framsat .h
#include <errno.h>
#include <string.h>
// #include <libtiming.h>
#include <stdlib.h>

#include "../inc/libtm.h"

#define DO_DEBUG 1

#define ArrayLength(Array) (sizeof(Array) / sizeof(Array[0]))
#ifdef DO_DEBUG
#define DEBUG(string) string
#endif

struct __attribute__((__packed__)) iMTQ_housekeeping_data
{
    uint16_t dig_volt;    /**< Measured voltage of digital supply  */
    uint16_t analog_volt; /**< Measured voltage of analogue supply  */
    uint16_t dig_curr;    /**< Measured current of digital supply  */
    uint16_t analog_curr; /**< Measured current of analogue supply  */
    int16_t meas_curr_x;  /**< Measured current through X-axis coil  */
    int16_t meas_curr_y;  /**< Measured current through Y-axis coil  */
    int16_t meas_curr_z;  /**< Measured current through Z-axis coil  */
    int16_t coil_temp_x;  /**< Measured temperature in X-axis coil  */
    int16_t coil_temp_y;  /**< Measured temperature in Y-axis coil  */
    int16_t coil_temp_z;  /**< Measured temperature in Z-axis coil  */
    int16_t MCU_temp;     /**< Temperature measurement of the MCU  */
};

struct __attribute__((__packed__)) vec3
{
    float X;
    float Y;
    float Z;
};

struct __attribute__((__packed__)) ADCS_sensor_data
{
    struct vec3 IMU_Mean_Gyro_XYZ;
    struct vec3 IMU_Mean_Magnetometer;
    struct vec3 Controller_output_raw;
    struct vec3 Controller_output_to_imtq;
    struct iMTQ_housekeeping_data iMTQ_housekeeping;
    struct vec3 envModels_Mag_field;
    struct vec3 envModels_Sun_position;
    struct vec3 envModels_radius_ECI;
    struct vec3 envModels_quarternion_ECI;
};

// NOTE(ingar): I should talk with Sigmund about how to do the error for iMTQ
// housekeeping and the p-controller read
//
// Bitmask mapping (least siginificant bits): bits [1, 9] are errors reading a
// sensor, with its sensor_ids value mapping to the bit number. Bit 10 is an
// error when writing a file.
static uint16_t _error_bitmask;
static struct iMTQ_housekeeping_data _latest_iMTQ_housekeeping_data;

//TODO(ingar): Reset error flag on file write?
// 
int libtm_set_error_flag(uint8_t n_bit)
{
    uint16_t mask = 1;
    if (n_bit > N_SENSORS + 1) // +1 for filewrite error
    {
        DEBUG(printf("Invalid sensor id; expected a value between 1 and %d inclusive, was actually %d\n",
                     N_SENSORS + 1, n_bit);)
        return -1;
    }

    _error_bitmask |= 1 << n_bit - 1;
    return 0;
}

// NOTE(ingar): Make struct public and allow user to pass in a struct instead?
// Do some form of value validation?
int libtm_set_iMTQ_housekeeping_data(uint16_t dig_volt, uint16_t analog_volt,
                                     uint16_t dig_curr, uint16_t analog_curr,
                                     int16_t meas_curr_x, int16_t meas_curr_y,
                                     int16_t meas_curr_z, int16_t coil_temp_x,
                                     int16_t coil_temp_y, int16_t coil_temp_z,
                                     int16_t MCU_temp)
{
    _latest_iMTQ_housekeeping_data.dig_volt = dig_volt;
    _latest_iMTQ_housekeeping_data.analog_volt = analog_volt;
    _latest_iMTQ_housekeeping_data.dig_curr = dig_curr;
    _latest_iMTQ_housekeeping_data.analog_curr = analog_curr;
    _latest_iMTQ_housekeeping_data.meas_curr_x = meas_curr_x;
    _latest_iMTQ_housekeeping_data.meas_curr_y = meas_curr_y;
    _latest_iMTQ_housekeeping_data.meas_curr_z = meas_curr_z;
    _latest_iMTQ_housekeeping_data.coil_temp_x = coil_temp_x;
    _latest_iMTQ_housekeeping_data.coil_temp_y = coil_temp_y;
    _latest_iMTQ_housekeeping_data.coil_temp_z = coil_temp_z;
    _latest_iMTQ_housekeeping_data.MCU_temp = MCU_temp;

    return 0;
}

int main()
{
    // Should fail when i=11, succeed otherwise
    // for (uint16_t i = 1; i < 12; ++i)
    // {
    //     libtm_set_error_flag(i);
    //     _error_bitmask = 0;
    // }

    // Should succeed, and _error_bitmask should remain the same every iteration
    // for (uint16_t i = 0; i < 3; ++i)
    // {
    //     libtm_set_error_flag(5);
    // }
    

    return 0;
}