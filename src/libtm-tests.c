#include <sys/time.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "../inc/libtm.h"

#define DO_DEBUG 0

#define ArrayLength(Array) (sizeof(Array) / sizeof(Array[0]))
#ifdef DO_DEBUG
#define DEBUG(string) string
#endif

// 2 * 9 = 18  bytes
struct sensor_write_intervals_ms
{
    uint16_t IMU_Mean_Gyro_XYZ;
    uint16_t IMU_Mean_Magnetometer;
    uint16_t Controller_output_raw;
    uint16_t Controller_output_to_imtq;
    uint16_t envModels_Mag_field;
    uint16_t envModels_Sun_position;
    uint16_t envModels_radius_ECI;
    uint16_t envModels_quarternion_ECI;
    uint16_t iMTQ_housekeeping;
};


struct __attribute__((__packed__)) vec3
{
    float x;
    float y;
    float z;
};

// 11 * 2 = 22 bytes
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

// 12 * 8 + 22 = 118 bytes
struct __attribute__((__packed__)) ADCS_sensor_data
{
    struct vec3 IMU_Mean_Gyro_XYZ;
    struct vec3 IMU_Mean_Magnetometer;
    struct vec3 Controller_output_raw;
    struct vec3 Controller_output_to_imtq;
    struct vec3 envModels_Mag_field;
    struct vec3 envModels_Sun_position;
    struct vec3 envModels_radius_ECI;
    struct vec3 envModels_quarternion_ECI;
    struct iMTQ_housekeeping_data iMTQ_housekeeping;
};

