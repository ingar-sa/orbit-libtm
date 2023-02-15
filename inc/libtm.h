#ifndef LIBTM_H
#define LIBTM_H

enum libtm_sensor_ids
{
    IMU_MEAN_GYRO_XYZ = 1,
    IMU_MEAN_MAGNETOMETER,
    CONTROLLER_OUTPUT_RAW,
    CONTROLLER_OUTPUT_TO_IMTQ,
    ENVMODELS_MAG_FIELD,
    ENVMODELS_SUN_POSITION,
    ENVMODELS_RADIUS_ECI,
    ENVMODELS_QUARTERNION_ECI,
    IMTQ_HOUSEKEEPING,
    N_SENSORS = 9
};

// NOTE(ingar): The timeval struct is 16 bytes on my machine, however
// it should be only 8 bytes on eos (2 uint32_t)
// 3 * 4 = 12 bytes
struct __attribute__((__packed__)) libtm_vec3
{
    float x;
    float y;
    float z;
};

// 1 + 8 + 12 = 21 bytes
struct __attribute__((__packed__)) libtm_vec3_packet
{
    uint8_t sensor_id;
    struct timeval timestamp_last_write;
    struct libtm_vec3 data;
};

// 11 * 2 = 22 bytes
struct __attribute__((__packed__)) libtm_iMTQ_housekeeping_data
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

// Needed to make the packet structs because otherwise the pointer arithmetic
// on the data would be fucked up

// 1 + 8 + 22 = 31 bytes
struct __attribute__((__packed__)) libtm_iMTQ_housekeeping_packet
{
    // We will always know this id, but we whould probably have the
    // id in the struct anyway for consistency
    uint8_t sensor_id;
    struct timeval timestamp_last_write;
    struct libtm_iMTQ_housekeeping_data data;
};


#endif // LIBTM_H