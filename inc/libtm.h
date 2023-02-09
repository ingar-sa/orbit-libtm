#include <stdint.h>

enum sensor_ids
{
    IMU_MEAN_GYRO_XYZ = 1,
    IMU_MEAN_MAGNETOMETER,
    CONTROLLER_OUTPUT_RAW,
    CONTROLLER_OUTPUT_TO_IMTQ,
    IMTQ_HOUSEKEEPING,
    ENVMODELS_MAG_FIELD,
    ENVMODELS_SUN_POSITION,
    ENVMODELS_RADIUS_ECI,
    ENVMODELS_QUARTERNION_ECI,
    N_SENSORS = 9
};

//NOTE(ingar): Should probably not be in here, since we're just going to have static
// variables of these struct that are written to by functions
// Maybe iMTQ housekeeping data should be public, since Sigmund is going to write 
// into that
// I think we just write a function that takes in all the variables as arguments


// struct __attribute__((__packed__)) iMTQ_housekeeping_data
// {
//     uint16_t dig_volt;    /**< Measured voltage of digital supply  */
//     uint16_t analog_volt; /**< Measured voltage of analogue supply  */
//     uint16_t dig_curr;    /**< Measured current of digital supply  */
//     uint16_t analog_curr; /**< Measured current of analogue supply  */
//     int16_t meas_curr_x;  /**< Measured current through X-axis coil  */
//     int16_t meas_curr_y;  /**< Measured current through Y-axis coil  */
//     int16_t meas_curr_z;  /**< Measured current through Z-axis coil  */
//     int16_t coil_temp_x;  /**< Measured temperature in X-axis coil  */
//     int16_t coil_temp_y;  /**< Measured temperature in Y-axis coil  */
//     int16_t coil_temp_z;  /**< Measured temperature in Z-axis coil  */
//     int16_t MCU_temp;     /**< Temperature measurement of the MCU  */
// };

// struct __attribute__((__packed__)) Vec3
// {
//     float X;
//     float Y;
//     float Z;
// };

// struct __attribute__((__packed__)) ADCS_sensor_data
// {
//     Vec3 IMU_Mean_Gyro_XYZ;
//     Vec3 IMU_Mean_Magnetometer;
//     Vec3 Controller_output_raw;
//     Vec3 Controller_output_to_imtq;
//     iMTQ_housekeeping_data iMTQ_housekeeping;
//     Vec3 envModels_Mag_field;
//     Vec3 envModels_Sun_position;
//     Vec3 envModels_radius_ECI;
//     Vec3 envModels_quarternion_ECI;
// };