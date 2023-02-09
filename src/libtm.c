// #include <dirent.h>
// #include <errno.h>
// #include <fcntl.h>
// #include <stdio.h>
// #include <sys/stat.h>
// #include <sys/types.h>
// #include <unistd.h>
// // #include <macros.h> // Framsat .h
// #include <errno.h>
// #include <string.h>
// // #include <libtiming.h>
// #include <stdlib.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "../inc/libtm.h"

#define DO_DEBUG 1

#define ArrayLength(Array) (sizeof(Array) / sizeof(Array[0]))
#ifdef DO_DEBUG
#define DEBUG(string) string
#endif

struct __attribute__((__packed__)) vec3
{
    float x;
    float y;
    float z;
};

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

static struct ADCS_sensor_data _latest_polled_data;

int libtm_set_sensor_data(float x, float y, float z, uint8_t sensor_id)
{
    if (sensor_id >= IMTQ_HOUSEKEEPING)
    {
        DEBUG(printf("Invalid sensor id; expected a value between 1 and %d "
                     "inclusive, was actually %d\n",
                     N_SENSORS + 1, IMTQ_HOUSEKEEPING - 1);)
        return -1;
    }

    struct vec3 *sensor_data = (struct vec3 *)&_latest_polled_data;

    sensor_data[sensor_id - 1].x = x;
    sensor_data[sensor_id - 1].y = y;
    sensor_data[sensor_id - 1].z = z;
}

void libtm_set_iMTQ_housekeeping_data(const void *iMTQ_housekeeping_data)
{
    // printf("Address of envModels_quarternion_ECI: %p\n", &_latest_polled_data.envModels_quarternion_ECI);
    // printf("Address of imtq: %p\n", &_latest_polled_data.iMTQ_housekeeping);
    // printf("Diff: %ld\n", (uint64_t)&_latest_polled_data.iMTQ_housekeeping - (uint64_t)&_latest_polled_data.envModels_quarternion_ECI);
    memcpy((void *)&_latest_polled_data.iMTQ_housekeeping, iMTQ_housekeeping_data,
           sizeof(struct iMTQ_housekeeping_data));
}

void print_ADCS_data()
{
    struct vec3 *sensor_data = (struct vec3 *)&_latest_polled_data;
    for (int i = 0; i < IMTQ_HOUSEKEEPING - 1; ++i)
    {
        printf("Sensor %d values: X=%f Y=%f Z=%f\n", i, sensor_data[i].x, sensor_data[i].y, sensor_data[i].z);
    }

    printf("\n");

    uint16_t *imtq_data = (uint16_t *)&_latest_polled_data.iMTQ_housekeeping;
    for (int i = 0; i < 11; ++i)
    {
        if (i < 4)
        {
            printf("Field %d values: %d\n", i, imtq_data[i]);
        }
        else
        {
            printf("Field %d values: %d\n", i, (int16_t)imtq_data[i]);
        }
    }
}

static void libtm_run_tests()
{
    struct iMTQ_housekeeping_data iMTQ_housekeeping_data;
    iMTQ_housekeeping_data.dig_volt = 1;
    iMTQ_housekeeping_data.analog_volt = 2;
    iMTQ_housekeeping_data.dig_curr = 3;
    iMTQ_housekeeping_data.analog_curr = 4;
    iMTQ_housekeeping_data.meas_curr_x = 5;
    iMTQ_housekeeping_data.meas_curr_y = 6;
    iMTQ_housekeeping_data.meas_curr_z = 7;
    iMTQ_housekeeping_data.coil_temp_x = 8;
    iMTQ_housekeeping_data.coil_temp_y = 9;
    iMTQ_housekeeping_data.coil_temp_z = 10;
    iMTQ_housekeeping_data.MCU_temp = 11;

    libtm_set_iMTQ_housekeeping_data((const void *)&iMTQ_housekeeping_data);

    libtm_set_sensor_data(1, 2, 3, 1);
    libtm_set_sensor_data(4, 5, 6, 2);
    libtm_set_sensor_data(7, 8, 9, 3);
    libtm_set_sensor_data(10, 11, 12, 4);
    libtm_set_sensor_data(13, 14, 15, 5);
    libtm_set_sensor_data(16, 17, 18, 6);
    libtm_set_sensor_data(19, 20, 21, 7);
    libtm_set_sensor_data(22, 23, 24, 8);

    print_ADCS_data();

    printf("\n");

    uint16_t new_imtq_data[11] = {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    libtm_set_iMTQ_housekeeping_data((const void *)new_imtq_data);

    libtm_set_sensor_data(22, 23, 24, 1);
    libtm_set_sensor_data(25, 26, 27, 2);
    libtm_set_sensor_data(28, 29, 30, 3);
    libtm_set_sensor_data(31, 32, 33, 4);
    libtm_set_sensor_data(34, 35, 36, 5);
    libtm_set_sensor_data(37, 38, 39, 6);
    libtm_set_sensor_data(40, 41, 42, 7);
    libtm_set_sensor_data(43, 44, 45, 8);

    print_ADCS_data();
}

int main()
{
    libtm_run_tests();

    return 0;
}