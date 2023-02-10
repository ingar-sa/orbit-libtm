// #include <dirent.h>
// #include <errno.h>
// #include <fcntl.h>
// #include <stdio.h>
// #include <sys/stat.h>
// #include <unistd.h>
// // #include <macros.h> // Framsat .h
// #include <errno.h>
// #include <string.h>
// // #include <libtiming.h>
// #include <stdlib.h>
// #include <sys/types.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "../inc/libtm.h"

#define DO_DEBUG 0

#define ArrayLength(Array) (sizeof(Array) / sizeof(Array[0]))
#ifdef DO_DEBUG
#define DEBUG(string) string
#endif

// TODO(ingar): give this struct a better name since it's used for multiple
// things
//  2 * 9 = 18  bytes
struct __attribute__((__packed__)) sensor_write_intervals_ms
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

// 3 * 4 = 12 bytes
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

static struct ADCS_sensor_data _latest_polled_data;
static struct sensor_write_intervals_ms _sensor_write_intervals_ms;
static struct sensor_write_intervals_ms
    _elapsed_time_since_last_sensor_write_ms;

void libtm_write_to_file()
{
    struct timeval timeval_current_time;
    gettimeofday(&timeval_current_time, NULL);

    uint8_t sensors_to_write[N_SENSORS] = {0};
    uint8_t n_sensors_to_write = 0;
    uint16_t *sensor_write_intervals = (uint16_t *)&_sensor_write_intervals_ms;
    uint16_t *elapsed_time_since_last_sensor_write =
        (uint16_t *)&_elapsed_time_since_last_sensor_write_ms;

    for (int i = 0; i < N_SENSORS; ++i)
    {
        elapsed_time_since_last_sensor_write[i] +=
            timeval_current_time.tv_usec * 1000;

        if ((sensor_write_intervals[i] == 0) ||
            (elapsed_time_since_last_sensor_write[i] < sensor_write_intervals[i]))
        {
            continue;
        }

        
    }
}

void libtm_set_sensor_write_interval(uint16_t interval_ms, uint8_t sensor_id)
{
    if (sensor_id < 1 || sensor_id > N_SENSORS)
    {
        DEBUG(printf("Invalid sensor id when setting sensor write interval; "
                     "expected a value between 1 and %d "
                     "inclusive, was actually %d\n",
                     N_SENSORS, sensor_id);)
        return;
    }

    uint16_t *sensor_write_intervals = (uint16_t *)&_sensor_write_intervals_ms;
    sensor_write_intervals[sensor_id - 1] = interval_ms;
}

int libtm_set_sensor_data(float x, float y, float z, uint8_t sensor_id)
{
    if (sensor_id < 1 || sensor_id >= IMTQ_HOUSEKEEPING)
    {
        DEBUG(printf("Invalid sensor id when setting sensor data; expected a value "
                     "between 1 and %d "
                     "inclusive, was actually %d\n",
                     IMTQ_HOUSEKEEPING - 1, sensor_id);)
        return -1;
    }

    struct vec3 *sensor_data = (struct vec3 *)&_latest_polled_data;

    sensor_data[sensor_id - 1].x = x;
    sensor_data[sensor_id - 1].y = y;
    sensor_data[sensor_id - 1].z = z;
}

// NOTE(ingar): Should the byte be sent in separately from the buffer.
void libtm_set_iMTQ_housekeeping_data(const void *iMTQ_housekeeping_data)
{
    memcpy((void *)&_latest_polled_data.iMTQ_housekeeping, iMTQ_housekeeping_data,
           sizeof(struct iMTQ_housekeeping_data));
}

static void libtm_run_tests();
static void print_ADCS_data();

int main()
{
    // libtm_run_tests();

    return 0;
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

    libtm_set_sensor_data(1, 2, 3, 0);
    libtm_set_sensor_data(1, 2, 3, 1);
    libtm_set_sensor_data(4, 5, 6, 2);
    libtm_set_sensor_data(7, 8, 9, 3);
    libtm_set_sensor_data(10, 11, 12, 4);
    libtm_set_sensor_data(13, 14, 15, 5);
    libtm_set_sensor_data(16, 17, 18, 6);
    libtm_set_sensor_data(19, 20, 21, 7);
    libtm_set_sensor_data(22, 23, 24, 8);
    libtm_set_sensor_data(1, 2, 3, 9);
    // print_ADCS_data();

    DEBUG(printf("\n");)

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

    // print_ADCS_data();

    libtm_set_sensor_write_interval(50, 0);
    libtm_set_sensor_write_interval(50, 10);
    for (int i = 1; i <= N_SENSORS; ++i)
    {

        libtm_set_sensor_write_interval(50 * i, i);
    }

    uint16_t *sensor_write_intervals = (uint16_t *)&_sensor_write_intervals_ms;
    for (int i = 0; i < N_SENSORS; ++i)
    {
        DEBUG(printf("Sensor %d write interval: %d\n", i + 1,
                     sensor_write_intervals[i]);)
    }
}

static void print_ADCS_data()
{
    struct vec3 *sensor_data = (struct vec3 *)&_latest_polled_data;
    for (int i = 0; i < IMTQ_HOUSEKEEPING - 1; ++i)
    {
        DEBUG(printf("Sensor %d values: X=%f Y=%f Z=%f\n", i + 1, sensor_data[i].x,
                     sensor_data[i].y, sensor_data[i].z);)
    }

    DEBUG(printf("\n");)

    uint16_t *imtq_data = (uint16_t *)&_latest_polled_data.iMTQ_housekeeping;
    for (int i = 0; i < 11; ++i)
    {
        if (i < 4)
        {
            DEBUG(printf("Field %d values: %d\n", i + 1, imtq_data[i]);)
        }
        else
        {
            DEBUG(printf("Field %d values: %d\n", i + 1, (int16_t)imtq_data[i]);)
        }
    }
}
