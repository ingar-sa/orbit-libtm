// #include <dirent.h>
// #include <errno.h>
// #include <sys/stat.h>
// // #include <macros.h> // Framsat .h
// #include <errno.h>
// #include <string.h>
// // #include <libtiming.h>
// #include <stdlib.h>
// #include <sys/types.h>

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "../inc/libtm.h"

#define DO_DEBUG 0
#define TESTING 0

#define ArrayLength(Array) (sizeof(Array) / sizeof(Array[0]))
#if DO_DEBUG
#define DEBUG(string) string
#else
#define DEBUG(string)
#endif

#define TM_MAX_FILESIZE_BYTES (211 420)
#define TM_FILEAMOUNT_ALLOWED 4000

// TODO(ingar): give this struct a better name since it's used for multiple
// things
//  8 * 9 = 72 bytes
struct __attribute__((__packed__)) sensor_time_data_ms
{
    struct timeval IMU_Mean_Gyro_XYZ;
    struct timeval IMU_Mean_Magnetometer;
    struct timeval Controller_output_raw;
    struct timeval Controller_output_to_imtq;
    struct timeval envModels_Mag_field;
    struct timeval envModels_Sun_position;
    struct timeval envModels_radius_ECI;
    struct timeval envModels_quarternion_ECI;
    struct timeval iMTQ_housekeeping;
};

// NOTE(ingar): Just make this 
//  8 * 21 + 22 = 186 bytes
struct __attribute__((__packed__)) ADCS_sensor_data
{
    struct libtm_packet IMU_Mean_Gyro_XYZ;
    struct libtm_packet IMU_Mean_Magnetometer;
    struct libtm_packet Controller_output_raw;
    struct libtm_packet Controller_output_to_imtq;
    struct libtm_packet envModels_Mag_field;
    struct libtm_packet envModels_Sun_position;
    struct libtm_packet envModels_radius_ECI;
    struct libtm_packet envModels_quarternion_ECI;
    struct libtm_packet iMTQ_housekeeping;
};

struct file_write_buffer
{
    uint8_t remaining_bytes;
    uint8_t buffer[256];
};

// WARNING(ingar): I do a lot of casting these structs to arrays of their types.
// Therefore, I should be careful of alignment issues.
static struct ADCS_sensor_data _latest_polled_data = {0};
static struct sensor_time_data_ms _sensor_write_intervals_ms = {0};
static struct file_write_buffer _file_write_buffer = {0};

#if 1
// NOTE(ingar): Rename this? Make this public?
// Still not sure where the "tick" is going to happen
void _libtm_write()
{
    struct timeval *sensor_write_intervals = (struct timeval *)&_sensor_write_intervals_ms;
    struct libtm_packet *latest_polled_data = (struct libtm_packet *)&_latest_polled_data;
    
    for (int i = 0; i < IMTQ_HOUSEKEEPING; ++i)
    {
        struct libtm_packet sensor_packet = latest_polled_data[i];
        struct timeval sensor_write_interval = sensor_write_intervals[i];
        struct libtm_packet_header sensor_header = sensor_packet.header;

        if (sensor_header.timestamp.tv_sec <= sensor_write_interval.tv_sec &&
            sensor_header.timestamp.tv_usec < sensor_write_interval.tv_usec) // NOTE(ingar): Make this <= ?
        {
            continue; // Not enough time has passed since last write
        }

        if (_file_write_buffer.remaining_bytes + sensor_packet.size >= 256)
        {
            _write_buffer_to_file();
            memset(&_file_write_buffer, 0, sizeof(struct file_write_buffer)); // Reset buffer
            _file_write_buffer.remaining_bytes = 256;
        }

        memcpy(&_file_write_buffer.buffer[256 - _file_write_buffer.remaining_bytes],
               &sensor_packet, sensor_packet.size);
    }
}
#endif

void _write_buffer_to_file()
{
}

void libtm_set_sensor_write_interval(struct timeval time_interval,
                                     uint8_t sensor_id)
{
    if (sensor_id < 1 || sensor_id > N_SENSORS)
    {
        DEBUG(printf("Invalid sensor id when setting sensor write interval; "
                     "expected a value between 1 and %d "
                     "inclusive, was actually %d\n",
                     N_SENSORS, sensor_packet.header.sensor_id);)
        return;
    }
    
    struct timeval *sensor_write_intervals = (struct timeval *)&_sensor_write_intervals_ms;
    sensor_write_intervals[sensor_id - 1] = time_interval;
}

void libtm_set_sensor_data(struct libtm_packet sensor_packet)
{
    if (sensor_packet.header.sensor_id < 1 || sensor_packet.header.sensor_id > N_SENSORS)
    {
        DEBUG(printf("Invalid sensor id when setting sensor write interval; "
                     "expected a value between 1 and %d "
                     "inclusive, was actually %d\n",
                     N_SENSORS, sensor_packet.header.sensor_id);)
        return;
    }

    struct libtm_packet *latest_polled_data = (struct libtm_packet *)&_latest_polled_data;
    latest_polled_data[sensor_packet.header.sensor_id - 1] = sensor_packet;
}


#if TESTING
static void libtm_run_tests();
static void print_ADCS_data();
#endif

int main()
{
#if TESTING
    libtm_run_tests();
    print_ADCS_data();
#endif

    return 0;
}

#if TESTING
static void libtm_run_tests()
{
    struct timeval imtq_time_interval = {10, 0};
    struct libtm_iMTQ_housekeeping_data iMTQ_housekeeping_data;
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

    struct libtm_iMTQ_housekeeping_packet iMTQ_housekeeping_packet = {
        .sensor_id = IMTQ_HOUSEKEEPING,
        .timestamp_last_write = imtq_time_interval,
        .data = iMTQ_housekeeping_data};

    libtm_set_iMTQ_housekeeping_data((const void *)&iMTQ_housekeeping_packet);

    struct libtm_vec3_packet sensor_data = {.sensor_id = 1,
                                            .timestamp_last_write = imtq_time_interval,
                                            .data = {1, 2, 3}};

    struct libtm_vec3_packet sensor_data2 = {.sensor_id = 2,
                                             .timestamp_last_write = imtq_time_interval,
                                             .data = {4, 5, 6}};

    struct libtm_vec3_packet sensor_data3 = {.sensor_id = 3,
                                             .timestamp_last_write = imtq_time_interval,
                                             .data = {7, 8, 9}};

    struct libtm_vec3_packet sensor_data4 = {.sensor_id = 4,
                                             .timestamp_last_write = imtq_time_interval,
                                             .data = {10, 11, 12}};

    struct libtm_vec3_packet sensor_data5 = {.sensor_id = 5,
                                             .timestamp_last_write = imtq_time_interval,
                                             .data = {13, 14, 15}};

    struct libtm_vec3_packet sensor_data6 = {.sensor_id = 6,
                                             .timestamp_last_write = imtq_time_interval,
                                             .data = {16, 17, 18}};

    struct libtm_vec3_packet sensor_data7 = {.sensor_id = 7,
                                             .timestamp_last_write = imtq_time_interval,
                                             .data = {19, 20, 21}};

    struct libtm_vec3_packet sensor_data8 = {.sensor_id = 8,
                                             .timestamp_last_write = imtq_time_interval,
                                             .data = {22, 23, 24}};

    libtm_set_sensor_data((const void *)&sensor_data, 1);
    libtm_set_sensor_data((const void *)&sensor_data2, 2);
    libtm_set_sensor_data((const void *)&sensor_data3, 3);
    libtm_set_sensor_data((const void *)&sensor_data4, 4);
    libtm_set_sensor_data((const void *)&sensor_data5, 5);
    libtm_set_sensor_data((const void *)&sensor_data6, 6);
    libtm_set_sensor_data((const void *)&sensor_data7, 7);
    libtm_set_sensor_data((const void *)&sensor_data8, 8);
}

static void print_ADCS_data()
{
    struct libtm_vec3_packet *sensor_packets =
        (struct libtm_vec3_packet *)&_latest_polled_data;
    for (int i = 0; i < IMTQ_HOUSEKEEPING - 1; ++i)
    {
        struct libtm_vec3_packet sensor_packet = sensor_packets[i];
        printf("Sensor %d values: tv_sec: %ld, tv_usec: %ld, X: %f, Y: %f, Z: %f\n",
               sensor_packet.sensor_id, sensor_packet.timestamp_last_write.tv_sec,
               sensor_packet.timestamp_last_write.tv_usec, sensor_packet.data.x,
               sensor_packet.data.y, sensor_packet.data.z);
    }

    printf("\n");

    struct libtm_iMTQ_housekeeping_packet iMTQ_housekeeping_packet =
        _latest_polled_data.iMTQ_housekeeping;
    printf("iMTQ Housekeeping values:\n tv_sec: %ld, tv_usec: %ld, dig_volt: %d,\n "
           "analog_volt: %d, dig_curr: %d, analog_curr: %d, meas_curr_x: %d,\n "
           "meas_curr_y: %d, meas_curr_z: %d, coil_temp_x: %d, coil_temp_y: %d,\n "
           "coil_temp_z: %d, MCU_temp: %d\n",
           iMTQ_housekeeping_packet.timestamp_last_write.tv_sec,
           iMTQ_housekeeping_packet.timestamp_last_write.tv_usec,
           iMTQ_housekeeping_packet.data.dig_volt,
           iMTQ_housekeeping_packet.data.analog_volt,
           iMTQ_housekeeping_packet.data.dig_curr,
           iMTQ_housekeeping_packet.data.analog_curr,
           iMTQ_housekeeping_packet.data.meas_curr_x,
           iMTQ_housekeeping_packet.data.meas_curr_y,
           iMTQ_housekeeping_packet.data.meas_curr_z,
           iMTQ_housekeeping_packet.data.coil_temp_x,
           iMTQ_housekeeping_packet.data.coil_temp_y,
           iMTQ_housekeeping_packet.data.coil_temp_z,
           iMTQ_housekeeping_packet.data.MCU_temp);
}
#endif