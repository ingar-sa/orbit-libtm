#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "../inc/libtm.h"

#define DO_DEBUG 0
#define TEST 1

#if DO_DEBUG
#define DEBUG(string) string
#else
#define DEBUG(string)
#endif

#if TEST
#include <time.h>
#include <stdlib.h>
#include <errno.h>

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
#endif

#define FILE_PATH_BASE "telem/file"
#define MAX_FILESIZE_BYTES 50961960 // 48.6 MB
#define MAX_FILEAMOUNT_ALLOWED 32

#define PACKET_SIZE 21
#define VEC3_SIZE 12
#define IMTQ_HK_SIZE 22
#define LIBTM_HEADER_SIZE 9

struct file_write_buffer
{
    int16_t remaining_bytes;
    uint8_t buffer[256];
};

static struct file_write_buffer _file_write_buffer = {0};

static uint64_t *_sensor_write_intervals_us;
static uint64_t _sensor_last_write_time_us[IMTQ_HOUSEKEEPING] = {0};

static uint64_t *_file_ring_buffer;
static uint8_t *_active_file_index;
static uint32_t _active_file_remaining_bytes;
static char _active_file_path[12] = {0};
static int _active_file_fd;

static void _libtm_set_active_file(void)
{
    close(_active_file_fd);
    sprintf(_active_file_path, "%s%d", FILE_PATH_BASE, *_active_file_index);
    _active_file_remaining_bytes = MAX_FILESIZE_BYTES;
    _active_file_fd = open(_active_file_path, O_WRONLY | O_CREAT | O_TRUNC);
    if (_active_file_fd < 0)
    {
        DEBUG(printf("Error opening file: %s\n", strerror(errno));)
    }
}

static void _write_buffer_to_file(void)
{
    write(_active_file_fd, _file_write_buffer.buffer, 256 - _file_write_buffer.remaining_bytes);
    memset(&_file_write_buffer, 0, sizeof(_file_write_buffer));
    _file_write_buffer.remaining_bytes = 256;
}

static inline void *_get_file_write_buffer_head(void)
{
    return &_file_write_buffer.buffer[256 - _file_write_buffer.remaining_bytes];
}

void libtm_write_sensor_data(struct libtm_packet sensor_packet)
{
    if (sensor_packet.header.sensor_id < 1 || sensor_packet.header.sensor_id > N_SENSORS)
    {
        DEBUG(printf("Invalid sensor id when setting sensor write interval; "
                     "expected a value between 1 and %d "
                     "inclusive, was actually %d\n",
                     N_SENSORS, sensor_packet.header.sensor_id);)
        return;
    }

    uint64_t time_delta = sensor_packet.header.timestamp_us - _sensor_last_write_time_us[sensor_packet.header.sensor_id - 1];
    if (time_delta < _sensor_write_intervals_us[sensor_packet.header.sensor_id - 1])
    {
        return;
    }

    if (sensor_packet.header.sensor_id < IMTQ_HOUSEKEEPING &&
        _file_write_buffer.remaining_bytes < PACKET_SIZE)
    {
        _write_buffer_to_file();
        _file_ring_buffer[*_active_file_index] = sensor_packet.header.timestamp_us;
    }
    else if (sensor_packet.header.sensor_id == IMTQ_HOUSEKEEPING &&
             _file_write_buffer.remaining_bytes < 2 * PACKET_SIZE)
    {
        _write_buffer_to_file();
        _file_ring_buffer[*_active_file_index] = sensor_packet.header.timestamp_us;
    }

    _sensor_last_write_time_us[sensor_packet.header.sensor_id - 1] = sensor_packet.header.timestamp_us;

    printf("Packet: %d %ld ", sensor_packet.header.sensor_id, sensor_packet.header.timestamp_us);
    if (sensor_packet.header.sensor_id == IMTQ_HOUSEKEEPING)
    {
        void *sensor_data = sensor_packet.data;
#if TEST
        struct libtm_iMTQ_housekeeping_data *hk_data = sensor_data;
        printf("%d %d %d %d %d %d %d %d %d %d %d\n", hk_data->dig_volt, hk_data->analog_volt, hk_data->dig_curr,
               hk_data->analog_curr, hk_data->meas_curr_x,
               hk_data->meas_curr_y, hk_data->meas_curr_z,
               hk_data->coil_temp_x, hk_data->coil_temp_y,
               hk_data->coil_temp_z, hk_data->MCU_temp);
#endif
        // NOTE(ingar): This loop will work for an arbitrary size of sensor data, but
        // the library overall is still written with the assumption that we know exactly
        // which sensors are being used. It could be generalized to work with any data
        // by having a way to specify the size of the smallest data that could be put in
        // at any given time, and replacing VEC3_SIZE with that value. Mith do this later.
        // struct libtm_iMTQ_housekeeping_data *hk_data = sensor_data;

        for (int i = 0; i < sensor_packet.data_size / VEC3_SIZE; ++i)
        {
            memcpy(_get_file_write_buffer_head(), &sensor_packet.header, LIBTM_HEADER_SIZE); // Write header
            _file_write_buffer.remaining_bytes -= LIBTM_HEADER_SIZE;

            memcpy(_get_file_write_buffer_head(), sensor_data, VEC3_SIZE); // Write data
            sensor_data += VEC3_SIZE;
            _file_write_buffer.remaining_bytes -= VEC3_SIZE;
        }

        int remainder = sensor_packet.data_size % VEC3_SIZE;

        memcpy(_get_file_write_buffer_head(), &sensor_packet.header, LIBTM_HEADER_SIZE);
        _file_write_buffer.remaining_bytes -= LIBTM_HEADER_SIZE;

        memcpy(_get_file_write_buffer_head(), sensor_data, remainder);
        _file_write_buffer.remaining_bytes -= remainder;

        memset(_get_file_write_buffer_head(), 0, VEC3_SIZE - remainder); // Add padding
        _file_write_buffer.remaining_bytes -= VEC3_SIZE - remainder;
    }
    else
    {
#if TEST
        printf("%f %f %f\n", ((float *)sensor_packet.data)[0], ((float *)sensor_packet.data)[1], ((float *)sensor_packet.data)[2]);
#endif
        memcpy(_get_file_write_buffer_head(), &sensor_packet.header, LIBTM_HEADER_SIZE);
        _file_write_buffer.remaining_bytes -= LIBTM_HEADER_SIZE;

        memcpy(_get_file_write_buffer_head(), sensor_packet.data, sensor_packet.data_size);
        _file_write_buffer.remaining_bytes -= VEC3_SIZE;
    }

    if (_active_file_remaining_bytes < 2 * PACKET_SIZE)
    {
        ++(*_active_file_index);
        if (*_active_file_index > MAX_FILEAMOUNT_ALLOWED)
        {
            *_active_file_index = 0;
        }

        _libtm_set_active_file();
    }
}

void libtm_init(uint64_t *file_ring_buffer, uint8_t *active_file_index, uint64_t *sensor_write_intervals_us)
{
    _file_ring_buffer = file_ring_buffer;
    _active_file_index = active_file_index;
    _sensor_write_intervals_us = sensor_write_intervals_us;
    _file_write_buffer.remaining_bytes = 256;
    _libtm_set_active_file();
}

///////////////////////////////////////////////////////////////////////////////
// Test functions
///////////////////////////////////////////////////////////////////////////////

#if TEST

static void _print_file(int n_data)
{
    int fd = open(_active_file_path, O_RDONLY);
    if (fd < 0)
    {
        printf("Error opening file: %s\n", strerror(errno));
    }

    int n_imtq = n_data / 9;
    int buffer_size = n_data * PACKET_SIZE + n_imtq * PACKET_SIZE;
    uint8_t buffer[buffer_size];
    uint8_t *buf_pointer = buffer;
    int bytes_read = read(fd, buffer, buffer_size);

    if (bytes_read < 0)
    {
        printf("Error reading file: %s\n", strerror(errno));
    }

    for (int iteration = 1; n_data > 0; ++iteration, --n_data)
    {
        struct libtm_packet_header *packet_header = (struct libtm_packet_header *)buf_pointer;
        buf_pointer += LIBTM_HEADER_SIZE;
        if (packet_header->sensor_id < 1 || packet_header->sensor_id > N_SENSORS)
        {
            printf("Print file invalid sensor id: %d\n", packet_header->sensor_id);
            continue;
        }
        printf("%d ", iteration);
        printf("Header: %d %ld ", packet_header->sensor_id, packet_header->timestamp_us);
        if (packet_header->sensor_id == IMTQ_HOUSEKEEPING)
        {

            uint16_t *first_four = (uint16_t *)buf_pointer;
            printf("%d %d %d %d ", first_four[0], first_four[1], first_four[2], first_four[3]);
            buf_pointer += 8;

            int16_t *last_in_first_packet = (int16_t *)buf_pointer;
            printf("%d %d", last_in_first_packet[0], last_in_first_packet[1]);
            buf_pointer += 4 + LIBTM_HEADER_SIZE;

            int16_t *second_packet = (int16_t *)buf_pointer;
            printf("%d %d %d %d %d\n", second_packet[0], second_packet[1], second_packet[2], second_packet[3], second_packet[4]);
            buf_pointer += 12;
        }
        else
        {
            float *vec3 = (float *)buf_pointer;
            printf("%f %f %f\n", vec3[0], vec3[1], vec3[2]);
            buf_pointer += 12;
        }
    }
}

float vec3[3] = {0};
struct libtm_iMTQ_housekeeping_data imtq = {0};

void generate_random_test_data(int iterations)
{
    struct timespec ts = {.tv_sec = 0, .tv_nsec = 10000000};

    for (int sensor_id = 1; iterations > 0; --iterations)
    {
        if (sensor_id > N_SENSORS)
        {
            sensor_id = 1;
        }
        int random = rand() % 5;

        struct timeval tv;
        gettimeofday(&tv, NULL);
        uint64_t timestamp_us = tv.tv_sec * 1000000 + tv.tv_usec;

        struct libtm_packet packet = {0};
        packet.header.sensor_id = sensor_id;
        packet.header.timestamp_us = timestamp_us;

        if (sensor_id == IMTQ_HOUSEKEEPING)
        {
            imtq.dig_volt = 1 + random;
            imtq.analog_volt = 2 + random;
            imtq.dig_curr = 3 + random;
            imtq.analog_curr = 4 + random;
            imtq.meas_curr_x = 5 + random;
            imtq.meas_curr_y = 6 + random;
            imtq.meas_curr_z = 7 + random;
            imtq.coil_temp_x = 8 + random;
            imtq.coil_temp_y = 9 + random;
            imtq.coil_temp_z = 10 + random;
            imtq.MCU_temp = 11 + random;

            packet.data = &imtq;
            packet.data_size = IMTQ_HK_SIZE;
        }
        else
        {
            vec3[0] = 1.0 + random;
            vec3[1] = 2.0 + random;
            vec3[2] = 3.0 + random;

            packet.data = &vec3;
            packet.data_size = sizeof(vec3);
        }

        ++sensor_id;
        printf("Debug data generation iteration %d\n", iterations);
        libtm_write_sensor_data(packet);
        nanosleep(&ts, NULL);
    }
}

int main()
{
    srand(time(NULL));
    uint64_t file_ring_buffer[5] = {0};
    uint8_t active_file_index = 0;

    uint64_t sensor_write_intervals_us[N_SENSORS] = {
        40000,
        40000,
        40000,
        40000,
        40000,
        40000,
        40000,
        40000,
        40000,
    };

    libtm_init(file_ring_buffer, &active_file_index, sensor_write_intervals_us);
    generate_random_test_data(256);
    _print_file(256);

    return 0;
}

#endif