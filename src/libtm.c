#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "../inc/libtm.h"

#define DO_DEBUG 0

#if DO_DEBUG
#define DEBUG(string) string
#else
#define DEBUG(string)
#endif

#define FILE_PATH_BASE "telem/file"
#define MAX_FILESIZE_BYTES 50961960 // 48.6 MB
#define MAX_FILEAMOUNT_ALLOWED 32

#define VEC3_SIZE 12
#define VEC3_PACKET_SIZE 21

#define IMTQ_SIZE 22
#define IMTQ_PACKET_SIZE 42

#define LIBTM_HEADER_SIZE 9

struct file_write_buffer
{
    uint16_t remaining_bytes;
    uint8_t buffer[256];
};

static struct file_write_buffer _file_write_buffer = {0};

static uint64_t *_sensor_write_intervals_us;
static uint64_t _sensor_last_write_time_us[IMTQ_HOUSEKEEPING] = {0};

static uint64_t *_file_ring_buffer;
static uint8_t *_active_file_index;
static uint32_t _active_file_remaining_bytes;
static char _active_file_path[16] = {0};
static int _active_file_fd;

void libtm_set_active_file(void)
{
    close(_active_file_fd);
    sprintf(_active_file_path, "%s%d", FILE_PATH_BASE, *_active_file_index);
    _active_file_fd = open(_active_file_path, O_WRONLY | O_CREAT | O_TRUNC);
    _active_file_remaining_bytes = MAX_FILESIZE_BYTES;
}

void _write_buffer_to_file(void)
{
    write(_active_file_fd, _file_write_buffer.buffer, 256 - _file_write_buffer.remaining_bytes);
    memset(&_file_write_buffer, 0, sizeof(_file_write_buffer));
    _file_write_buffer.remaining_bytes = 256;
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
    if (time_delta > _sensor_write_intervals_us[sensor_packet.header.sensor_id - 1])
    {
        return; // Note enough time has passed since last write
    }

    if (sensor_packet.header.sensor_id < IMTQ_HOUSEKEEPING &&
        _file_write_buffer.remaining_bytes < VEC3_PACKET_SIZE)
    {
        _write_buffer_to_file();
        _file_ring_buffer[*_active_file_index] = sensor_packet.header.timestamp_us;
    }
    else if (sensor_packet.header.sensor_id == IMTQ_HOUSEKEEPING &&
             _file_write_buffer.remaining_bytes < IMTQ_PACKET_SIZE)
    {
        _write_buffer_to_file();
        _file_ring_buffer[*_active_file_index] = sensor_packet.header.timestamp_us;
    }

    _sensor_last_write_time_us[sensor_packet.header.sensor_id - 1] = sensor_packet.header.timestamp_us;

    if (sensor_packet.header.sensor_id == IMTQ_HOUSEKEEPING)
    {
        // Write header and first half of data
        memcpy(&_file_write_buffer.buffer[256 - _file_write_buffer.remaining_bytes], &sensor_packet, VEC3_PACKET_SIZE);
        _file_write_buffer.remaining_bytes -= VEC3_PACKET_SIZE;

        // Write header for second packet
        memcpy(&_file_write_buffer.buffer[256 - _file_write_buffer.remaining_bytes], &sensor_packet, sizeof(struct libtm_packet_header));
        _file_write_buffer.remaining_bytes -= sizeof(struct libtm_packet_header);

        // Write second half of data
        memcpy(&_file_write_buffer.buffer[256 - _file_write_buffer.remaining_bytes],
               sensor_packet.data + VEC3_SIZE, VEC3_SIZE);
        _file_write_buffer.remaining_bytes -= VEC3_SIZE;

        // Add padding
        memset(&_file_write_buffer.buffer[256 - _file_write_buffer.remaining_bytes], 0, 2);
        _file_write_buffer.remaining_bytes -= 2;
    }
    else
    {
        memcpy(&_file_write_buffer.buffer[256 - _file_write_buffer.remaining_bytes], &sensor_packet, VEC3_PACKET_SIZE);
        _file_write_buffer.remaining_bytes -= VEC3_PACKET_SIZE;
    }

    if (_active_file_remaining_bytes < IMTQ_PACKET_SIZE)
    {
        ++(*_active_file_index);
        if (*_active_file_index > MAX_FILEAMOUNT_ALLOWED)
        {
            *_active_file_index = 0;
        }

        libtm_set_active_file();
    }
}

void libtm_init(uint64_t *file_ring_buffer, uint8_t *active_file_index)
{
    _active_file_index = active_file_index;
    _file_ring_buffer = file_ring_buffer;
    libtm_set_active_file();
    _active_file_fd = open(_active_file_path, O_WRONLY | O_CREAT | O_APPEND);
}

int main()
{
    return 0;
}