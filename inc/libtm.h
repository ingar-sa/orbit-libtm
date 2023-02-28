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

struct __attribute__((packed)) libtm_packet_header {
    uint8_t sensor_id;
    uint64_t timestamp_us;
};

struct libtm_packet {
    struct libtm_packet_header header;
    void *data;
    uint8_t data_size;
};

/**
 *  @brief Writes a sensor packet to a buffer in libtm.
 *  The buffer is written to file if it is full.
 * 
 *  @param sensor_packet The sensor packet to write to the buffer.
*/
void libtm_write_sensor_data(struct libtm_packet sensor_packet);

/**
 *  @brief Initializes libtm. Sets the relevant variables and opens 
 *  the file indicated by active_file_index.
 * 
 *  @param file_ring_buffer A pointer to the file ring buffer.
 *  @param active_file_index A pointer to the active file index.
 *  @param sensor_write_intervals_us A pointer to the sensor write intervals.
*/
void libtm_init(uint64_t *file_ring_buffer, uint8_t *active_file_index, uint64_t *sensor_write_intervals_us);

#endif // LIBTM_H