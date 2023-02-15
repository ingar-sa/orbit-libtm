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

// 1 + 4 = 5 bytes
struct __attribute__((packed)) libtm_packet_header {
    uint8_t sensor_id;
    struct timeval timestamp;
};

// 5 + 4 + 12 = 21 bytes (for vec3)
// 5 + 4 + 22 = 31 bytes (for iMTQ housekeeping)
struct __attribute__((packed)) libtm_packet {
    struct libtm_packet_header header;
    uint32_t size; // Size of packet
    void *data;
};

#endif // LIBTM_H