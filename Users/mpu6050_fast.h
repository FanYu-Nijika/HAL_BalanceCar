#ifndef __MPU6050_FAST_H
#define __MPU6050_FAST_H

#include <stdint.h>

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu6050_fast_data_t;

int mpu6050_fast_init(void);
int mpu6050_fast_read(mpu6050_fast_data_t *data);
float mpu6050_fast_get_pitch_accel(const mpu6050_fast_data_t *data);
float mpu6050_fast_get_pitch_gyro_rate(const mpu6050_fast_data_t *data);

#endif
