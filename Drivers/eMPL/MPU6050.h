#ifndef __MPU6050_H
#define __MPU6050_H

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "STM32_I2C.h"

int MPU_init(void);								// 初始化MPU6050模块，返回0表示失败
int MPU_getdata(void);						// 获取六轴数据和姿态角，更新全局变量
void MPU6050_ReturnTemp(float*Temperature);	// 读取MPU6050器件温度

extern float q0, q1, q2, q3;			// 四元数
extern __IO float fAX, fAY, fAZ;				// 三个姿态角（pitch俯仰角, roll滚转角, yaw偏航角）
extern __IO short ax, ay, az, gx, gy, gz;	// 六轴数据

/* backward-compatible API for balance_control.c */
typedef struct {
    short ax, ay, az;
    short gx, gy, gz;
} mpu6050_data_t;

void mpu6050_init(void);
int  mpu6050_read_data(mpu6050_data_t *data);
float mpu6050_get_pitch_accel(const mpu6050_data_t *data);
float mpu6050_get_pitch_gyro_rate(const mpu6050_data_t *data);

#endif
