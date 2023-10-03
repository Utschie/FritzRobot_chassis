#ifndef __IMU_H__
#define __IMU_H__


#ifdef __cplusplus
extern "C" {
#endif

#include "mytype.h"
#define MPU_DELAY(x) HAL_Delay(x)

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data_t;

typedef struct
{
	float ax;
	float ay;
	float az;

	float temp;

	float wx; 
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} imu_t;

extern mpu_data_t mpu_data;
extern imu_t      imu;

uint8_t mpu_device_init(void);
void mpu_get_data(void);
void mpu_offset_call(void);	
	
	
#ifdef __cplusplus
}
#endif

#endif /* __IMU__ */