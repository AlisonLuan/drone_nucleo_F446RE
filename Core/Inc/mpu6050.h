#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN ET */
/**
 * @brief Raw sensor data structure
 */
typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t temp;
} MPU6050_Data_t;

/**
 * @brief Physical units for sensor data
 */
typedef struct {
	float accel_x; /* m/s^2 */
	float accel_y; /* m/s^2 */
	float accel_z; /* m/s^2 */
	float gyro_x;  /* deg/s */
	float gyro_y;  /* deg/s */
	float gyro_z;  /* deg/s */
	float temp;    /* Celsius */
} MPU6050_Physical_t;
/* USER CODE END ET */

/* USER CODE BEGIN EFP */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);
void MPU6050_ConvertToPhysical(const MPU6050_Data_t *raw, MPU6050_Physical_t *out);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
