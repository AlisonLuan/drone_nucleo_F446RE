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
/* USER CODE END ET */

/* USER CODE BEGIN EFP */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
