#include "mpu6050.h"

/* USER CODE BEGIN 0 */
#define MPU6050_ADDR            (0x68 << 1)
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	uint8_t data;

	/* Wake up the sensor */
	data = 0;
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	/* Set sample rate to 1kHz/(1+0) = 1kHz */
	data = 0;
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	/* Set DLPF to 42Hz */
	data = 0x03;
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_CONFIG, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	/* Set gyro full scale to +-500 deg/s */
	data = 0x08;
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	/* Set accel full scale to +-4g */
	data = 0x08;
	if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadAll(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
	uint8_t buf[14];
	if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

	data->accel_x = (int16_t)(buf[0] << 8 | buf[1]);
	data->accel_y = (int16_t)(buf[2] << 8 | buf[3]);
	data->accel_z = (int16_t)(buf[4] << 8 | buf[5]);
	data->temp    = (int16_t)(buf[6] << 8 | buf[7]);
	data->gyro_x  = (int16_t)(buf[8] << 8 | buf[9]);
	data->gyro_y  = (int16_t)(buf[10] << 8 | buf[11]);
	data->gyro_z  = (int16_t)(buf[12] << 8 | buf[13]);

	return HAL_OK;
}

void MPU6050_ConvertToPhysical(const MPU6050_Data_t *raw, MPU6050_Physical_t *out)
{
	const float accel_lsb = 8192.0f;   /* LSB/g for +-4g */
	const float gyro_lsb  = 65.5f;     /* LSB/(deg/s) for +-500dps */
	const float g = 9.80665f;          /* m/s^2 per g */

	out->accel_x = (raw->accel_x / accel_lsb) * g;
	out->accel_y = (raw->accel_y / accel_lsb) * g;
	out->accel_z = (raw->accel_z / accel_lsb) * g;

	out->gyro_x  = raw->gyro_x / gyro_lsb;
	out->gyro_y  = raw->gyro_y / gyro_lsb;
	out->gyro_z  = raw->gyro_z / gyro_lsb;

	out->temp = (raw->temp / 340.0f) + 36.53f;
}
/* USER CODE END 1 */
