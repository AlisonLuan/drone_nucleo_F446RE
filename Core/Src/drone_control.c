#include "drone_control.h"
#include <string.h>
#include <stdio.h>

/* USER CODE BEGIN 0 */
#define ACCEL_LP_ALPHA   0.2f
#define GYRO_HP_ALPHA    0.8f
#define PWM_RAMP_KP      0.2f
#define PWM_MAX_STEP     1
#define SHORT_DELAY_CYCLES 1000
/* USER CODE END 0 */

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

extern volatile uint32_t PWM_D9;
extern volatile uint32_t PWM_D6;
extern volatile uint32_t PWM_D5;
extern volatile uint32_t PWM_D3;

extern volatile uint32_t PWM_D9_Target;
extern volatile uint32_t PWM_D6_Target;
extern volatile uint32_t PWM_D5_Target;
extern volatile uint32_t PWM_D3_Target;

extern MPU6050_Physical_t imu_avg;
extern MPU6050_Physical_t imu_window[];
extern MPU6050_Physical_t imu_sum;
extern MPU6050_Physical_t imu_bias;
extern uint8_t imu_index;
extern uint8_t imu_count;

extern volatile uint8_t control_enabled;
extern volatile uint8_t button_pressed;
extern uint32_t button_press_time;

static uint32_t last_button_time = 0;

/* Simple cycle-based delay used during I2C bus recovery */
static void ShortDelay(void)
{
        for (volatile uint32_t i = 0; i < SHORT_DELAY_CYCLES; ++i)
        {
                __NOP();
        }
}

/* Filter state for accelerometer low-pass and gyroscope high-pass */
static MPU6050_Physical_t accel_lp = {0};
static MPU6050_Physical_t gyro_hp = {0};
static MPU6050_Physical_t gyro_prev_in = {0};

void IMU_Filter(const MPU6050_Physical_t *in, MPU6050_Physical_t *out)
{
        /* low-pass / high-pass smoothing using macros */
        out->accel_x = ACCEL_LP_ALPHA * (in->accel_x - imu_bias.accel_x) +
                       (1.0f - ACCEL_LP_ALPHA) * accel_lp.accel_x;
        out->accel_y = ACCEL_LP_ALPHA * (in->accel_y - imu_bias.accel_y) +
                       (1.0f - ACCEL_LP_ALPHA) * accel_lp.accel_y;
        out->accel_z = ACCEL_LP_ALPHA * (in->accel_z - imu_bias.accel_z) +
                       (1.0f - ACCEL_LP_ALPHA) * accel_lp.accel_z;
        accel_lp = *out;

        out->gyro_x = GYRO_HP_ALPHA * (gyro_hp.gyro_x + in->gyro_x - gyro_prev_in.gyro_x);
        out->gyro_y = GYRO_HP_ALPHA * (gyro_hp.gyro_y + in->gyro_y - gyro_prev_in.gyro_y);
        out->gyro_z = GYRO_HP_ALPHA * (gyro_hp.gyro_z + in->gyro_z - gyro_prev_in.gyro_z);

        gyro_prev_in = *in;
        gyro_hp     = *out;
        out->temp   = in->temp;
}

void Debug_Send(const char *msg)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void IMU_UpdateAverage(const MPU6050_Physical_t *sample)
{
	if (imu_count < IMU_WINDOW_SIZE)
	{
		imu_window[imu_index] = *sample;
		imu_sum.accel_x += sample->accel_x;
		imu_sum.accel_y += sample->accel_y;
		imu_sum.accel_z += sample->accel_z;
		imu_sum.gyro_x  += sample->gyro_x;
		imu_sum.gyro_y  += sample->gyro_y;
		imu_sum.gyro_z  += sample->gyro_z;
		imu_sum.temp    += sample->temp;
		imu_count++;
	}
	else
	{
		imu_sum.accel_x -= imu_window[imu_index].accel_x;
		imu_sum.accel_y -= imu_window[imu_index].accel_y;
		imu_sum.accel_z -= imu_window[imu_index].accel_z;
		imu_sum.gyro_x  -= imu_window[imu_index].gyro_x;
		imu_sum.gyro_y  -= imu_window[imu_index].gyro_y;
		imu_sum.gyro_z  -= imu_window[imu_index].gyro_z;
		imu_sum.temp    -= imu_window[imu_index].temp;

		imu_window[imu_index] = *sample;

		imu_sum.accel_x += sample->accel_x;
		imu_sum.accel_y += sample->accel_y;
		imu_sum.accel_z += sample->accel_z;
		imu_sum.gyro_x  += sample->gyro_x;
		imu_sum.gyro_y  += sample->gyro_y;
		imu_sum.gyro_z  += sample->gyro_z;
		imu_sum.temp    += sample->temp;
	}

	imu_index = (imu_index + 1) % IMU_WINDOW_SIZE;
	float div = (float)imu_count;

	imu_avg.accel_x = imu_sum.accel_x / div;
	imu_avg.accel_y = imu_sum.accel_y / div;
	imu_avg.accel_z = imu_sum.accel_z / div;
	imu_avg.gyro_x  = imu_sum.gyro_x  / div;
	imu_avg.gyro_y  = imu_sum.gyro_y  / div;
	imu_avg.gyro_z  = imu_sum.gyro_z  / div;
        imu_avg.temp    = imu_sum.temp    / div;
}

float PID_Update(PID_t *pid, float error, float dt)
{
        pid->integral += error * dt;
        float derivative = (error - pid->last_err) / dt;
        pid->last_err = error;
        return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}


void SoftStartPWM(uint32_t *current, const uint32_t target)
{
        uint32_t desired = target;

        /* Optional branch to ramp down smoothly when control is disabled */
        if (!control_enabled)
        {
                memset(&accel_lp, 0, sizeof(accel_lp));
                memset(&gyro_hp,  0, sizeof(gyro_hp));
                desired = 0;
        }

	int32_t error = (int32_t)desired - (int32_t)(*current);
        int32_t step = (int32_t)(PWM_RAMP_KP * error);

	if (step > PWM_MAX_STEP) step = PWM_MAX_STEP;
	else if (step < -PWM_MAX_STEP) step = -PWM_MAX_STEP;

	*current += step;

	if ((step > 0 && *current > desired) || (step < 0 && *current < desired))
	{
		*current = desired;
	}
}

void UpdatePWM(void)
{
	SoftStartPWM((uint32_t*)&PWM_D9, PWM_D9_Target);
	SoftStartPWM((uint32_t*)&PWM_D6, PWM_D6_Target);
	SoftStartPWM((uint32_t*)&PWM_D5, PWM_D5_Target);
	SoftStartPWM((uint32_t*)&PWM_D3, PWM_D3_Target);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM_D9);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_D6);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_D5);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_D3);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		uint32_t now = HAL_GetTick();
		if (now - last_button_time > BUTTON_DEBOUNCE_MS)
		{
			button_pressed = 1;
			button_press_time = now;
			last_button_time = now;
		}
	}
}

void I2C_ResetBus(void)
{
	HAL_I2C_DeInit(&hi2c1);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	for (int i = 0; i < 9; i++)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		ShortDelay();
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		ShortDelay();
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
	ShortDelay();

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
}

