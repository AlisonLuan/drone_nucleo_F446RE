#include "drone_control.h"
#include <string.h>
#include <stdio.h>

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
extern uint8_t imu_index;
extern uint8_t imu_count;

extern volatile uint8_t control_enabled;

static uint32_t last_button_time = 0;

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

#define PWM_MAX_STEP 1
#define Kp 0.2f

void SoftStartPWM(uint32_t *current, const uint32_t target)
{
    uint32_t desired = target;

    /* Optional branch to ramp down smoothly when control is disabled */
    if (!control_enabled)
    {
        desired = 0;
    }

    int32_t error = (int32_t)desired - (int32_t)(*current);
    int32_t step = (int32_t)(Kp * error);

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
        if (now - last_button_time > 200)
        {
            control_enabled = !control_enabled;
            if (control_enabled)
                Debug_Send("Control Enabled\r\n");
            else
                Debug_Send("Control Disabled\r\n");

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
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

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

