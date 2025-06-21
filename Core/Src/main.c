/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>
#include "mpu6050.h"
#include <stdio.h>
#include "drone_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_PWM        1000
#define MAX_PWM        2000
#define PID_KP_PITCH   0.7f
#define PID_KI_PITCH   0.02f
#define PID_KD_PITCH   0.01f

#define PID_KP_ROLL    0.7f
#define PID_KI_ROLL    0.02f
#define PID_KD_ROLL    0.01f

#define PID_KP_YAW     0.7f
#define PID_KI_YAW     0.02f
#define PID_KD_YAW     0.01f

#define BASE_THROTTLE  1200.0f
#define MAX_I2C_RETRIES 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t PWM_D9 = 0;
volatile uint32_t PWM_D6 = 0;
volatile uint32_t PWM_D5 = 0;
volatile uint32_t PWM_D3 = 0;

volatile uint32_t PWM_D9_Target = 0;
volatile uint32_t PWM_D6_Target = 0;
volatile uint32_t PWM_D5_Target = 0;
volatile uint32_t PWM_D3_Target = 0;

MPU6050_Data_t imu_data;
MPU6050_Physical_t imu_phys;
MPU6050_Physical_t imu_filt;
MPU6050_Physical_t imu_avg;
MPU6050_Physical_t imu_window[IMU_WINDOW_SIZE];
MPU6050_Physical_t imu_sum;
uint8_t imu_index = 0;
uint8_t imu_count = 0;

volatile uint8_t control_enabled = 0;
volatile ControlState control_state = CONTROL_DISARMED;
volatile uint8_t button_pressed = 0;
uint32_t button_press_time = 0;
float target_pitch = 0.0f;
float target_roll  = 0.0f;
float target_yaw   = 0.0f;
float throttle_base = BASE_THROTTLE;
volatile float debug_throttle = -1.0f; /* Allows throttle override via debugger */

float last_error_pitch = 0.0f;
float integral_pitch   = 0.0f;
float last_error_roll  = 0.0f;
float integral_roll    = 0.0f;
float last_error_yaw   = 0.0f;
float integral_yaw     = 0.0f;
uint32_t last_pid_time = 0;
uint8_t i2c_retry_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void UpdatePWM(void);
void Debug_Send(const char *msg);
void IMU_UpdateAverage(const MPU6050_Physical_t *sample);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void IMU_Filter(const MPU6050_Physical_t *in, MPU6050_Physical_t *out);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Functions implemented in drone_control.c */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	Debug_Send("System Init Complete\r\n");
	MPU6050_Init(&hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
        uint32_t lastBlink = 0;
        last_pid_time = HAL_GetTick();
        while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		uint32_t now = HAL_GetTick();

		// Toggle LED every 500 ms without blocking
                if (now - lastBlink >= 500)
                {
                        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                        lastBlink = now;
                }

                if (button_pressed && HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET)
                {
                        uint32_t duration = now - button_press_time;
                        button_pressed = 0;
                        if (duration >= BUTTON_LONG_PRESS_MS)
                        {
                                Debug_Send("Calibration\r\n");
                                integral_pitch = 0.0f;
                                integral_roll = 0.0f;
                                integral_yaw = 0.0f;
                                last_error_pitch = 0.0f;
                                last_error_roll = 0.0f;
                                last_error_yaw = 0.0f;
                                control_state = CONTROL_DISARMED;
                        }
                        else if (duration >= BUTTON_DEBOUNCE_MS)
                        {
                                if (control_state == CONTROL_ARMED)
                                {
                                        control_state = CONTROL_DISARMED;
                                        Debug_Send("Disarmed\r\n");
                                }
                                else
                                {
                                        control_state = CONTROL_ARMED;
                                        Debug_Send("Armed\r\n");
                                }
                        }
                }
                control_enabled = (control_state == CONTROL_ARMED);

                if (MPU6050_ReadAll(&hi2c1, &imu_data) != HAL_OK || HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_NONE)
                {
                        char buf[40];
                        snprintf(buf, sizeof(buf), "I2C error:%lu\r\n", HAL_I2C_GetError(&hi2c1));
                        Debug_Send(buf);
                        PWM_D9_Target = 0;
                        PWM_D6_Target = 0;
                        PWM_D5_Target = 0;
                        PWM_D3_Target = 0;
                        UpdatePWM();
                        I2C_ResetBus();
                        if (++i2c_retry_count > MAX_I2C_RETRIES)
                        {
                                Error_Handler();
                        }
                        continue;
                }
                else
                {
                        i2c_retry_count = 0;
                }
                MPU6050_ConvertToPhysical(&imu_data, &imu_phys);
                IMU_Filter(&imu_phys, &imu_filt);
                IMU_UpdateAverage(&imu_filt);

                float pitch = atanf(imu_avg.accel_y /
                                sqrtf(imu_avg.accel_x * imu_avg.accel_x +
                                                imu_avg.accel_z * imu_avg.accel_z)) * 180.0f / M_PI;
                float roll  = atanf(-imu_avg.accel_x / imu_avg.accel_z) * 180.0f / M_PI;

                float dt = (now - last_pid_time) / 1000.0f;
                last_pid_time = now;
                if (dt <= 0.0f) dt = 0.001f;

                float error_pitch = target_pitch - pitch;
                integral_pitch += error_pitch * dt;
                float derivative_pitch = (error_pitch - last_error_pitch) / dt;
                last_error_pitch = error_pitch;
                float output_pitch = PID_KP_PITCH * error_pitch +
                                    PID_KI_PITCH * integral_pitch +
                                    PID_KD_PITCH * derivative_pitch;

                float error_roll  = target_roll - roll;
                integral_roll += error_roll * dt;
                float derivative_roll = (error_roll - last_error_roll) / dt;
                last_error_roll = error_roll;
                float output_roll  = PID_KP_ROLL * error_roll +
                                   PID_KI_ROLL * integral_roll +
                                   PID_KD_ROLL * derivative_roll;

                float throttle = throttle_base;
                if (debug_throttle >= 0.0f)
                {
                        throttle = debug_throttle;
                }

		float m1 = throttle + output_pitch + output_roll;
		float m2 = throttle + output_pitch - output_roll;
		float m3 = throttle - output_pitch + output_roll;
		float m4 = throttle - output_pitch - output_roll;

		if (m1 > MAX_PWM) m1 = MAX_PWM; else if (m1 < MIN_PWM) m1 = MIN_PWM;
		if (m2 > MAX_PWM) m2 = MAX_PWM; else if (m2 < MIN_PWM) m2 = MIN_PWM;
		if (m3 > MAX_PWM) m3 = MAX_PWM; else if (m3 < MIN_PWM) m3 = MIN_PWM;
		if (m4 > MAX_PWM) m4 = MAX_PWM; else if (m4 < MIN_PWM) m4 = MIN_PWM;

		if(control_enabled)
		{
		PWM_D9_Target = (uint32_t)m1;
		PWM_D6_Target = (uint32_t)m2;
		PWM_D5_Target = (uint32_t)m3;
		PWM_D3_Target = (uint32_t)m4;
		}
		else
		{
			PWM_D9_Target = 0;
					PWM_D6_Target = 0;
					PWM_D5_Target = 0;
					PWM_D3_Target = 0;
		}

		UpdatePWM(); // This runs as fast as possible
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* IMU_UpdateAverage moved to drone_control.c */

#define PWM_MAX_STEP 1  // passo máximo permitido por ciclo
#define Kp 0.2f            // ganho proporcional (ajuste conforme necessário)

/* SoftStartPWM moved to drone_control.c */


/* UpdatePWM moved to drone_control.c */

/* HAL_GPIO_EXTI_Callback moved to drone_control.c */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	Debug_Send("Error_Handler\r\n");
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
