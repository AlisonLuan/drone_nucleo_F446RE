#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "main.h"

#define IMU_WINDOW_SIZE 10

typedef struct {
    float kp;
    float ki;
    float kd;
    float last_err;
    float integral;
} PID_t;

float PID_Update(PID_t *pid, float error, float dt);


void Debug_Send(const char *msg);
void SoftStartPWM(uint32_t *current, const uint32_t target);
void UpdatePWM(void);
void IMU_UpdateAverage(const MPU6050_Physical_t *sample);
void IMU_Filter(const MPU6050_Physical_t *in, MPU6050_Physical_t *out);
void I2C_ResetBus(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* DRONE_CONTROL_H */
