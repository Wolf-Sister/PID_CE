#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
#include "arm_math.h"
#include "usart.h"

extern uint8_t uartTxReady;

typedef struct {
    float Kp;       // 比例系数
    float Ki;       // 积分系数
    float Kd;       // 微分系数
    float setpoint; // 目标值
    float integral; // 积分值
    float prev_error; // 上一次误差
    float prev_prev_error; // 新增的成员
    float output_sum;      // 输出和
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd);
float PID_Compute(PID_Controller *pid, float measured_value);
void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd);

#ifdef __cplusplus
}
#endif

#endif
