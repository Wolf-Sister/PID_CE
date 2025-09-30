#include "PID.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


//增量式PID初始化
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_sum = 0.0f;
}
// 增量式PID计算
float PID_Compute(PID_Controller *pid, float measured_value) {
    float error, delta_output;
    float delta_p, delta_i, delta_d;
    static float prev_error = 0.0f, prev_prev_error = 0.0f;
    // 计算当前误差
    error = pid->setpoint - measured_value;
    // 增量式PID公式
    delta_p = pid->Kp * (error - prev_error);
    delta_i = pid->Ki * error;
    delta_d = pid->Kd * (error - 2 * prev_error + prev_prev_error);
    delta_output = delta_p + delta_i + delta_d;
    // 更新误差历史
    prev_prev_error = prev_error;
    prev_error = error;
    return delta_output;
}
// 修改PID参数
void PID_SetTunings(PID_Controller *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}
