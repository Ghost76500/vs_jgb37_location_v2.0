#ifndef __PID_H
#define __PID_H

#include "stm32f1xx_hal.h"

extern float Target, Actual, Out;
extern float Kp, Ki, Kd;
extern float Error0, Error1, Error2;
extern float motor_speed;
extern float motor_position;

// 双环控制参数
extern float Position_Kp, Position_Ki, Position_Kd;
extern float Speed_Kp, Speed_Ki, Speed_Kd;
extern float Position_Actual, Speed_Target;

float Speed_Output(void);
float Position_Output(void);
float Position_Speed_Output(void);  // 双环控制
void Read_Encoder_Speed(void);

extern char msg[100]; // 用于UART发送数据

#endif // __PID_H