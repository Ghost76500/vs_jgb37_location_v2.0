#include "pid.h"
#include "stm32f1xx_hal_uart.h"
#include "tim.h"
#include "encoder.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include <math.h>
#include "stdlib.h"

float Target = 0.0f, Actual = 0.0f, Out = 0.0f;
 float Kp = 0.5f, Ki = 0.06f, Kd = 0.0f; // jgb37-520-333RPM PID参数 Speed
// float Kp = 0.8f, Ki = 0.0f, Kd = 0.0f; // JGA25-370-200RPM PID参数 Speed
// float Kp = 0.09f, Ki = 0.0f, Kd = 0.0f; // JGB37-520-333RPM PID参数 Position
float Error0 = 0.0f, Error1 = 0.0f, Error2 = 0.0f, ErrorInt = 0.0f;
float motor_speed = 0; // 电机速度变量
float motor_position = 0; // 电机位置变量

// 双环控制专用参数
//float Position_Kp = 0.07f, Position_Ki = 0.0f, Position_Kd = 0.0f;  // 外环位置PID  响应稍慢，好用
float Position_Kp = 0.2f, Position_Ki = 0.0f, Position_Kd = 0.01f;  // 外环位置PID
float Speed_Kp = 0.5f, Speed_Ki = 0.06f, Speed_Kd = 0.0f;           // 内环速度PID
float Pos_Error0 = 0.0f, Pos_Error1 = 0.0f, Pos_Error2 = 0.0f;     // 位置环误差
float Spd_Error0 = 0.0f, Spd_Error1 = 0.0f, Spd_Error2 = 0.0f;     // 速度环误差
float Position_Actual = 0.0f;  // 累积位置
float Speed_Target = 0.0f;     // 速度环目标（由位置环输出）
float Current_Speed = 0.0f;    // 当前速度
float Pos_Integral_UART = 0.0f;  // 位置环积分项
float Pos_c = 0.0f; // 积分分离变量

char msg[100]; // 用于UART发送数据

// 增量式PID速度控制计算
float Speed_Output(void)
{
    if(tim_read == 0) return Out; // 未到读取时间，直接返回上次输出值
    tim_read = 0; // 清除标志位
    
    motor_speed = Read_Pulse_Count(&htim2);
    Actual = motor_speed;
    
    Error2 = Error1;
    Error1 = Error0;
    Error0 = Target - Actual;

    Out += Kp * (Error0 - Error1) + Ki * Error0 + Kd * (Error0 - 2 * Error1 + Error2);

    // 限幅处理
    if (Out > 100) {Out = 100;}
    if (Out < -100) {Out = -100;} // 定速控制

    return Out; // 返回输出值
}

// 位置式PID位置控制计算
float Position_Output(void)
{
    if(tim_read == 0) return Out; // 未到读取时间，直接返回上次输出值
    tim_read = 0; // 清除标志位
    
    motor_position = Read_Pulse_Count(&htim2);
    Actual += motor_position;
    
    Error1 = Error0;
    Error0 = Target - Actual;

    if (Ki != 0)
	{
		ErrorInt += Error0;
	}
	else
	{
		ErrorInt = 0;
	}

    Out = Kp * Error0 + Ki * ErrorInt + Kd * (Error0 - Error1);

    // 限幅处理
    if (Out > 100) {Out = 100;}
    if (Out < -100) {Out = -100;} // 增量式定位置控制

    
    return Out; // 返回输出值
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)Out);
}


// 外环位置环，内环速度环双环控制
float Position_Speed_Output(void)
{
    if(tim_read == 0) return Out; // 未到读取时间，直接返回上次输出值
    tim_read = 0; // 清除标志位
    
    // 读取电机编码器速度增量
    motor_speed = Read_Pulse_Count(&htim2);
    Current_Speed = motor_speed;
    // 累积为位置
    Position_Actual += motor_speed;
    
    // === 外环：位置环用位置式PID直接输出目标速度 ===
    Pos_Error2 = Pos_Error1;
    Pos_Error1 = Pos_Error0;
    Pos_Error0 = Target - Position_Actual;  // 目标位置 - 实际位置
    
    // 位置环改用位置式PID，直接输出目标速度（不累加）
    static float Pos_Integral = 0.0f;  // 位置环积分项
    Pos_Integral += Pos_Error0;
    
    // 积分限幅防止饱和
    if (Pos_Integral > 500.0f) Pos_Integral = 500.0f;
    if (Pos_Integral < -500.0f) Pos_Integral = -500.0f;
    Pos_Integral_UART = Pos_Integral; // 供UART打印使用

    Speed_Target = Position_Kp * Pos_Error0 
                 + Position_Ki * Pos_Integral 
                 + Position_Kd * (Pos_Error0 - Pos_Error1);
    
    // 限制目标速度范围（根据电机最大速度调整）
    if (Speed_Target > 200.0f) Speed_Target = 200.0f;
    if (Speed_Target < -200.0f) Speed_Target = -200.0f;
    
    // === 内环：速度环计算PWM输出 ===
    Spd_Error2 = Spd_Error1;
    Spd_Error1 = Spd_Error0;
    Spd_Error0 = Speed_Target - motor_speed;  // 目标速度 - 实际速度
    
    // 速度环增量式PID输出PWM占空比
    Out += Speed_Kp * (Spd_Error0 - Spd_Error1) 
         + Speed_Ki * Spd_Error0 
         + Speed_Kd * (Spd_Error0 - 2 * Spd_Error1 + Spd_Error2);
    
    // PWM输出限幅
    if (Out > 100.0f) Out = 100.0f;
    if (Out < -100.0f) Out = -100.0f;
    
    // 更新全局变量供UART打印
    Actual = Position_Actual;
    
   // if (motor_speed < 30) motor_speed = 0; // 死区处理
   // if (Current_Speed > 230 && Target > Actual && (Target - Actual) < 150) Out /= 5; // 靠近目标位置时减速

    return Out;
}

/*
 *  读取编码器速度并通过UART发送
 */
void Read_Encoder_Speed(void)
{
    if(tim_uart == 0) return; // 未到发送时间，直接返回
    
    // 检查UART是否空闲再发送数据，避免DMA冲突导致卡死
    if (huart1.gState == HAL_UART_STATE_READY)
    {
        sprintf(msg, "%f,%f,%f,%f,%f\r\n", Target, Actual, Out, Current_Speed, Pos_Integral_UART);
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg, strlen(msg));
    }

	tim_uart = 0; // 清除UART发送标志位
}   