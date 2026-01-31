#ifndef MOTOR_SIMPLE_H
#define MOTOR_SIMPLE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 电机控制模式：对应你现有三种控制函数 */
typedef enum
{
    MOTOR_MODE_SPEED_INC = 0,     // 增量式速度环（Speed_Output）
    MOTOR_MODE_POSITION_POS,      // 位置式位置环（Position_Output）
    MOTOR_MODE_POS_SPD_CASCADE    // 外位置环 + 内速度环（Position_Speed_Output）
} MotorMode_t;

/* 结构一：把你原来的所有“全局变量”收拢到一个结构体里
 * 命名 Motor_t：t 表示 type（类型）
 */
typedef struct
{
    /* ========== 硬件资源 ========== */
    TIM_HandleTypeDef *enc_tim;      // 编码器定时器（例如 &htim2）
    TIM_HandleTypeDef *pwm_tim;      // PWM 定时器（例如 &htim3）
    uint32_t pwm_ch;                 // PWM 通道（例如 TIM_CHANNEL_1）

    GPIO_TypeDef *in1_port;          // 方向引脚 1 的 GPIO 端口（例如 GPIOB）
    uint16_t      in1_pin;           // 方向引脚 1 的 pin（例如 GPIO_PIN_0）
    GPIO_TypeDef *in2_port;          // 方向引脚 2 的 GPIO 端口（例如 GPIOB）
    uint16_t      in2_pin;           // 方向引脚 2 的 pin（例如 GPIO_PIN_1）

    /* ========== 控制模式与基础量 ========== */
    MotorMode_t mode;                // 当前控制模式
    float Target;                    // 目标量（速度模式=目标速度；位置模式=目标位置）
    float Actual;                    // 实际量（速度模式=当前速度；位置模式=累计位置）
    float Out;                       // PID 输出（PWM百分比，-100~100）

    /* ========== 单环 PID（你原 Kp Ki Kd + Error0/1/2 + ErrorInt） ========== */
    float Kp, Ki, Kd;                // PID 参数（单环用）
    float Error0, Error1, Error2;    // 误差序列
    float ErrorInt;                  // 积分项（位置式 PID 用）

    /* ========== 速度/位置测量量（你原 motor_speed/motor_position） ========== */
    float motor_speed;               // 本次采样速度（脉冲增量/采样周期）
    float motor_position;            // 本次采样位置增量（脉冲增量）

    /* ========== 双环参数与中间变量（你原 Position_Kp... Speed_Kp... 等） ========== */
    float Position_Kp, Position_Ki, Position_Kd;  // 外环位置 PID
    float Speed_Kp, Speed_Ki, Speed_Kd;           // 内环速度 PID

    float Pos_Error0, Pos_Error1, Pos_Error2;     // 位置环误差
    float Spd_Error0, Spd_Error1, Spd_Error2;     // 速度环误差

    float Position_Actual;          // 累积位置（双环用）
    float Speed_Target;             // 外环输出的目标速度（双环用）
    float Current_Speed;            // 当前速度（双环用）
    float Pos_Integral;             // 外环积分项（双环用，替代你 static Pos_Integral）
    float Pos_Integral_UART;        // 给 UART 打印用的快照

    /* ========== 限幅 ========== */
    float out_limit;                // 输出限幅（一般 100）
    float speed_target_limit;       // 目标速度限幅（你原 200）
    float pos_integral_limit;       // 位置积分限幅（你原 500）

} Motor_t;

/* 结构一：全局实例公开，任何 .c 都可以用 g_motor */
extern Motor_t g_motor;

/* 基础接口 */
void Motor_InitSimple(Motor_t *m);
int16_t Motor_ReadPulseDelta(Motor_t *m);

float Motor_SpeedOutput(Motor_t *m);
float Motor_PositionOutput(Motor_t *m);
float Motor_PosSpdCascadeOutput(Motor_t *m);

void Motor_ApplyPWM(Motor_t *m, float out_percent);

#endif
