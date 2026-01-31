#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <stdint.h>
#include "stm32f1xx_hal.h"   // TIM_HandleTypeDef, GPIO_TypeDef 等类型在这里

/* 控制模式枚举
 * 命名理由：
 * - MOTOR_ 前缀：表明属于电机控制模块
 * - MODE_：表明是“模式”
 */
typedef enum
{
    MOTOR_MODE_OPEN_LOOP = 0,      /* 开环：外部直接给 PWM 输出（不做 PID） */
    MOTOR_MODE_SPEED_PID_INC,      /* 速度环：增量式 PID（贴近你 Speed_Output） */
    MOTOR_MODE_POSITION_PID,       /* 位置环：位置式 PID（贴近你 Position_Output） */
    MOTOR_MODE_POS_SPD_CASCADE      /* 双环：外位置环 + 内速度环（贴近你 Position_Speed_Output） */
} MotorCtrl_Mode_t;

/* ===== 结构 3 核心：不透明类型声明（外部看不到字段） ===== */
typedef struct MotorCtrl MotorCtrl_t;

/* 电机硬件配置（对外可见）
 * 命名理由：
 * - Config：配置
 * - 每个字段都明确“用途+单位/语义”
 */
typedef struct
{
    TIM_HandleTypeDef *pwm_tim;    /* PWM 输出用的定时器句柄，例如 &htim3 */
    uint32_t pwm_channel;          /* PWM 输出通道，例如 TIM_CHANNEL_1 */

    GPIO_TypeDef *in1_port;        /* 方向引脚 IN1 所在 GPIO 端口，例如 GPIOB */
    uint16_t      in1_pin;         /* 方向引脚 IN1 的 pin，例如 GPIO_PIN_0 */
    GPIO_TypeDef *in2_port;        /* 方向引脚 IN2 所在 GPIO 端口 */
    uint16_t      in2_pin;         /* 方向引脚 IN2 的 pin */

    TIM_HandleTypeDef *enc_tim;    /* 编码器计数定时器句柄，例如 &htim2（Encoder Mode） */

    float out_limit_percent;       /* 输出限幅（百分比），一般给 100.0f */
    float speed_target_limit;      /* 双环中“目标速度”限幅（单位：每次采样的脉冲增量），你原来是 200 */
    float pos_integral_limit;      /* 位置环积分限幅，你原来用的是 ±500 */
} MotorCtrl_Config_t;

/* 运行数据快照（对外可读，用于串口打印/上位机监控）
 * 命名理由：
 * - Telemetry：遥测/监控数据
 * - 字段名贴近你原全局变量，便于迁移
 */
typedef struct
{
    float target;              /* 目标：速度模式=目标速度(脉冲/采样)，位置模式=目标位置(脉冲累计) */
    float actual;              /* 实际：速度模式=当前速度(脉冲/采样)，位置模式=当前位置(脉冲累计) */
    float out_percent;         /* PID 输出：PWM 百分比（-out_limit ~ +out_limit） */

    /* 双环常用观测量 */
    float current_speed;       /* 当前速度（脉冲/采样） */
    float position_actual;     /* 累积位置（脉冲累计） */
    float speed_target;        /* 双环外环输出的目标速度（脉冲/采样） */
    float pos_integral;        /* 双环外环积分项（便于你原 UART 打印） */
} MotorCtrl_Telemetry_t;

/* ===== 对外 API ===== */

/* 获取模块内部电机对象句柄（结构 3 常用做法：内部静态对象池）
 * - index：第几个电机对象（0/1/...），本例内部池大小见 .c
 */
MotorCtrl_t* MotorCtrl_Get(uint8_t index);

/* 初始化电机对象：保存硬件配置、清零内部状态
 * - 返回 HAL_OK 表示成功
 */
HAL_StatusTypeDef MotorCtrl_Init(MotorCtrl_t *m, const MotorCtrl_Config_t *cfg);

/* 设置控制模式 */
void MotorCtrl_SetMode(MotorCtrl_t *m, MotorCtrl_Mode_t mode);

/* 设置目标（Target）
 * - 速度模式：target = 目标速度（单位：脉冲/采样周期）
 * - 位置模式：target = 目标位置（单位：脉冲累计）
 */
void MotorCtrl_SetTarget(MotorCtrl_t *m, float target);

/* 设置 PID 参数（分别对应你原来的三套参数）
 * 命名理由：SetXxxPID 直观表达“设置哪个 PID”
 */
void MotorCtrl_SetSpeedPID(MotorCtrl_t *m, float kp, float ki, float kd);
void MotorCtrl_SetPositionPID(MotorCtrl_t *m, float kp, float ki, float kd);
void MotorCtrl_SetCascadePID(MotorCtrl_t *m,
                             float pos_kp, float pos_ki, float pos_kd,
                             float spd_kp, float spd_ki, float spd_kd);

/* 更新控制器（计算出 out_percent）
 * 调用时机建议：由你原来的 tim_read 标志控制（到采样周期再调用）
 * - 返回：本次计算得到的 PWM 输出百分比
 */
float MotorCtrl_Update(MotorCtrl_t *m);

/* 将 out_percent 应用到硬件（写 PWM CCR + 方向 GPIO）
 * 调用时机建议：由你原来的 tim_SetPWM 标志控制（到输出周期再调用）
 */
void MotorCtrl_ApplyOutput(MotorCtrl_t *m, float out_percent);

/* 读取遥测数据（用于串口打印）
 * - out：由调用者提供的输出结构体指针
 */
void MotorCtrl_GetTelemetry(const MotorCtrl_t *m, MotorCtrl_Telemetry_t *out);

#endif /* MOTOR_CTRL_H */
