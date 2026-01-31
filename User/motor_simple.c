#include "motor_simple.h"
#include <math.h>

/* 结构一：全局对象的“定义”只能在一个 .c 里出现一次 */
Motor_t g_motor;

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 初始化：把默认参数、限幅、模式设好
 * 你也可以在 main 里初始化硬件句柄后，再调用这个
 */
void Motor_InitSimple(Motor_t *m)
{
    if (!m) return;

    m->mode = MOTOR_MODE_POS_SPD_CASCADE;

    m->Target = 0.0f;
    m->Actual = 0.0f;
    m->Out    = 0.0f;

    /* 单环 PID 默认值（对应你原 Speed Kp/Ki/Kd 的那组） */
    m->Kp = 0.5f;
    m->Ki = 0.06f;
    m->Kd = 0.0f;

    m->Error0 = m->Error1 = m->Error2 = 0.0f;
    m->ErrorInt = 0.0f;

    /* 双环默认值（对应你原参数） */
    m->Position_Kp = 0.2f;
    m->Position_Ki = 0.0f;
    m->Position_Kd = 0.01f;

    m->Speed_Kp = 0.5f;
    m->Speed_Ki = 0.06f;
    m->Speed_Kd = 0.0f;

    m->Pos_Error0 = m->Pos_Error1 = m->Pos_Error2 = 0.0f;
    m->Spd_Error0 = m->Spd_Error1 = m->Spd_Error2 = 0.0f;

    m->Position_Actual = 0.0f;
    m->Speed_Target = 0.0f;
    m->Current_Speed = 0.0f;
    m->Pos_Integral = 0.0f;
    m->Pos_Integral_UART = 0.0f;

    m->out_limit = 100.0f;
    m->speed_target_limit = 200.0f;
    m->pos_integral_limit = 500.0f;
}

/* 读取编码器增量（并清零），等价于你原 Read_Pulse_Count(&htim2) */
int16_t Motor_ReadPulseDelta(Motor_t *m)
{
    if (!m || !m->enc_tim) return 0;

    int16_t temp = (int16_t)__HAL_TIM_GetCounter(m->enc_tim);
    __HAL_TIM_SetCounter(m->enc_tim, 0);
    return temp;
}

/* 增量式速度环 PID（对应你原 Speed_Output） */
float Motor_SpeedOutput(Motor_t *m)
{
    if (!m) return 0.0f;

    m->motor_speed = (float)Motor_ReadPulseDelta(m);
    m->Actual = m->motor_speed;

    m->Error2 = m->Error1;
    m->Error1 = m->Error0;
    m->Error0 = m->Target - m->Actual;

    m->Out += m->Kp * (m->Error0 - m->Error1)
            + m->Ki * (m->Error0)
            + m->Kd * (m->Error0 - 2.0f * m->Error1 + m->Error2);

    m->Out = clampf(m->Out, -m->out_limit, m->out_limit);
    return m->Out;
}

/* 位置式位置环 PID（对应你原 Position_Output） */
float Motor_PositionOutput(Motor_t *m)
{
    if (!m) return 0.0f;

    m->motor_position = (float)Motor_ReadPulseDelta(m);
    m->Actual += m->motor_position;   // 累计位置

    m->Error1 = m->Error0;
    m->Error0 = m->Target - m->Actual;

    if (m->Ki != 0.0f) m->ErrorInt += m->Error0;
    else m->ErrorInt = 0.0f;

    m->Out = m->Kp * m->Error0
           + m->Ki * m->ErrorInt
           + m->Kd * (m->Error0 - m->Error1);

    m->Out = clampf(m->Out, -m->out_limit, m->out_limit);
    return m->Out;
}

/* 双环：外位置环（位置式）输出速度目标 + 内速度环（增量式）输出 PWM
 * 对应你原 Position_Speed_Output（把 static Pos_Integral 变成 m->Pos_Integral）
 */
float Motor_PosSpdCascadeOutput(Motor_t *m)
{
    if (!m) return 0.0f;

    /* 读速度增量（脉冲/采样周期） */
    m->motor_speed = (float)Motor_ReadPulseDelta(m);
    m->Current_Speed = m->motor_speed;

    /* 积分成位置 */
    m->Position_Actual += m->motor_speed;

    /* 外环：位置式 PID -> Speed_Target */
    m->Pos_Error2 = m->Pos_Error1;
    m->Pos_Error1 = m->Pos_Error0;
    m->Pos_Error0 = m->Target - m->Position_Actual;

    m->Pos_Integral += m->Pos_Error0;
    m->Pos_Integral = clampf(m->Pos_Integral, -m->pos_integral_limit, m->pos_integral_limit);
    m->Pos_Integral_UART = m->Pos_Integral;

    m->Speed_Target = m->Position_Kp * m->Pos_Error0
                    + m->Position_Ki * m->Pos_Integral
                    + m->Position_Kd * (m->Pos_Error0 - m->Pos_Error1);

    m->Speed_Target = clampf(m->Speed_Target, -m->speed_target_limit, m->speed_target_limit);

    /* 内环：增量式 PID -> Out */
    m->Spd_Error2 = m->Spd_Error1;
    m->Spd_Error1 = m->Spd_Error0;
    m->Spd_Error0 = m->Speed_Target - m->motor_speed;

    m->Out += m->Speed_Kp * (m->Spd_Error0 - m->Spd_Error1)
            + m->Speed_Ki * (m->Spd_Error0)
            + m->Speed_Kd * (m->Spd_Error0 - 2.0f * m->Spd_Error1 + m->Spd_Error2);

    m->Out = clampf(m->Out, -m->out_limit, m->out_limit);

    /* 你原来 Actual=Position_Actual：用于打印/上位机 */
    m->Actual = m->Position_Actual;

    return m->Out;
}

/* 应用 PWM：替代你原 Motor1_SetPWM（方向 + CCR）
 * out_percent：-100~100
 */
void Motor_ApplyPWM(Motor_t *m, float out_percent)
{
    if (!m || !m->pwm_tim) return;

    out_percent = clampf(out_percent, -m->out_limit, m->out_limit);

    /* 用 ARR 把百分比转 CCR，避免你之前直接把 0~100 写进 CCR 的问题 */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(m->pwm_tim);
    uint32_t ccr = (uint32_t)(fabsf(out_percent) / 100.0f * (float)arr);
    if (ccr > arr) ccr = arr;

    if (out_percent >= 0.0f)
    {
        __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_ch, ccr);
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_RESET);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(m->pwm_tim, m->pwm_ch, ccr);
        HAL_GPIO_WritePin(m->in1_port, m->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->in2_port, m->in2_pin, GPIO_PIN_SET);
    }
}
