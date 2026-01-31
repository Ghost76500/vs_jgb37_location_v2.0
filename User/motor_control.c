#include "motor_control.h"
#include <string.h>   // memset
#include <math.h>     // fabsf
#include "stm32f1xx_hal.h" // __get_PRIMASK, __disable_irq, __enable_irq
#include "tim.h"      // TIM 相关宏

/*
 * 模块概览：
 * - 面向对象风格的电机控制器，支持多实例（静态对象池 s_motor_pool）。
 * - 控制模式：开环、速度增量式 PID、位置式 PID、位置-速度双环。
 * - 调用链：
 *     1) MotorCtrl_Init -> 配置定时器、GPIO 限幅等。
 *     2) MotorCtrl_SetMode / MotorCtrl_SetTarget / MotorCtrl_Set*PID -> 设定运行参数。
 *     3) 周期任务中调用 MotorCtrl_Update 获取 out_percent；随后 MotorCtrl_ApplyOutput 输出到 PWM 与方向管脚。
 * - 线程安全：读取编码器时使用极短临界区，避免计数器在读取过程中被中断修改。
 */

/* ===== 内部 PID 结构（外部不可见）=====
 * 命名理由：
 * - PidInc：增量式 PID（你速度环那种 Out += ...）
 * - PidPos：位置式 PID（你位置环那种 Out = Kp*e + Ki*I + Kd*de）
 */
typedef struct
{
    float kp;     /* 比例系数 Kp：响应强度 */
    float ki;     /* 积分系数 Ki：消除静差 */
    float kd;     /* 微分系数 Kd：抑制变化、改善动态 */

    float e0;     /* 当前误差 Error0（你原来叫 Error0） */
    float e1;     /* 上次误差 Error1 */
    float e2;     /* 上上次误差 Error2 */

    float out;    /* 输出 Out：对增量式 PID 来说是“累加后的输出” */
} PidInc_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float e0;
    float e1;

    float integral;   /* 积分累加项（原来叫 ErrorInt / Pos_Integral） */
    float out;        /* 输出 Out：位置式 PID 直接计算得到 */
} PidPos_t;

/* ===== 结构 3：真实的 MotorCtrl 结构体定义（只在 .c）=====
 * 外部完全看不到这些字段
 */
struct MotorCtrl
{
    MotorCtrl_Config_t cfg;     /* 硬件与限幅配置，命名 cfg = config，工程常用 */

    MotorCtrl_Mode_t mode;      /* 当前控制模式 */

    /* 与你原全局变量对应的“目标/实际/输出” */
    float target;               /* Target：目标（速度或位置） */
    float actual;               /* Actual：实际（速度或位置） */
    float out_percent;          /* Out：输出百分比（-100..100） */

    /* 编码器相关 */
    float current_speed;        /* 当前速度（脉冲/采样）== 你原 motor_speed/Current_Speed */
    float position_actual;      /* 累积位置（脉冲累计）== 你原 Position_Actual */

    /* 双环内部量 */
    float speed_target;         /* 速度环目标（由位置环输出）== 你原 Speed_Target */
    float pos_integral_uart;    /* 外环积分项快照（便于你 UART 打印） */

    /* PID 控制器对象 */
    PidInc_t spd_inc;           /* 速度环：增量式 PID（贴近你 Speed_Output / 内环） */
    PidPos_t pos_pid;           /* 单位置环：位置式 PID（贴近你 Position_Output） */

    /* 双环外环用位置式 PID（为了贴近你的 Position_Speed_Output 外环写法） */
    PidPos_t pos_outer;
};

/* ===== 模块内部：静态对象池 =====
 * 命名理由：
 * - s_ 前缀表示 static file-scope（仅本文件）
 * - motor_pool 表示“电机对象池”
 * 给 2 个对象
 */
static MotorCtrl_t s_motor_pool[2];

/* ===== 内部工具函数：读取编码器计数（并清零）=====
 * 贴近你原来的 Read_Pulse_Count，但做成模块私有
 */
static int16_t motor_read_pulse_delta(TIM_HandleTypeDef *enc_tim)
{
    /* temp：临时变量，存“本次采样的增量脉冲数”
     * 类型 int16_t：贴近你原写法（TIM 计数器读出来常用 16 位）
     */
    int16_t temp;

    /* 为了避免“读计数器的同时有中断改动”的极端情况，做一个很短的临界区 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    temp = (int16_t)__HAL_TIM_GetCounter(enc_tim);
    __HAL_TIM_SetCounter(enc_tim, 0);

    if (primask == 0u) { __enable_irq(); }

    return temp;
}

/* 输出限幅工具 */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ===== 对外 API 实现 ===== */

/* 获取对象池中的控制器实例；超界返回空指针，调用者自行判空。 */
MotorCtrl_t* MotorCtrl_Get(uint8_t index)
{
    if (index >= (uint8_t)(sizeof(s_motor_pool) / sizeof(s_motor_pool[0])))
    {
        return 0; /* 越界返回空指针，调用者应检查 */
    }
    return &s_motor_pool[index];
}

/* 初始化控制器：保存配置并清零所有运行时状态，保证干净起步。 */
HAL_StatusTypeDef MotorCtrl_Init(MotorCtrl_t *m, const MotorCtrl_Config_t *cfg)
{
    if (m == 0 || cfg == 0) return HAL_ERROR;

    /* 保存配置：把 cfg 拷贝进内部（避免外部 cfg 生命周期问题） */
    m->cfg = *cfg;

    /* 清零运行状态（比逐字段清更不容易漏） */
    m->mode = MOTOR_MODE_OPEN_LOOP;

    m->target = 0.0f;
    m->actual = 0.0f;
    m->out_percent = 0.0f;

    m->current_speed = 0.0f;
    m->position_actual = 0.0f;
    m->speed_target = 0.0f;
    m->pos_integral_uart = 0.0f;

    /* 清零 PID 结构体 */
    memset(&m->spd_inc,   0, sizeof(m->spd_inc));
    memset(&m->pos_pid,   0, sizeof(m->pos_pid));
    memset(&m->pos_outer, 0, sizeof(m->pos_outer));

    return HAL_OK;
}

void MotorCtrl_SetMode(MotorCtrl_t *m, MotorCtrl_Mode_t mode)
{
    if (m == 0) return;
    m->mode = mode;
}

void MotorCtrl_SetTarget(MotorCtrl_t *m, float target)
{
    if (m == 0) return;
    m->target = target;
}

void MotorCtrl_SetSpeedPID(MotorCtrl_t *m, float kp, float ki, float kd)
{
    if (m == 0) return;

    /* spd_inc：速度环增量式 PID 参数 */
    m->spd_inc.kp = kp;
    m->spd_inc.ki = ki;
    m->spd_inc.kd = kd;
}

void MotorCtrl_SetPositionPID(MotorCtrl_t *m, float kp, float ki, float kd)
{
    if (m == 0) return;

    /* pos_pid：单位置环位置式 PID 参数 */
    m->pos_pid.kp = kp;
    m->pos_pid.ki = ki;
    m->pos_pid.kd = kd;
}

void MotorCtrl_SetCascadePID(MotorCtrl_t *m,
                             float pos_kp, float pos_ki, float pos_kd,
                             float spd_kp, float spd_ki, float spd_kd)
{
    if (m == 0) return;

    /* pos_outer：双环外环位置式 PID 参数 */
    m->pos_outer.kp = pos_kp;
    m->pos_outer.ki = pos_ki;
    m->pos_outer.kd = pos_kd;

    /* spd_inc：双环内环增量式 PID 参数（与你原 Speed_Kp/Speed_Ki/Speed_Kd 对应） */
    m->spd_inc.kp = spd_kp;
    m->spd_inc.ki = spd_ki;
    m->spd_inc.kd = spd_kd;
}

/* ===== 三种控制算法更新函数（统一从 MotorCtrl_Update 调用）===== */

/* 速度环：增量式 PID（贴近你 Speed_Output）
 * - 输入：当前速度 current_speed（脉冲/采样）
 * - 输出：out_percent（PWM百分比，累加输出）
 */
static float update_speed_pid_inc(MotorCtrl_t *m, float current_speed)
{
    PidInc_t *pid = &m->spd_inc;

    pid->e2 = pid->e1;
    pid->e1 = pid->e0;
    pid->e0 = m->target - current_speed;

    /* 增量式：Out += ...（与你原公式一致） */
    pid->out += pid->kp * (pid->e0 - pid->e1)
              + pid->ki * (pid->e0)
              + pid->kd * (pid->e0 - 2.0f * pid->e1 + pid->e2);

    /* 输出限幅（单位：百分比） */
    pid->out = clampf(pid->out, -m->cfg.out_limit_percent, m->cfg.out_limit_percent);

    /* 外部可观测量 */
    m->actual = current_speed;
    m->out_percent = pid->out;

    return m->out_percent;
}

/* 单位置环：位置式 PID（贴近你 Position_Output）
 * - 你原来 Actual += motor_position（累计位置）
 */
static float update_position_pid(MotorCtrl_t *m, float delta_pulse)
{
    PidPos_t *pid = &m->pos_pid;

    /* 位置累计：当前位置 = 上次位置 + 本次增量 */
    m->position_actual += delta_pulse;

    pid->e1 = pid->e0;
    pid->e0 = m->target - m->position_actual;

    /* 积分：只有 Ki != 0 才积（贴近你原逻辑） */
    if (pid->ki != 0.0f)
    {
        pid->integral += pid->e0;
    }
    else
    {
        pid->integral = 0.0f;
    }

    /* 位置式输出：Out = Kp*e + Ki*I + Kd*(de) */
    pid->out = pid->kp * pid->e0
             + pid->ki * pid->integral
             + pid->kd * (pid->e0 - pid->e1);

    pid->out = clampf(pid->out, -m->cfg.out_limit_percent, m->cfg.out_limit_percent);

    /* 外部可观测量 */
    m->actual = m->position_actual;
    m->out_percent = pid->out;

    return m->out_percent;
}

/* 双环：外位置环（位置式）输出速度目标 + 内速度环（增量式）输出 PWM
 * 贴近你 Position_Speed_Output 的结构
 */
static float update_pos_spd_cascade(MotorCtrl_t *m, float current_speed)
{
    /* 位置由速度积分得到（与你原来 Position_Actual += motor_speed 一致） */
    m->position_actual += current_speed;

    /* ---------- 外环：位置式 PID -> speed_target ---------- */
    {
        PidPos_t *pos = &m->pos_outer;

        pos->e1 = pos->e0;
        pos->e0 = m->target - m->position_actual;

        /* 积分累加 + 限幅（你原来 ±500） */
        pos->integral += pos->e0;
        pos->integral = clampf(pos->integral, -m->cfg.pos_integral_limit, m->cfg.pos_integral_limit);

        m->pos_integral_uart = pos->integral; /* 便于你 UART 打印 */

        /* 外环输出：目标速度 */
        m->speed_target = pos->kp * pos->e0
                        + pos->ki * pos->integral
                        + pos->kd * (pos->e0 - pos->e1);

        /* 目标速度限幅（你原来 ±200） */
        m->speed_target = clampf(m->speed_target, -m->cfg.speed_target_limit, m->cfg.speed_target_limit);
    }

    /* ---------- 内环：增量式速度 PID -> out_percent ---------- */
    {
        PidInc_t *spd = &m->spd_inc;

        spd->e2 = spd->e1;
        spd->e1 = spd->e0;
        spd->e0 = m->speed_target - current_speed;

        spd->out += spd->kp * (spd->e0 - spd->e1)
                  + spd->ki * (spd->e0)
                  + spd->kd * (spd->e0 - 2.0f * spd->e1 + spd->e2);

        spd->out = clampf(spd->out, -m->cfg.out_limit_percent, m->cfg.out_limit_percent);

        m->out_percent = spd->out;
    }

    /* 外部可观测量（与你原来 Actual = Position_Actual 类似） */
    m->current_speed = current_speed;
    m->actual = m->position_actual;

    return m->out_percent;
}

float MotorCtrl_Update(MotorCtrl_t *m)
{
    if (m == 0) return 0.0f;
    if (m->cfg.enc_tim == 0) return 0.0f;

    /* 读取编码器增量（脉冲/采样）
     * 命名 delta_pulse：强调“增量”而不是绝对位置
     */
    float delta_pulse = (float)motor_read_pulse_delta(m->cfg.enc_tim);

    /* 速度的定义：本例与你现有工程一致 = 每次采样读到的脉冲增量
     * 命名 current_speed：强调“当前速度测量值”
     */
    float current_speed = delta_pulse;
    m->current_speed = current_speed;

    /* 根据当前模式选择控制链：开环/速度环/位置环/位置-速度双环 */
    switch (m->mode)
    {
    case MOTOR_MODE_OPEN_LOOP:
        /* 开环：不更新 PID，只保持 out_percent（由外部 SetTarget 或直接 ApplyOutput 决定）
         * 这里返回当前 out_percent，方便外部统一调用
         */
        return m->out_percent;

    case MOTOR_MODE_SPEED_PID_INC:
        return update_speed_pid_inc(m, current_speed);

    case MOTOR_MODE_POSITION_PID:
        return update_position_pid(m, delta_pulse);

    case MOTOR_MODE_POS_SPD_CASCADE:
        return update_pos_spd_cascade(m, current_speed);

    default:
        return 0.0f;
    }
}

/* 将百分比输出同步到 PWM/方向脚，同时做限幅与占空比换算。 */
void MotorCtrl_ApplyOutput(MotorCtrl_t *m, float out_percent)
{
    if (m == 0) return;
    if (m->cfg.pwm_tim == 0) return;

    /* 保存输出（用于遥测/调试） */
    m->out_percent = clampf(out_percent, -m->cfg.out_limit_percent, m->cfg.out_limit_percent);

    /* 把百分比转换成 CCR
     * - 读取 ARR（自动重装载值）得到 PWM 周期
     * - compare = |out| / 100 * ARR
     */
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(m->cfg.pwm_tim);
    float duty = fabsf(m->out_percent) / 100.0f;
    uint32_t compare = (uint32_t)(duty * (float)arr);

    if (compare > arr) compare = arr;

    /* 方向控制（贴近你 Motor1_SetPWM / Motor2_SetPWM）
     * 约定：
     * - out_percent >= 0：正转（IN1=1, IN2=0）
     * - out_percent <  0：反转（IN1=0, IN2=1）
     * - out_percent == 0：停止（PWM=0，方向脚都拉低，避免直通）
     */
    if (m->out_percent > 0.0f)
    {
        __HAL_TIM_SET_COMPARE(m->cfg.pwm_tim, m->cfg.pwm_channel, compare);
        HAL_GPIO_WritePin(m->cfg.in1_port, m->cfg.in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->cfg.in2_port, m->cfg.in2_pin, GPIO_PIN_RESET);
    }
    else if (m->out_percent < 0.0f)
    {
        __HAL_TIM_SET_COMPARE(m->cfg.pwm_tim, m->cfg.pwm_channel, compare);
        HAL_GPIO_WritePin(m->cfg.in1_port, m->cfg.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->cfg.in2_port, m->cfg.in2_pin, GPIO_PIN_SET);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(m->cfg.pwm_tim, m->cfg.pwm_channel, 0);
        HAL_GPIO_WritePin(m->cfg.in1_port, m->cfg.in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->cfg.in2_port, m->cfg.in2_pin, GPIO_PIN_RESET);
    }
}

void MotorCtrl_GetTelemetry(const MotorCtrl_t *m, MotorCtrl_Telemetry_t *out)
{
    if (m == 0 || out == 0) return;

    out->target          = m->target;
    out->actual          = m->actual;
    out->out_percent     = m->out_percent;

    out->current_speed   = m->current_speed;
    out->position_actual = m->position_actual;
    out->speed_target    = m->speed_target;
    out->pos_integral    = m->pos_integral_uart;
}