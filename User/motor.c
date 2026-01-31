#include "motor.h"
#include "stm32f1xx_hal.h"
#include "encoder.h"
#include "pid.h"
#include "tim.h"
#include <stdint.h>



// 电机1设置PWM输出函数(包含方向控制)
void Motor1_SetPWM(float Out)
{
    // 
    // if(tim_SetPWM == 0) return; // 未到设置时间，直接返回
    // 设置TIM3通道1的比较值
    if (Out >= 0)
    { // 正转
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)Out);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // PB1低电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);   // PB0高电平
    }
    else
    { // 反转 
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)(-Out));
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // PB1低电平
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // PB1高电平

    }
    //tim_SetPWM = 0; // 清除PWM设置标志位
}

void Motor2_SetPWM(float Out)
{
    // 
    if(tim_SetPWM == 0) return; // 未到设置时间，直接返回
    // 设置TIM3通道1的比较值
    if (Out >= 0)
    { // 正转
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)Out);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // PB1低电平
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // PB0高电平
    }
    else
    { // 反转
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)(-Out));
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // PB1低电平
        //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // PB1高电平
    }
    tim_SetPWM = 0; // 清除PWM设置标志位
}



