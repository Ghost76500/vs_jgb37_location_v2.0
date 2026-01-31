#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

int16_t Read_Pulse_Count(TIM_HandleTypeDef *htim);      // 读取速度（增量）
int Read_Position(TIM_HandleTypeDef *htim);   // 读取位置（累计值）
int Read_button(TIM_HandleTypeDef *htim);     // 读取按钮状态
int16_t Read_Location(TIM_HandleTypeDef *htim); // 读取位置（增量）

extern volatile uint8_t tim_read;  // 读取速度标志位
extern volatile uint8_t tim_uart; // UART发送标志位
extern volatile uint8_t tim_SetPWM; // PWM设置标志位

extern int count_button;



#endif // ENCODER_H