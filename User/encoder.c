#include "encoder.h"
#include "pid.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include <stdint.h>
#include <sys/_intsup.h>

volatile uint8_t tim_read = 0; // 读取速度标志位
volatile uint8_t tim_uart = 0; // UART发送标志位
volatile uint8_t tim_SetPWM = 0; // PWM设置标志位
int count_button = 0; // 旋转编码器值
// char msg_button[50] = ""; // 旋转编码器值储存

// 读取电机编码器速度（增量，每次读取后清零）
int16_t Read_Pulse_Count(TIM_HandleTypeDef *htim)
{
	int16_t temp;
	temp=(int16_t)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
  
	return temp;
}


// 读取旋转编码器位置（累计值，不清零）
int Read_Position(TIM_HandleTypeDef *htim)
{
	int16_t temp;
	temp = (int16_t)__HAL_TIM_GetCounter(htim);
	temp *= 500; // 自定义每格脉冲，根据实际编码器分辨率调整，jgb37-333rpm一圈1320脉冲
//	if (temp > 250) temp = 250; // 限幅处理
//	if (temp > 290) temp = 290; // 限幅处理
//	if (temp < 0) temp = 0;
	return temp;
}

int Read_button(TIM_HandleTypeDef *htim)
{
	static int temp;
	temp=(short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
  
	return temp;
}

/*sprintf(msg_button, "%d", count_button);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)msg_button, strlen(msg_button)); // 通过DMA发送数据
  */