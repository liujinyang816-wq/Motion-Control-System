/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PH1-OSC_OUT (PH1)   ------> RCC_OSC_OUT
*/
void MX_GPIO_Init(void)
{

 	/* 定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitTypeDef  GPIO_InitStructure;

	////////////////      LED	   ////////////////
	/* 开启LED相关的GPIO端口时钟*/
	DO2_GPIO_CLK_ENABLE();
	DO3_GPIO_CLK_ENABLE();
	DO4_GPIO_CLK_ENABLE();
	DO5_GPIO_CLK_ENABLE();
	DO6_GPIO_CLK_ENABLE();
	DO7_GPIO_CLK_ENABLE();
	DO8_GPIO_CLK_ENABLE();
	DO9_GPIO_CLK_ENABLE();
	DO10_GPIO_CLK_ENABLE();
	DO11_GPIO_CLK_ENABLE();
	DO12_GPIO_CLK_ENABLE();
	DO13_GPIO_CLK_ENABLE();
	DO14_GPIO_CLK_ENABLE();
	DO15_GPIO_CLK_ENABLE();
	DO16_GPIO_CLK_ENABLE();
	RUN_GPIO_CLK_ENABLE();
	ERR_GPIO_CLK_ENABLE();

	ETH_RST_GPIO_CLK_ENABLE();

    /* ── 配置所有 DO 输出引脚的基础参数 ── */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

	GPIO_InitStruct.Pin = DO2_PIN;
	HAL_GPIO_Init(DO2_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO3_PIN;
	HAL_GPIO_Init(DO3_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO4_PIN;
	HAL_GPIO_Init(DO4_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO5_PIN;
	HAL_GPIO_Init(DO5_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO6_PIN;
	HAL_GPIO_Init(DO6_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO7_PIN;
	HAL_GPIO_Init(DO7_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO8_PIN;
	HAL_GPIO_Init(DO8_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO9_PIN;
	HAL_GPIO_Init(DO9_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO10_PIN;
	HAL_GPIO_Init(DO10_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO11_PIN;
	HAL_GPIO_Init(DO11_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO12_PIN;
	HAL_GPIO_Init(DO12_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO13_PIN;
	HAL_GPIO_Init(DO13_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO14_PIN;
	HAL_GPIO_Init(DO14_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO15_PIN;
	HAL_GPIO_Init(DO15_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DO16_PIN;
	HAL_GPIO_Init(DO16_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = RUN_PIN;
	HAL_GPIO_Init(RUN_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ERR_PIN;
	HAL_GPIO_Init(ERR_GPIO_PORT, &GPIO_InitStruct);

    /* ── ETH_RST (PB10): DM9161 PHY 复位控制 ── */
	GPIO_InitStruct.Pin = ETH_RST;
	HAL_GPIO_Init(ETH_RST_GPIO_PORT, &GPIO_InitStruct);

    /* ── ETH_RST_Compatible (PB12): DM9161 兼容板复位控制 ── */
	GPIO_InitStruct.Pin = ETH_RST_Compatible;
	HAL_GPIO_Init(ETH_RST_GPIO_PORT, &GPIO_InitStruct);


	LED_ALLON;//全部LED亮
		////////////////      KEY	   ////////////////
	/* 开启按键GPIO口的时钟*/
	DI1_GPIO_CLK_ENABLE();
	DI2_GPIO_CLK_ENABLE();
	DI3_GPIO_CLK_ENABLE();
	DI4_GPIO_CLK_ENABLE();
	DI5_GPIO_CLK_ENABLE();
	DI6_GPIO_CLK_ENABLE();
	DI7_GPIO_CLK_ENABLE();
	DI8_GPIO_CLK_ENABLE();
	DI9_GPIO_CLK_ENABLE();
	DI10_GPIO_CLK_ENABLE();
	DI11_GPIO_CLK_ENABLE();
	DI12_GPIO_CLK_ENABLE();
	DI13_GPIO_CLK_ENABLE();
	DI14_GPIO_CLK_ENABLE();
	DI15_GPIO_CLK_ENABLE();
	DI16_GPIO_CLK_ENABLE();


	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI1_PIN;
	/*设置引脚为输入模式*/
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	/*设置引脚不上拉也不下拉*/
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI1_GPIO_PORT, &GPIO_InitStructure);
	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI2_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI2_GPIO_PORT, &GPIO_InitStructure);
	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI3_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI3_GPIO_PORT, &GPIO_InitStructure);
	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI4_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI4_GPIO_PORT, &GPIO_InitStructure);
	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI5_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI5_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI6_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI6_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI7_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI7_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI8_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI8_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI9_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI9_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI10_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI10_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI11_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI11_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI12_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI12_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI13_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI13_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI14_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI14_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI15_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI15_GPIO_PORT, &GPIO_InitStructure);

	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = DI16_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(DI16_GPIO_PORT, &GPIO_InitStructure);

    /* ── DM9161 PHY 硬件复位时序 ──
     * ETH_RST_ON  → 释放复位 (高电平)
     * 延时 100ms  → 等待 PHY 稳定
     * ETH_RST_OFF → 拉低复位 (低电平)
     * 延时 100ms  → 保持复位
     * ETH_RST_ON  → 释放复位
     */
	ETH_RST_ON;
	ETH_RST_Compatible_ON
	HAL_Delay(100);
	ETH_RST_OFF;
	ETH_RST_Compatible_OFF
	HAL_Delay(100);
	ETH_RST_ON;
	ETH_RST_Compatible_ON

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
