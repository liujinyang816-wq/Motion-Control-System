/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "osal.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile uint32_t CPU_RunTime;

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()     (CPU_RunTime = 0ul)
#define portGET_RUN_TIME_COUNTER_VALUE()             CPU_RunTime  

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* printf control: set to 1 to enable, 0 to disable */
#define printf_cmd 1

#define SYNC0TIME 5000

/* ── 轴数定义 ── */
#define MAX_AXES          8     /* 硬件最大轴数 */
#define LINKED_AXES_MAX   4     /* 联动轴最大数量 (X/Y/Z/R) */
#define AUX_AXES_MAX      4     /* 辅助轴最大数量 (U/V/W/S) */

/* 运行时变量 (main.c 中定义) */
extern uint8_t g_active_axes;       /* 激活轴总数 (1~8, 默认4) */
extern uint8_t g_active_coupled;    /* 激活联动轴数 (1~4, 默认4) */
extern uint8_t g_active_aux;        /* 激活辅助轴数 (0~4, 默认0) */

/* 兼容旧代码: NUM_AXES 现在是运行期变量, 循环自动适配 */
#define NUM_AXES  g_active_axes

/* 位置控制参数 (编码器脉冲) */
#define SCURVE_STOP_THRESH  4     /* S曲线收尾阈值(≈1RPM), motion_profile.c 使用, 不依赖轴索引 */
#define MOVE_RPM          100   /* 串口定位默认速度 (可通过 SPEED:N 动态修改) */
#define HOMING_RPM        50     /* 回零速度 RPM */
#define AXIS_HOME_POS_X   0     /* 自定义X轴初始位置 */
#define AXIS_HOME_POS_Y   0     /* 自定义Y轴初始位置 */
#define AXIS_HOME_POS_Z   0     /* 自定义Z轴初始位置 (若启用) */
#define AXIS_HOME_POS_R   0     /* 自定义R轴初始位置 (若启用) */
#define POS_ARRIVE_THRESH 5     /* 到位判定阈值 (约1RPM单周期脉冲) */
#define PULSES_PER_REV         50000  /* 直线模式每圈脉冲数（保持不变） */
#define PULSES_PER_REV_ROTARY  36000  /* 旋转模式每圈脉冲数（新增） */

/* Axis ID enum */
#define AXIS_ID_X   0
#define AXIS_ID_Y   1
#define AXIS_ID_Z   2
#define AXIS_ID_R   3
#define AXIS_ID_U   4
#define AXIS_ID_V   5
#define AXIS_ID_W   6
#define AXIS_ID_S   7

/* ── 运动系统扩展宏 ── */
#define SEGMENT_QUEUE_SIZE  64    /**< 前瞻队列深度 */
#define MAX_FEEDRATE_MMPM   50000 /**< 最大进给速度 mm/min */
#define PARAM_COUNT         256   /**< 参数总数 */


#define RMII_TXD1_Pin GPIO_PIN_14
#define RMII_TXD1_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG

#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG

#define LAN_nRST_Pin GPIO_PIN_10
#define LAN_nRST_GPIO_Port GPIOB

#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC

#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA

#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC

#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA

#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC

#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
