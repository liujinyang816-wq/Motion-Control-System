/**
 ******************************************************************************
 * @file           : fdcan.h
 * @brief          : STM32H7 FDCAN1 驱动头文件
 * @author         : 正点原子 (ALIENTEK)
 * @version        : V1.0
 * @date           : 2018-06-29
 * ----------------------------------------------------------------------------
 * FDCAN1 配置：
 *   - PB8 = FDCAN1_RX (AF9), PB9 = FDCAN1_TX (AF9)
 *   - 时钟源: PLL1Q = 200MHz
 *   - 经典 CAN 2.0 模式
 ******************************************************************************
 */
#ifndef _FDCAN_H
#define _FDCAN_H

#include "main.h"

/** FDCAN1 RX0 中断使能: 0=禁用(轮询), 1=启用中断接收 */
#define FDCAN1_RX0_INT_ENABLE   0

/* ============================================================================
 * 函数声明
 * ============================================================================ */

unsigned char FDCAN1_Mode_Init(uint16_t presc, unsigned char ntsjw,
                                uint16_t ntsg1, unsigned char ntsg2,
                                uint32_t mode);               /**< 初始化FDCAN1 */
unsigned char FDCAN1_Send_Msg(unsigned char *msg, uint32_t len);   /**< 发送CAN消息 */
unsigned char FDCAN1_Receive_Msg(unsigned char *buf);              /**< 接收CAN消息(轮询) */

#endif
