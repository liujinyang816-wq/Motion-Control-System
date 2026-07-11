/**
 ******************************************************************************
 * @file           : rs485.h
 * @brief          : RS485 收发驱动头文件 (DMA 版本)
 ******************************************************************************
 */
#ifndef __RS485_H
#define __RS485_H
#include "main.h"

/* ── 全局变量声明 ── */
extern uint8_t RS485_RX_BUF[260];          /**< 接收缓冲区 (与 MB_RX_BUF_SIZE 对齐, 防溢出) */
extern uint8_t RS485_TX_BUF[260];          /**< DMA 发送缓冲区 (必须非DTCM，DMA可访问) */
extern uint16_t RS485_RX_CNT;              /**< 已接收字节计数 (uint16_t: 缓冲区 260B, uint8_t 最大 255 不够) */
extern uint8_t RS485_Receive_Over;          /**< 接收完成标志: 1=一帧就绪 */
extern uint8_t RS485_TX_Busy;              /**< DMA 发送忙标志: 1=正在发送 */
extern uint32_t RS485_TC_Timeout;          /**< TC 超时计数器 (在 Modbus_Poll 中递减) */

/* ── DMA 句柄 ── */
extern DMA_HandleTypeDef hdma_usart3_rx;   /**< USART3 RX DMA 句柄 */
extern DMA_HandleTypeDef hdma_usart3_tx;   /**< USART3 TX DMA 句柄 */

/* ── UART 句柄 ── */
extern UART_HandleTypeDef USART3_RS485Handler;  /**< USART3 HAL 句柄 (stm32h7xx_it.c ISR 需要引用) */

/* ── 功能开关 ── */
#define EN_USART3_RX    1                  /**< 接收中断使能: 1=启用 (DMA依赖USART3 IDLE中断) */
#define EN_USART3_DMA   1                  /**< DMA 收发开关: 1=DMA模式, 0=回退中断模式 */

/* ── 函数声明 ── */
void RS485_Init(uint32_t bound);
uint8_t RS485_Send_Data(uint8_t *buf, uint8_t len);  /**< @retval 1=发送成功, 0=忙(DMA模式下前一次发送未完成) */
void RS485_Receive_Data(uint8_t *buf, uint8_t *len);
void RS485_TX_Set(uint8_t en);
void RS485_TC_CheckTimeout(void);          /**< TC 超时检查 (每周期在 Modbus_Poll 中调用) */

/* ── HAL 回调声明 ── */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
