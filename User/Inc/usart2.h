/**
 ******************************************************************************
 * @file           : usart2.h
 * @brief          : USART2 调试串口驱动头文件
 * @author         : zh.
 * @date           : 2019-11-12
 * ----------------------------------------------------------------------------
 * USART2 结构体封装：
 *   - receive_buffer[] : 接收环形缓冲区
 *   - receive_ok_flag  : 一帧接收完成标志
 *   - 函数指针实现多态接口 (initialize, send_string, printf)
 ******************************************************************************
 */
#ifndef __usart2_h__
#define __usart2_h__

#include "stm32h7xx_hal.h"

/* ============================================================================
 * 宏定义
 * ============================================================================ */

#define UART_BUFFER_SIZE 200         /**< 收发缓冲区大小（字节） */

/* ============================================================================
 * USART2_T 结构体 — USART2抽象接口
 * ============================================================================ */
typedef struct
{
    char receive_buffer[UART_BUFFER_SIZE]; /**< 接收环形缓冲区 */
    char receive_data;                     /**< 当前接收字节（中断接收存储） */
    int  counter;                          /**< 接收缓冲区写入位置 */

    int  receive_ok_flag;                  /**< 接收完成标志: 1=一帧已就绪 */
    int  baudrate;                         /**< 当前波特率 */
    int  error;                            /**< 错误码 */

    /* ── 函数指针：多态接口 ── */
    int (*initialize)(unsigned long int);  /**< 初始化串口 (参数: 波特率) */
    int (*send_string)(char *);            /**< 发送字符串 */
    int (*printf)(const char *, ...);      /**< 格式化输出 */
} USART2_T;

/* ============================================================================
 * 全局变量和函数声明
 * ============================================================================ */

extern void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle);
extern void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle);
extern UART_HandleTypeDef huart2;           /**< USART2 HAL 句柄 */
extern USART2_T usart2;                    /**< USART2 全局实例 */

#endif
