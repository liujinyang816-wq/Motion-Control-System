/**
 ******************************************************************************
 * @file           : usart2.c
 * @brief          : USART2 调试串口驱动 — 基于结构体封装的 UART 抽象层
 * @author         : zh.
 * @date           : 2019-11-12
 * ----------------------------------------------------------------------------
 * 功能：
 *   1. USART2 初始化（可配置波特率）
 *   2. 字符串发送
 *   3. 中断接收（环形缓冲区）
 *   4. printf 格式化输出
 *
 * USART2 用途：辅助调试/通信串口（与 USART1 控制命令串口独立）
 *
 * 接收模式：中断非阻塞 (UART_IT_RXNE)
 *   - 每次接收1字节触发中断
 *   - 数据存储在 USART2_T.receive_buffer[]
 *   - \r\n 作为帧结束标志
 ******************************************************************************
 */

#include "usart2.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>

/* ============================================================================
 * 函数原型声明
 * ============================================================================ */
static int send_string_to_usart2(char *str);
static int initialize_usart2(unsigned long int baudrate);
static int my_printf2(const char *fmt, ...);

/* ============================================================================
 * USART2 全局实例
 * ============================================================================ */
USART2_T usart2 = {
    .receive_ok_flag = 0,
    .counter = 0,
    .send_string = send_string_to_usart2,       // 函数指针绑定
    .initialize = initialize_usart2,
    .printf = my_printf2
};

extern UART_HandleTypeDef huart2;                /**< USART2 HAL 句柄 (在 usart.c 中) */

/* ============================================================================
 * initialize_usart2() — USART2 初始化
 * ============================================================================
 * @param  baudrate : 目标波特率 (如 115200)
 * @retval 0 : 成功
 *
 * 步骤：
 *   1. 设置波特率
 *   2. 调用 HAL_UART_Init() 初始化
 *   3. 启动中断接收（每次1字节）
 */
static int initialize_usart2(unsigned long int baudrate)
{
    huart2.Init.BaudRate = baudrate;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        while (1);                               // 初始化失败 → 死循环
    }
    /* 启动中断接收模式：每次接收1字节，存入 usart2.receive_data */
    HAL_UART_Receive_IT(&huart2, (unsigned char *)&usart2.receive_data, 1);

    return 0;
}

/* ============================================================================
 * send_string_to_usart2() — 发送字符串
 * ============================================================================
 * @param  str : 以 '\0' 结尾的字符串
 * @retval 0 : 成功
 *
 * 逐字节阻塞发送，等待每个字节发送完成（TC标志）
 */
static int send_string_to_usart2(char *str)
{
    while (*str != '\0')
    {
        while (!(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == 1));  // 等待TC
        HAL_UART_Transmit(&huart2, (unsigned char*)str++, 1, 1000);
    }
    while (!(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == 1));      // 等待最后字节完成

    return 0;
}

/* ============================================================================
 * HAL_UART_RxCpltCallback() — USART2 接收中断回调
 * ============================================================================
 * 每收到1字节触发，将数据存入环形缓冲区
 *
 * 帧解析：
 *   - 检测 \r\n 作为一帧结束
 *   - 设置 receive_ok_flag = 1 通知上层处理
 *   - 计数器归零，准备接收下一帧
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        HAL_UART_IRQHandler(&huart2);

        /* 等待串口就绪后重新启动中断接收 */
        while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY);
        while (HAL_UART_Receive_IT(&huart2, (unsigned char *)&usart2.receive_data, 1) != HAL_OK);

        /* 存入环形缓冲区 */
        usart2.receive_buffer[usart2.counter++] = usart2.receive_data;
        if (usart2.counter == UART_BUFFER_SIZE) usart2.counter = 0;  // 缓冲区溢出保护

        /* ── 帧结束检测 ──
         * 当收到 \r\n (CR+LF) 时，标记一帧接收完成
         * 将 \r 替换为 \0 作为字符串终止符
         */
        if (usart2.receive_buffer[usart2.counter - 1] == '\n'
         && usart2.receive_buffer[usart2.counter - 2] == '\r')
        {
            usart2.receive_buffer[usart2.counter - 1] = 0;   // 终止符替换 \n
            usart2.counter = 0;                                // 计数器归零
            usart2.receive_ok_flag = 1;                        // 通知上层
        }
    }
}

/* ============================================================================
 * my_printf2() — USART2 格式化输出
 * ============================================================================
 * @param  fmt : printf 格式字符串
 * @param  ... : 可变参数
 * @retval 0 : 成功
 *
 * 类似 printf，但输出到 USART2 而非 USART1
 * 使用 vsprintf 格式化 → send_string_to_usart2 发送
 */
static int my_printf2(const char *fmt, ...)
{
    __va_list arg_ptr;
    char buf[UART_BUFFER_SIZE];

    memset(buf, '\0', sizeof(buf));             // 清空格式化缓冲区

    va_start(arg_ptr, fmt);
    vsprintf(buf, fmt, arg_ptr);                // 格式化到 buf
    va_end(arg_ptr);

    send_string_to_usart2(buf);                 // 通过 USART2 发送

    return 0;
}
