/**
 ******************************************************************************
 * @file           : serial_cmd.h
 * @brief          : 串口命令解析器 — 命令缓冲区和解析函数声明
 * ----------------------------------------------------------------------------
 * USART1 中断接收命令 → 缓冲 → ParseCommand() → 执行轴控指令
 ******************************************************************************
 */
#ifndef _SERIAL_CMD_H_
#define _SERIAL_CMD_H_

#include "osal.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/* ── 串口命令缓冲区 (由 USART1_IRQHandler 写入) ── */
extern char   cmd_buf[64];
extern uint8  cmd_idx;
extern uint8  cmd_ready;

/* ── 命令解析任务句柄 (由 USART1 ISR 唤醒) ── */
extern TaskHandle_t xCmdTaskHandle;

/* ── 辅助函数 (被 modbus_slave.c 等引用) ── */
int32 Axis_ParsePosition(const char *cmd, int *ok);
int   Axis_ParseRpm(const char *cmd, int *rpm, int *sign, int *ok);
int   Axis_NameToIndex(char c);
int   Axis_StrIEq(const char *a, const char *b);
void  Axis_TrimSpaces(char *s);
int   ParseCSV(const char *s, int32 *out, int max);

/* ── 命令解析任务 ── */
void vCmd_Task(void *pvParameters);

#endif /* _SERIAL_CMD_H_ */
