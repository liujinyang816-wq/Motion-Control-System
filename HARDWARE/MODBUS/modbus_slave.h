/**
 ******************************************************************************
 * @file    modbus_slave.h
 * @brief   MODBUS RTU 从站协议栈 — 纯协议层
 ******************************************************************************
 */

#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H

#include "main.h"
#include "modbus_regs.h"

#define FW_VERSION         0x0100

#define MB_SLAVE_ADDR      1                //本站地址
#define MB_RX_BUF_SIZE     260             // ModBus 接收缓冲区大小
#define MB_FRAME_TIMEOUT   2               // ModBus 帧超时时间（单位：ms）

#define MB_FC_READ_REGS    0x03             //0X03读寄存器
#define MB_FC_WRITE_REG    0x06             //0x06写单个寄存器
#define MB_FC_WRITE_REGS   0x10             //0x10写多个寄存器

extern uint16_t mb_regs[MB_REG_COUNT];
extern uint32_t g_uptime_seconds;

void Modbus_Init(void);
void Modbus_Poll(void);
void Modbus_UpdateRegs(void);
void CmdExec_CheckTriggers(void);

#endif
