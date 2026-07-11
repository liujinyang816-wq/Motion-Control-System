/**
 ******************************************************************************
 * @file           : DM9000.h
 * @brief          : DM9000AEP 以太网控制器驱动头文件
 * @author         : armfly (www.armfly.com)
 * ----------------------------------------------------------------------------
 * DM9000 是一款集成 MAC + PHY 的快速以太网控制器
 * 通过 FMC/FSMC 并行总线与 MCU 连接，支持 8/16 位数据宽度
 *
 * 本文件定义：
 *   1. DM9000 完整寄存器地址映射
 *   2. 寄存器位定义宏
 *   3. 全局变量和函数声明
 ******************************************************************************
 */
#ifndef _DM9000_H_
#define _DM9000_H_

#include <inttypes.h>

/* ============================================================================
 * DM9000 寄存器地址映射（所有寄存器均为8位访问）
 * ============================================================================ */

/* ── 网络控制/状态寄存器组 (NCR/NSR) ── */
#define DM9000_REG_NCR        0x00    /**< 网络控制寄存器: 复位、唤醒、启动控制 */
#define DM9000_REG_NSR        0x01    /**< 网络状态寄存器: 链路状态、速度、双工 */
#define DM9000_REG_TCR        0x02    /**< 发送控制寄存器: bit0=1启动发送 */
#define DM9000_REG_TSR1       0x03    /**< 发送状态寄存器1: 冲突计数、发送状态 */
#define DM9000_REG_TSR2       0x04    /**< 发送状态寄存器2: 延迟冲突、载波丢失 */
#define DM9000_REG_RCR        0x05    /**< 接收控制寄存器: 混杂模式、过滤控制 */
#define DM9000_REG_RSR        0x06    /**< 接收状态寄存器: 帧类型、错误状态 */
#define DM9000_REG_ROCR       0x07    /**< 接收溢出计数寄存器 */

/* ── 流量控制和背压寄存器 ── */
#define DM9000_REG_BPTR       0x08    /**< 背压门限寄存器 */
#define DM9000_REG_FCTR       0x09    /**< 流量控制门限寄存器 */
#define DM9000_REG_FCR        0x0A    /**< 流量控制寄存器 */

/* ── PHY访问寄存器组 (EPCR/EPAR/EPDRH/EPDRL) ── */
#define DM9000_REG_EPCR       0x0B    /**< EEPROM & PHY 控制寄存器 */
#define DM9000_REG_EPAR       0x0C    /**< EEPROM & PHY 地址寄存器 */
#define DM9000_REG_EPDRL      0x0D    /**< EEPROM & PHY 数据寄存器低字节 */
#define DM9000_REG_EPDRH      0x0E    /**< EEPROM & PHY 数据寄存器高字节 */
#define DM9000_REG_WAR        0x0F    /**< 唤醒地址寄存器 */

/* ── MAC地址和哈希表寄存器 (PAR/MAR) ── */
#define DM9000_REG_PAR        0x10    /**< 物理地址寄存器(起始): PAR0~PAR5 = MAC地址 */
#define DM9000_REG_MAR        0x16    /**< 多播地址寄存器(起始): MAR0~MAR7 = 哈希表 */

/* ── GPIO 和 芯片ID 寄存器 ── */
#define DM9000_REG_GPCR       0x1E    /**< GPIO控制寄存器 */
#define DM9000_REG_GPR        0x1F    /**< GPIO数据寄存器: bit0=PHY电源控制 */
#define DM9000_REG_VID_L      0x28    /**< 厂商ID低字节 (应为0x46) */
#define DM9000_REG_VID_H      0x29    /**< 厂商ID高字节 (应为0x0A) */
#define DM9000_REG_PID_L      0x2A    /**< 产品ID低字节 */
#define DM9000_REG_PID_H      0x2B    /**< 产品ID高字节 */
#define DM9000_REG_CHIPR      0x2C    /**< 芯片版本寄存器 */

/* ── 扩展功能寄存器 ── */
#define DM9000_REG_TCR2       0x2D    /**< 发送控制寄存器2: LED模式 */
#define DM9000_REG_OTCR       0x2E    /**< 操作测试控制寄存器: 时钟频率设置 */
#define DM9000_REG_SMCR       0x2F    /**< 特殊模式控制寄存器 */
#define DM9000_REG_ETXCSR     0x30    /**< 早期发送控制/状态寄存器 */
#define DM9000_REG_TCSCR      0x31    /**< 发送校验和控制寄存器 */
#define DM9000_REG_RCSCSR     0x32    /**< 接收校验和控制/状态寄存器 */

/* ── 内存访问命令寄存器 (FIFO操作) ── */
#define DM9000_REG_MRCMDX     0xF0    /**< 内存读命令(不更新指针): 用于检查数据就绪 */
#define DM9000_REG_MRCMD      0xF2    /**< 内存读命令(更新指针): 实际读取数据 */
#define DM9000_REG_MRRL       0xF4    /**< 内存读指针低字节 */
#define DM9000_REG_MRRH       0xF5    /**< 内存读指针高字节 */
#define DM9000_REG_MWCMDX     0xF6    /**< 内存写命令(不更新指针) */
#define DM9000_REG_MWCMD      0xF8    /**< 内存写命令(更新指针): 实际写入数据 */
#define DM9000_REG_MWRL       0xFA    /**< 内存写指针低字节 */
#define DM9000_REG_MWRH       0xFB    /**< 内存写指针高字节 */
#define DM9000_REG_TXPLL      0xFC    /**< 发送包长度低字节 */
#define DM9000_REG_TXPLH      0xFD    /**< 发送包长度高字节 */

/* ── 中断状态/屏蔽寄存器 ── */
#define DM9000_REG_ISR        0xFE    /**< 中断状态寄存器: 写1清除对应位 */
#define DM9000_REG_IMR        0xFF    /**< 中断屏蔽寄存器: bit0=接收中断, bit1=发送中断 */

/* ============================================================================
 * 芯片和寄存器常量定义
 * ============================================================================ */

/** DM9000A 有效芯片ID (VID=0x0A46, PID=0x9000) */
#define DM9000A_ID_OK       0x0A469000

/* ── 总线模式选择 ── */
#define DM9000_BYTE_MODE      0x01    /**< 8位总线模式 */
#define DM9000_WORD_MODE      0x00    /**< 16位总线模式（本项目使用） */

/* ── PHY访问前缀 ── */
#define DM9000_PHY            0x40    /**< PHY地址前缀 (bit6=1表示PHY寄存器访问) */

/* ── 接收就绪状态 ── */
#define DM9000_PKT_RDY        0x01    /**< 接收数据就绪 (MRCMDX返回值) */
#define DM9000_PKT_NORDY      0x00    /**< 无接收数据 */
#define DM9000_REG_RESET      0x03    /**< NCR复位值 (bit1:bit0 = 11 = 内部复位) */

/* ── 中断标志位定义 ── */
#define DM9000_RX_INTR        0x01    /**< 接收中断 (ISR/IMR bit0) */
#define DM9000_TX_INTR        0x02    /**< 发送中断 (ISR/IMR bit1) */
#define DM9000_OVERFLOW_INTR  0x04    /**< 接收溢出中断 (ISR/IMR bit2) */
#define DM9000_LINK_CHANG     0x20    /**< 链路状态变化中断 (ISR bit5) */
#define DM9000_LINK_STATUE    0x40    /**< 链路状态: 0x40=已连接 (NSR bit6) */

/* ── PHY电源控制 ── */
#define DM9000_PHY_ON         0x00    /**< GPIO数据: PHY上电 (GPR bit0=0) */
#define DM9000_PHY_OFF        0x01    /**< GPIO数据: PHY断电 (GPR bit0=1) */

/* ── 接收控制寄存器预设值 ──
 * 0x31: 接收所有帧(含CRC错误/对齐错误)
 * 0x33: 混杂模式, 接收所有帧(不分目标地址)  ── */
#define DM9000_RCR_SET        0x33    /**< RCR: 混杂模式+广播+多播+单播全部接收 */
#define DM9000_RCR_OFF        0x00    /**< RCR: 禁用接收 */

/* ── 发送控制预设 ── */
#define DM9000_TCR_SET        0x01    /**< TCR: bit0=1触发发送, 发送完成后自动清零 */

/* ── 其他寄存器预设值 ── */
#define DM9000_BPTR_SET       0x37    /**< 背压门限: 3KB */
#define DM9000_FCTR_SET       0x38    /**< 流量控制门限: 3KB */
#define DM9000_TCR2_SET       0x80    /**< LED模式1: 全双工常亮, 半双工闪烁 */
#define DM9000_OTCR_SET       0x80    /**< 工作频率: 100MHz */
#define DM9000_ETXCSR_SET     0x83    /**< 早期发送: 启用 */
#define DM9000_FCR_SET        0x28    /**< 流量控制: 启用发送暂停帧 */
#define DM9000_TCSCR_SET      0x07    /**< TCP/UDP校验和生成: 启用 */
#define DM9000_RCSCSR_SET     0x03    /**< 接收校验和验证: 启用 */

/* ── 中断屏蔽预设 ── */
#define DM9000_IMR_SET        0x81    /**< 中断屏蔽: 仅启用接收中断(bit0), 其余屏蔽 */
#define DM9000_IMR_OFF        0x80    /**< 中断屏蔽: 全部屏蔽 (bit7=1强制屏蔽) */

/* ── 硬件复位引脚控制宏 ── */
#define DM9000_RST  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)  /**< 复位: PC9拉低 */
#define DM9000_SET  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)    /**< 释放: PC9拉高 */

/* ============================================================================
 * 全局变量声明
 * ============================================================================ */
extern uint16_t  receiveLen_DM9000;              /**< DM9000收到的帧长度(不含CRC) */
extern uint8_t   receiveBuffer_DM9000[1528];     /**< DM9000接收缓冲区 */
extern uint16_t  receiveLen;                     /**< 上层使用的接收长度 */
extern uint8_t   receiveBuffer[1528];            /**< 上层使用的接收缓冲区 */

/* ============================================================================
 * 函数声明
 * ============================================================================ */
void     DM9000_Init(void);                              /**< DM9000初始化入口 */
void     dm9k_send_packet(uint8_t *p_char, uint16_t length); /**< 发送以太网帧 */
uint16_t dm9k_receive_packet(void);                      /**< 接收以太网帧(轮询) */
uint8_t  dm9k_ReadReg(uint8_t reg);                      /**< 读取DM9000寄存器 */
uint32_t dm9k_ReadID(void);                              /**< 读取芯片ID */
void     DM9000_Initnic(void);                           /**< 网卡芯片初始化 */
uint16_t dm9k_phy_read(uint8_t phy_reg);                 /**< 读取PHY寄存器 */
void     dm9k_err_reset(void);                           /**< DM9000 软复位 (清缓冲区/恢复寄存器) */
void     dm9k_reset(void);                                /**< DM9000 硬复位 (GPIO拉低+完整初始化) */

#endif
