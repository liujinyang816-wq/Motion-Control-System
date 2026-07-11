/**
 ******************************************************************************
 * @file           : DM9161.h
 * @brief          : DM9161 10/100M 以太网 PHY 芯片驱动头文件
 * ----------------------------------------------------------------------------
 * 寄存器定义基于 IEEE 802.3 MII 标准和 DM9161 数据手册
 *
 * MDIO 通信：
 *   - PHY 地址 = 1 (DM9161addr)
 *   - 通过 STM32H7 ETH MAC 的 MDIO 接口访问
 *   - 标准 MII 寄存器 0x00~0x0F + DM9161 扩展寄存器 0x10~0x1F
 ******************************************************************************
 */
#ifndef _DM9161_H_
#define _DM9161_H_

/* ============================================================================
 * PHY 地址和状态返回码
 * ============================================================================ */

/** DM9161 MDIO 地址 (由 PHYAD0~PHYAD4 引脚硬件决定) */
#define DM9161addr 1

/* ── 链路状态返回值（正数 = 有效状态） ── */
#define DM9161_STATUS_READ_ERROR            ((int32_t)-5) /**< MDIO 读取失败 */
#define DM9161_STATUS_WRITE_ERROR           ((int32_t)-4) /**< MDIO 写入失败 */
#define DM9161_STATUS_ADDRESS_ERROR         ((int32_t)-3) /**< PHY 地址错误 */
#define DM9161_STATUS_RESET_TIMEOUT         ((int32_t)-2) /**< 软复位超时 */
#define DM9161_STATUS_ERROR                 ((int32_t)-1) /**< 通用错误 */
#define DM9161_STATUS_OK                    ((int32_t) 0) /**< 操作成功 */
#define DM9161_STATUS_LINK_DOWN             ((int32_t) 1) /**< 链路断开 */
#define DM9161_STATUS_100MBITS_FULLDUPLEX   ((int32_t) 2) /**< 100M 全双工 */
#define DM9161_STATUS_100MBITS_HALFDUPLEX   ((int32_t) 3) /**< 100M 半双工 */
#define DM9161_STATUS_10MBITS_FULLDUPLEX    ((int32_t) 4) /**< 10M 全双工 */
#define DM9161_STATUS_10MBITS_HALFDUPLEX    ((int32_t) 5) /**< 10M 半双工 */
#define DM9161_STATUS_AUTONEGO_NOTDONE      ((int32_t) 6) /**< 自动协商未完成 */

/* ============================================================================
 * MII 标准寄存器地址（IEEE 802.3 定义）
 * ============================================================================ */

#define DM9161_BCR      ((uint16_t)0x0000U)  /**< 基本控制寄存器 (Basic Control Register) */
#define DM9161_BSR      ((uint16_t)0x0001U)  /**< 基本状态寄存器 (Basic Status Register) */
#define DM9161_PHYI1R   ((uint16_t)0x0002U)  /**< PHY 标识符1 (OUI高16位) */
#define DM9161_PHYI2R   ((uint16_t)0x0003U)  /**< PHY 标识符2 (OUI低6位+型号) */
#define DM9161_ANAR     ((uint16_t)0x0004U)  /**< 自动协商广播寄存器 (Auto-Neg Advertisement) */
#define DM9161_ANLPAR   ((uint16_t)0x0005U)  /**< 自动协商对端能力 (Link Partner Ability) */
#define DM9161_ANER     ((uint16_t)0x0006U)  /**< 自动协商扩展寄存器 */
#define DM9161_ANNPTR   ((uint16_t)0x0007U)  /**< 自动协商下一页发送 */
#define DM9161_ANNPRR   ((uint16_t)0x0008U)  /**< 自动协商对端下一页接收 */
#define DM9161_MMDACR   ((uint16_t)0x000DU)  /**< MMD 访问控制 */
#define DM9161_MMDAADR  ((uint16_t)0x000EU)  /**< MMD 访问地址/数据 */

/* ============================================================================
 * DM9161 扩展寄存器地址
 * ============================================================================ */

#define DM9161_ENCTR    ((uint16_t)0x0010U)  /**< 增强控制寄存器 */
#define DM9161_MCSR     ((uint16_t)0x0011U)  /**< 模式控制/状态寄存器 */
#define DM9161_SMR      ((uint16_t)0x0012U)  /**< 特殊模式寄存器 (含PHY地址) */
#define DM9161_TPDCR    ((uint16_t)0x0018U)  /**< 双绞线诊断控制 */
#define DM9161_TCSR     ((uint16_t)0x0019U)  /**< 双绞线诊断状态 */
#define DM9161_SECR     ((uint16_t)0x001AU)  /**< 信号能量控制 */
#define DM9161_SCSIR    ((uint16_t)0x001BU)  /**< 信号质量状态 */
#define DM9161_CLR      ((uint16_t)0x001CU)  /**< 电缆长度寄存器 */
#define DM9161_ISFR     ((uint16_t)0x001DU)  /**< 中断状态标志寄存器 */
#define DM9161_IMR      ((uint16_t)0x001EU)  /**< 中断屏蔽寄存器 */
#define DM9161_PHYSCSR  ((uint16_t)0x001FU)  /**< PHY 专用状态寄存器 (协商结果) */

/* SMR 位域定义 */
#define DM9161_SMR_MODE       ((uint16_t)0x00E0U)  /**< SMR 模式选择掩码 */
#define DM9161_SMR_PHY_ADDR   ((uint16_t)0x001FU)  /**< SMR PHY 地址掩码 */

/* ============================================================================
 * BCR (Basic Control Register, 0x00) 位定义
 * ============================================================================ */

#define DM9161_BCR_SOFT_RESET         ((uint16_t)0x8000U) /**< bit15: 软复位 (1=复位, 自动清零) */
#define DM9161_BCR_LOOPBACK           ((uint16_t)0x4000U) /**< bit14: 环回模式 */
#define DM9161_BCR_SPEED_SELECT       ((uint16_t)0x2000U) /**< bit13: 速率选择 (1=100M, 0=10M) */
#define DM9161_BCR_AUTONEGO_EN        ((uint16_t)0x1000U) /**< bit12: 自动协商使能 */
#define DM9161_BCR_POWER_DOWN         ((uint16_t)0x0800U) /**< bit11: 低功耗模式 */
#define DM9161_BCR_ISOLATE            ((uint16_t)0x0400U) /**< bit10: 电气隔离 */
#define DM9161_BCR_RESTART_AUTONEGO   ((uint16_t)0x0200U) /**< bit9: 重新启动自动协商 */
#define DM9161_BCR_DUPLEX_MODE        ((uint16_t)0x0100U) /**< bit8: 双工模式 (1=全双工, 0=半双工) */

/* ============================================================================
 * BSR (Basic Status Register, 0x01) 位定义
 * ============================================================================ */

#define DM9161_BSR_100BASE_T4       ((uint16_t)0x8000U) /**< bit15: 支持 100BASE-T4 */
#define DM9161_BSR_100BASE_TX_FD    ((uint16_t)0x4000U) /**< bit14: 支持 100BASE-TX 全双工 */
#define DM9161_BSR_100BASE_TX_HD    ((uint16_t)0x2000U) /**< bit13: 支持 100BASE-TX 半双工 */
#define DM9161_BSR_10BASE_T_FD      ((uint16_t)0x1000U) /**< bit12: 支持 10BASE-T 全双工 */
#define DM9161_BSR_10BASE_T_HD      ((uint16_t)0x0800U) /**< bit11: 支持 10BASE-T 半双工 */
#define DM9161_BSR_100BASE_T2_FD    ((uint16_t)0x0400U) /**< bit10: 支持 100BASE-T2 全双工 */
#define DM9161_BSR_100BASE_T2_HD    ((uint16_t)0x0200U) /**< bit9: 支持 100BASE-T2 半双工 */
#define DM9161_BSR_EXTENDED_STATUS  ((uint16_t)0x0100U) /**< bit8: 扩展状态信息可用 */
#define DM9161_BSR_AUTONEGO_CPLT    ((uint16_t)0x0020U) /**< bit5: 自动协商完成 */
#define DM9161_BSR_REMOTE_FAULT     ((uint16_t)0x0010U) /**< bit4: 远端故障 */
#define DM9161_BSR_AUTONEGO_ABILITY ((uint16_t)0x0008U) /**< bit3: 支持自动协商 */
#define DM9161_BSR_LINK_STATUS      ((uint16_t)0x0004U) /**< bit2: 链路状态 (1=已连接) */
#define DM9161_BSR_JABBER_DETECT    ((uint16_t)0x0002U) /**< bit1: Jabber 检测 */
#define DM9161_BSR_EXTENDED_CAP     ((uint16_t)0x0001U) /**< bit0: 扩展寄存器能力 */

/* ============================================================================
 * PHYSCSR (0x1F) 位定义 — DM9161 专用
 * ============================================================================ */

#define DM9161_PHYSCSR_AUTONEGO_DONE   ((uint16_t)0x1000U) /**< bit12: 自动协商完成标志 */
#define DM9161_PHYSCSR_HCDSPEEDMASK    ((uint16_t)0x001CU) /**< bits[4:2]: 速率/双工协商结果掩码 */
#define DM9161_PHYSCSR_10BT_HD         ((uint16_t)0x0004U) /**< 协商结果: 10M 半双工 */
#define DM9161_PHYSCSR_10BT_FD         ((uint16_t)0x0014U) /**< 协商结果: 10M 全双工 */
#define DM9161_PHYSCSR_100BTX_HD       ((uint16_t)0x0008U) /**< 协商结果: 100M 半双工 */
#define DM9161_PHYSCSR_100BTX_FD       ((uint16_t)0x0018U) /**< 协商结果: 100M 全双工 */

/* ============================================================================
 * 函数声明
 * ============================================================================ */

int DM9161_Init(void);              /**< DM9161 初始化（软复位+配置） */
int DM9161_GetLinkState(void);      /**< 获取链路状态（速率+双工模式） */

#endif
