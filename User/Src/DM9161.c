/**
 ******************************************************************************
 * @file           : DM9161.c
 * @brief          : DM9161 10/100M 以太网 PHY 芯片驱动
 * ----------------------------------------------------------------------------
 * DM9161 是 Davicom 公司的快速以太网物理层收发器 (PHY)
 * 通过 MDIO 接口与 STM32H7 的 ETH MAC 通信
 *
 * 主要功能：
 *   1. PHY 软复位
 *   2. 自动协商完成检测
 *   3. 链路状态检测（速率+双工模式）
 *   4. 支持 10/100M 全双工/半双工自动协商
 *
 * MDIO 地址：DM9161addr = 1（由硬件引脚 PHYAD0~4 决定）
 *
 * 速率协商结果：
 *   - DM9161_STATUS_100MBITS_FULLDUPLEX: 100M 全双工 ← 通常协商结果
 *   - DM9161_STATUS_100MBITS_HALFDUPLEX: 100M 半双工
 *   - DM9161_STATUS_10MBITS_FULLDUPLEX:  10M 全双工
 *   - DM9161_STATUS_10MBITS_HALFDUPLEX:  10M 半双工
 ******************************************************************************
 */
#include "DM9161.h"
#include "stdio.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_eth.h"
#include "main.h"

extern ETH_HandleTypeDef heth;       /**< STM32H7 以太网外设句柄 */

/* ============================================================================
 * DM9161_Init() — DM9161 PHY 初始化
 * ============================================================================
 * @retval  0 : 初始化成功
 * @retval -1 : 初始化失败（MDIO通信错误）
 *
 * 初始化流程：
 *   1. 配置 MDIO 时钟范围（适配 HCLK 频率）
 *   2. 通过 BCR 软复位 PHY
 *   3. 轮询等待软复位完成（BCR bit15 自动清零）
 *
 * 注意：
 *   - DM9161 在上电后需要约 100ms 的稳定时间
 *   - 软复位后 PHY 会自动进入自动协商模式
 *   - 软复位通常需要 100~500ms 完成
 */
int DM9161_Init(void)
{
    uint32_t regvalue = 0;
    HAL_StatusTypeDef rtn;
    int32_t retry;

    /* ── 第1步: 配置 MDIO 时钟分频 ──
     * STM32H7 MDIO 时钟必须 <= 2.5MHz
     * HAL_ETH_SetMDIOClockRange() 根据 HCLK 自动选择合适的分频值
     */
    HAL_ETH_SetMDIOClockRange(&heth);

    #if printf_cmd
    printf("start soft reset\r\n");
    #endif

    /* ── 第2步: 触发 PHY 软复位 ──
     * BCR (Basic Control Register, 0x00) bit15 = 1 → 启动软复位
     * 复位期间 PHY 不响应 MDIO 通信
     */
    rtn = HAL_ETH_WritePHYRegister(&heth, DM9161addr, DM9161_BCR,
                                   DM9161_BCR_SOFT_RESET);
    if (rtn != HAL_OK)
    {
        #if printf_cmd
        printf("[DM9161] Write BCR Soft Reset FAILED (rtn=%d)\r\n", rtn);
        #endif
        return -1;
    }

    /* ── 第3步: 轮询等待软复位完成 ──
     * BCR bit15 = 1 → 复位进行中
     * BCR bit15 = 0 → 复位完成
     *
     * 注意: 如果 MDIO 通信失败，regvalue 返回 0x0000，
     * bit15=0 → 循环立即退出（假成功）。
     * 因此在退出后验证 BCR != 0x0000。
     */
    #if printf_cmd
    printf("check soft reset is finish or not\r\n");
    #endif
    retry = 0;
    do
    {
        rtn = HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_BCR, &regvalue);
        if (rtn != HAL_OK)
        {
            #if printf_cmd
            printf("[DM9161] Read BCR during reset wait FAILED (rtn=%d)\r\n", rtn);
            #endif
            return -1;
        }
        retry++;
        if (retry > 1000)
        {
            #if printf_cmd
            printf("[DM9161] Soft reset TIMEOUT (BCR=0x%04lX)\r\n", regvalue);
            #endif
            return -1;
        }
    } while (regvalue & DM9161_BCR_SOFT_RESET);  // 等待 bit15 清零

    #if printf_cmd
    printf("soft reset is finish\r\n");
    #endif

    /* ── 第4步: 诊断 —— 读 PHY ID 寄存器验证 MDIO 通信 ──
     * DM9161 PHY ID:
     *   PHYI1R (0x02) = 0x0181 (Davicom OUI 高16位)
     *   PHYI2R (0x03) = 0xB881 (OUI 低6位 + 型号 0x29)
     *
     * 如果这两个寄存器也返回 0x0000，说明 MDIO 通信完全失败
     */
    {
        uint32_t bcr, bsr, physcsr, phyi1r, phyi2r;

        rtn  = HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_BCR, &bcr);
        rtn |= HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_BSR, &bsr);
        rtn |= HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_PHYSCSR, &physcsr);
        rtn |= HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_PHYI1R, &phyi1r);
        rtn |= HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_PHYI2R, &phyi2r);

        #if printf_cmd
        printf("[DM9161] BCR     =0x%04lX\r\n", bcr);
        #endif
        #if printf_cmd
        printf("[DM9161] BSR     =0x%04lX\r\n", bsr);
        #endif
        #if printf_cmd
        printf("[DM9161] PHYSCSR =0x%04lX\r\n", physcsr);
        #endif
        #if printf_cmd
        printf("[DM9161] PHYI1R  =0x%04lX (expected 0x0181)\r\n", phyi1r);
        #endif
        #if printf_cmd
        printf("[DM9161] PHYI2R  =0x%04lX (expected 0xB881)\r\n", phyi2r);
        #endif

        /* ── 诊断判断 ── */
        if (rtn != HAL_OK)
        {
            #if printf_cmd
            printf("[DM9161] ERROR: MDIO read returned HAL_ERROR\r\n");
            #endif
            return -1;
        }

        if (phyi1r == 0x0000 && phyi2r == 0x0000)
        {
            #if printf_cmd
            printf("[DM9161] ERROR: PHY ID all zero — MDIO communication FAILED!\r\n");
            #endif
            #if printf_cmd
            printf("[DM9161] Check: PHY reset GPIO, MDIO pull-up, PHY address\r\n");
            #endif
            return -1;
        }

        if (phyi1r != 0x0181 || phyi2r != 0xB881)
        {
            #if printf_cmd
            printf("[DM9161] WARNING: PHY ID mismatch, but MDIO is working\r\n");
            #endif
            #if printf_cmd
            printf("[DM9161] Expected Davicom DM9161, got VID=0x%04lX PID=0x%04lX\r\n",
                   phyi1r, phyi2r);
            #endif
            /* 不阻断初始化 — 可能是不同型号但兼容的 PHY */
        }
    }

    return 0;
}

/* ============================================================================
 * DM9161_GetLinkState() — 获取 PHY 链路状态
 * ============================================================================
 * @retval DM9161_STATUS_LINK_DOWN       (1) : 链路断开
 * @retval DM9161_STATUS_100MBITS_FULLDUPLEX (2) : 100M 全双工
 * @retval DM9161_STATUS_100MBITS_HALFDUPLEX (3) : 100M 半双工
 * @retval DM9161_STATUS_10MBITS_FULLDUPLEX  (4) : 10M 全双工
 * @retval DM9161_STATUS_10MBITS_HALFDUPLEX  (5) : 10M 半双工
 * @retval DM9161_STATUS_AUTONEGO_NOTDONE    (6) : 自动协商未完成
 * @retval DM9161_STATUS_READ_ERROR     (-5) : MDIO通信错误
 *
 * 检测流程：
 *   1. 两次读取 BSR (Basic Status Register) 检查链路状态
 *   2. 如果启用自动协商 → 读取 PHYSCSR 获取协商结果
 *   3. 如果未启用自动协商 → 读取 BCR 获取强制模式
 *
 * 标准 MII 寄存器说明：
 *   BCR  (0x00): 基本控制寄存器 — 复位、环回、速率、双工、自动协商使能
 *   BSR  (0x01): 基本状态寄存器 — 链路状态、协商完成、能力位
 *   PHYSCSR (0x1F): PHY专用状态寄存器 — DM9161 的协商结果详情
 */
int DM9161_GetLinkState(void)
{
    uint32_t readval = 0;
    int32_t rtn;

    /* ── 第1步: 读取 BSR 两次 ──
     * 根据 IEEE 802.3 规范，需要连续读取两次 BSR
     * 因为第一次读取会锁存当前状态，第二次读取才是锁存后的值
     */
    rtn = HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_BSR, &readval);
    if (rtn != HAL_OK)
    {
        #if printf_cmd
        printf("HAL_ETH_ReadPHYRegister DM9161_BSR Error\r\n");
        #endif
        return DM9161_STATUS_READ_ERROR;
    }

    rtn = HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_BSR, &readval);
    if (rtn != HAL_OK)
    {
        #if printf_cmd
        printf("HAL_ETH_ReadPHYRegister DM9161_BSR Error\r\n");
        #endif
        return DM9161_STATUS_READ_ERROR;
    }

    /* ── 第2步: 检查物理链路状态 ──
     * BSR bit2 = LINK_STATUS:
     *   0 = 链路断开（网线未插、对端未上电、信号质量差）
     *   1 = 链路已建立
     */
    if ((readval & DM9161_BSR_LINK_STATUS) == 0)
    {
        return DM9161_STATUS_LINK_DOWN;
    }

    /* ── 第3步: 读取 BCR 检查自动协商是否启用 ──
     * BCR bit12 = AUTONEGO_EN:
     *   1 = 自动协商已启用
     *   0 = 强制速率/双工模式
     */
    rtn = HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_BCR, &readval);
    if (rtn != HAL_OK)
    {
        return DM9161_STATUS_READ_ERROR;
    }

    if ((readval & DM9161_BCR_AUTONEGO_EN) != DM9161_BCR_AUTONEGO_EN)
    {
        /* ── 分支A: 未启用自动协商（强制模式） ──
         * bit13=SPEED_SELECT: 1=100M, 0=10M
         * bit8=DUPLEX_MODE:   1=全双工, 0=半双工
         */
        if (((readval & DM9161_BCR_SPEED_SELECT) == DM9161_BCR_SPEED_SELECT)
         && ((readval & DM9161_BCR_DUPLEX_MODE) == DM9161_BCR_DUPLEX_MODE))
        {
            return DM9161_STATUS_100MBITS_FULLDUPLEX;
        }
        else if ((readval & DM9161_BCR_SPEED_SELECT) == DM9161_BCR_SPEED_SELECT)
        {
            return DM9161_STATUS_100MBITS_HALFDUPLEX;
        }
        else if ((readval & DM9161_BCR_DUPLEX_MODE) == DM9161_BCR_DUPLEX_MODE)
        {
            return DM9161_STATUS_10MBITS_FULLDUPLEX;
        }
        else
        {
            return DM9161_STATUS_10MBITS_HALFDUPLEX;
        }
    }
    else
    {
        /* ── 分支B: 自动协商模式 ──
         * 读取 PHYSCSR (PHY Specific Control/Status Register, 0x1F)
         * 这是 DM9161 的扩展寄存器，包含协商完成标志和协商结果
         */
        rtn = HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_PHYSCSR, &readval);
        if (rtn != HAL_OK)
        {
            return DM9161_STATUS_READ_ERROR;
        }

        /* ── 检查自动协商是否完成 ──
         * PHYSCSR bit12 = AUTONEGO_DONE:
         *   0 = 协商进行中
         *   1 = 协商完成
         */
        if ((readval & DM9161_PHYSCSR_AUTONEGO_DONE) == 0)
        {
            return DM9161_STATUS_AUTONEGO_NOTDONE;
        }

        /* ── 解析协商结果 ──
         * PHYSCSR bits[4:2] = HCDSPEEDMASK:
         *   100BTX_FD (0x18): 100M 全双工
         *   100BTX_HD (0x08): 100M 半双工
         *   10BT_FD   (0x14): 10M 全双工
         *   10BT_HD   (0x04): 10M 半双工
         */
        if ((readval & DM9161_PHYSCSR_HCDSPEEDMASK) == DM9161_PHYSCSR_100BTX_FD)
        {
            return DM9161_STATUS_100MBITS_FULLDUPLEX;
        }
        else if ((readval & DM9161_PHYSCSR_HCDSPEEDMASK) == DM9161_PHYSCSR_100BTX_HD)
        {
            return DM9161_STATUS_100MBITS_HALFDUPLEX;
        }
        else if ((readval & DM9161_PHYSCSR_HCDSPEEDMASK) == DM9161_PHYSCSR_10BT_FD)
        {
            return DM9161_STATUS_10MBITS_FULLDUPLEX;
        }
        else
        {
            return DM9161_STATUS_10MBITS_HALFDUPLEX;
        }
    }
}
