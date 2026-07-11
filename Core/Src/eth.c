/**
 ******************************************************************************
 * @file           : eth.c
 * @brief          : STM32H7 以太网 MAC 层驱动 + DM9161 PHY 适配
 * ----------------------------------------------------------------------------
 * 本文件实现 STM32H7 ETH 外设的初始化和底层数据收发
 *
 * 功能模块：
 *   1. MX_ETH_Init()           — ETH MAC 初始化 (RMII, DMA描述符)
 *   2. HAL_ETH_MspInit()       — 底层硬件 GPIO/时钟配置 (HAL回调)
 *   3. PHY_Init()              — DM9161 PHY 初始化和链路协商
 *   4. low_level_output()      — 底层以太网帧发送 (DMA)
 *   5. low_level_input()       — 底层以太网帧接收 (预留)
 *   6. bfin_EMAC_send/recv()   — SOEM 协议栈适配接口
 *
 * 硬件架构：
 *   STM32H7 ETH MAC (RMII模式)
 *        ↕ MDIO
 *   DM9161 PHY (0x01)
 *        ↕ 隔离变压器
 *   以太网 RJ45 接口
 *
 * DMA 内存布局（固定地址）：
 *   0x30040000 - Rx DMA 描述符表
 *   0x30040060 - Tx DMA 描述符表
 *   0x30040200 - Rx 缓冲队列
 *   0x30044000 - Tx 缓冲队列
 *
 * MPU 配置要求（在 main.c 中配置）：
 *   - 0x30040000~0x30043FFF: 非缓存 (DMA 描述符)
 *   - 0x30044000~0x30047FFF: 可缓存 (Tx缓冲)
 ******************************************************************************
 */

#include "eth.h"
#include "DM9000.h"

/* ============================================================================
 * DMA 描述符和缓冲区定义
 * ============================================================================
 * 以下数组位于固定物理地址（通过 __attribute__((at())) 或 #pragma）
 * 这是 STM32H7 ETH DMA 的硬件要求
 */

#if defined (__ICCARM__)   /* ── IAR 编译器 ── */
#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT];   /**< Rx DMA描述符表 */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT];   /**< Tx DMA描述符表 */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /**< Rx数据缓冲 */

#elif defined (__CC_ARM)   /* ── Keil MDK-ARM 编译器（本项目使用） ── */
__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT];
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT];
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE];
__attribute__((at(0x30044000))) uint8_t Tx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE];

#elif defined (__GNUC__)   /* ── GCC 编译器 ── */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection")));
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection")));
#endif

ETH_TxPacketConfig TxConfig;    /**< ETH 发送包配置 */

/* USER CODE BEGIN 0 */
#include "DM9161.h"
#include "string.h"
#include "gpio.h"
#include "main.h"

uint8_t  RecvLength = 0;        /**< 已接收的帧长度 */
uint32_t current_pbuf_idx = 0;  /**< 当前接收缓冲区索引 (环形缓冲) */

ETH_HandleTypeDef heth;          /**< STM32H7 ETH HAL 句柄 */
/* USER CODE END 0 */

/* ============================================================================
 * PHY_HardwareReset() — DM9161 硬件复位 (PB10/PB12)
 * ============================================================================ */
static void PHY_HardwareReset(void)
{
    ETH_RST_OFF;
    ETH_RST_Compatible_OFF;
    HAL_Delay(50);
    ETH_RST_ON;
    ETH_RST_Compatible_ON;
    HAL_Delay(150);
}

/* ============================================================================
 * PHY_InitDM9161() — 软复位 + PHY ID 校验, 失败时重试
 * ============================================================================ */
static int PHY_InitDM9161(void)
{
    uint32_t phyi1r = 0;
    uint32_t phyi2r = 0;

    HAL_Delay(100);

    for (int attempt = 0; attempt < 5; attempt++)
    {
        if (attempt > 0)
        {
            #if printf_cmd
            printf("[PHY] DM9161 retry %d: hardware reset\r\n", attempt);
            #endif
            PHY_HardwareReset();
        }

        if (DM9161_Init() != 0)
            continue;

        if (HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_PHYI1R, &phyi1r) != HAL_OK)
            continue;
        if (HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_PHYI2R, &phyi2r) != HAL_OK)
            continue;

        if (phyi1r == 0x0000U && phyi2r == 0x0000U)
            continue;

        #if printf_cmd
        printf("[PHY] DM9161 OK on attempt %d (PHYI1R=0x%04lX PHYI2R=0x%04lX)\r\n",
               attempt + 1, phyi1r, phyi2r);
        #endif
        return 1;
    }

    #if printf_cmd
    printf("[PHY] DM9161 init FAILED after 5 attempts\r\n");
    #endif
    return 0;
}

/* ============================================================================
 * MX_ETH_Init() — ETH MAC 初始化
 * ============================================================================
 * 由 CubeMX 自动生成，配置：
 *   - MAC 地址: 01:01:01:01:01:01 (临时地址，EtherCAT 会重新配置)
 *   - 接口: RMII (Reduced MII, 2位数据线)
 *   - DMA 描述符指针
 *   - 接收缓冲长度: 1524 字节 (最大以太网帧 + 对齐)
 *   - 发送配置: IP校验和 + CRC填充
 *
 * 初始化后：
 *   - MAC 处于禁用状态（等待 HAL_ETH_Start_IT 启动）
 *   - DMA 描述符尚未关联缓冲（由 PHY_Init 中完成）
 */
void MX_ETH_Init(void)
{
    /* ── 配置 MAC 基本参数 ── */
    heth.Instance = ETH;
    heth.Init.MACAddr[0] = 0x01;   // MAC 地址字节0
    heth.Init.MACAddr[1] = 0x01;   // MAC 地址字节1
    heth.Init.MACAddr[2] = 0x01;   // MAC 地址字节2
    heth.Init.MACAddr[3] = 0x01;   // MAC 地址字节3
    heth.Init.MACAddr[4] = 0x01;   // MAC 地址字节4
    heth.Init.MACAddr[5] = 0x01;   // MAC 地址字节5
    heth.Init.MediaInterface = HAL_ETH_RMII_MODE;   // RMII 接口模式
    heth.Init.TxDesc = DMATxDscrTab;                 // 发送DMA描述符表
    heth.Init.RxDesc = DMARxDscrTab;                 // 接收DMA描述符表
    heth.Init.RxBuffLen = 1524;                      // 接收最大帧长

    if (HAL_ETH_Init(&heth) != HAL_OK)
    {
        Error_Handler();  // ETH 初始化失败 → 死循环
    }

    /* ── 配置默认发送参数 ──
     * 设置 IP 校验和自动计算 + 自动 CRC 填充
     * 注意: EtherCAT 帧不使用 IP 协议，但此配置对非EtherCAT帧有效
     */
    memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
    TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM
                        | ETH_TX_PACKETS_FEATURES_CRCPAD;
    TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
    TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
}

/* ============================================================================
 * HAL_ETH_MspInit() — ETH 底层硬件初始化（HAL回调）
 * ============================================================================
 * 由 HAL_ETH_Init() 自动调用
 *
 * 配置：
 *   1. 使能 ETH MAC/TX/RX 时钟
 *   2. 使能 RMII 相关的 GPIO 时钟
 *   3. 配置 RMII 引脚 (PG11/13/14, PC1/4/5, PA1/2/7)
 *
 * RMII 引脚映射：
 *   TXD0=PG13, TXD1=PG14, TX_EN=PG11
 *   MDC=PC1, RXD0=PC4, RXD1=PC5
 *   REF_CLK=PA1, MDIO=PA2, CRS_DV=PA7
 *   复用功能: AF11 (ETH)
 */
void HAL_ETH_MspInit(ETH_HandleTypeDef *ethHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (ethHandle->Instance == ETH)
    {
        /* ── 使能 ETH 外设时钟 ── */
        __HAL_RCC_ETH1MAC_CLK_ENABLE();    // ETH MAC 时钟
        __HAL_RCC_ETH1TX_CLK_ENABLE();     // ETH TX 时钟
        __HAL_RCC_ETH1RX_CLK_ENABLE();     // ETH RX 时钟

        /* ── 使能 GPIO 时钟 ── */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();

        /**
         * ETH GPIO 引脚映射:
         *   PG14 → ETH_TXD1
         *   PG13 → ETH_TXD0
         *   PG11 → ETH_TX_EN
         *   PC1  → ETH_MDC
         *   PA1  → ETH_REF_CLK
         *   PC4  → ETH_RXD0
         *   PA2  → ETH_MDIO
         *   PC5  → ETH_RXD1
         *   PA7  → ETH_CRS_DV
         */

        /* ── PG 组: TX 数据引脚 ── */
        GPIO_InitStruct.Pin = RMII_TXD1_Pin | RMII_TXD0_Pin | RMII_TX_EN_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;            // 复用推挽
        GPIO_InitStruct.Pull = GPIO_NOPULL;                // 无上下拉
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;      // 高速
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;         // AF11 = ETH
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        /* ── PC 组: MDC + RX 数据 ── */
        GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* ── PA 组: REF_CLK + MDIO + CRS_DV ── */
        GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* ============================================================================
 * HAL_ETH_MspDeInit() — ETH 底层反初始化（HAL回调）
 * ============================================================================
 */
void HAL_ETH_MspDeInit(ETH_HandleTypeDef *ethHandle)
{
    if (ethHandle->Instance == ETH)
    {
        /* ── 关闭时钟 ── */
        __HAL_RCC_ETH1MAC_CLK_DISABLE();
        __HAL_RCC_ETH1TX_CLK_DISABLE();
        __HAL_RCC_ETH1RX_CLK_DISABLE();

        /* ── 释放 GPIO ── */
        HAL_GPIO_DeInit(GPIOG, RMII_TXD1_Pin | RMII_TXD0_Pin | RMII_TX_EN_Pin);
        HAL_GPIO_DeInit(GPIOC, RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin);
        HAL_GPIO_DeInit(GPIOA, RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin);
    }
}

/* USER CODE BEGIN 1 */

/* ============================================================================
 * PHY_Init() — DM9161 PHY 初始化和链路协商
 * ============================================================================
 * 初始化流程：
 *   1. 设置 MAC 为混杂模式（接收所有帧，EtherCAT 要求）
 *   2. 关联 DMA 接收描述符与数据缓冲区
 *   3. 软复位 DM9161 PHY (通过 DM9161_Init)
 *   4. 轮询检测链路状态（最多10次，每次100ms）
 *   5. 根据协商结果配置 MAC 速率和双工模式
 *   6. 启动 ETH MAC 并使能中断
 *   7. 重建接收描述符
 *
 * 混杂模式原因：
 *   EtherCAT 使用特殊以太类型 (0x88A4)，某些从站可能不响应
 *   标准地址过滤，因此开启混杂模式确保所有帧被接收
 */
void PHY_Init(void)
{
    uint32_t idx, duplex, speed = 0;
    int32_t PHYLinkState;
    uint8_t i = 0;
    ETH_MACConfigTypeDef MACConf;
    ETH_MACFilterConfigTypeDef filterDef;

    /* ── 步骤1: 设置 MAC 为混杂模式 ──
     * 混杂模式: 不过滤目标 MAC 地址，接收所有帧
     * 这对 EtherCAT 主站至关重要（EtherCAT 使用特殊帧格式）
     */
    HAL_ETH_GetMACFilterConfig(&heth, &filterDef);
    filterDef.PromiscuousMode = ENABLE;         // 启用混杂模式
    HAL_ETH_SetMACFilterConfig(&heth, &filterDef);

    /* ── 步骤2: 将 DMA 描述符绑定到接收缓冲区 ──
     * 每个 DMA 描述符指向一个接收缓冲
     * 接收时 DMA 自动将数据写入对应缓冲
     */
    for (idx = 0; idx < ETH_RX_DESC_CNT; idx++)
    {
        HAL_ETH_DescAssignMemory(&heth, idx, Rx_Buff[idx], NULL);
    }

    /* ── 步骤3: DM9161 PHY 初始化（软复位 + 重试） ── */
    if (!PHY_InitDM9161())
    {
        #if printf_cmd
        printf("[PHY] WARNING: DM9161 not responding, continue link poll\r\n");
        #endif
    }

    /* ── 步骤4: 轮询等待链路建立 ──
     * 最多重试50次，每次间隔100ms (共5秒)
     * DM9161 软复位后自动协商需要 1~3 秒
     */
    do
    {
        HAL_Delay(100);
        PHYLinkState = DM9161_GetLinkState();
        #if printf_cmd
        printf("DM9161_GetLinkState = %d\r\n", PHYLinkState);
        #endif

        ERR_TOGGLE;

        i++;
        if (i > 50)
        {
            #if printf_cmd
            printf("DM9161 STATUS is ERROR\r\n");
            #endif
            break;
        }
    } while (PHYLinkState <= DM9161_STATUS_LINK_DOWN);

    /* ── 步骤5: 根据协商结果配置 MAC ── */
    switch (PHYLinkState)
    {
    case DM9161_STATUS_100MBITS_FULLDUPLEX:
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        #if printf_cmd
        printf("DM9161_STATUS_100MBITS_FULLDUPLEX\r\n");
        #endif
        break;
    case DM9161_STATUS_100MBITS_HALFDUPLEX:
        duplex = ETH_HALFDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        #if printf_cmd
        printf("DM9161_STATUS_100MBITS_HALFDUPLEX\r\n");
        #endif
        break;
    case DM9161_STATUS_10MBITS_FULLDUPLEX:
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_10M;
        #if printf_cmd
        printf("DM9161_STATUS_10MBITS_FULLDUPLEX\r\n");
        #endif
        break;
    case DM9161_STATUS_10MBITS_HALFDUPLEX:
        duplex = ETH_HALFDUPLEX_MODE;
        speed = ETH_SPEED_10M;
        #if printf_cmd
        printf("DM9161_STATUS_10MBITS_HALFDUPLEX\r\n");
        #endif
        break;
    default:
        /* 异常情况：默认使用 100M 全双工 */
        duplex = ETH_FULLDUPLEX_MODE;
        speed = ETH_SPEED_100M;
        #if printf_cmd
        printf("ETH_FULLDUPLEX_MODE ETH_SPEED_100M\r\n");
        #endif
        break;
    }

    /* ── 步骤6: 应用 MAC 配置 ── */
    HAL_ETH_GetMACConfig(&heth, &MACConf);
    MACConf.DuplexMode = duplex;
    MACConf.Speed = speed;
    MACConf.TransmitQueueMode = ETH_TRANSMITTHRESHOLD_128;  // 发送队列阈值128字节
    HAL_ETH_SetMACConfig(&heth, &MACConf);

    /* ── 步骤7: 启动 ETH MAC ──
     * HAL_ETH_Start_IT: 启动并使能中断（接收/发送完成中断）
     */
    HAL_ETH_Start_IT(&heth);

    /* ── 步骤8: 重建接收描述符链 ── */
    HAL_ETH_BuildRxDescriptors(&heth);
}

/* ============================================================================
 * low_level_output() — 底层以太网帧发送
 * ============================================================================
 * @param p      : 发送数据指针
 * @param length : 数据长度（字节）
 *
 * 这是 SOEM 协议栈调用的底层发送函数
 *
 * 发送流程：
 *   1. 准备 TxBuffer (描述符 + 数据指针)
 *   2. 清理 D-Cache (确保数据对 DMA 可见)
 *   3. 调用 HAL_ETH_Transmit 启动 DMA 发送
 *   4. 设置 sendfinishflag 标记发送进行中
 *
 * 注意：
 *   - 使用 STM32H7 HAL 的 DMA 发送模式（非阻塞）
 *   - 超时时间: 5ms（SOEM 建议值）
 *   - D-Cache 必须 clean+invalidate（STM32H7 Cache 一致性要求）
 */
uint32_t sendfinishflag = 0;    /**< 发送完成标志: 0=空闲, 1=发送中 */

void low_level_output(uint8_t *p, uint32_t length)
{
    uint32_t framelen = 0;
    ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT];

    /* ── 准备发送缓冲描述符 ── */
    memset(Txbuffer, 0, ETH_TX_DESC_CNT * sizeof(ETH_BufferTypeDef));
    Txbuffer[0].buffer = p;          // 数据指针
    Txbuffer[0].len = length;        // 数据长度
    framelen += length;

    TxConfig.Length = framelen;      // 设置发送长度
    TxConfig.TxBuffer = Txbuffer;    // 绑定缓冲区

    /* ── 清理 D-Cache ──
     * STM32H7 使用 write-through 模式，但为了 DMA 一致性
     * 仍然需要 clean+invalidate 确保数据已写入物理内存
     */
    SCB_CleanInvalidateDCache();

    /* ── 启动 DMA 发送 ── */
    HAL_ETH_Transmit(&heth, &TxConfig, 5);  // 超时 5ms
    sendfinishflag = 1;                      // 标记发送进行中
}

/* ============================================================================
 * low_level_input() — 底层以太网帧接收（预留接口）
 * ============================================================================
 * 当前为空实现，实际接收通过 bfin_EMAC_recv() 完成
 * 此函数在 HAL_ETH_RxCpltCallback 中被调用
 */
void low_level_input(void)
{
    // 预留：中断驱动的接收处理
}

/* ============================================================================
 * HAL_ETH_RxCpltCallback() — 接收完成中断回调
 * ============================================================================
 * 当 DMA 完成一帧接收时由 HAL 中断处理调用
 * 当前仅打印调试信息
 */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
    low_level_input();
    #if printf_cmd
    printf("rx isr\r\n");  // 调试: 接收到帧
    #endif
}

/* ============================================================================
 * HAL_ETH_TxCpltCallback() — 发送完成中断回调
 * ============================================================================
 * 当 DMA 完成一帧发送时由 HAL 中断处理调用
 * 清除 sendfinishflag，允许下一帧发送
 */
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
    sendfinishflag = 0;     // 清除发送标志
    #if printf_cmd
    printf("tx isr\r\n");   // 调试: 发送完成
    #endif
}

/* ============================================================================
 * bfin_EMAC_send() — SOEM 协议栈发送接口
 * ============================================================================
 * @param packet : 以太网帧数据指针
 * @param length : 帧长度
 * @retval 0 : 成功
 *
 * 这是 SOEM (Simple Open EtherCAT Master) 协议栈调用的发送接口
 * 将帧数据从 SOEM 缓冲区复制到 DMA Tx 缓冲，然后调用底层发送
 *
 * 参数说明：
 *   - void *packet: SOEM 内部帧缓冲区
 *   - int length: 完整以太网帧长度 (不含前导码和FCS)
 */
int bfin_EMAC_send(void *packet, int length)
{
    /* ── 从 SOEM 缓冲区复制到 DMA Tx 缓冲区 ──
     * 使用 Tx_Buff[0] 作为发送缓冲区
     * Tx_Buff 位于 0x30044000 (可缓存区域)
     */
    memcpy(&Tx_Buff[0][0], packet, length);

    /* ── 调用底层发送 ── */
    low_level_output(Tx_Buff[0], length);

    return 0;
}

/* ============================================================================
 * bfin_EMAC_recv() — SOEM 协议栈接收接口
 * ============================================================================
 * @param packet : 接收数据缓冲区 (SOEM 提供)
 * @param size   : 缓冲区大小
 * @retval >0   : 接收到的帧长度
 * @retval -1   : 无数据/接收失败
 *
 * 这是 SOEM 协议栈调用的接收接口
 *
 * 接收流程：
 *   1. 清理 D-Cache（确保读到最新的 DMA 数据）
 *   2. 获取接收数据缓冲指针
 *   3. 获取帧长度
 *   4. 失效 D-Cache（使缓存行失效，确保读取物理内存）
 *   5. 从 Rx 缓冲复制到 SOEM 缓冲区
 *   6. 更新接收缓冲索引（环形缓冲）
 *   7. 重建接收 DMA 描述符
 *
 * 注意：
 *   - current_pbuf_idx 实现环形缓冲索引
 *   - 每次接收后必须重建 Rx 描述符（归还 DMA 所有权）
 */
int bfin_EMAC_recv(uint8_t *packet, size_t size)
{
    ETH_BufferTypeDef RxBuff;
    uint32_t framelength = 0;

    /* ── 清理 D-Cache ── */
    SCB_CleanInvalidateDCache();

    /* ── 获取接收数据缓冲 ── */
    HAL_StatusTypeDef status = HAL_ETH_GetRxDataBuffer(&heth, &RxBuff);

    if (status == HAL_OK)
    {
        /* ── 获取接收帧长度 ── */
        HAL_ETH_GetRxDataLength(&heth, &framelength);

        /* ── 使 D-Cache 中对应的接收缓冲区域失效 ──
         * 确保后续读取的数据来自 DMA 写入的最新值
         */
        SCB_InvalidateDCache_by_Addr((uint32_t *)Rx_Buff,
                                      ETH_RX_DESC_CNT * ETH_MAX_PACKET_SIZE);

        /* ── 复制到 SOEM 缓冲区 ── */
        memcpy(packet, Rx_Buff[current_pbuf_idx], framelength);

        /* ── 更新环形缓冲索引 ── */
        if (current_pbuf_idx < (ETH_RX_DESC_CNT - 1))
        {
            current_pbuf_idx++;
        }
        else
        {
            current_pbuf_idx = 0;  // 回卷到起始
        }

        /* ── 重建接收描述符 ──
         * 将 DMA 描述符所有权归还给硬件
         * 必须在每次读取后调用
         */
        HAL_ETH_BuildRxDescriptors(&heth);

        receiveLen = framelength;   // 记录接收长度（全局变量）
        return framelength;
    }

    return -1;  // 无数据
}
/* USER CODE END 1 */
