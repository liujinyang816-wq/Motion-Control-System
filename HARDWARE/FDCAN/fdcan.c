/**
 ******************************************************************************
 * @file           : fdcan.c
 * @brief          : STM32H7 FDCAN1 驱动 — CAN 总线收发
 * @author         : 正点原子 (ALIENTEK)
 * @version        : V1.0
 * @date           : 2018-06-29
 * ----------------------------------------------------------------------------
 * FDCAN (Flexible Data-rate CAN) — 支持 CAN 2.0 和 CAN FD
 * 本项目使用经典 CAN 2.0 模式 (Classic CAN)
 *
 * 硬件连接：
 *   - FDCAN1_RX: PB8 (AF9)
 *   - FDCAN1_TX: PB9 (AF9)
 *
 * CAN 总线配置 (默认)：
 *   - 波特率: 500 Kbps
 *   - 时钟源: PLL1Q = 200MHz
 *   - 分频: presc=10, tsg1=8, tsg2=31 → (200M/10)/(8+31+1) = 500Kbps
 *
 * 功能：
 *   1. FDCAN1_Mode_Init()   — 初始化 CAN 控制器
 *   2. FDCAN1_Send_Msg()    — 发送 CAN 消息 (标准帧, ID=0x12)
 *   3. FDCAN1_Receive_Msg() — 接收 CAN 消息 (轮询)
 ******************************************************************************
 */
#include "fdcan.h"
#include "usart.h"

/* ── FDCAN 全局变量 ── */
FDCAN_HandleTypeDef FDCAN1_Handler;      /**< FDCAN1 HAL 句柄 */
FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;   /**< 接收帧头信息 */
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;   /**< 发送帧头信息 */

/* ============================================================================
 * FDCAN1_Mode_Init() — FDCAN1 初始化
 * ============================================================================
 * @param presc : 预分频值 (1~512)
 * @param ntsjw : 同步跳转宽度 (1~128)
 * @param ntsg1 : 时间段1 (2~256)
 * @param ntsg2 : 时间段2 (2~128)
 * @param mode  : 工作模式 (FDCAN_MODE_NORMAL / FDCAN_MODE_EXTERNAL_LOOPBACK)
 * @retval 0 : 成功, 1: HAL初始化失败, 2: 滤波器配置失败
 *
 * CAN 波特率 = FDCAN时钟 / (presc × (ntsg1 + ntsg2 + ntsjw))
 *
 * 配置说明：
 *   - 经典CAN模式 (FDCAN_FRAME_CLASSIC)
 *   - 使用 TX FIFO 操作模式
 *   - 接收 FIFO0: 1个元素, 8字节数据宽度
 *   - 标准帧过滤器: 接受所有帧 (ID1=0, ID2=0, Mask模式)
 */
uint8_t FDCAN1_Mode_Init(uint16_t presc, uint8_t ntsjw,
                         uint16_t ntsg1, uint8_t ntsg2, uint32_t mode)
{
    FDCAN_FilterTypeDef FDCAN1_RXFilter;

    /* ── 第1步: 反初始化之前的配置 ── */
    HAL_FDCAN_DeInit(&FDCAN1_Handler);

    /* ── 第2步: 配置 FDCAN 参数 ── */
    FDCAN1_Handler.Instance = FDCAN1;
    FDCAN1_Handler.Init.FrameFormat = FDCAN_FRAME_CLASSIC;       // 经典CAN 2.0
    FDCAN1_Handler.Init.Mode = mode;                              // 普通/环回模式
    FDCAN1_Handler.Init.AutoRetransmission = DISABLE;             // 禁止自动重发
    FDCAN1_Handler.Init.TransmitPause = DISABLE;                  // 禁止发送暂停
    FDCAN1_Handler.Init.ProtocolException = DISABLE;              // 禁止协议异常

    /* ── CAN 位时序配置 ── */
    FDCAN1_Handler.Init.NominalPrescaler = presc;                 // 预分频
    FDCAN1_Handler.Init.NominalSyncJumpWidth = ntsjw;            // 同步跳转宽度
    FDCAN1_Handler.Init.NominalTimeSeg1 = ntsg1;                 // 相位段1+传播段
    FDCAN1_Handler.Init.NominalTimeSeg2 = ntsg2;                 // 相位段2

    /* ── 消息RAM配置 ── */
    FDCAN1_Handler.Init.MessageRAMOffset = 0;                     // 消息RAM偏移
    FDCAN1_Handler.Init.StdFiltersNbr = 0;                        // 标准帧过滤器数量
    FDCAN1_Handler.Init.ExtFiltersNbr = 0;                        // 扩展帧过滤器数量
    FDCAN1_Handler.Init.RxFifo0ElmtsNbr = 1;                      // RX FIFO0: 1个元素
    FDCAN1_Handler.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;     // 每个8字节
    FDCAN1_Handler.Init.RxBuffersNbr = 0;                         // RX Buffer数量
    FDCAN1_Handler.Init.TxEventsNbr = 0;                          // TX Event数量
    FDCAN1_Handler.Init.TxBuffersNbr = 0;                         // TX Buffer数量
    FDCAN1_Handler.Init.TxFifoQueueElmtsNbr = 1;                  // TX FIFO: 1个元素
    FDCAN1_Handler.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; // TX FIFO模式
    FDCAN1_Handler.Init.TxElmtSize = FDCAN_DATA_BYTES_8;          // 发送8字节

    if (HAL_FDCAN_Init(&FDCAN1_Handler) != HAL_OK) return 1;

    /* ── 第3步: 配置接收过滤器 ──
     * 使用 Mask 模式: ID1 和 ID2 共同定义过滤器
     * 此处设 ID1=0, ID2=0 → 接受所有标准帧
     */
    FDCAN1_RXFilter.IdType = FDCAN_STANDARD_ID;                  // 标准ID (11位)
    FDCAN1_RXFilter.FilterIndex = 0;                             // 过滤器索引0
    FDCAN1_RXFilter.FilterType = FDCAN_FILTER_MASK;              // 掩码模式
    FDCAN1_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;      // 存入FIFO0
    FDCAN1_RXFilter.FilterID1 = 0x0000;                          // ID1 (掩码模式)
    FDCAN1_RXFilter.FilterID2 = 0x0000;                          // ID2 (掩码模式)

    if (HAL_FDCAN_ConfigFilter(&FDCAN1_Handler, &FDCAN1_RXFilter) != HAL_OK)
        return 2;

    /* ── 第4步: 启动FDCAN ── */
    HAL_FDCAN_Start(&FDCAN1_Handler);
    HAL_FDCAN_ActivateNotification(&FDCAN1_Handler,
                                    FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    return 0;
}

/* ============================================================================
 * HAL_FDCAN_MspInit() — FDCAN 底层硬件初始化（HAL回调）
 * ============================================================================
 * 由 HAL_FDCAN_Init() 自动调用
 *
 * 配置：
 *   - 使能 FDCAN1 外设时钟
 *   - 配置 PB8(RX), PB9(TX) 为 AF9(FDCAN1)
 *   - 配置时钟源为 PLL1Q (200MHz)
 *   - 可选：使能 RX0 中断
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    GPIO_InitTypeDef GPIO_Initure;
    RCC_PeriphCLKInitTypeDef FDCAN_PeriphClk;

    /* ── 使能时钟 ── */
    __HAL_RCC_FDCAN_CLK_ENABLE();              // FDCAN 外设时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();              // GPIOB 时钟 (PB8/PB9)

    /* ── 配置 FDCAN 时钟源为 PLL1Q ── */
    FDCAN_PeriphClk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    FDCAN_PeriphClk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    HAL_RCCEx_PeriphCLKConfig(&FDCAN_PeriphClk);

    /* ── 配置 CAN TX (PB9) 和 RX (PB8) ── */
    GPIO_Initure.Pin = GPIO_PIN_8 | GPIO_PIN_9;    // PB8=RX, PB9=TX
    GPIO_Initure.Mode = GPIO_MODE_AF_PP;            // 复用推挽
    GPIO_Initure.Pull = GPIO_PULLUP;                // 上拉 (CAN隐性电平)
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_MEDIUM;    // 中等速度
    GPIO_Initure.Alternate = GPIO_AF9_FDCAN1;       // AF9 = FDCAN1
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);

#if FDCAN1_RX0_INT_ENABLE
    /* ── 可选: 配置接收中断 ── */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 2);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
#endif
}

/* ============================================================================
 * HAL_FDCAN_MspDeInit() — FDCAN 底层反初始化 (HAL回调)
 * ============================================================================
 */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    __HAL_RCC_FDCAN_FORCE_RESET();             // 强制复位
    __HAL_RCC_FDCAN_RELEASE_RESET();           // 释放复位

#if FDCAN1_RX0_INT_ENABLE
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
#endif
}

/* ============================================================================
 * FDCAN1_Send_Msg() — 发送 CAN 消息
 * ============================================================================
 * @param msg : 8字节消息内容
 * @param len : 数据长度 (FDCAN_DLC_BYTES_2 ~ FDCAN_DLC_BYTES_8)
 * @retval 0 : 发送成功
 * @retval 1 : 发送失败
 *
 * 固定参数：
 *   - 标准帧 (11位ID)
 *   - CAN ID = 0x12
 *   - 数据帧（非远程帧）
 *   - 经典 CAN 2.0 格式
 */
uint8_t FDCAN1_Send_Msg(uint8_t *msg, uint32_t len)
{
    /* ── 配置发送帧头 ── */
    FDCAN1_TxHeader.Identifier = 0x12;                    // CAN ID = 0x12
    FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;           // 标准帧 (11位)
    FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;       // 数据帧
    FDCAN1_TxHeader.DataLength = len;                     // 数据长度 (DLC)
    FDCAN1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        // 无速率切换
    FDCAN1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;         // 经典CAN格式
    FDCAN1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
    FDCAN1_TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&FDCAN1_Handler, &FDCAN1_TxHeader, msg) != HAL_OK)
        return 1;  // 发送失败

    return 0;
}

/* ============================================================================
 * FDCAN1_Receive_Msg() — 接收 CAN 消息（轮询模式）
 * ============================================================================
 * @param buf : 8字节接收缓冲区
 * @retval 0 : 无数据
 * @retval >0: 实际数据长度 (DLC >> 16)
 *
 * 从 RX FIFO0 读取一条消息
 * 如果 FIFO 为空，返回 0
 */
uint8_t FDCAN1_Receive_Msg(uint8_t *buf)
{
    if (HAL_FDCAN_GetRxMessage(&FDCAN1_Handler, FDCAN_RX_FIFO0,
                                &FDCAN1_RxHeader, buf) != HAL_OK)
        return 0;  // 无数据

    return FDCAN1_RxHeader.DataLength >> 16;  // 返回实际长度
}

#if FDCAN1_RX0_INT_ENABLE
/* ============================================================================
 * FDCAN1_IT0_IRQHandler() — FDCAN1 接收中断服务
 * ============================================================================
 * 调用 HAL 的中断处理函数，最终触发 RxFifo0Callback
 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&FDCAN1_Handler);
}

/* ============================================================================
 * HAL_FDCAN_RxFifo0Callback() — FIFO0 接收回调
 * ============================================================================
 * 接收到新消息时自动调用
 * 打印消息 ID、长度和数据内容 → 用于调试
 * 重新激活接收通知以接收下一条消息
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    u8 i = 0;
    u8 rxdata[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)  // 新消息
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                &FDCAN1_RxHeader, rxdata);

        /* ── 打印消息详情 ── */
        #if printf_cmd
        printf("id:%#x\r\n", FDCAN1_RxHeader.Identifier);
        #endif
        #if printf_cmd
        printf("len:%d\r\n", FDCAN1_RxHeader.DataLength >> 16);
        #endif
        for (i = 0; i < 8; i++)
            #if printf_cmd
            printf("rxdata[%d]:%d\r\n", i, rxdata[i]);
            #endif

        /* 重新激活通知 */
        HAL_FDCAN_ActivateNotification(hfdcan,
                                        FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}
#endif
