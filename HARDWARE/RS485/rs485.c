/**
 ******************************************************************************
 * @file           : rs485.c
 * @brief          : RS485 收发驱动 — DMA 版本 (USART3 + DMA1 Stream1/3)
 ******************************************************************************
 * 硬件连接：
 *   - USART3_TX: PC10 (AF7)
 *   - USART3_RX: PC11 (AF7)
 *   - RS485_DIR: PC12 (GPIO, 0=接收, 1=发送)
 *
 * DMA 通道 (STM32H7 DMAMUX):
 *   - RX: DMA1 Stream1, Request = DMA_REQUEST_USART3_RX (DMAMUX ID 29)
 *   - TX: DMA1 Stream3, Request = DMA_REQUEST_USART3_TX (DMAMUX ID 30)
 *
 * RX 流程:
 *   1. HAL_UARTEx_ReceiveToIdle_DMA() 启动接收
 *   2. DMA 后台自动搬运 USART3 DR → RS485_RX_BUF[]
 *   3. 总线空闲 ≥1字节时间 → USART3 IDLE 中断
 *   4. HAL 自动停 DMA → HAL_UARTEx_RxEventCallback()
 *   5. 回调中记录 RS485_RX_CNT → 立即重新调用 ReceiveToIdle_DMA()
 *
 * TX 流程:
 *   1. DIR=1 (发送模式) + 启动超时保护 (RS485_TC_Timeout=50)
 *   2. HAL_UART_Transmit_DMA() → 立即返回, DMA 后台搬运
 *   3. DMA 搬运完成 → HAL 内部 UART_DMATransmitCplt → 开启 TCIE
 *   4. 硬件 TC 置位 → USART3_IRQHandler → HAL_UART_IRQHandler
 *      → UART_EndTransmit_IT → HAL_UART_TxCpltCallback()
 *      → 清 TC 标志 + DIR=0 (接收模式) + TX_Busy=0
 *
 * ⚠️ 关键陷阱:
 *   - IDLE 后 HAL 停止 DMA, 必须重新调用 ReceiveToIdle_DMA() 否则后续帧丢失
 *   - DMA 搬完 ≠ 移位寄存器发完, 必须等 TC 再切方向, 否则截断停止位
 *   - TC 标志由 HAL_UART_TxCpltCallback 显式清除, 防止残留触发下次传输
 *   - 方向切换全部在 HAL_UART_TxCpltCallback 中完成, USART3_IRQHandler 无需额外处理
 *   - TX DMA 中断优先级必须最低(15), 防止抢占 RX/IDLE
 *   - 所有 ISR 统一放在 stm32h7xx_it.c, 不在本文件中定义
 ******************************************************************************
 */
#include "rs485.h"
#include <string.h>             /* memcpy for RS485_Send_Data DMA-safe copy */

UART_HandleTypeDef USART3_RS485Handler;  /**< USART3 HAL 句柄 */

/* ── DMA 句柄 ── */
DMA_HandleTypeDef hdma_usart3_rx;                   //< USART3 RX DMA 句柄
DMA_HandleTypeDef hdma_usart3_tx;                   //< USART3 TX DMA 句柄

/* ── 接收缓冲区 ── */
uint8_t RS485_RX_BUF[260];        /**< 接收缓冲区 (与 MB_RX_BUF_SIZE 对齐, 防止大帧溢出)
                                   *   ⚠️ 由 SOEM_H7_custom.sct 强制放入 AXI SRAM (DMA 可访问) */
uint8_t RS485_TX_BUF[260];        /**< DMA 发送缓冲区: RS485_Send_Data 先 memcpy 再启动 DMA
                                   *   原因: 调用者的栈缓冲区可能在 DTCM, DMA 不可达
                                   *   ⚠️ 由 SOEM_H7_custom.sct 强制放入 AXI SRAM (DMA 可访问) */
uint16_t RS485_RX_CNT = 0;       /**< uint16_t: 缓冲区 260B, uint8_t 最大 255 不够容纳极端帧 */
uint8_t RS485_Receive_Over = 0;
uint8_t  RS485_TX_Busy = 0;     /**< DMA 发送忙标志: 1=正在发送, 禁止重复调用 */
uint32_t RS485_TC_Timeout = 0;  /**< TC 超时计数器 (50ms @ 1kHz, 0=未启用) */

/* ═══════════════════════════════════════════════════════════════════
   注意: USART3_IRQHandler / DMA1_Stream1_IRQHandler / DMA1_Stream3_IRQHandler
   已统一迁移到 stm32h7xx_it.c，不要在 rs485.c 中重复定义！
   ═══════════════════════════════════════════════════════════════════ */

/* ========================================================================
   RS485_Init() — RS485 初始化 (含 DMA 配置)
   ======================================================================== */
void RS485_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_Initure;

    /* ── 使能时钟 ── */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

#if EN_USART3_DMA
    __HAL_RCC_DMA1_CLK_ENABLE();   /* DMA1 时钟 (DMA 模式需要) */
#endif

    /* ── TX (PC10) + RX (PC11): AF7 ── */
    GPIO_Initure.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_Initure.Mode      = GPIO_MODE_AF_PP;
    GPIO_Initure.Pull      = GPIO_PULLUP;
    GPIO_Initure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_Initure.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);

    /* ── DIR (PC12): 推挽输出, 默认低 (接收) ── */
    GPIO_Initure.Pin       = GPIO_PIN_12;
    GPIO_Initure.Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_Initure.Pull      = GPIO_PULLDOWN;
    GPIO_Initure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);

    /* ── USART3: 8-N-1 ── */
    USART3_RS485Handler.Instance          = USART3;
    USART3_RS485Handler.Init.BaudRate     = bound;
    USART3_RS485Handler.Init.WordLength   = UART_WORDLENGTH_8B;
    USART3_RS485Handler.Init.StopBits     = UART_STOPBITS_1;
    USART3_RS485Handler.Init.Parity       = UART_PARITY_NONE;
    USART3_RS485Handler.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    USART3_RS485Handler.Init.Mode         = UART_MODE_TX_RX;
    HAL_UART_Init(&USART3_RS485Handler);

    __HAL_UART_CLEAR_IT(&USART3_RS485Handler, UART_CLEAR_TCF);

#if EN_USART3_RX
#if EN_USART3_DMA
    /* ═══════════════════════════════════════════════════════════════
       DMA 模式: 配置 RX DMA + 启动 ReceiveToIdle_DMA
       ═══════════════════════════════════════════════════════════════ */
    /* ── 配置 DMA1_Stream1: 外设→内存, USART3_RX ──
     * 注意: 不要设循环模式！ReceiveToIdle_DMA 内部用普通模式,
     * IDLE 后手动重新启动 */
    hdma_usart3_rx.Instance                 = DMA1_Stream1;
    hdma_usart3_rx.Init.Request             = DMA_REQUEST_USART3_RX;  /* DMAMUX Request ID */
    hdma_usart3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;       /* 外设地址固定(DR寄存器) */
    hdma_usart3_rx.Init.MemInc              = DMA_MINC_ENABLE;        /* 内存地址递增 */
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;      // 外设数据宽度 1B
    hdma_usart3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;      // 内存数据宽度 1B
    hdma_usart3_rx.Init.Mode                = DMA_NORMAL;             /* 普通模式 */
    hdma_usart3_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart3_rx);

    /* 将 DMA 句柄关联到 UART 句柄 (HAL 内部用) */
    __HAL_LINKDMA(&USART3_RS485Handler, hdmarx, hdma_usart3_rx);

    /* ── 配置 DMA1_Stream3: 内存→外设, USART3_TX ── */
    hdma_usart3_tx.Instance                 = DMA1_Stream3;
    hdma_usart3_tx.Init.Request             = DMA_REQUEST_USART3_TX;  /* DMAMUX Request ID */
    hdma_usart3_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority            = DMA_PRIORITY_LOW;       /* TX 用低优先级 */
    HAL_DMA_Init(&hdma_usart3_tx);

    __HAL_LINKDMA(&USART3_RS485Handler, hdmatx, hdma_usart3_tx);

    /* ── 启动 IDLE + DMA 接收 ── */
    HAL_UARTEx_ReceiveToIdle_DMA(&USART3_RS485Handler, RS485_RX_BUF, 260);          // 启动 DMA 接收, 260B 缓冲区

    /* ── NVIC: USART3 + DMA1_Stream1/3 ── */
    HAL_NVIC_SetPriority(USART3_IRQn,       14, 0);   /* IDLE 中断, 优先级14 */
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,  14, 1);   /* RX DMA, 优先级14 (与 IDLE 同级) */
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn,  15, 0);   /* TX DMA, 优先级15 (最低) */
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    /* 注意: TX DMA 优先级15 (最低), 防止抢占 RX/IDLE 导致丢帧 */

#else
    /* ═══════════════════════════════════════════════════════════════
       中断模式 (EN_USART3_DMA=0): 保留原有 RXNE 逐字节接收
       ═══════════════════════════════════════════════════════════════ */
    __HAL_UART_ENABLE_IT(&USART3_RS485Handler, UART_IT_RXNE);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, 15, 0);
#endif
#endif

    RS485_TX_Set(0);  /* 默认接收模式 */
}

/* ========================================================================
   RS485_Send_Data() — 发送数据块
   @retval 1=发送成功, 0=忙(前一次DMA发送未完成, 调用者应稍后重试或记录错误)
   ======================================================================== */
uint8_t RS485_Send_Data(uint8_t *buf, uint8_t len)
{
#if EN_USART3_DMA
    /* ── 忙状态保护: 上一次 DMA 发送未完成时拒绝新发送 ── */
    if (RS485_TX_Busy) {
        return 0;  /* 返回失败, 调用者(MB_Send)可以据此做超时/重试处理 */
    }
    RS485_TX_Busy = 1;

    /* ── 拷贝到 DMA 安全缓冲区 (AXI SRAM) ──
     * 关键: 调用者 buf 可能在 DTCM (栈/全局), DMA 不可达。
     * 必须先拷贝到 RS485_TX_BUF (已由链接脚本强制放入 AXI SRAM)。 */
    memcpy(RS485_TX_BUF, buf, len);

    /* ── DCache 清理 (STM32H7 D-Cache 已启用) ──
     * AXI SRAM 经 MPU Region3 配为 Write-Through, 理论上无需 Clean。
     * 但部分 H7 版本 Write-Through 对 DMA 不可靠, 显式 Clean 保安全。 */
    SCB_CleanDCache_by_Addr((uint32_t *)RS485_TX_BUF,
                            (int32_t)(len + 31) & ~31);  /* 向上对齐到 32B cache line */

    RS485_TX_Set(1);   /* 切发送模式 */
    RS485_TC_Timeout = 50;  /* 启动超时: 从发送开始计时, TC 中断未触发时由 CheckTimeout 兜底恢复 */
    HAL_UART_Transmit_DMA(&USART3_RS485Handler, RS485_TX_BUF, len);
    /* ⚠️ 不在这里切回 DIR=0！等 HAL_UART_TxCpltCallback (TC 确认后) 再切 */
    return 1;
#else
    RS485_TX_Set(1);   /* 切发送模式 */
    HAL_UART_Transmit(&USART3_RS485Handler, buf, len, 1000);
    RS485_RX_CNT = 0;
    RS485_TX_Set(0);   /* 阻塞模式下直接切回 */
    return 1;
#endif
}

/* ========================================================================
   RS485_Receive_Data() — 接收数据块
   ========================================================================
 * ⚠️ 仅在中断模式 (EN_USART3_DMA=0) 下使用此函数。
 * DMA 模式下由 Modbus_Poll() 直接读取 RS485_RX_BUF/RS485_RX_CNT，
 * 不应调用此函数，否则 Delay_ms(10) 轮询会与 DMA 回调产生竞争。
   ======================================================================== */
void RS485_Receive_Data(uint8_t *buf, uint8_t *len)
{
#if EN_USART3_DMA
    /* DMA 模式: 此函数不应被调用, 直接返回空 */
    *len = 0;
    (void)buf;
#else
    uint16_t rxlen = RS485_RX_CNT;
    uint8_t i = 0;
    *len = 0;
    Delay_ms(10);
    if (rxlen == RS485_RX_CNT && rxlen) {
        for (i = 0; i < rxlen; i++)
            buf[i] = RS485_RX_BUF[i];
        *len = RS485_RX_CNT;
        RS485_RX_CNT = 0;
        RS485_Receive_Over = 1;
    }
#endif
}

/* ========================================================================
   HAL_UART_ErrorCallback() — 通信错误恢复
   ========================================================================
 * DMA 传输错误 (TEIF)、帧错误 (FE)、噪声错误 (NE)、溢出错误 (ORE)
 * 均触发此回调。核心操作: 重新启动 DMA 接收, 防止通信永久中断。
   ======================================================================== */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &USART3_RS485Handler)
    {
        RS485_RX_CNT = 0;
        RS485_TX_Busy = 0;   /* 清除忙标志, 防止错误导致发送永久阻塞 */
        RS485_TC_Timeout = 0; /* 清除 TC 超时计数 */
        /* 重新启动 DMA 接收, 恢复通信 */
        if (HAL_UARTEx_ReceiveToIdle_DMA(&USART3_RS485Handler, RS485_RX_BUF, 260) != HAL_OK) {
            /* 严重错误: DMA 重启失败, RS485 接收永久中断。
             * 可考虑设置全局错误标志供上层检测; HAL_BUSY 理论上不应出现
             * (IDLE 中断已停止上一次 DMA), HAL_ERROR 表示硬件异常。 */
        }
    }
}

/* ========================================================================
   RS485_TC_CheckTimeout() — TC 超时兜底检查
   ========================================================================
 * 在 Modbus_Poll() 中每周期调用一次 (~1kHz)。
 * 若 TC 中断因异常未触发，超时后强制恢复总线，防止 RS485 永久卡在发送模式。
 * 超时值 50ms = 最长帧 (259B × 87µs ≈ 23ms) 的 2 倍余量。
   ======================================================================== */
void RS485_TC_CheckTimeout(void)
{
    if (RS485_TC_Timeout > 0) {
        if (--RS485_TC_Timeout == 0) {
            /* 超时！TC 中断未触发，强制恢复 */
            __HAL_UART_DISABLE_IT(&USART3_RS485Handler, UART_IT_TC);
            RS485_TX_Set(0);    /* 强制切回接收模式 */
            RS485_TX_Busy = 0;  /* 解除发送锁 */
        }
    }
}

/* ========================================================================
   RS485_TX_Set() — 收发方向控制
   ======================================================================== */
void RS485_TX_Set(uint8_t en)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, en);
}

/* ========================================================================
   HAL_UARTEx_RxEventCallback() — IDLE 中断回调
   ========================================================================
 * HAL 在检测到 USART3 IDLE 后自动停止 DMA, 并调用此回调。
 * Size = HAL 内部计算的本次 DMA 实际接收字节数。
 *
 * ⚠️ 关键: 必须在此回调中重新调用 ReceiveToIdle_DMA() 恢复接收,
 *   否则后续帧全部丢失。
   ======================================================================== */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &USART3_RS485Handler && Size > 0)
    {
        /* ── DCache 失效 (STM32H7 D-Cache 已启用) ──
         * DMA 已写新数据到 RS485_RX_BUF (AXI SRAM), CPU 的 D-Cache 可能仍有旧数据。
         * 必须先 Invalidate, 否则 RS485_RX_CNT/Modbus_Poll 读到过期缓存。 */
        SCB_InvalidateDCache_by_Addr((uint32_t *)RS485_RX_BUF,
                                      (int32_t)(Size + 31) & ~31);

        RS485_RX_CNT = Size;         /* 记录接收长度, Modbus_Poll 会读取并清零 */
        RS485_Receive_Over = 1;      /* 置位完成标志, 通知上层有新帧到达 */
    }
    /* 重新启动 DMA 接收, 准备收下一帧 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(&USART3_RS485Handler, RS485_RX_BUF, 260) != HAL_OK) {
        /* DMA 重启失败 (HAL_BUSY/HAL_ERROR)。
         * IDLE → 重新启动之间时间极短, 正常情况不应该失败。
         * 若出现, RS485 接收将中断, 只能通过 ErrorCallback 或其他异常恢复。 */
    }
}

/* ========================================================================
   HAL_UART_TxCpltCallback() — 发送完成回调 (TC 确认)
   ========================================================================
 * 调用时机: HAL 内部 UART_EndTransmit_IT() 确认 TC 硬件置位后。
 *          此时移位寄存器已发完最后一个停止位，总线物理空闲。
 *
 * 注意: HAL 进入此回调前已关闭 TCIE (UART_EndTransmit_IT → CLEAR_BIT(CR1, TCIE)),
 *       因此无需再次 __HAL_UART_DISABLE_IT(TC)。
 *
 * ⚠️ 关键操作顺序:
 *   1. 清 TC 硬件标志 — 防残留标志在下次 DMA 完成时误触发 TC 中断
 *   2. 清超时计数 — TC 正常触发，无需兜底
 *   3. DIR=0 — 切回接收模式，释放 RS485 总线
 *   4. TX_Busy=0 — 解除发送锁，允许下次发送
   ======================================================================== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &USART3_RS485Handler)
    {
        __HAL_UART_CLEAR_FLAG(&USART3_RS485Handler, UART_CLEAR_TCF);  /* 清 TC 硬件标志 */
        RS485_TC_Timeout = 0;     /* 清超时 (TC 正常触发) */
        RS485_TX_Set(0);          /* 切回接收模式 */
        RS485_TX_Busy = 0;        /* 解除发送锁 */
    }
}
