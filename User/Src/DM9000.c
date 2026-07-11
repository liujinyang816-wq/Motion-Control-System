/**
 ******************************************************************************
 * @file           : DM9000.c
 * @brief          : DM9000AEP 以太网MAC控制器底层驱动
 * @author         : armfly (www.armfly.com)
 * @version        : V1.1
 * @date           : 2013-06-20
 * ----------------------------------------------------------------------------
 * 硬件连接：DM9000 通过 FMC (Flexible Memory Controller) 总线与 STM32H7 连接
 *   - FMC Bank3 (NE3) 作为片选信号
 *   - 16位数据总线宽度
 *   - 寄存器地址和数据通过 FSMC 地址线 A18 区分
 *
 * 功能摘要：
 *   1. FMC 总线配置（16位 SRAM 模式，模式A）
 *   2. DM9000 硬件复位与软件复位
 *   3. 寄存器读写（8位/16位）
 *   4. PHY 寄存器读写（通过 DM9000 间接访问）
 *   5. 以太网帧收发（含地址过滤、广播过滤）
 *   6. MAC 地址配置和多播哈希表
 *   7. 中断处理
 ******************************************************************************
 */
#include "bsp_timer.h"
#include "DM9000.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "string.h"

/* ============================================================================
 * 编译选项宏定义
 * ============================================================================ */

//#define DM9000A_FLOW_CONTROL       // 启用流量控制
//#define DM9000A_UPTO_100M          // 强制100M模式
//#define Fifo_Point_Check           // FIFO指针校验
//#define Point_Error_Reset          // 指针错误时自动复位

#define Fix_Note_Address             // 固定地址模式
#define FifoPointCheck               // 启用FIFO指针检查

/* ── 接收选项 ── */
#define Rx_Int_enable                // 启用接收中断（当前未使用中断模式）
#define Max_Int_Count       1        // 单次中断最大处理帧数
#define Max_Ethernet_Lenth   1536    // 最大以太网帧长（含CRC）
#define Broadcast_Jump                // 广播帧长度过滤
#define Max_Broadcast_Lenth   500    // 广播帧最大允许长度

/* ── 发送选项 ── */
#define Max_Send_Pack        2       // 最大连续发送帧数

/* ============================================================================
 * FMC 总线地址映射
 * ============================================================================
 * FMC Bank3 基地址：0x68000000 (64MB空间)
 * DM9000 使用地址线 A18 区分 CMD 和 DATA：
 *   - NET_REG_ADDR:  A18=0 → 索引端口（写寄存器地址）
 *   - NET_REG_DATA:  A18=1 → 数据端口（读写寄存器数据）
 * 因此：
 *   - 地址端口 = 0x68400000 (A18=0)
 *   - 数据端口 = 0x68400000 + 0x00080000 (A18=1, 即bit19)
 */
#define NET_BASE_ADDR       0x68400000
#define NET_REG_ADDR        (*((volatile uint16_t *) NET_BASE_ADDR))
#define NET_REG_DATA        (*((volatile uint16_t *) (NET_BASE_ADDR + 0x00080000)))

#define ETH_ADDR_LEN        6       // MAC地址长度（6字节）

/* ============================================================================
 * 全局变量
 * ============================================================================ */

/** 默认 MAC 地址（可自定义修改） */
static unsigned char DEF_MAC_ADDR[ETH_ADDR_LEN] = {0x01, 0x02, 0x01, 0x02, 0x01, 0x02};

uint8_t SendPackOk = 0;                  // 发送完成标志
uint8_t s_FSMC_Init_Ok = 0;              // FMC初始化完成标志

uint16_t receiveLen_DM9000;              // DM9000接收到的帧长度（不含CRC）
uint8_t  receiveBuffer_DM9000[1528];     // DM9000接收缓冲区
uint16_t receiveLen;                     // 上层使用的接收长度
uint8_t  receiveBuffer[1528];            // 上层使用的接收缓冲区

/* ============================================================================
 * 函数声明
 * ============================================================================ */
static void DM9K_CtrlLinesConfig(void);  // FMC GPIO控制线配置
static void DM9K_FSMCConfig(void);       // FMC总线时序配置
void DM9000_Initnic(void);               // DM9000网卡初始化

/* ============================================================================
 * DM9000_Init() — 初始化入口
 * ============================================================================
 * 调用流程：GPIO配置 → FMC时序配置 → DM9000芯片初始化
 * 被 main() 在系统初始化阶段调用
 */
void DM9000_Init(void)
{
    DM9K_CtrlLinesConfig();             // 第一步：配置FMC接口GPIO
    DM9K_FSMCConfig();                  // 第二步：配置FMC总线时序参数
    s_FSMC_Init_Ok = 1;                 // 标记FMC初始化完成
    DM9000_Initnic();                   // 第三步：软复位DM9000并配置MAC
}

/* ============================================================================
 * DM9K_CtrlLinesConfig() — FMC接口GPIO初始化
 * ============================================================================
 * 配置以下引脚为FMC复用功能：
 *   GPIOD: D0-D1, D3-D4, D8-D10, D13-D15, NOE, NWE, A18(RS)
 *   GPIOE: D4-D15 (高8位数据线)
 *   GPIOG: PG10 = NE3 (FMC Bank3片选)
 *   GPIOC: PC9 = DM9000硬件复位控制
 *
 * FMC总线使用16位数据宽度，复用地址/数据总线模式
 */
void DM9K_CtrlLinesConfig(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    /* ── 使能所有需要的GPIO时钟和FMC时钟 ── */
    __HAL_RCC_FMC_CLK_ENABLE();         // FMC外设时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();       // GPIOC: 复位引脚
    __HAL_RCC_GPIOD_CLK_ENABLE();       // GPIOD: 数据线D0-D15 + 控制线
    __HAL_RCC_GPIOE_CLK_ENABLE();       // GPIOE: 数据线D4-D15
    __HAL_RCC_GPIOF_CLK_ENABLE();       // GPIOF: 备用
    __HAL_RCC_GPIOG_CLK_ENABLE();       // GPIOG: NE3片选

    /* ── GPIOD 配置 ──
     * PD0=D2, PD1=D3, PD4=NOE(读使能), PD5=NWE(写使能)
     * PD8=D13, PD9=D14, PD10=D15, PD13=A18(RS=CMD/DATA选择)
     * PD14=D0, PD15=D1
     * 全部配置为推挽复用输出，上拉，最高速度
     */
    GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5
                     | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
                     | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_Initure.Mode = GPIO_MODE_AF_PP;           // 复用推挽输出
    GPIO_Initure.Pull = GPIO_PULLUP;               // 上拉
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 最高速度等级
    GPIO_Initure.Alternate = GPIO_AF12_FMC;        // 复用功能FMC
    HAL_GPIO_Init(GPIOD, &GPIO_Initure);

    /* ── GPIOE 配置 ──
     * PE7=D4, PE8=D5, PE9=D6, PE10=D7, PE11=D8
     * PE12=D9, PE13=D10, PE14=D11, PE15=D12
     */
    GPIO_Initure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
                     | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
                     | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &GPIO_Initure);

    /* ── GPIOG 配置 ──
     * PG10 = NE3 (FMC Bank3 片选信号，低电平有效)
     */
    GPIO_Initure.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOG, &GPIO_Initure);

    /* ── GPIOC 配置 ──
     * PC9 = DM9000 硬件复位控制引脚（推挽输出）
     */
    GPIO_Initure.Pin = GPIO_PIN_9;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;       // 推挽输出
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Initure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_Initure);

    /* ── 执行硬件复位时序 ── */
    DM9000_RST;                         // PC9拉低 → DM9000复位
    Delay_ms(1);
    DM9000_SET;                         // PC9拉高 → 复位释放
    Delay_ms(200);                      // 等待DM9000内部就绪（至少需要100ms）
}

/* ============================================================================
 * DM9K_FSMCConfig() — FMC总线时序配置
 * ============================================================================
 * 配置 STM32H7 的 FMC 控制器以访问 DM9000：
 *   - Bank3 (NE3), 基地址 0x68000000
 *   - 16位数据总线, 地址/数据复用模式
 *   - 模式A (SRAM类型), 异步访问
 *   - FMC时钟 = HCLK3 = 200MHz, 每个FMC周期 = 5ns
 *
 * 时序计算（基于200MHz FMC时钟，即5ns/周期）：
 *   - AddressSetupTime=8  → 地址建立时间 = 8×5ns = 40ns
 *   - AddressHoldTime=2   → 地址保持时间 = 2×5ns = 10ns
 *   - DataSetupTime=10    → 数据建立时间 = 10×5ns = 50ns
 *   - BusTurnAround=15    → 总线周转时间 = 15×5ns = 75ns
 */
static void DM9K_FSMCConfig(void)
{
    SRAM_HandleTypeDef hsram = {0};
    FMC_NORSRAM_TimingTypeDef SRAM_Timing = {0};

    hsram.Instance  = FMC_NORSRAM_DEVICE;
    hsram.Extended  = FMC_NORSRAM_EXTENDED_DEVICE;

    /* ── 配置FMC读写时序（单位：FMC时钟周期） ── */
    SRAM_Timing.AddressSetupTime       = 8;   // 地址建立时间: 8×5ns=40ns
    SRAM_Timing.AddressHoldTime        = 2;   // 地址保持时间: 2×5ns=10ns
    SRAM_Timing.DataSetupTime          = 10;  // 数据建立时间: 10×5ns=50ns
    SRAM_Timing.BusTurnAroundDuration  = 15;  // 总线周转: 15×5ns=75ns
    SRAM_Timing.CLKDivision            = 0;   // 不分频（仅同步模式有效）
    SRAM_Timing.DataLatency            = 0;   // 数据延迟（仅同步模式有效）
    SRAM_Timing.AccessMode             = FMC_ACCESS_MODE_A;  // 模式A

    /* ── 配置FMC Bank参数 ── */
    hsram.Init.NSBank             = FMC_NORSRAM_BANK3;        // 使用Bank3 (NE3)
    hsram.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_ENABLE;  // 地址/数据复用
    hsram.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;     // SRAM类型
    hsram.Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16; // 16位总线
    hsram.Init.BurstAccessMode    = FMC_BURST_ACCESS_MODE_DISABLE; // 禁止突发
    hsram.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
    hsram.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;    // 允许写
    hsram.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;       // 无等待信号
    hsram.Init.ExtendedMode       = FMC_EXTENDED_MODE_DISABLE;     // 读写相同时序
    hsram.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram.Init.WriteBurst         = FMC_WRITE_BURST_DISABLE;       // 禁止写突发
    hsram.Init.ContinuousClock    = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    hsram.Init.WriteFifo          = FMC_WRITE_FIFO_ENABLE;         // 启用写FIFO

    /* ── 初始化FMC SRAM控制器 ── */
    if (HAL_SRAM_Init(&hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK)
    {
        Error_Handler(__FILE__, __LINE__);  // 初始化失败 → 进入错误处理
    }
}

/* ============================================================================
 * dm9k_ReadReg() — 读取DM9000内部寄存器
 * ============================================================================
 * @param  reg : 寄存器地址（0x00~0xFF）
 * @retval 寄存器值（8位）
 *
 * 操作过程：
 *   1. 向地址端口写入寄存器索引
 *   2. 从数据端口读取寄存器值
 * 注意：此函数不可重入，需在中断中保护
 */
uint8_t dm9k_ReadReg(uint8_t reg)
{
    NET_REG_ADDR = reg;                 // 写入寄存器地址
    return (NET_REG_DATA);              // 读取并返回寄存器值
}

/* ============================================================================
 * dm9k_WriteReg() — 写入DM9000内部寄存器
 * ============================================================================
 * @param  reg       : 目标寄存器地址
 * @param  writedata : 要写入的数据值（8位）
 *
 * 操作过程：
 *   1. 向地址端口写入寄存器索引
 *   2. 向数据端口写入寄存器值
 */
void dm9k_WriteReg(uint8_t reg, uint8_t writedata)
{
    NET_REG_ADDR = reg;                 // 选择目标寄存器
    NET_REG_DATA = writedata;           // 写入数据
}

/* ============================================================================
 * dm9k_hash_table() — 配置MAC地址和多播哈希表
 * ============================================================================
 * 设置DM9000的：
 *   1. 物理地址 (PAR0~PAR5)：源MAC地址
 *   2. 多播地址寄存器 (MAR0~MAR7)：全部设为0xFF → 接收所有多播帧
 *
 * 如果需要过滤特定多播地址，需要根据多播地址计算哈希值并设置对应MAR位
 */
void dm9k_hash_table(void)
{
    uint8_t i;

    /* ── 写入6字节MAC地址到PAR寄存器（Physical Address Register） ── */
    for (i = 0; i < 6; i++)
    {
        dm9k_WriteReg(DM9000_REG_PAR + i, DEF_MAC_ADDR[i]);
    }

    /* ── 全部多播地址通过（设为0xFF = 接收所有多播帧） ── */
    for (i = 0; i < 8; i++)
    {
        dm9k_WriteReg(DM9000_REG_MAR + i, 0xFF);
    }
}

/* ============================================================================
 * dm9k_err_reset() — DM9000错误恢复复位
 * ============================================================================
 * 当检测到接收/发送异常时调用，执行轻量级复位：
 *   1. 两次连续NCR复位
 *   2. 禁用中断
 *   3. 清除状态寄存器
 *   4. 恢复基本配置
 *
 * 与 dm9k_reset() 的区别：不执行硬件复位和PHY重新初始化
 */
void dm9k_err_reset(void)
{
    /* ── 两次NCR软件复位 ── */
    dm9k_WriteReg(DM9000_REG_NCR, 0x03);   // 第一次：置位复位位
    Delay_us(10);
    dm9k_WriteReg(DM9000_REG_NCR, 0x03);   // 第二次：再次复位
    Delay_us(10);

    /* ── 恢复基本配置 ── */
    dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);   // 关闭所有中断
    dm9k_WriteReg(DM9000_REG_TCR2, DM9000_TCR2_SET);  // LED模式1: 全双工常亮，半双工闪烁

    /* ── 清除各状态寄存器 ── */
    dm9k_WriteReg(DM9000_REG_NSR, 0x2c);    // 清除网络状态寄存器
    dm9k_WriteReg(DM9000_REG_TCR, 0x00);     // 清除发送控制寄存器
    dm9k_WriteReg(DM9000_REG_ISR, 0x0f);     // 清除中断状态寄存器

    /* ── 重新启用接收和中断屏蔽 ── */
    dm9k_WriteReg(DM9000_REG_RCR, DM9000_RCR_SET);   // 启用接收（含混杂模式/广播/多播）
    dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);   // 保持中断关闭（轮询模式）

    SendPackOk = 0;                      // 重置发送完成标志
}

/* ============================================================================
 * dm9k_reset() — DM9000完整复位
 * ============================================================================
 * 执行完整的DM9000初始化复位流程：
 *   1. 硬件复位（GPIO拉低→延时→拉高）
 *   2. 关闭内部PHY电源
 *   3. 两次NCR软件复位
 *   4. 配置基本寄存器
 *
 * 参考：DM9000 Application Notes V1.22 第29页
 */
void dm9k_reset(void)
{
    /* ── 步骤1: 硬件复位 ── */
    DM9000_RST;                         // 拉低复位引脚
    Delay_ms(20);                       // 保持20ms
    DM9000_SET;                         // 释放复位
    Delay_ms(100);                      // 等待DM9000完全就绪

    /* ── 步骤2: 关闭内部PHY（为后续PHY初始化做准备） ── */
    dm9k_WriteReg(DM9000_REG_GPCR, 0x01);     // GPIO控制: 启用GPIO功能
    dm9k_WriteReg(DM9000_REG_GPR, 0);          // GPIO数据: bit0=0 → PHY断电
    Delay_ms(20);                               // 等待PHY完全掉电

    /* ── 步骤3: 第一次NCR软件复位 ── */
    dm9k_WriteReg(DM9000_REG_NCR, 0x03);       // NCR[0]=1 → 启动软件复位
    do {
        Delay_ms(1);
    } while (dm9k_ReadReg(DM9000_REG_NCR) & 1); // 轮询等待复位完成（bit0自动清零）
    dm9k_WriteReg(DM9000_REG_NCR, 0x00);        // 退出复位状态，进入正常模式

    /* ── 步骤4: 第二次NCR软件复位（增强可靠性） ── */
    dm9k_WriteReg(DM9000_REG_NCR, 0x03);
    do {
        Delay_ms(1);
    } while (dm9k_ReadReg(DM9000_REG_NCR) & 1);
    dm9k_WriteReg(DM9000_REG_NCR, 0x00);

    /* ── 步骤5: 配置默认寄存器值 ── */
    dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);        // 禁用所有中断
    dm9k_WriteReg(DM9000_REG_TCR2, DM9000_TCR2_SET);       // LED模式配置
    dm9k_WriteReg(DM9000_REG_NSR, 0x2c);                   // 清除网络状态
    dm9k_WriteReg(DM9000_REG_TCR, 0x00);                   // 清除发送控制
    dm9k_WriteReg(DM9000_REG_ISR, 0x0f);                   // 清除中断状态
    dm9k_WriteReg(DM9000_REG_RCR, DM9000_RCR_SET);         // 配置接收控制
    dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);         // 保持中断关闭
    SendPackOk = 0;
}

/* ============================================================================
 * dm9k_phy_write() — 通过DM9000间接写入PHY寄存器
 * ============================================================================
 * @param  phy_reg   : PHY寄存器地址（0x00~0x1F）
 * @param  writedata : 16位写入数据
 *
 * DM9000内部集成了PHY（物理层），通过EPCR/EPAR/EPDRH/EPDRL间接访问：
 *   1. 写入PHY地址到EPAR
 *   2. 写入高8位到EPDRH，低8位到EPDRL
 *   3. 设置EPCR触发写操作
 *   4. 轮询等待操作完成
 */
void dm9k_phy_write(uint8_t phy_reg, uint16_t writedata)
{
    /* ── 设置PHY寄存器地址（含PHY地址偏移0x40） ── */
    dm9k_WriteReg(DM9000_REG_EPAR, phy_reg | DM9000_PHY);

    /* ── 写入16位PHY数据（高字节在前） ── */
    dm9k_WriteReg(DM9000_REG_EPDRH, (writedata >> 8) & 0xff);
    dm9k_WriteReg(DM9000_REG_EPDRL, writedata & 0xff);

    /* ── 触发PHY写操作（EPCR[1]=1）并等待完成 ── */
    dm9k_WriteReg(DM9000_REG_EPCR, 0x0a);           // EPCR: 启动PHY写
    while (dm9k_ReadReg(DM9000_REG_EPCR) & 0x01);    // 等待bit0清零 → 操作完成
    Delay_ms(50);
    dm9k_WriteReg(DM9000_REG_EPCR, 0x00);            // 清除EPCR
}

/* ============================================================================
 * dm9k_phy_read() — 通过DM9000间接读取PHY寄存器
 * ============================================================================
 * @param  phy_reg : PHY寄存器地址（0x00~0x1F）
 * @retval 16位PHY寄存器值
 *
 * 操作流程与dm9k_phy_write()对称，通过EPCR触发读操作
 */
uint16_t dm9k_phy_read(uint8_t phy_reg)
{
    uint16_t readdata;

    /* ── 设置PHY寄存器地址 ── */
    dm9k_WriteReg(DM9000_REG_EPAR, phy_reg | DM9000_PHY);

    /* ── 触发PHY读操作（EPCR[2]=1）并等待完成 ── */
    dm9k_WriteReg(DM9000_REG_EPCR, 0x0C);            // EPCR: 启动PHY读
    while (dm9k_ReadReg(DM9000_REG_EPCR) & 0x01);     // 等待完成

    dm9k_WriteReg(DM9000_REG_EPCR, 0x00);             // 清除EPCR

    /* ── 读取16位PHY数据（高字节+低字节拼接） ── */
    readdata = (dm9k_ReadReg(DM9000_REG_EPDRH) << 8)
             | (dm9k_ReadReg(DM9000_REG_EPDRL));

    return readdata;
}

/* ============================================================================
 * DM9000_Initnic() — DM9000网卡芯片初始化
 * ============================================================================
 * 完整的网卡初始化流程：
 *   1. 软件复位DM9000
 *   2. 关闭内部PHY电源
 *   3. 配置PHY为自动协商模式
 *   4. 重新开启PHY
 *   5. 配置MAC地址和多播过滤
 */
void DM9000_Initnic(void)
{
    /* ── 步骤1: 复位DM9000 ── */
    dm9k_reset();

    /* ── 步骤2: 关闭PHY，进行PHY配置 ── */
    dm9k_WriteReg(DM9000_REG_GPR, DM9000_PHY_OFF);     // GPIO数据: PHY断电

    /* ── 步骤3: PHY寄存器初始化 ── */
    dm9k_phy_write(0x00, 0x8000);                      // BCR: 软复位PHY
    dm9k_phy_write(0x04, 0x01e1);                      // ANAR: 自动协商广播
                                                         //   bit8=100Base-TX FD, bit7=100Base-TX HD
                                                         //   bit6=10Base-T FD, bit5=10Base-T HD

    /* ── 步骤4: PHY模式选择 ──
     * 0x0000: 固定 10M 半双工
     * 0x0100: 固定 10M 全双工
     * 0x2000: 固定 100M 半双工
     * 0x2100: 固定 100M 全双工
     * 0x1000: 自动协商（推荐） ── */
    dm9k_phy_write(0x00, 0x1000);                      // BCR: 自动协商模式

    Delay_ms(2000);                                     // 等待PHY稳定

    /* ── 步骤5: 重新开启PHY ── */
    dm9k_WriteReg(DM9000_REG_GPR, DM9000_PHY_ON);      // GPIO数据: PHY上电
    Delay_ms(2000);                                     // 等待PHY就绪

    /* ── 步骤6: 配置MAC地址和多播哈希表 ── */
    dm9k_hash_table();
}

/* ============================================================================
 * dm9k_receive_packet() — 接收以太网帧
 * ============================================================================
 * @retval  >0 : 接收到的帧长度（不含4字节CRC）
 * @retval   0 : 无帧可接收
 *
 * 在轮询模式下工作（非中断），每次调用检查并接收一帧：
 *   1. 读取MRCMDX检查数据就绪状态
 *   2. 检查帧长度合法性（最大1536字节）
 *   3. 广播帧额外长度过滤（最大500字节）
 *   4. 从FIFO读取帧数据到接收缓冲区
 *   5. 更新内存读指针（MRRH/MRRL），跳过CRC
 *   6. 可选FIFO指针校验
 *
 * 注意事项：
 *   - 接收数据长度若为奇数，内存指针需额外+1（对齐）
 *   - 内存指针范围 0x0000~0x3FFF，超出需回卷
 */
uint16_t dm9k_receive_packet(void)
{
    uint16_t ReceiveData[1600];         // 临时接收缓冲区（最大800字=1600字节）
    uint8_t  rx_int_count = 0;          // 当前中断处理周期已接收帧计数
    uint32_t rx_checkbyte;              // 接收就绪检查字节
    uint16_t rx_status, rx_length;      // 帧状态和长度
    uint8_t  jump_packet;               // 跳包标志
    uint16_t i;
    uint16_t calc_len;                  // 计算出的字长度
    uint16_t calc_MRR;                  // 计算出的内存读指针

    do
    {
        jump_packet = 0;

        /* ── 第1步: 检查是否有数据就绪 ── */
        dm9k_ReadReg(DM9000_REG_MRCMDX);                 // 预读触发

        /* ── 读取当前内存读指针位置 ── */
        calc_MRR = (dm9k_ReadReg(DM9000_REG_MRRH) << 8)
                 + dm9k_ReadReg(DM9000_REG_MRRL);

        rx_checkbyte = dm9k_ReadReg(DM9000_REG_MRCMDX);  // 第二次读取获取状态

        if (rx_checkbyte == DM9000_PKT_RDY)              // 0x01 = 数据就绪
        {
            /* ── 第2步: 读取帧状态和长度 ── */
            NET_REG_ADDR = DM9000_REG_MRCMD;              // 设置内存读命令
            rx_status = NET_REG_DATA;                     // 帧状态字
            rx_length = NET_REG_DATA;                     // 帧长度（含4字节CRC）

            /* ── 第3步: 帧长度合法性检查 ── */
            if (rx_length > Max_Ethernet_Lenth)           // 超过最大帧长 → 丢弃
                jump_packet = 1;

#ifdef Broadcast_Jump
            /* ── 广播/多播帧额外长度限制 ── */
            if (rx_status & 0x4000)                       // bit14 = 广播/多播标志
            {
                if (rx_length > Max_Broadcast_Lenth)
                    jump_packet = 1;
            }
#endif

            /* ── 第4步: 计算下一帧起始地址 ──
             * 逻辑地址空间0x0000~0x3FFF（16KB SRAM）
             * 帧起始 = 当前指针 + 帧长 + 4(头) + 奇偶对齐
             */
            calc_MRR += (rx_length + 4);
            if (rx_length & 0x01) calc_MRR++;            // 奇数长度 → +1对齐
            if (calc_MRR > 0x3fff) calc_MRR -= 0x3400;    // 超出 → 回卷

            /* ── 第5步: 跳包处理 ── */
            if (jump_packet == 0x01)
            {
                /* 跳过当前帧，将读指针移动到下一帧起始 */
                dm9k_WriteReg(DM9000_REG_MRRH, (calc_MRR >> 8) & 0xff);
                dm9k_WriteReg(DM9000_REG_MRRL, calc_MRR & 0xff);
                continue;
            }

            /* ── 第6步: 从FIFO读取帧数据 ──
             * 每次读取1个word（16位），共读取 (rx_length+1)/2 次
             */
            calc_len = (rx_length + 1) >> 1;
            for (i = 0; i < calc_len; i++)
                ReceiveData[i] = NET_REG_DATA;

            /* ── 第7步: 复制到全局缓冲区（去掉最后4字节CRC） ── */
            receiveLen_DM9000 = rx_length - 4;            // 有效数据长度 = 帧长 - CRC
            memcpy((unsigned char*)receiveBuffer_DM9000,
                   (uint8_t *)ReceiveData, receiveLen_DM9000);

            rx_int_count++;                               // 接收计数+1

#ifdef FifoPointCheck
            /* ── 第8步(可选): FIFO指针校验 ──
             * 比较计算出的指针和芯片内部实际指针，不一致则修正
             */
            if (calc_MRR != ((dm9k_ReadReg(DM9000_REG_MRRH) << 8)
                           + dm9k_ReadReg(DM9000_REG_MRRL)))
            {
#ifdef Point_Error_Reset
                dm9k_reset();  // 指针错误 → 硬复位
                return receiveLen_DM9000;
#endif
                /* 指针修正：将芯片指针强制设置为计算值 */
                dm9k_WriteReg(DM9000_REG_MRRH, (calc_MRR >> 8) & 0xff);
                dm9k_WriteReg(DM9000_REG_MRRL, calc_MRR & 0xff);
            }
#endif
            /* ── 第9步: 更新内存读指针到下一帧位置 ── */
            dm9k_WriteReg(DM9000_REG_MRRH, (calc_MRR >> 8) & 0xff);
            dm9k_WriteReg(DM9000_REG_MRRL, calc_MRR & 0xff);

            return receiveLen_DM9000;
        }
        else
        {
            /* ── 无数据或错误处理 ── */
            if (rx_checkbyte == DM9000_PKT_NORDY)        // 0x00 = 无数据
            {
                dm9k_WriteReg(DM9000_REG_ISR, 0x3f);      // 清除中断状态
            }
            else
            {
                dm9k_err_reset();                         // 异常状态 → 错误复位
            }
            return 0;
        }
    } while (rx_int_count < Max_Int_Count);               // 最多处理1帧

    return 0;
}

/* ============================================================================
 * dm9k_send_packet() — 发送以太网帧
 * ============================================================================
 * @param  p_char : 发送数据缓冲区指针
 * @param  length : 发送数据长度（不含CRC，芯片自动添加）
 *
 * 发送流程：
 *   1. 关闭发送中断
 *   2. 将数据写入发送FIFO（每次1个word）
 *   3. 设置发送长度到TXPLH/TXPLL
 *   4. 触发发送（TCR bit0=1）
 *   5. 等待发送完成（超时100ms）
 *   6. 清除状态寄存器
 */
void dm9k_send_packet(uint8_t *p_char, uint16_t length)
{
    uint16_t SendLength = length;
    uint16_t *SendData = (uint16_t *) p_char;              // 按16位对齐发送
    uint16_t i;
    uint16_t calc_len;
    __IO uint16_t calc_MWR;
    uint32_t timer2;

    /* ── 关闭发送中断 ── */
    dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);

    /* ── 写入发送FIFO ──
     * 设置MWCMD命令，然后逐个word写入数据
     */
    NET_REG_ADDR = DM9000_REG_MWCMD;                      // 内存写命令

    calc_len = (SendLength + 1) >> 1;                      // 计算word数量(向上取整)
    for (i = 0; i < calc_len; i++)
        NET_REG_DATA = SendData[i];                        // 每次写入1个word

    /* ── 设置发送帧长度 ── */
    dm9k_WriteReg(DM9000_REG_TXPLH, (SendLength >> 8) & 0xff);  // 长度高字节
    dm9k_WriteReg(DM9000_REG_TXPLL, SendLength & 0xff);          // 长度低字节

    /* ── 触发发送 ── */
    dm9k_WriteReg(DM9000_REG_TCR, DM9000_TCR_SET);        // TCR bit0=1 → 启动发送

    /* ── 等待发送完成（超时保护: 100ms） ── */
    timer2 = bsp_GetTickCount();
    while (dm9k_ReadReg(DM9000_REG_TCR) & 0x01)            // 轮询TCR bit0
    {
        if (bsp_GetTickCount() - timer2 >= 100)             // 超时100ms
            break;
    }

    /* ── 清除状态并重新屏蔽中断 ── */
    dm9k_WriteReg(DM9000_REG_NSR, 0x2c);                   // 清除网络状态
    dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);          // 保持中断关闭
}

/* ============================================================================
 * dm9k_interrupt() — DM9000中断服务处理（当前项目未使用）
 * ============================================================================
 * 中断模式下的接收处理入口（当前项目采用轮询模式，此函数保留备用）
 *
 * 中断处理流程：
 *   1. 保存当前寄存器地址上下文
 *   2. 禁用中断 → 读取ISR中断状态
 *   3. 如果是接收中断 → 调用 dm9k_receive_packet()
 *   4. 恢复中断使能和寄存器上下文
 */
void dm9k_interrupt(void)
{
    uint8_t  save_reg;
    uint16_t isr_status;

    save_reg = NET_REG_ADDR;                                // 保存当前使用的寄存器索引
    dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_OFF);          // 临时关闭所有中断
    isr_status = dm9k_ReadReg(DM9000_REG_ISR);              // 读取中断状态

    if (isr_status & DM9000_RX_INTR)                        // bit0 = 接收中断
    {
        // dm9k_receive_packet();                           // 中断模式下在此处接收
    }

    dm9k_WriteReg(DM9000_REG_IMR, DM9000_IMR_SET);          // 恢复中断使能
    NET_REG_ADDR = save_reg;                                // 恢复寄存器上下文
}

/* ============================================================================
 * dm9k_ReadID() — 读取DM9000芯片ID
 * ============================================================================
 * @retval 32位芯片ID: [VID2:VID1:PID2:PID1]
 *
 * 预期合法值：0x0A469000
 *   - VID (Vendor ID) = 0x0A46 (Davicom)
 *   - PID (Product ID) = 0x9000 (DM9000A/B/C系列)
 *
 * 如果返回值不是0x0A469000，说明：
 *   - FMC总线配置错误
 *   - DM9000硬件连接故障
 *   - 芯片损坏
 */
uint32_t dm9k_ReadID(void)
{
    uint8_t vid1, vid2, pid1, pid2;

    /* ── 依次读取4个ID寄存器 ── */
    vid1 = dm9k_ReadReg(DM9000_REG_VID_L) & 0xFF;  // 厂商ID低字节
    vid2 = dm9k_ReadReg(DM9000_REG_VID_H) & 0xFF;  // 厂商ID高字节
    pid1 = dm9k_ReadReg(DM9000_REG_PID_L) & 0xFF;  // 产品ID低字节
    pid2 = dm9k_ReadReg(DM9000_REG_PID_H) & 0xFF;  // 产品ID高字节

    /* ── 拼接为32位ID返回 ── */
    return (vid2 << 24) | (vid1 << 16) | (pid2 << 8) | pid1;
}
