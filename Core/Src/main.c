/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 4~8轴 SV660N EtherCAT 主站运动控制系统 (X/Y/Z/R + U/V/W/S)
 * ----------------------------------------------------------------------------
 * 【系统概述】
 * 本工程基于 STM32H7 + SOEM 协议栈实现 4轴 EtherCAT 主站，
 * 通过 CiA 402 CSP (Cyclic Synchronous Position) 模式控制 SV660N 伺服驱动器。
 *
 * 【硬件架构】
 *   STM32H7 (400MHz)
 *     ├── FMC → DM9000 MAC + DM9161 PHY → EtherCAT 总线
 *     ├── TIM1 → DC 同步中断 (5ms 周期)
 *     ├── USART1 → 串口命令输入 (115200-8N1)
 *     ├── USART3 → RS485 通信
 *     └── FDCAN1 → CAN 总线
 *
 * 【EtherCAT 从站拓扑】
 *   主站 (STM32H7)
 *     ├── 从站1: SV660N → X轴 (CSP)
 *     ├── 从站2: SV660N → Y轴 (CSP)
 *     ├── 从站3: SV660N → Z轴 (CSP)
 *     ├── 从站4: SV660N → R轴 (CSP, 旋转轴)
 *     ├── 从站5: SV660N → U轴 (CSP, 摇动光洁度)
 *     ├── 从站6: SV660N → V轴 (CSP, 摇动光洁度)
 *     ├── 从站7: SV660N → W轴 (CSP, 升降抓电极)
 *     └── 从站8: SV660N → S轴 (CSP, 刀库旋转)
 *
 * 【代码架构】
 *   main.c          — 系统初始化 + FreeRTOS 任务创建
 *   ethercat_slave.c — SV660N PDO 映射配置 (Servosetup)
 *   ethercat_task.c — EtherCAT 实时控制任务 (vEtherCAT_Task)
 *   motion_api.c    — 轴控核心 (Axis_Init/ControlCycle/StopOne/...)
 *   serial_cmd.c    — 串口命令解析 (ParseCommand/vCmd_Task)
 *   modbus_slave.c  — ModBus 从站 (Modbus_UpdateRegs/CmdExec_CheckTriggers)
 *   motion_profile.c — S 曲线 7 段规划器
 *   param_store.c   — Flash 参数存储
 *
 * 【控制周期】
 *   TIM1 中断 → 5ms (DC SYNC0) → 唤醒 vEtherCAT_Task
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
#include "string.h"
#include "usart2.h"
#include "stdio.h"
#include "DM9000.h"
#include "DM9161.h"
#include "bsp_timer.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "rs485.h"
#include "fdcan.h"
#include "modbus_slave.h"
#include "motor_axis.h"
#include "param_defs.h"
#include "motion_api.h"
#include "ethercat_slave.h"
#include "ethercat_task.h"
#include "serial_cmd.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

/* ── FreeRTOS Task Handles ── */
TaskHandle_t xEtherCATTaskHandle = NULL;

/* ── FreeRTOS Idle Task Memory ── */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* ── FreeRTOS Stack Overflow Hook ── */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    #if printf_cmd
    printf("\r\n!!! STACK OVERFLOW in task: %s !!!\r\n", pcTaskName);
    #endif
    taskDISABLE_INTERRUPTS();
    for (;;) {}
}

/* ================================================================
   全局变量 (保留在 main.c — 被多模块共享)
   ================================================================ */

/** IOmap[512]: SOEM IO 映射表 */
char IOmap[512];

uint32_t vid = 0;               /**< DM9000 芯片 ID */
int dorun = 0;                  /**< 运行标志: 1=所有轴已使能 */
uint8 fault_reset_requested = 0; /**< 故障复位请求标志 */

uint8_t socket_mode;            /**< 网口模式 */

MotorAxis_t axis[MAX_AXES];    /**< 轴控制数组 (8轴, 运行期 g_active_axes 控制实际使用数) */

/* ── 运行时轴数配置 (HMI 可通过 Modbus 动态修改) ── */
uint8_t g_active_axes    = 4;     /**< 激活轴总数 (默认4, 兼容现有HMI) */
uint8_t g_active_coupled = 4;     /**< 激活联动轴数 (默认4) */
uint8_t g_active_aux     = 0;     /**< 激活辅助轴数 (默认0) */

/* ── RS485 和 CAN 缓冲区 ── */
uint8 rs485buf[64];
uint8 len485;
uint8 can_tx_buf[8];
uint8 can_rx_buf[8];


/* ================================================================
   PVD_Init() — 掉电检测初始化 (FLASH_STORAGE_DISABLED 时不启用)
   ================================================================ */
#if !FLASH_STORAGE_DISABLED
static void PVD_Init(void)
{
    /* ── 1. 配置 PVD: 阈值 2.85V (Level 6), 使能检测 ── */
    /* PLS[2:0] = 110 (Level 6 = ~2.85V), PVDEN = 1 */
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_PLS_Msk) | PWR_CR1_PLS_LEV6 | PWR_CR1_PVDEN;
    __DSB();

    /* ── 2. 配置 EXTI 线 16 (PVD 输出) — 下降沿触发 ── */
    EXTI->FTSR1  |=  EXTI_FTSR1_TR16;       /* 下降沿触发中断 */
    EXTI->RTSR1  &= ~EXTI_RTSR1_TR16;       /* 关上升沿触发 */
    EXTI->IMR1   |=  EXTI_IMR1_IM16;        /* 不屏蔽 line 16 中断 */

    /* ── 3. 清除可能残留的挂起标志 ── */
    EXTI->PR1 = (1U << 16);                 /* 写1清除 line 16 挂起 */

    /* ── 4. 最高中断优先级 ── */
    NVIC_SetPriority(PVD_AVD_IRQn, 0);
    NVIC_EnableIRQ(PVD_AVD_IRQn);

    #if printf_cmd
    printf("[PVD] Enabled, threshold=2.85V, priority=0\r\n");
    #endif
}
#endif /* !FLASH_STORAGE_DISABLED */

/* ================================================================
   main() — 系统主函数
   ================================================================
 * 启动流程:
 *   1. MPU + Cache 配置
 *   2. HAL 初始化 + 系统时钟 400MHz
 *   3. 外设初始化 (GPIO/ETH/TIM/USART)
 *   4. Flash 参数加载
 *   5. DM9000 + DM9161 初始化
 *   6. RS485 + ModBus + CAN 初始化
 *   7. 定时器中断启动
 *   8. 网络链路检测
 *   9. EtherCAT 从站发现 + PDO 配置
 *   10. DC 同步 + 状态机推进到 OP
 *   11. CiA 402 使能序列
 *   12. FreeRTOS 任务创建 + 调度器启动
 */
int main(void)
{
    uint32_t dm9161status = 0;
    uint32_t i;
    uint16_t slc;

    /* =================================================================
     * 阶段1: 系统初始化
     * ================================================================= */
    MPU_Config();
    Cache_Enable();

    dorun = 0;

    HAL_Init();

    /* 系统时钟: HSE 25MHz → PLL → 400MHz */
    Stm32_Clock_Init(160, 5, 2, 4);

    /* 外设初始化 */
    MX_GPIO_Init();
    MX_ETH_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();

    /* 调试串口 */
    DEBUG_USART_Config();


    /* Flash 参数加载 (Flash 禁用时仅用 RAM 默认值) */
    Param_LoadAll();
    Param_EnsureDefaults();
    #if printf_cmd
    printf("Params loaded, version=%lu\r\n", Param_GetVersion());
    #endif

#if !FLASH_STORAGE_DISABLED
    /* 掉电保存槽初始化 (读取上次掉电时保存的轴位置) */
    Position_PowerFailInit();
#endif

printf("你好");

    /* 以太网控制器初始化 */
    DM9000_Init();
    vid = dm9k_ReadID();
    #if printf_cmd
    printf("\r\nDM9000 VID=%x\n\r", vid);
    #endif
    PHY_Init();

    /* 通信接口 */
    RS485_Init(115200);
    Modbus_Init();
    FDCAN1_Mode_Init(10, 8, 31, 8, FDCAN_MODE_NORMAL);

	/* 启动定时器中断 */
    __enable_irq();
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);

    /* =================================================================
     * 阶段2: 网络链路检测
     * =================================================================
     * 循环检测直到网线插入并建立物理链路
     *
     * socket_mode 含义:
     *   0 = 无连接 (未插网线) → 继续循环
     *   1 = 单网口模式 (DM9161 直连)
     *   2 = 双网口模式 (DM9000 + DM9161 冗余)
     *
     * 检测逻辑:
     *   DM9161 BSR bit2 = 1 (链路已建立) + DM9000 NSR bit6 = 0x40 → 双网口
     *   DM9161 BSR bit2 = 1 (链路已建立) + DM9000 NSR bit6 = 0    → 单网口
     */
    socket_mode = 1;                    // 默认单网口
    do
    {
        /* 读取 DM9161 PHY 状态寄存器 (BSR) */
        HAL_ETH_ReadPHYRegister(&heth, DM9161addr, DM9161_BSR, &dm9161status);

        if ((dm9161status & 0x0004)                              // DM9161 链路已建立
         && (dm9k_ReadReg(DM9000_REG_NSR) & DM9000_LINK_STATUE)) // DM9000 也检测到链路
            socket_mode = 2;                                      // → 双网口冗余模式
        else if ((dm9161status & 0x0004)                         // 仅 DM9161 链路
         && (!(dm9k_ReadReg(DM9000_REG_NSR) & DM9000_LINK_STATUE)))
            socket_mode = 1;                                      // → 单网口模式
        else {
            socket_mode = 0;                                      // → 无连接
            #if printf_cmd
            printf("Please plug in the Ethernet cable first\r\n");
            #endif
        }
        Delay_ms(100);
        ERR_TOGGLE;                     // 闪烁错误灯 (等待连接)
    } while (socket_mode == 0);         // 循环直到检测到连接

    ERR_OFF;
	
    ec_init();
    #if printf_cmd
    printf("\r\nSocket Mode: %d\r\n", socket_mode);
    #endif
    Delay_ms(1000);
    RUN_ON;

    /* =================================================================
     * 阶段3: EtherCAT 从站发现和配置
     * ================================================================= */
    if (ec_config_init(TRUE) > 0)
    {
        #if printf_cmd
        printf("%d slaves found and configured.\r\n", ec_slavecount);
        #endif

        /* 注册 PDO 配置回调 */
        if (ec_slavecount >= 1)
        {
            for (slc = 1; slc <= ec_slavecount; slc++)
            {
                ec_slave[slc].PO2SOconfig = &Servosetup;
            }
        }

        /* DC 分布式时钟同步 */
        ec_configdc();
        for (slc = 1; slc <= ec_slavecount; slc++)
        {
            ec_dcsync0(slc, TRUE, SYNC0TIME, 250000);
            #if printf_cmd
            printf("DC SYNC0 enabled on slave %d\r\n", slc);
            #endif
        }

        /* IO 映射 */
        ec_config_map(&IOmap);
        #if printf_cmd
        printf("Slaves mapped, state to SAFE_OP.\r\n");
        #endif

        /* 等待 SAFE_OP */
        ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
        ec_readstate();
        #if printf_cmd
        printf("Slave 0 State=0x%04x\r\n", ec_slave[0].state);
        #endif

        /* 轴初始化 */
        for (int ax = 0; ax < MAX_AXES; ax++)
        {
            uint16 slv = ax + 1;
            if (slv > ec_slavecount) {
                #if printf_cmd
                printf("%c: no slave (idx=%d), skipping\r\n", "XYZRUVWS"[ax], slv);
                #endif
                continue;
            }
            Axis_Init(ax, slv);
        }
          axis[0].axis_type = 0;  /* X轴 直线 */
          axis[1].axis_type = 0;  /* Y轴 直线 */
          axis[2].axis_type = 0;  /* Z轴 直线 */
          axis[3].axis_type = 1;  /* R轴 旋转 */
          axis[4].axis_type = 0;  /* U轴 直线 */
          axis[5].axis_type = 0;  /* V轴 直线 */
          axis[6].axis_type = 0;  /* W轴 直线 */
          axis[7].axis_type = 1;  /* S轴 旋转 */


        /* ── 齿轮比初始化 + 位置安全预同步 ──
         * μ m↔脉冲转换由 coord_sys.c 完成, 不经过伺服齿轮比 */
        #if printf_cmd
        printf("\r\n--- Refresh Axis Gear Ratio ---\r\n");
        #endif
        RefreshAxisGearRatio();
        Servo_WriteAllGearRatios();

        /* =================================================================
         * 阶段4: 状态机推进到 OP
         *   保持 ISR 运行提供 DC SYNC0
         *   用 FPRD (非 BRD) 逐站检查状态，避免与 ISR 抢 EMAC
         * ================================================================= */
        #if printf_cmd
        printf("\r\nRequest operational state...\r\n");
        #endif
        ec_slave[0].state = EC_STATE_OPERATIONAL;

        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_writestate(0);
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

        if (ec_slave[0].state != EC_STATE_OPERATIONAL)
        {
            int op_retry = 0;
            do
            {
                ec_writestate(0);

                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);

                /* 逐站 FPRD 检查，不依赖 BRD */
                int all_op = 1;
                for (int si = 1; si <= ec_slavecount; si++) {
                    ec_statecheck(si, EC_STATE_OPERATIONAL, 20000);
                    if (ec_slave[si].state != EC_STATE_OPERATIONAL)
                        all_op = 0;
                }
                if (all_op) {
                    ec_slave[0].state = EC_STATE_OPERATIONAL;
                    break;
                }

                op_retry++;
                if (op_retry > 200)
                {
                    #if printf_cmd
                    printf("OP timeout! slave0=0x%04x\r\n", ec_slave[0].state);
                    #endif
                    ec_readstate();
                    for (int si = 1; si <= ec_slavecount; si++)
                    {
                        #if printf_cmd
                        printf("  slv%d: state=0x%04x AL=0x%04x\r\n",
                               si, ec_slave[si].state, ec_slave[si].ALstatuscode);
                        #endif
                    }
                    break;
                }
            }
            while (ec_slave[0].state != EC_STATE_OPERATIONAL);
        }

        #if printf_cmd
        printf("Slave 0 State=0x%04x\r\n", ec_slave[0].state);
        #endif

        if (ec_slave[0].state == EC_STATE_OPERATIONAL)
        {
            #if printf_cmd
            printf("Operational state reached for all slaves.\r\n");
            #endif

            /* 进 OP 后、使能前再次同步 TargetPos = CurrentPosition, 防止爆冲 */
            #if printf_cmd
            printf("\r\n--- Pre-Enable Position Sync ---\r\n");
            #endif
            HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            Axis_SyncAllTargetsFromEncoder();
            for (int ax = 0; ax < NUM_AXES; ax++) {
                if (ax + 1 > ec_slavecount) continue;
                axis[ax].pdo_out->ControlWord = 0x0006;
                axis[ax].pdo_out->TargetMode  = 8;
            }
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

            /* CiA 402 使能序列 */
            #if printf_cmd
            printf("\r\n--- CiA 402 Enable Sequence ---\r\n");
            #endif
            Axis_EnableAll();

            /* 自动清除上电故障 */
            for (int ax = 0; ax < NUM_AXES; ax++) {
                if (ax + 1 > ec_slavecount) continue;
                uint16 sw = axis[ax].pdo_in->StatusWord;
                if (sw & 0x0008) {
                    uint16 err = axis[ax].pdo_in->ErrorCode;
                    #if printf_cmd
                    printf("%c: auto-clear fault 0x%04x\r\n", "XYZRUVWS"[ax], err);
                    #endif
                    axis[ax].pdo_out->ControlWord = 0x0080;
                    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
                    HAL_Delay(20);
                    ec_receive_processdata(EC_TIMEOUTRET);
                    axis[ax].pdo_out->ControlWord = 0x0006;
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    HAL_Delay(20);
                    axis[ax].pdo_out->ControlWord = 0x0007;
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    HAL_Delay(20);
                    axis[ax].pdo_out->ControlWord = 0x000F;
                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                    HAL_Delay(20);
                    axis[ax].actual_pos = axis[ax].pdo_in->CurrentPosition;
                    axis[ax].target_pos = axis[ax].actual_pos;
                    axis[ax].cmd_target_pos = axis[ax].actual_pos;
                    axis[ax].pdo_out->TargetPos = axis[ax].target_pos;
                    axis[ax].pdo_out->TargetMode = 8;
                    axis[ax].enabled = 1;
                    #if printf_cmd
                    printf("%c: re-enabled pos=%d\r\n", "XYZRUVWS"[ax], (int)axis[ax].target_pos);
                    #endif
                }
            }

#if !FLASH_STORAGE_DISABLED
            /* 位置持久化恢复: 若编码器上电返回0, 从掉电槽/param_ram恢复 */
            Position_RestoreIfNeeded();

            /* 掉电检测: PVD 监控 VDD, 跌落时自动保存轴位置 */
            PVD_Init();
#endif

            /* ══════════════════════════════════════════════════════════════════
               步骤 2/2: 最后阶段 — R/S 轴软限位兜底 (哨兵值判断)
               与 param_store.c 步骤 5 逻辑一致: 仅在参数从未被用户自定义过时
               (= 2000000000) 才设默认值。用户通过 HMI 修改并保存过的值不会被覆盖。
               ══════════════════════════════════════════════════════════════════ */
            if (param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, 3)] == 2000000000)
                param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, 3)] = GetAxisPPR(3) - 1;  /* R轴: 35999 */
            if (param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, 3)] == -2000000000)
                param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, 3)] = 0;
            if (param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, 7)] == 2000000000)
                param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, 7)] = GetAxisPPR(7) - 1;  /* S轴: 35999 */
            if (param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, 7)] == -2000000000)
                param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, 7)] = 0;

                dorun = 1;
        }
        else
        {
            #if printf_cmd
            printf("Not all slaves reached operational state.\r\n");
            #endif
            ec_readstate();  /* 刷新各站 state 和 ALStatusCode */
            for (i = 1; i <= ec_slavecount; i++)
            {
                if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                {
                    #if printf_cmd
                    printf("Slave %d State=0x%04x ALStatusCode=0x%04x\r\n",
                           i, ec_slave[i].state, ec_slave[i].ALstatuscode);
                    #endif
                }
            }
        }
    }
    else
    {
        #if printf_cmd
        printf("No slaves found!\r\n");
        #endif
    }

    /* ================================================================
       FreeRTOS 任务创建 & 启动调度器
       ================================================================ */
    if (xTaskCreate(vEtherCAT_Task, "EtherCAT", 1280, NULL, 6, &xEtherCATTaskHandle) != pdPASS)
    {
        #if printf_cmd
        printf("FATAL: Failed to create EtherCAT task!\r\n");
        #endif
        Error_Handler();
    }

    if (xTaskCreate(vCmd_Task, "CmdParse", 1280, NULL, 3, &xCmdTaskHandle) != pdPASS)
    {
        #if printf_cmd
        printf("FATAL: Failed to create CmdParse task!\r\n");
        #endif
        Error_Handler();
    }

    vTaskStartScheduler();

    while (1) {}
}

/* ================================================================
   系统初始化函数
   ================================================================ */

void Cache_Enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
    SCB->CACR |= 1 << 2;  /* D-Cache write-through */
}

/* ================================================================
 * Stm32_Clock_Init() — 系统时钟初始化
 * ================================================================
 * @param plln : PLL N 倍频系数 (160)
 * @param pllm : PLL M 预分频  (5)
 * @param pllp : PLL P 分频    (2) → 系统时钟
 * @param pllq : PLL Q 分频    (4) → 外设时钟源
 *
 * 【时钟树 (默认参数: 160, 5, 2, 4)】
 *   HSE = 25MHz (外部晶振)
 *   VCO = 25 × 160/5 = 800MHz
 *   SYSCLK = 800/2   = 400MHz (CPU 时钟)
 *   HCLK   = 400/2   = 200MHz (AHB 总线)
 *   PCLK1  = 200/2   = 100MHz (APB1)
 *   PCLK2  = 200/2   = 100MHz (APB2)
 *   PLL1Q  = 800/4   = 200MHz (FDCAN 时钟源)
 *
 * 【电压调节】
 *   SCALE1 = VOS1 (高性能模式, 支持 400MHz)
 *   需要等待 VOS 就绪标志 (PWR_D3CR_VOSRDY)
 */
void Stm32_Clock_Init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* ── 配置电压调节器为 SCALE1 ──
     * SCALE1: 最高性能, 支持 CPU 400MHz */
    MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);              // 禁用 SCU
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) {}  // 等待就绪

    /* ── 配置 HSE + PLL ── */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // 使用外部晶振 (25MHz)
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;                     // 开启 HSE
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;                    // 关闭内部 HSI (省电)
    RCC_OscInitStruct.CSIState = RCC_CSI_OFF;                    // 关闭内部 CSI
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                 // 开启 PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;         // PLL 源 = HSE
    RCC_OscInitStruct.PLL.PLLN = plln;                           // VCO 倍频
    RCC_OscInitStruct.PLL.PLLM = pllm;                           // 输入预分频
    RCC_OscInitStruct.PLL.PLLP = pllp;                           // 系统时钟分频
    RCC_OscInitStruct.PLL.PLLQ = pllq;                           // 外设时钟分频
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;          // 宽 VCO 范围
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;          // VCO 输入范围2
    ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
    if (ret != HAL_OK) while (1);  // 时钟配置失败 → 死循环

    /* ── 配置系统时钟分频 ── */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 |
                                   RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;     // 系统时钟 = PLL
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;             // 400MHz
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;               // 200MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;               // 100MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;               // 100MHz
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;               // 100MHz
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV4;               // 50MHz
    ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Flash 等待=2
    if (ret != HAL_OK) while (1);

    /* ── 使能补偿单元和系统配置控制器 ── */
    __HAL_RCC_CSI_ENABLE();              // 使能 CSI (用于时钟安全)
    __HAL_RCC_SYSCFG_CLK_ENABLE();       // 使能系统配置时钟
    HAL_EnableCompensationCell();        // 使能 I/O 补偿单元 (高速IO需要)
}

/* ================================================================
 * MPU_Config() — 内存保护单元配置
 * ================================================================
 * STM32H7 的 Cortex-M7 有 L1 Cache, 需要 MPU 控制不同内存区域的缓存策略
 *
 * 【4个 MPU 区域配置:】
 *
 * Region 0: 0x30040000 (16KB) — ETH DMA 描述符
 *   属性: Non-Cacheable, Bufferable
 *   原因: DMA 描述符必须非缓存, 否则 CPU 读到的可能是缓存旧值
 *
 * Region 1: 0x30044000 (16KB) — ETH TX 缓冲区
 *   属性: Cacheable, Non-Bufferable
 *   原因: TX 数据由 CPU 写入, write-through 保证 DMA 可见
 *
 * Region 2: 0x68000000 (64MB) — FMC 扩展 IO (DM9000)
 *   属性: Non-Cacheable, Bufferable (Device Memory)
 *   原因: DM9000 是外部设备, 必须非缓存
 *
 * Region 3: 0x24000000 (512KB) — SRAM4
 *   属性: Cacheable, Non-Bufferable, Shareable
 *   原因: 通用数据, 可缓存提升性能
 */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};
    HAL_MPU_Disable();

    /* ── Region 0: ETH DMA 描述符 (0x30040000, 16KB, 非缓存) ── */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x30040000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;   // ← 非缓存
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* ── Region 1: ETH TX 缓冲 (0x30044000, 16KB, 可缓存) ── */
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress = 0x30044000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;        // ← 可缓存
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* ── Region 2: FMC 扩展 IO (0x68000000, 64MB, 非缓存)
     * DM9000 通过 FMC 总线访问, 必须使用 Device 属性 */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x68000000;
    MPU_InitStruct.Size = ARM_MPU_REGION_SIZE_64MB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;    // ← 非缓存
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER2;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* ── Region 3: SRAM4 (0x24000000, 512KB, 可缓存) ── */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = 0x24000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;        // ← 可缓存
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER3;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* 使能 MPU (特权模式默认背景区域) */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* ================================================================
   HAL_TIM_PeriodElapsedCallback() — 定时器周期中断回调
   ================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim1.Instance)
    {
        if (dorun == 1)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            ec_send_processdata();

            if (xEtherCATTaskHandle != NULL) {
                vTaskNotifyGiveFromISR(xEtherCATTaskHandle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
        else
        {
            ec_send_processdata();
        }
    }
    if (htim->Instance == TIM7)
    {
        HAL_IncTick();
    }
}

/* ================================================================
   Error_Handler() — 系统错误处理
   ================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
