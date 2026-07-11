/**
 ******************************************************************************
 * @file           : ethercat_slave.c
 * @brief          : SV660N PDO 映射配置 (CiA 402 CSP)
 * ----------------------------------------------------------------------------
 * 通过 SDO 协议配置每个 SV660N 从站的 PDO 映射
 *
 * RxPDO (主站→从站, 0x1600, SM2):
 *   - 0x6040:00 (16bit) — ControlWord
 *   - 0x607A:00 (32bit) — TargetPosition
 *   - 0x6060:00 (8bit)  — ModesOfOperation
 *
 * TxPDO (从站→主站, 0x1A00, SM3):
 *   - 0x6041:00 (16bit) — StatusWord
 *   - 0x6064:00 (32bit) — ActualPosition
 *   - 0x606C:00 (32bit) — ActualVelocity
 *   - 0x603F:00 (16bit) — ErrorCode
 *   - 0x6061:00 (8bit)  — ModesOfOperationDisplay
 ******************************************************************************
 */
#include "ethercat_slave.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "DM9000.h"
#include "motor_axis.h"
#include "motion_api.h"
#include "param_defs.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

/* ── 外部引用 ── */
extern MotorAxis_t axis[MAX_AXES];
extern int dorun;
extern TaskHandle_t xEtherCATTaskHandle;
extern char IOmap[];

/* ================================================================
   Servosetup() — SV660N PDO 映射配置 (CiA 402 CSP)
   ================================================================
 * @param  slave : 从站索引号 (1-based)
 * @retval 1 : 配置成功 (retval 累计错误数, 此处忽略)
 *
 * 此函数在 ec_config_init() → ec_config_map() 阶段被调用
 * 通过 SDO 协议配置每个 SV660N 从站的 PDO 映射
 *
 * 【SDO 编码格式 0xIIII00SS】
 *   IIII = 对象字典索引 (如 0x6040 = ControlWord)
 *   SS   = 位宽 (0x10=16bit, 0x20=32bit, 0x08=8bit)
 *
 * 【CiA 402 模式选择】
 *   0x6060 = 8 → CSP (Cyclic Synchronous Position) 循环同步位置模式
 */
int Servosetup(uint16 slave)
{
    int retval;
    uint16 u16val;
    uint8  u8val;
    uint32 u32val;
    retval = 0;

    /* ── 配置 RxPDO (0x1C12 → SM2) ── */
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1600;
    retval += ec_SDOwrite(slave, 0x1c12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c12, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    /* ── 配置 RxPDO 映射条目 (0x1600) ──
     * 子索引1: 0x60400010 → ControlWord (16bit)
     * 子索引2: 0x607A0020 → TargetPosition (32bit)
     * 子索引3: 0x60600008 → ModesOfOperation (8bit)
     * 子索引0: 3 → 3个映射条目
     */
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u32val = 0x60400010;   /* ControlWord: 对象0x6040, 16bit */
    retval += ec_SDOwrite(slave, 0x1600, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x607A0020;   /* TargetPosition: 对象0x607A, 32bit */
    retval += ec_SDOwrite(slave, 0x1600, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60600008;   /* ModesOfOperation: 对象0x6060, 8bit */
    retval += ec_SDOwrite(slave, 0x1600, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u8val = 3;
    retval += ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    /* ── 配置 TxPDO (0x1C13 → SM3) ── */
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u16val = 0x1a00;
    retval += ec_SDOwrite(slave, 0x1c13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    u8val = 1;
    retval += ec_SDOwrite(slave, 0x1c13, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    /* ── 配置 TxPDO 映射条目 (0x1A00) ──
     * 子索引1: 0x60410010 → StatusWord (16bit)
     * 子索引2: 0x60640020 → ActualPosition (32bit)
     * 子索引3: 0x606C0020 → ActualVelocity (32bit)
     * 子索引4: 0x603F0010 → ErrorCode (16bit)
     * 子索引5: 0x60610008 → ModesOfOperationDisplay (8bit)
     * 子索引0: 5 → 5个映射条目
     */
    u8val = 0;
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);
    u32val = 0x60410010;   /* StatusWord: 对象0x6041, 16bit */
    retval += ec_SDOwrite(slave, 0x1A00, 0x01, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60640020;   /* ActualPosition: 对象0x6064, 32bit */
    retval += ec_SDOwrite(slave, 0x1A00, 0x02, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x606C0020;   /* ActualVelocity: 对象0x606C, 32bit */
    retval += ec_SDOwrite(slave, 0x1A00, 0x03, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x603F0010;   /* ErrorCode: 对象0x603F, 16bit */
    retval += ec_SDOwrite(slave, 0x1A00, 0x04, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u32val = 0x60610008;   /* ModesOfOperationDisplay: 对象0x6061, 8bit */
    retval += ec_SDOwrite(slave, 0x1A00, 0x05, FALSE, sizeof(u32val), &u32val, EC_TIMEOUTRXM);
    u8val = 5;
    retval += ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    /* ── 设置 CSP 模式 (0x6060 = 8) ── */
    u8val = 8;
    retval += ec_SDOwrite(slave, 0x6060, 0x00, FALSE, sizeof(u8val), &u8val, EC_TIMEOUTRXM);

    return 1;
}

/* ================================================================
   EtherCAT_RecoverOP() — 恢复 EtherCAT 通信到 OP 状态
   ================================================================
 * 当 Flash 擦除或其它原因导致 EtherCAT 通信中断后,
 * 此函数将状态机逐步恢复到 OP 状态，然后重新使能所有轴。
 *
 * 恢复流程:
 *   1. 禁止 TIM1 ISR, 唤醒 NIC (20帧 PDO 交换)
 *   2. 请求 INIT → 等待, 校验失败则退出
 *   3. ec_config_init(FALSE) 重新初始化从站 (SM/FMMU 配置)
 *   4. 重新注册 PO2SOconfig 回调 + ec_config_map() 映射 IO
 *   5. Axis_Init() 重新绑定轴 PDO 指针 (ec_slave[] 已重建)
 *   6. DC 时钟同步恢复 (ec_configdc + ec_dcsync0)
 *   7. 请求 PRE-OP → 等待
 *   8. 请求 SAFE-OP → 等待 (带 PDO 交换)
 *   9. 请求 OP → 等待 (带重试循环)
 *  10. 预加载轴位置并调用 Axis_EnableAll() 使能
 *  11. 恢复 TIM1 ISR, 设置 dorun=1
 *
 * ⚠️ 关键修复: INIT 状态会丢失从站的 SM/FMMU/PDO 映射配置,
 *   必须先通过 ec_config_init() + ec_config_map() 重新建立,
 *   否则从站无法进入 SAFE-OP/OP, 导致 statecheck 超时卡死.
 * ⚠️ 此函数会短暂禁用 TIM1_UP_IRQn, 调用者不需要额外处理
 */
int EtherCAT_RecoverOP(void)
{
    #if printf_cmd
    printf("EtherCAT recovery: restoring OP state...\r\n");
    #endif

    /* Step 1: 干净断开 EtherCAT
     * 先关 TIM1 中断 → 不再发任何 EtherCAT 帧
     * → 从站检测到总线静默 → DC 超时 → SAFEOP+ERR
     * 但 DM9000 处于干净状态, 不会有 RX 溢出/寄存器错乱 */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

    /* Step 2: 完整重初始化 DM9000 + 重绑 SOEM
     * DM9000_Initnic(): 硬件复位 + PHY 配置 + MAC 设置
     * (dm9k_reset 只做硬件复位, 缺少 PHY 协商, 导致帧转发异常) */
    #if printf_cmd
    printf("  Re-syncing EtherCAT...\r\n");
    #endif
    DM9000_Initnic();
    ec_init();

    /* Step 3: 完整 EtherCAT 状态机重启
     * SAFEOP+ERR(0x0014) 无法直接清除, 先下到 INIT 再重新上。
     * 注意: TIM1 全程禁用, 禁止 ISR 发帧干扰状态机推进 */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

    /* ── 阶段 A: 下到 INIT (清所有错误状态) ── */
    #if printf_cmd
    printf("  -> INIT...\r\n");
    #endif
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE * 4);
    for (int retry = 0; retry < 50; retry++) {
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_INIT, 50000);
        if (ec_slave[0].state == EC_STATE_INIT) break;
    }
    #if printf_cmd
    printf("  -> INIT: state=0x%04x\r\n", ec_slave[0].state);
    #endif

    if (ec_slave[0].state != EC_STATE_INIT) {
        #if printf_cmd
        printf("  FAILED: cannot reach INIT\r\n");
        #endif
        HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
        return 0;
    }

    /* ── 阶段 B: INIT → PREOP ── */
    #if printf_cmd
    printf("  -> PREOP...\r\n");
    #endif
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);
    for (int retry = 0; retry < 50; retry++) {
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_PRE_OP, 50000);
        if (ec_slave[0].state == EC_STATE_PRE_OP) break;
    }
    #if printf_cmd
    printf("  -> PREOP: state=0x%04x\r\n", ec_slave[0].state);
    #endif

    if (ec_slave[0].state != EC_STATE_PRE_OP) {
        #if printf_cmd
        printf("  FAILED: cannot reach PREOP\r\n");
        #endif
        HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
        return 0;
    }

    /* ── 阶段 C: DC 配置 (PREOP 状态, 必须在 IO 映射之前) ──
     * 与正常启动顺序一致: DC → Map → SAFEOP → OP
     * ec_configdc() 写 SDO 配置 DC 时钟, ec_dcsync0() 配置 SYNC0
     * 必须在 ec_config_map() 之前, 否则 SDO 写入可能破坏 IO 映射 */
    #if printf_cmd
    printf("  -> DC configure...\r\n");
    #endif
    ec_configdc();
    for (int slc = 1; slc <= ec_slavecount; slc++) {
        ec_dcsync0(slc, TRUE, SYNC0TIME, 250000);
    }
    #if printf_cmd
    printf("  -> DC configured\r\n");
    #endif

    /* ── 阶段 D: IO 映射 → SAFEOP ── */
    #if printf_cmd
    printf("  -> IO map + SAFEOP...\r\n");
    #endif
    ec_config_map(&IOmap);

    /* ec_config_map 后 SOEM 更新了 ec_slave[].outputs/inputs,
     * 必须刷新各轴的 PDO 指针, 否则 PDO 读写指向旧地址 */
    for (int ax = 0; ax < MAX_AXES; ax++) {
        int slv = axis[ax].slave_idx;
        if (slv >= 1 && slv <= (int)ec_slavecount) {
            axis[ax].pdo_out = (PDO_Output *)ec_slave[slv].outputs;
            axis[ax].pdo_in  = (PDO_Input  *)ec_slave[slv].inputs;
        }
    }

    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    for (int retry = 0; retry < 100; retry++) {
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_SAFE_OP, 50000);
        if (ec_slave[0].state == EC_STATE_SAFE_OP) break;
    }
    #if printf_cmd
    printf("  -> SAFEOP: state=0x%04x\r\n", ec_slave[0].state);
    #endif

    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        #if printf_cmd
        printf("  WARNING: SAFEOP not reached (state=0x%04x), trying OP anyway...\r\n",
               ec_slave[0].state);
        #endif
    }

    /* ── 阶段 D.5: 恢复伺服齿轮比 + 位置预同步 ──
     * ⚠️ Servo_WriteAllGearRatios() 内部完成:
     *    禁用ISR → SDO写入0x6091 → PDO交换(等生效) → 位置回写 → 恢复ISR
     *    调用后 ISR 重新运行, 齿轮比已生效, TargetPos已同步为CurrentPosition */
    #if printf_cmd
    printf("  -> Restoring servo gear ratios via SDO...\r\n");
    #endif
    Servo_WriteAllGearRatios();

    /* 恢复 MCU 侧有效 PPR（含机械齿轮比 × 模式 PPR）
     * Servo_WriteAllGearRatios() 已将齿轮比 SDO 写入伺服并完成 PDO 交换,
     * 但 MCU 侧 effective_ppr 尚未同步。此处从 param_ram 读取机械齿轮比,
     * 结合各轴 axis_type 重算 effective_ppr, 确保后续 MCS 显示/定位换算正确。 */
    RefreshAxisGearRatio();

    /* ── 阶段 E: 开 TIM1 让 DC 时钟同步 → 推 OP ──
     * DC SYNC0 需要 EtherCAT 帧按 5ms 周期持续发送,
     * 必须开 TIM1 让 ISR 发帧, 等 DC 稳定后再请求 OP */
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
    HAL_Delay(100);  /* 等 ~20 个 SYNC0 周期, DC 时钟锁定 */

    int ec_ok = 0;
    if (ec_slave[0].state == EC_STATE_SAFE_OP) {
        #if printf_cmd
        printf("  -> OP...\r\n");
        #endif
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        ec_writestate(0);

        /* OP 推状态时 ISR 也在发帧, 不会冲突
         * 状态可能返回 0x0008(OP) 或 0x000c(OP+过渡),
         * 用位掩码 0x000F 取低4位后判断 >= OP 即可 */
        ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        for (int retry = 0; retry < 200; retry++) {
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            if ((ec_slave[0].state & 0x000F) >= EC_STATE_OPERATIONAL) {
                ec_ok = 1;
                break;
            }
            HAL_Delay(5);
        }
    }
    #if printf_cmd
    printf("  -> OP: state=0x%04x %s\r\n",
           ec_slave[0].state, ec_ok ? "(OK)" : "(FAILED)");
    #endif

    if (!ec_ok) {
        #if printf_cmd
        printf("  FAILED: cannot reach OP, ALStatusCode=0x%04x\r\n",
               ec_slave[0].ALstatuscode);
        #endif
        return 0;
    }

    #if printf_cmd
    printf("  Operational state reached.\r\n");
    #endif

    /* Step 4: 同步轴当前位置并准备使能 */
    #if printf_cmd
    printf("  Re-enabling axes...\r\n");
    #endif

    for (int ax = 0; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        axis[ax].pdo_out->ControlWord = 0x0000;
        axis[ax].pdo_out->TargetMode  = 8;
        axis[ax].fault_reset_step = 0;
        axis[ax].fault_reported = 0;
    }

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    for (int ax = 0; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        /* 初始同步: 齿轮比尚未写入, 直接原样回写 (任何齿轮比下都不产生位移) */
        int32 raw_pos = axis[ax].pdo_in->CurrentPosition;
        axis[ax].actual_pos     = raw_pos;
        axis[ax].target_pos     = raw_pos;
        axis[ax].cmd_target_pos = raw_pos;
        axis[ax].pdo_out->TargetPos   = raw_pos;  /* 原样回写, 零跳变 */
        axis[ax].pdo_out->TargetMode  = 8;
        axis[ax].pdo_out->ControlWord = 0x0006;
    }

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    /* CiA 402 使能: Shutdown → SwitchOn → EnableOp */
    for (int ax = 0; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        axis[ax].pdo_out->ControlWord = 0x0007;
    }
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    HAL_Delay(10);

    for (int ax = 0; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        axis[ax].pdo_out->ControlWord = 0x000F;
    }
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    HAL_Delay(10);

    for (int ax = 0; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        axis[ax].enabled = 1;
        #if printf_cmd
        printf("  %c: RE-ENABLED (pos=%d)\r\n",
               "XYZRUVWS"[ax], (int)axis[ax].actual_pos);
        #endif
    }

    #if printf_cmd
    printf("EtherCAT recovery: DONE\r\n");
    #endif
    return 1;
}

/* ── 前向声明 ── */
/* (gcd_u32 已移除 — 齿轮比不再从 pitch 计算) */

#define SERVO_GEAR_NUM_FIXED  524288
#define SERVO_GEAR_DEN_FIXED  3125

/* 旋转模式电子齿轮比: 262144:1125 → 36000 pulses/rev
 * 推导: 8388608 × 1125 / 262144 = 32 × 1125 = 36000  ✅ */
#define SERVO_GEAR_NUM_ROTARY  262144
#define SERVO_GEAR_DEN_ROTARY  1125

/**
 * @brief  根据轴模式更新 servo_gear_num/den 字段
 * @param  ax   : 轴索引 (3~7)
 * @param  type : 0=直线(50000ppr), 1=旋转(36000ppr)
 * @note   仅更新结构体字段, SDO 写入由调用者通过 Servo_WriteGearRatio() 完成
 */
void Axis_UpdateModeGearRatio(int ax, uint8_t type)
{
    if (type == 1) {
        axis[ax].servo_gear_num = SERVO_GEAR_NUM_ROTARY;
        axis[ax].servo_gear_den = SERVO_GEAR_DEN_ROTARY;
    } else {
        axis[ax].servo_gear_num = SERVO_GEAR_NUM_FIXED;
        axis[ax].servo_gear_den = SERVO_GEAR_DEN_FIXED;
    }
}

/* ================================================================
   RefreshAxisGearRatio() — 从参数表读取各轴齿轮比, 计算有效每圈脉冲
   ================================================================
 * effective_ppr = GetAxisPPR(ax) * gear_num / gear_den
 * 用于补偿机械传动比 (电机转 gear_num 圈 = 输出轴转 gear_den 圈)
 * 与 coord_sys.c 直线轴公式保持一致
 */
void RefreshAxisGearRatio(void)
{
    for (int ax = 0; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;

        int32 gear_num = Param_Get(AXIS_PARAM(PARAM_AXIS_GEAR_NUM, ax));
        int32 gear_den = Param_Get(AXIS_PARAM(PARAM_AXIS_GEAR_DEN, ax));
        if (gear_num <= 0) gear_num = 1;
        if (gear_den <= 0) gear_den = 1;

        /* effective_ppr = PULSES_PER_REV * gear_num / gear_den
         * (电机转 gear_num 圈 = 输出轴转 gear_den 圈)
         * 与 coord_sys.c 直线轴公式保持一致 */
        int64 eppr_64 = (int64)GetAxisPPR(ax) * (int64)gear_num / (int64)gear_den;
        if (eppr_64 > (int64)0x7FFFFFFF || eppr_64 <= 0) {
            #if printf_cmd
            printf("%c: gear=%ld/%ld → eppr overflow, using 1:1\r\n",
                   "XYZRUVWS"[ax], (long)gear_num, (long)gear_den);
            #endif
            axis[ax].effective_ppr = GetAxisPPR(ax);
        } else {
            axis[ax].effective_ppr = (int32)eppr_64;
        }
        #if printf_cmd
        printf("%c: gear=%ld/%ld effective_ppr=%ld (%ld%%)\r\n",
               "XYZRUVWS"[ax],
               (long)gear_num, (long)gear_den,
               (long)axis[ax].effective_ppr,
               (long)(axis[ax].effective_ppr * 100LL / GetAxisPPR(ax)));
        #endif
    }
}


/* ================================================================
   Servo_WriteGearRatio() — 通过 SDO 将电子齿轮比写入伺服驱动器
   ================================================================
 * 写入 CiA 402 对象字典:
 *   0x6091:01 → servo_gear_num (电机侧圈数)
 *   0x6091:02 → servo_gear_den (输出轴侧圈数)
 *
 * @param slave : EtherCAT 从站索引 (1-based)
 * @param ax    : 轴索引
 * @retval 1=成功 0=失败
 *
 * ⚠️ 调用前从站必须在 PREOP/SAFEOP/OP 状态 (SDO 可用)
 * ⚠️ 建议在伺服未使能时调用 (参数写入受限)
 */
int Servo_WriteGearRatio(uint16 slave, int ax)
{
    int retval = 0;
    int32 num = axis[ax].servo_gear_num;
    int32 den = axis[ax].servo_gear_den;

    if (num <= 0 || den <= 0) {
        #if printf_cmd
        printf("%c: invalid servo_gear (%ld/%ld), skip SDO\r\n",
               "XYZRUVWS"[ax], (long)num, (long)den);
        #endif
        return 0;
    }

    #if printf_cmd
    printf("%c: SDO write gear ratio 0x6091:01=%ld 0x6091:02=%ld ... ",
           "XYZRUVWS"[ax], (long)num, (long)den);
    #endif

    /* 写 0x6091:01 — Gear Ratio numerator */
    int ret1 = ec_SDOwrite(slave, 0x6091, 0x01, FALSE,
                           sizeof(num), &num, EC_TIMEOUTRXM);
    if (ret1 != 0) {
        retval++;
    }

    /* 写 0x6091:02 — Gear Ratio denominator */
    int ret2 = ec_SDOwrite(slave, 0x6091, 0x02, FALSE,
                           sizeof(den), &den, EC_TIMEOUTRXM);
    if (ret2 != 0) {
        retval++;
    }

    if (retval == 0) {
        #if printf_cmd
        printf("OK\r\n");
        #endif
        return 1;
    } else {
        #if printf_cmd
        printf("FAIL (ret=%d, ret1=%d ret2=%d)\r\n", retval, ret1, ret2);
        #endif
        return 0;
    }
}

/* ================================================================
   Servo_WriteAllGearRatios() — 将所有轴的齿轮比写入伺服驱动器
   ================================================================
 * 写入 524288:3125 齿轮比到所有 SV660N 从站。
 *
 * 设计原则:
 *   - 伺服齿轮比固定为 524288:3125, PDO 目标位置用 50000/rev 用户单位
 *   - PDO 反馈用原始编码器脉冲 (Pn280=0, 不经齿轮比), MCU 直读
 *   - MCU 写入时 enc×3125/524288 缩放为 50000/rev
 *   - μm↔脉冲转换在 MCU 端 coord_sys.c 完成
 *
 * 流程:
 *   1. ISR 禁用 → SDO 写入 524288:3125
 *   2. SDO 读回验证 (确认伺服已接受)
 *   3. PDO 交换 (让齿轮比生效到目标位置)
 *   4. 位置回写: TargetPos ← enc×3125/524288 (零跳变)
 *   5. ISR 恢复
 */
void Servo_WriteAllGearRatios(void)
{
    int sdo_errors = 0;

    #if printf_cmd
    printf("\r\n--- Write Servo Gear Ratios (Linear:524288:3125→50000/rev, Rotary:262144:1125→36000/rev) ---\r\n");
    #endif
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

    /* ── 阶段 1: 初始化各轴伺服齿轮比参数 + SDO 写入 524288:3125 ── */
    for (int ax = 0; ax < MAX_AXES; ax++) {
        uint16 slv = (uint16)(ax + 1);
        if (slv > ec_slavecount) {
            #if printf_cmd
            printf("%c: no slave, skip\r\n", "XYZRUVWS"[ax]);
            #endif
            continue;
        }
        /* 设置伺服电子齿轮比 (SDO 写入和读回验证使用) */
        Axis_UpdateModeGearRatio(ax, axis[ax].axis_type);
        if (!Servo_WriteGearRatio(slv, ax))
            sdo_errors++;
    }

    /* ── 阶段 2: SDO 读回验证 — 确认伺服已接受齿轮比 ──
     * ec_SDOread 阻塞等待邮箱响应, 返回后伺服已确认写入。
     * 读回值必须等于写入值, 否则伺服可能拒绝/钳位了齿轮比。 ── */
    for (int ax = 0; ax < MAX_AXES; ax++) {
        uint16 slv = (uint16)(ax + 1);
        if (slv > ec_slavecount) continue;

        int32 rd_num = 0, rd_den = 0;
        int sz = 4;
        int ret1 = ec_SDOread(slv, 0x6091, 0x01, FALSE, &sz, &rd_num, EC_TIMEOUTRXM);
        int ret2 = ec_SDOread(slv, 0x6091, 0x02, FALSE, &sz, &rd_den, EC_TIMEOUTRXM);

        if (ret1 <= 0 || ret2 <= 0) {
            #if printf_cmd
            printf("%c: SDO read-back FAILED (ret1=%d ret2=%d)\r\n",
                   "XYZRUVWS"[ax], ret1, ret2);
            #endif
            sdo_errors++;
        } else if (rd_num != axis[ax].servo_gear_num ||
                   rd_den != axis[ax].servo_gear_den) {
            #if printf_cmd
            printf("%c: gear MISMATCH: wrote %ld/%ld, read %ld/%ld\r\n",
                   "XYZRUVWS"[ax],
                   (long)axis[ax].servo_gear_num, (long)axis[ax].servo_gear_den,
                   (long)rd_num, (long)rd_den);
            #endif
            sdo_errors++;
        } else {
            #if printf_cmd
            printf("%c: gear verified %ld/%ld OK\r\n",
                   "XYZRUVWS"[ax], (long)rd_num, (long)rd_den);
            #endif
        }
    }

    if (sdo_errors > 0)
        #if printf_cmd
        printf("WARNING: %d gear ratio errors detected!\r\n", sdo_errors);
        #endif

    /* ── 阶段 3: PDO 交换, 让伺服刷新位置数据 ──
     * 齿轮比写入后伺服内部位置计算流水线需要更新,
     * 多交换几次确保新齿轮比已生效。 ── */
    for (int flush = 0; flush < 10; flush++) {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
    }

    /* ── 阶段 4: 位置安全同步 ──
     * 读取 CurrentPosition 原样回写 TargetPos (零跳变),
     * MCU 内部直接使用 PDO 原始值, 不做齿轮比转换。 ── */
    for (int ax = 0; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;

        int32 cur = axis[ax].pdo_in->CurrentPosition;
        axis[ax].actual_pos     = cur;
        axis[ax].target_pos     = cur;
        axis[ax].cmd_target_pos = cur;
        axis[ax].pdo_out->TargetPos   = cur;
        axis[ax].pdo_out->TargetMode  = 8;
        axis[ax].pdo_out->ControlWord = 0x0006;
    }

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
}

