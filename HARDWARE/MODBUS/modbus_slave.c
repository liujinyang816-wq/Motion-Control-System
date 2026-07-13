/**
 ******************************************************************************
 * @file    modbus_slave.c
 * @brief   MODBUS RTU 从站协议栈 — CRC16、帧解析、FC03/06/16 分发
 ******************************************************************************
 */

#include "modbus_slave.h"
#include "rs485.h"
#include "string.h"
#include "stdio.h"
#include "param_defs.h"
#include "motion_api.h"
#include "motor_axis.h"
#include "ethercatmain.h"
#include "ethercat_slave.h"

/* 外部引用 */
extern int32 param_ram[PARAM_COUNT];
extern MotorAxis_t axis[MAX_AXES];
extern uint8 fault_reset_requested;
extern uint8_t g_active_axes;
extern uint8_t g_active_coupled;
extern uint8_t g_active_aux;

/* ModBus 读取的系统时间 */
uint32_t g_uptime_seconds = 0;

uint16_t mb_regs[MB_REG_COUNT];

static uint8_t  mb_rx_buf[MB_RX_BUF_SIZE];      // ModBus 接收缓冲区
static uint8_t  mb_rx_len = 0;          //收到字节数
static uint32_t mb_last_tick = 0;       //最后一个字节到达的时刻
static int8_t   mirror_cur_ax = -1;    /**< 镜像区当前对应的轴索引,-1=未初始化, 写回时校验防止切轴污染 */

/* 轴参数镜像映射: 镜像寄存器偏移 → ParamId */
static const ParamId_t mirror_map[MB_REG_MIRROR_COUNT] = {
    PARAM_AXIS_MAX_RPM,        /* 224: 最大转速 */
    PARAM_AXIS_MAX_ACCEL,      /* 225: 最大加速度 */
    PARAM_AXIS_MAX_JERK,       /* 226: 最大Jerk */
    PARAM_AXIS_BACKLASH,       /* 227: 反向间隙 */
    PARAM_AXIS_PITCH,          /* 228: 丝杠导程 */
    PARAM_AXIS_GEAR_NUM,       /* 229: 齿轮比分子 */
    PARAM_AXIS_GEAR_DEN,       /* 230: 齿轮比分母 */
    PARAM_AXIS_INVERT_DIR,     /* 231: 反转方向 */
    PARAM_AXIS_JOG_VEL,        /* 232: 点动速度 */
    PARAM_AXIS_HOMING_VEL,     /* 233: 回零速度 */
    PARAM_AXIS_HOMING_ACC,     /* 234: 回零加速度 */
    PARAM_AXIS_POS_ARRIVE_WIN, /* 235: 到位窗口 */
    PARAM_AXIS_MAX_FOLLOW_ERR, /* 236: 最大跟随误差 */
};

/* ── ModBus 从站初始化: 清零寄存器、设置默认值 ── */
void Modbus_Init(void)
{
    memset(mb_regs, 0, sizeof(mb_regs));            //清零mb_regs寄存器数组
    mb_rx_len = 0;
    mb_last_tick = 0;
    mb_regs[MB_REG_POS_SPEED]  = 100;           // 默认参数定位速度为100RPM
    mb_regs[MB_REG_JOG_SPEED]  = 200;           // 默认点动速度为200RPM

    /* axis_type 寄存器默认值 (与 main.c 步骤4 ① 的 axis[].axis_type 保持一致)
     * 必须在此初始化, 否则 mb_regs 清零后首帧 Modbus 写操作会误触发模式切换 */
    mb_regs[MB_REG_AXIS_TYPE_R] = 1;  /* R轴默认旋转模式 */
    mb_regs[MB_REG_AXIS_TYPE_S] = 1;  /* S轴默认旋转模式 */
    /* U/V/W 默认 0 (直线模式), mb_regs 已由 memset 清零, 无需显式赋值 */
}

/* ── ModBus CRC-16 校验 (多项式 0xA001, 初始值 0xFFFF) ── */
static uint16_t CRC16(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) 
    {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) 
        {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;        
            else              crc >>= 1;
        }
    }
    return crc;
}

/* ── 通过 RS485 发送 ModBus RTU 响应帧 ── */
static void MB_Send(uint8_t *data, uint8_t len)
{
    if (!RS485_Send_Data(data, len)) {
        #if printf_cmd
        static uint32_t mb_send_fail_cnt = 0;
        if ((++mb_send_fail_cnt & 0xFF) == 1)  /* 每256次失败才打印一次, 防刷屏 */
            printf("MB_Send: RS485 TX busy, frame dropped (len=%d)\r\n", len);
        #endif
    }
}

/* ── 发送 ModBus 异常响应帧 (功能码 | 0x80, 异常码) ── */
static void MB_SendError(uint8_t func, uint8_t excode)
{
    uint8_t buf[5];
    buf[0] = MB_SLAVE_ADDR;
    buf[1] = func | 0x80;
    buf[2] = excode;
    uint16_t crc = CRC16(buf, 3);
    buf[3] = crc & 0xFF;
    buf[4] = crc >> 8;
    MB_Send(buf, 5);
}

/* ── FC03: 读保持寄存器 (起始地址 + 数量, 最大125个) ── */
static void Handle_FC03(uint8_t *rx, uint8_t rx_len)
{

    uint16_t start_addr = ((uint16_t)rx[2] << 8) | rx[3];       // 读取寄存器的起始地址
    uint16_t quantity   = ((uint16_t)rx[4] << 8) | rx[5];       // 读取寄存器的数量

    // 一帧数据最多读取125个，如果超过125发送异常码0x03（非法数据值）
    if (quantity < 1 || quantity > 125) { MB_SendError(MB_FC_READ_REGS, 0x03); return; }
    if (start_addr + quantity > MB_REG_COUNT) { MB_SendError(MB_FC_READ_REGS, 0x02); return; }

    // 寄存器是16位的，读取的字节=寄存器*2
    uint8_t byte_count = (uint8_t)(quantity * 2);       
    uint8_t tx[260];
    tx[0] = MB_SLAVE_ADDR;
    tx[1] = MB_FC_READ_REGS;
    tx[2] = byte_count;         

    // 发送上位机需要读取的数据
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t val = mb_regs[start_addr + i];
        tx[3 + i * 2]     = (uint8_t)(val >> 8);
        tx[3 + i * 2 + 1] = (uint8_t)(val & 0xFF);
    }

    uint16_t crc = CRC16(tx, 3 + byte_count);
    tx[3 + byte_count]     = crc & 0xFF;
    tx[3 + byte_count + 1] = crc >> 8;
    MB_Send(tx, 3 + byte_count + 2);
}

/* ── FC06: 写单个保持寄存器 ── */
static void Handle_FC06(uint8_t *rx, uint8_t rx_len)
{
    uint16_t reg_addr = ((uint16_t)rx[2] << 8) | rx[3];         // 写入的寄存器地址
    uint16_t reg_val  = ((uint16_t)rx[4] << 8) | rx[5];         // 写入的值

    if (reg_addr >= MB_REG_COUNT) { MB_SendError(MB_FC_WRITE_REG, 0x02); return; }

    mb_regs[reg_addr] = reg_val;

    uint8_t tx[8];
    memcpy(tx, rx, 6);
    uint16_t crc = CRC16(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = crc >> 8;
    MB_Send(tx, 8);
}

/* ── FC16: 写多个保持寄存器 (起始地址 + 数量 + 字节数 + 数据) ── */
static void Handle_FC16(uint8_t *rx, uint8_t rx_len)
{
    uint16_t start_addr = ((uint16_t)rx[2] << 8) | rx[3];
    uint16_t quantity   = ((uint16_t)rx[4] << 8) | rx[5];
    uint8_t  byte_count = rx[6];

    if (quantity < 1 || quantity > 125) { MB_SendError(MB_FC_WRITE_REGS, 0x03); return; }
    if (start_addr + quantity > MB_REG_COUNT || byte_count != quantity * 2) {
        MB_SendError(MB_FC_WRITE_REGS, 0x02); return;
    }

    for (uint16_t i = 0; i < quantity; i++)
        mb_regs[start_addr + i] = ((uint16_t)rx[7 + i * 2] << 8) | rx[8 + i * 2];

    uint8_t tx[8];
    tx[0] = MB_SLAVE_ADDR; tx[1] = MB_FC_WRITE_REGS;
    tx[2] = rx[2]; tx[3] = rx[3];
    tx[4] = rx[4]; tx[5] = rx[5];
    uint16_t crc = CRC16(tx, 6);
    tx[6] = crc & 0xFF; tx[7] = crc >> 8;
    MB_Send(tx, 8);
}

/* ================================================================
   Modbus_Poll() — MODBUS 帧接收与命令分发
   ================================================================
 * 每控制周期由 EtherCAT 任务调用 (20ms 周期)
 *
 * 【帧检测策略】
 *   中断接收字节 → 存入 mb_rx_buf[] → 开始计时
 *   最后一个字节后 MB_FRAME_TIMEOUT(2ms) 内无新数据 → 帧结束
 *   适用于 115200bps 下 MODBUS RTU 的 3.5 字符间隔规则
 *
 * 【处理流程】
 *   ① 从 RS485 中断缓冲区搬运字节
 *   ② 超时判断 → 帧完整
 *   ③ 校验: 站地址匹配 + 长度≥4 + CRC16 正确
 *   ④ 按功能码分发: FC03/FC06/FC16
 *   ⑤ 写操作后: 参数写同步 → 齿轮比刷新 → 命令触发检测
 */
void Modbus_Poll(void)
{
    RS485_TC_CheckTimeout();  /* 每周期检查 TC 超时 (DMA 模式兜底) */

    /* ① 从 RS485 ISR 缓冲区搬运新收到的字节 */
    if (RS485_RX_CNT > 0) {
        for (uint16_t i = 0; i < RS485_RX_CNT && mb_rx_len < MB_RX_BUF_SIZE; i++)
            mb_rx_buf[mb_rx_len++] = RS485_RX_BUF[i];
        RS485_RX_CNT = 0;
        mb_last_tick = HAL_GetTick();
    }

    /* ② 帧超时判断: 2ms 无新数据 → 一帧接收完成
     * MODBUS RTU 标准: 帧间间隔 ≥ 3.5 字符时间
     * 115200bps 下 3.5 字符 ≈ 0.3ms, 2ms 留有充足余量 */
    if (mb_rx_len > 0 && (HAL_GetTick() - mb_last_tick) >= MB_FRAME_TIMEOUT)
    {
        /* ③ 帧校验: 站地址 + 最小长度 + CRC16 */
        if (mb_rx_buf[0] != MB_SLAVE_ADDR)  { mb_rx_len = 0; return; }
        // 帧最小长度 = 地址(1) + 功能码(1) + CRC(2) = 4 字节
        if (mb_rx_len < 4)                   { mb_rx_len = 0; return; }

        /* CRC16 校验: 计算值与帧尾 CRC 比对 */
        uint16_t rx_crc = ((uint16_t)mb_rx_buf[mb_rx_len - 1] << 8)
                        |  (uint16_t)mb_rx_buf[mb_rx_len - 2];
        uint16_t calc_crc = CRC16(mb_rx_buf, mb_rx_len - 2);
        if (rx_crc != calc_crc) { mb_rx_len = 0; return; }

        /* ④ 按功能码分发处理 */
        uint8_t func = mb_rx_buf[1];
        switch (func) {
            case MB_FC_READ_REGS:  Handle_FC03(mb_rx_buf, mb_rx_len); break;
            case MB_FC_WRITE_REG:  Handle_FC06(mb_rx_buf, mb_rx_len); break;
            case MB_FC_WRITE_REGS: Handle_FC16(mb_rx_buf, mb_rx_len); break;
            default:               MB_SendError(func, 0x01);           break;
        }

        /* ⑤ 写操作后: 参数写同步 (mb_regs → param_ram) */
        if (func == MB_FC_WRITE_REG || func == MB_FC_WRITE_REGS)
        {
            /* ── 参数写同步: mb_regs[132..195] → param_ram[0..63] (联动轴 X/Y/Z/R) ── */
            for (int i = 0; i < MB_REG_PARAM_COUNT; i++)
                Param_Set((ParamId_t)i, (int32)(int16)mb_regs[MB_REG_PARAM_BASE + i]);

            /* ── 辅助轴参数写同步: mb_regs[263..326] → param_ram[64..127] (U/V/W/S) ── */
            for (int i = 0; i < MB_REG_AUX_PARAM_COUNT; i++)
                Param_Set((ParamId_t)(64 + i),
                          (int32)(int16)mb_regs[MB_REG_AUX_PARAM_BASE + i]);

            /* ── 扩展参数写同步: mb_regs[199..207] → param_ram[128..136] (全局参数) ── */
            for (int i = 0; i < MB_REG_EXT_GLOBAL_COUNT; i++)
                Param_Set((ParamId_t)(128 + i),
                          (int32)(int16)mb_regs[MB_REG_EXT_GLOBAL_BASE + i]);

            /* ── 轴参数镜像区写回: mb_regs[224..242] → param_ram[选中轴] ──
             * HMI 参数画面仅绑定一套镜像寄存器 (4x 225~243),
             * STM32 根据 AXIS_SELECT (4x 131) 路由到对应轴的 param_ram。
             * mirror_cur_ax 校验: 防止切换轴选择后旧轴的数据污染新轴参数 */
            {
                int ax = (int)mb_regs[MB_REG_AXIS_SELECT];                      //HMI 画面选中轴索引 (0=X,1=Y,2=Z,3=R)
                if (ax >= 0 && ax < MAX_AXES && ax < ec_slavecount && ax == mirror_cur_ax) 
                {
                    if (ax < 4) 
                    {
                        /* ==== 联动轴 X/Y/Z/R: 13个 Int16 + 3个 Int32 ====
                         * Int16 参数通过 mirror_map[] 路由到对应轴的 param_ram
                         * Int32 参数 (回零偏置/软限位) 占连续2个 MODBUS 寄存器, 需拼接 */
                        for (int i = 0; i < MB_REG_MIRROR_COUNT; i++) 
                        {
                            Param_Set(AXIS_PARAM(mirror_map[i], ax),
                                      (int32)(int16)mb_regs[MB_REG_MIRROR_BASE + i]);
                        }
                        /* Int32 参数镜像写回: HI<<16 | LO */
                        int32 vmx;
                        vmx = ((int32)(int16)mb_regs[MB_REG_MIRROR_HOME_OFF_HI] << 16)
                            | (mb_regs[MB_REG_MIRROR_HOME_OFF_LO] & 0xFFFF);
                        Param_Set(AXIS_PARAM(PARAM_AXIS_HOME_OFFSET, ax), vmx);
                        vmx = ((int32)(int16)mb_regs[MB_REG_MIRROR_SOFT_PLUS_HI] << 16)
                            | (mb_regs[MB_REG_MIRROR_SOFT_PLUS_LO] & 0xFFFF);
                        Param_Set(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, ax), vmx);
                        vmx = ((int32)(int16)mb_regs[MB_REG_MIRROR_SOFT_MINUS_HI] << 16)
                            | (mb_regs[MB_REG_MIRROR_SOFT_MINUS_LO] & 0xFFFF);
                        Param_Set(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, ax), vmx);
                    }
                    else 
                    {
                        /* ==== 辅助轴: 仅写回通用 Int16 参数, 根据轴类型路由 ==== */
                        for (int i = 0; i < MB_REG_MIRROR_COUNT; i++) 
                        {
                            ParamId_t pid = (ax < 4) ? AXIS_PARAM(mirror_map[i], ax): (ParamId_t)0;
                            Param_Set(pid, (int32)(int16)mb_regs[MB_REG_MIRROR_BASE + i]);
                        }
                    }
                }
            }

            /* ── 轴类型模式同步 (在 RefreshAxisGearRatio 之前) ── */
            static uint8_t prev_axis_type[MAX_AXES];
            static uint8_t prev_axis_inited = 0;
            if (!prev_axis_inited) {
                /* 首次调用: 从实际轴状态同步, 避免硬编码默认值造成误触发 */
                for (int ax = 0; ax < MAX_AXES; ax++)
                    prev_axis_type[ax] = axis[ax].axis_type;
                prev_axis_inited = 1;
            }
            for (int ax = 3; ax < MAX_AXES; ax++) {
                if (ax + 1 > ec_slavecount) continue;  /* 从站不存在, 跳过 */
                uint8_t new_type = (uint8_t)(mb_regs[MB_REG_AXIS_TYPE_R + (ax - 3)] & 0x01);
                if (new_type != prev_axis_type[ax]) {
                    /* 运动中禁止切换模式: actual_pos 持续变化, 单位切换导致跳变/飞车 */
                    if (axis[ax].motion_busy || axis[ax].homing) {
                        mb_regs[MB_REG_AXIS_TYPE_R + (ax - 3)] = prev_axis_type[ax];  /* 回写旧值, 拒绝切换 */
                        continue;
                    }
                    /* ① 更新 MCU 侧轴类型 + 齿轮比字段 */
                    axis[ax].axis_type = new_type;
                    Axis_UpdateModeGearRatio(ax, new_type);
                    /* ② SDO 写入伺服: 失败则回滚所有状态 (MCU + 伺服) */
                    if (!Servo_WriteGearRatio(axis[ax].slave_idx, ax)) {
                        /* ── MCU 侧回滚 ── */
                        axis[ax].axis_type = prev_axis_type[ax];           /* 恢复旧类型 */
                        Axis_UpdateModeGearRatio(ax, prev_axis_type[ax]);  /* 恢复旧齿轮比字段 */
                        RefreshAxisGearRatio();  /* 恢复 effective_ppr 到旧值 */
                        mb_regs[MB_REG_AXIS_TYPE_R + (ax - 3)] = prev_axis_type[ax]; /* 回写 HMI */
                        /* ── 伺服侧恢复: 防止部分写入导致的中间状态 ── */
                        Servo_WriteGearRatio(axis[ax].slave_idx, ax);
                        continue;  /* 跳过软限位更新 */
                    }
                    /* ③ SDO 成功 → 记录新类型, 更新软限位 */
                    prev_axis_type[ax] = new_type;
                    if (new_type == 1) {
                        param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, ax)] = PULSES_PER_REV_ROTARY - 1;
                        param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, ax)] = 0;
                    } else {
                        param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, ax)] = 30000000;
                        param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, ax)] = 0;
                    }
                }
            }
            /* 模式变化 → 重算 effective_ppr */
            RefreshAxisGearRatio();

            CmdExec_CheckTriggers();
        }

        mb_rx_len = 0;
    }
}

/* ================================================================
   Modbus_UpdateRegs() — 每控制周期更新只读寄存器区
   ================================================================
 * 由 EtherCAT 任务每 20ms 调用一次, 将轴状态刷新到 mb_regs[] 供 HMI 读取
 *
 * 【更新内容】
 *   A. 联动轴 X/Y/Z/R 状态 (位置/RPM/StatusWord/ErrorCode/轴状态)
 *   B. 辅助轴 U/V/W/S 状态 (同上, 对称布局)
 *   C. MCS mm/deg 换算 (脉冲 × 导程 → IEEE 754 Float, 联动轴 + 辅助轴)
 *   D. 系统标志汇总 (故障/就绪/抱闸/回零/报警计数)
 *   E. 参数区同步 (param_ram → mb_regs, 每 200ms)
 *   F. 画面2 映射 (选中轴数据拷贝)
 *
 * 【调用频率】
 *   每 4 个 EtherCAT 周期 (20ms), 与 HMI 刷新率 10Hz 匹配
 *   参数区同步每 200ms (避免 MODBUS 读取时数据闪烁)
 */
void Modbus_UpdateRegs(void)
{
    int32 pos;
    int32 vel;
    static uint32_t last_uptime_tick = 0;

    extern int dorun;

    if (dorun == 0) 
    {
        /* 初始化期间仍上报已捕获的编码器位置，避免 HMI 画面显示空白/0 */
        for (int ax = 0; ax < LINKED_AXES_MAX; ax++) {
            if (ax + 1 > ec_slavecount) continue;
            pos = axis[ax].actual_pos;
            mb_regs[ax * 2]     = (uint16)(pos & 0xFFFF);
            mb_regs[ax * 2 + 1] = (uint16)((pos >> 16) & 0xFFFF);
        }
        mb_regs[MB_REG_SLAVE_COUNT] = (uint16)ec_slavecount;
        mb_regs[MB_REG_FW_VERSION]  = FW_VERSION;
        mb_regs[MB_REG_SYS_STATE]   = 0;   /* 0 = 初始化中 */
        return;
    }

    {
        uint32_t now = HAL_GetTick();
        if (now - last_uptime_tick >= 1000) {
            last_uptime_tick += 1000;
            g_uptime_seconds++;
        }
    }

    /* ③ 联动轴 X/Y/Z/R 只读状态 (0~19)
     * 每轴 5 个寄存器: 位置(2) + RPM(1) + StatusWord(1) + ErrorCode(1) */
    for (int ax = 0; ax < LINKED_AXES_MAX; ax++)
    {
        if (ax + 1 > ec_slavecount) continue;

        /* 编码器位置: Int32 → 2 个连续 MODBUS 寄存器 (LO 在前) */
        pos = axis[ax].actual_pos;
        mb_regs[ax * 2]     = (uint16)(pos & 0xFFFF);
        mb_regs[ax * 2 + 1] = (uint16)((pos >> 16) & 0xFFFF);

        /* 当前转速: 脉冲/周期 → RPM */
        vel = axis[ax].velocity;
        mb_regs[MB_REG_X_RPM + ax] = (uint16)(int16)(CycleVel_to_RPM(vel, ax));

        /* CiA 402 StatusWord (0x6041) — HMI 拆位显示 */
        mb_regs[MB_REG_X_STATUS_WORD + ax] = axis[ax].status_word;

        /* 伺服驱动器错误码 (0x603F) */
        mb_regs[MB_REG_X_ERRCODE + ax] = axis[ax].pdo_in->ErrorCode;

        /* 轴状态解析: 0=离线 1=就绪 2=运动中 3=故障
         * HMI 根据此值切换轴状态指示灯颜色 */
        if (ax + 1 > ec_slavecount) {
            mb_regs[MB_REG_AXIS_STATE_X + ax] = 0;  /* 从站不存在 → 离线 */
        } else if (!axis[ax].enabled) {
            mb_regs[MB_REG_AXIS_STATE_X + ax] = 0;  /* 未使能 → 离线 */
        } else if (axis[ax].status_word & 0x0008) {
            mb_regs[MB_REG_AXIS_STATE_X + ax] = 3;  /* bit3=1 → 故障 */
        } else if (axis[ax].motion_busy || axis[ax].velocity != 0) {
            mb_regs[MB_REG_AXIS_STATE_X + ax] = 2;  /* 运动标志置位 → 运动中 */
        } else {
            mb_regs[MB_REG_AXIS_STATE_X + ax] = 1;  /* 使能且静止 → 就绪 */
        }
    }

    /* ══════════════════════════════════════════════════════════════
     * B. 辅助轴 U/V/W/S 只读状态 — PDU 50~81
     *    每轴 8 个寄存器: 位置(2) + RPM(1) + SW(1) + ErrorCode(1) + 预留(3)
     *    基线偏移 = MB_REG_POS_LO_U + (ax-4) × 8
     * ══════════════════════════════════════════════════════════════ */
    for (int ax = 4; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;

        int aux = ax - 4;                            // 辅助轴编号: U=0, V=1, W=2, S=3
        int base = MB_REG_POS_LO_U + aux * 8;        // 该轴 8 个寄存器的起始地址（含 3 个预留）

        /* 编码器位置 */
        pos = axis[ax].actual_pos;
        mb_regs[base]     = (uint16)(pos & 0xFFFF);
        mb_regs[base + 1] = (uint16)((pos >> 16) & 0xFFFF);

        /* RPM */
        vel = axis[ax].velocity;                                            //每个控制周期的脉冲增量
        mb_regs[base + 2] = (uint16)(int16)(CycleVel_to_RPM(vel, ax));        

        /* StatusWord + ErrorCode */
        mb_regs[base + 3] = axis[ax].status_word;
        mb_regs[base + 4] = axis[ax].pdo_in->ErrorCode;

        /* 轴状态 */
        if (!axis[ax].enabled) {
            mb_regs[MB_REG_AXIS_STATE_U + aux] = 0;
        } else if (axis[ax].status_word & 0x0008) {
            mb_regs[MB_REG_AXIS_STATE_U + aux] = 3;
        } else if (axis[ax].motion_busy || axis[ax].velocity != 0) {
            mb_regs[MB_REG_AXIS_STATE_U + aux] = 2;
        } else {
            mb_regs[MB_REG_AXIS_STATE_U + aux] = 1;
        }
    }

    /* ══════════════════════════════════════════════════════════════
     * C. 联动轴 MCS 位置 mm/deg 换算 — PDU 38~45
     *    脉冲 × 导程(um) / (1000 × eppr) → mm (IEEE 754 Float)
     *    R轴: 脉冲 × 360° / eppr → deg, 取模 [0, 360)
     * ══════════════════════════════════════════════════════════════ */
    {
        union { float f; uint32_t u32; uint16_t u16[2]; } fc;
        for (int ax = 0; ax < LINKED_AXES_MAX; ax++) 
        {
            int32 pulses = (ax < ec_slavecount) ? axis[ax].actual_pos : 0;              //当前编码器脉冲数
            int32 eppr = (ax < ec_slavecount && axis[ax].effective_ppr > 0)             //有效每圈脉冲数（=编码器分辨率÷电子齿轮比）
            ? axis[ax].effective_ppr : GetAxisPPR(ax);
            if (axis[ax].axis_type == 1)
            {
                /* 旋转模式: 脉冲 → 角度 deg */
                if (ax < ec_slavecount && eppr > 0) {
                    fc.f = (float)pulses * 360.0f / (float)eppr;
                } else
                    fc.f = 0.0f;
                while (fc.f >= 360.0f) fc.f -= 360.0f;
                while (fc.f < 0.0f) fc.f += 360.0f;
            }
            else
            {
                /* 直线模式: 脉冲 → mm */
                int32 pitch = Param_Get(AXIS_PARAM(PARAM_AXIS_PITCH, ax));              //丝杠导程 (um)
                if (pitch <= 0) pitch = 5000;                                           //默认导程 5mm
                fc.f = (float)pulses * (float)pitch / (1000.0f * (float)eppr);
            }
            mb_regs[MB_REG_MCS_X_MM_LO + ax * 2]     = (uint16_t)(fc.u32 & 0xFFFF);
            mb_regs[MB_REG_MCS_X_MM_LO + ax * 2 + 1] = (uint16_t)(fc.u32 >> 16);
        }
    }

    /* axis_type 上报: R/U/V/W/S (每周期更新, 不放在200ms参数同步区) */
    for (int ax = 3; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        mb_regs[MB_REG_AXIS_TYPE_R + (ax - 3)] = axis[ax].axis_type;
    }

    /* ══════════════════════════════════════════════════════════════
     * D. 辅助轴 MCS mm/deg 换算 — PDU 82~89
     *    U/V/W: 脉冲 → mm, S轴: 脉冲 → deg, 与 C 段公式一致
     * ══════════════════════════════════════════════════════════════ */
    {
        union { float f; uint32_t u32; uint16_t u16[2]; } fc;
        for (int ax = 4; ax < MAX_AXES; ax++) {
            int aux = ax - 4;
            int32 pulses = (ax < ec_slavecount) ? axis[ax].actual_pos : 0;
            int32 eppr = (ax < ec_slavecount && axis[ax].effective_ppr > 0)
                       ? axis[ax].effective_ppr : GetAxisPPR(ax);
            if (axis[ax].axis_type == 1) {
                /* 旋转模式: 脉冲 → 角度 deg */
                if (ax < ec_slavecount && eppr > 0) {
                    fc.f = (float)pulses * 360.0f / (float)eppr;
                } else
                    fc.f = 0.0f;
                while (fc.f >= 360.0f) fc.f -= 360.0f;
                while (fc.f < 0.0f) fc.f += 360.0f;
            } else {
                /* 直线模式: 脉冲 → mm */
                int32 pitch = Param_Get(AXIS_PARAM(PARAM_AXIS_PITCH, ax));
                if (pitch <= 0) pitch = 5000;
                fc.f = (float)pulses * (float)pitch / (1000.0f * (float)eppr);
            }
            mb_regs[MB_REG_MCS_U_MM_LO + aux * 2]     = (uint16_t)(fc.u32 & 0xFFFF);
            mb_regs[MB_REG_MCS_U_MM_LO + aux * 2 + 1] = (uint16_t)(fc.u32 >> 16);
        }
    }
    
    /* ══════════════════════════════════════════════════════════════
     * E. 系统标志汇总 + 故障检测
     *    SYS_FLAGS: bit0=故障 bit1=全部就绪 bit2=抱闸 bit3=回零中
     *    SYS_ALARM_COUNT: 活跃报警轴数
     *    AUX_FLAGS: bit0=U故障 bit1=V故障 bit2=W故障 bit3=S故障
     * ══════════════════════════════════════════════════════════════ */
    {
        uint16 flags = 0;                                                // 系统标志位（位组合）
        uint16 alarm_count = 0;                                         // 故障轴数量
        uint8 all_ready = 1;                                            // 是否全部就绪（先假设是）   
        uint16 aux_flags = 0;                                           // 辅助轴故障标志

        for (int ax = 0; ax < MAX_AXES; ax++) {
            if (ax + 1 > ec_slavecount) { if (ax < 4) all_ready = 0; continue; }
            if (axis[ax].status_word & 0x0008) {
                if (ax < 4) flags |= 0x0001;
                alarm_count++;
                if (ax >= 4) aux_flags |= (uint16)(1 << (ax - 4));
            }
            if ((axis[ax].status_word & 0x0001) == 0) all_ready = 0;
        }
        if (all_ready) flags |= 0x0002;
        if ((axis[2].direction != 0)) flags |= 0x0004;  /* 抱闸 */
        if (homing_active) flags |= 0x0008;              /* 回零中 */
        mb_regs[MB_REG_SYS_FLAGS] = flags;
        mb_regs[MB_REG_SYS_ALARM_COUNT] = alarm_count;
        mb_regs[MB_REG_SYS_FAULT] = (flags & 0x0001) ? 1 : 0;
        mb_regs[MB_REG_SYS_ALL_READY] = (flags & 0x0002) ? 1 : 0;
        mb_regs[MB_REG_SYS_AUX_FLAGS] = aux_flags;
    }

    mb_regs[MB_REG_SYS_STATE] = 2;
    if (dorun == 0) mb_regs[MB_REG_SYS_STATE] = 1;

    {
        uint16 fault = 0;
        for (int ax = 0; ax < MAX_AXES; ax++) {
            if (ax + 1 > ec_slavecount) continue;
            if (axis[ax].status_word & 0x0008) {
                fault = axis[ax].status_word; break;
            }
        }
        mb_regs[MB_REG_FAULT_CODE] = fault;
    }

    /* 抱闸状态 */
    mb_regs[MB_REG_BRAKE_STATUS] = (axis[2].direction != 0) ? 1 : 0;

    mb_regs[MB_REG_HOMING_ACTIVE] = homing_active ? 1 : 0;

    mb_regs[MB_REG_SLAVE_COUNT] = (uint16)ec_slavecount;

    mb_regs[MB_REG_MOVE_SPEED_RD] = (uint16)g_move_rpm;

    mb_regs[MB_REG_FW_VERSION] = FW_VERSION;

    mb_regs[MB_REG_UPTIME_LO] = (uint16)(g_uptime_seconds & 0xFFFF);
    mb_regs[MB_REG_UPTIME_HI] = (uint16)(g_uptime_seconds >> 16);

    /* ══════════════════════════════════════════════════════════════
     * F. 参数区同步: param_ram → mb_regs (每 200ms)
     *    包括联动轴参数 (132~195)、辅助轴参数 (263~326)、
     *    扩展全局参数 (199~207)、轴参数镜像区 (224~242)
     *    降频到 200ms 避免 HMI MODBUS 读取时数据闪烁
     * ══════════════════════════════════════════════════════════════ */
    {
        static uint16 param_sync_counter = 0;
        param_sync_counter++;
        if (param_sync_counter >= 10) {  /* 每200ms同步一次 */
            param_sync_counter = 0;
            /* 联动轴参数同步: param_ram[0..63] → mb_regs[132..195] */
            for (int i = 0; i < MB_REG_PARAM_COUNT; i++)
                mb_regs[MB_REG_PARAM_BASE + i] = (uint16)(int16)Param_Get((ParamId_t)i);

            /* 辅助轴参数同步: param_ram[64..127] → mb_regs[263..326] */
            for (int i = 0; i < MB_REG_AUX_PARAM_COUNT; i++)
                mb_regs[MB_REG_AUX_PARAM_BASE + i] = (uint16)(int16)Param_Get((ParamId_t)(64 + i));

            /* 扩展参数同步: param_ram[128..136] → mb_regs[199..207] */
            for (int i = 0; i < MB_REG_EXT_GLOBAL_COUNT; i++)
                mb_regs[MB_REG_EXT_GLOBAL_BASE + i] = (uint16)(int16)Param_Get((ParamId_t)(128 + i));

            /* ── 轴参数镜像区同步: 选中轴 param_ram → mb_regs[224..242] ──
             * HMI 画面读取 AXIS_SELECT 后, STM32 将对应轴的参数填充到镜像区
             * ax<4: 使用 mirror_map 路由 (联动轴共享参数布局)
             * ax≥4: 清空镜像区 (辅助轴参数通过自己的参数区 PDU 263~326 读取) */
            {
                int ax = (int)mb_regs[MB_REG_AXIS_SELECT];
                if (ax >= 0 && ax < MAX_AXES && ax < ec_slavecount) 
                {
                    if (ax < 4) 
                    {
                        /* 联动轴: 使用 mirror_map */
                        for (int i = 0; i < MB_REG_MIRROR_COUNT; i++) {
                            mb_regs[MB_REG_MIRROR_BASE + i] = (uint16)(int16)
                                Param_Get(AXIS_PARAM(mirror_map[i], ax));
                        }
                        /* Int32 参数镜像 */
                        int32 vmx;
                        vmx = Param_Get(AXIS_PARAM(PARAM_AXIS_HOME_OFFSET, ax));
                        mb_regs[MB_REG_MIRROR_HOME_OFF_LO] = (uint16)(vmx & 0xFFFF);
                        mb_regs[MB_REG_MIRROR_HOME_OFF_HI] = (uint16)((vmx >> 16) & 0xFFFF);
                        vmx = Param_Get(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, ax));
                        mb_regs[MB_REG_MIRROR_SOFT_PLUS_LO] = (uint16)(vmx & 0xFFFF);
                        mb_regs[MB_REG_MIRROR_SOFT_PLUS_HI] = (uint16)((vmx >> 16) & 0xFFFF);
                        vmx = Param_Get(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, ax));
                        mb_regs[MB_REG_MIRROR_SOFT_MINUS_LO] = (uint16)(vmx & 0xFFFF);
                        mb_regs[MB_REG_MIRROR_SOFT_MINUS_HI] = (uint16)((vmx >> 16) & 0xFFFF);
                    } 
                    else 
                    {
                        /* 辅助轴: 镜像区清零 (参数布局不同, HMI通过辅助轴参数区读取) */
                        for (int i = 0; i < MB_REG_MIRROR_COUNT; i++)
                            mb_regs[MB_REG_MIRROR_BASE + i] = 0;
                        mb_regs[MB_REG_MIRROR_HOME_OFF_LO] = 0;
                        mb_regs[MB_REG_MIRROR_HOME_OFF_HI] = 0;
                        mb_regs[MB_REG_MIRROR_SOFT_PLUS_LO] = 0;
                        mb_regs[MB_REG_MIRROR_SOFT_PLUS_HI] = 0;
                        mb_regs[MB_REG_MIRROR_SOFT_MINUS_LO] = 0;
                        mb_regs[MB_REG_MIRROR_SOFT_MINUS_HI] = 0;
                    }
                    mirror_cur_ax = (int8_t)ax;
                } 
                else 
                {
                    /* 轴选择无效或离线 → 清零镜像区 */
                    for (int i = 0; i < MB_REG_MIRROR_COUNT; i++)
                        mb_regs[MB_REG_MIRROR_BASE + i] = 0;
                    mb_regs[MB_REG_MIRROR_HOME_OFF_LO] = 0;
                    mb_regs[MB_REG_MIRROR_HOME_OFF_HI] = 0;
                    mb_regs[MB_REG_MIRROR_SOFT_PLUS_LO] = 0;
                    mb_regs[MB_REG_MIRROR_SOFT_PLUS_HI] = 0;
                    mb_regs[MB_REG_MIRROR_SOFT_MINUS_LO] = 0;
                    mb_regs[MB_REG_MIRROR_SOFT_MINUS_HI] = 0;
                    mirror_cur_ax = -1;
                }
            }
        }
    }

    /* ══════════════════════════════════════════════════════════════
     * G. PDU 29~31
     *    根据 AXIS_SELECT 自动拷贝选中轴数据, HMI 调试画面无需切换变量
     * ══════════════════════════════════════════════════════════════ */
    {
        uint16 sel = mb_regs[MB_REG_AXIS_SELECT];
        int ax = (sel < (uint16)NUM_AXES) ? (int)sel : 0;
        if (ax + 1 <= (int)ec_slavecount) {
            int32 p = axis[ax].actual_pos;
            mb_regs[MB_REG_DISP_POS_LO] = (uint16)(p & 0xFFFF);
            mb_regs[MB_REG_DISP_POS_HI] = (uint16)((p >> 16) & 0xFFFF);
            mb_regs[MB_REG_DISP_RPM]     = (uint16)(int16)(CycleVel_to_RPM(axis[ax].velocity, ax));
        }
    }
}

/* ================================================================
   CmdExec_CheckTriggers() — MODBUS 命令触发检测与执行
   ================================================================
 * 在 Modbus_Poll() 写操作完成后调用, 检测触发寄存器并执行对应动作
 *
 * 【触发-清除机制】
 *   HMI 写触发寄存器 = 1 → STM32 检测上升沿 → 执行动作 → 清除触发寄存器
 *   写操作寄存器 (FC06/FC16): 直接写入生效, 无需触发
 *
 * 【支持的命令类型】
 *   ① 全局参数更新: 定位速度变更
 *   ② 速度命令: SPEED_TRIG → 单轴/多轴连续旋转
 *   ③ 定位命令: POS_TRIG → 单轴绝对定位 (支持 mm 目标)
 *   ④ 圈数命令: REVS_TRIG → 按编码器圈数定位
 *   ⑤ 系统命令: SYS_TRIG → STOP/RESET/ZERO 回零
 *   ⑥ 多轴插补: MULTI_TRIG → 直线/圆弧插补
 *   ⑦ 点动命令: JOG_TRIG → 连续速度点动/步进点动 (含上升沿/下降沿状态机)
 *   ⑧ 轴配置: CFG_APPLY → 动态修改激活轴数量 (需停机)
 */
void CmdExec_CheckTriggers(void)
{
    int32 pos;
    int rpm, sign;

    extern int   g_move_rpm;        // 定位速度
    extern int   g_home_rpm;
    extern uint8 homing_active;
    extern uint8 fault_reset_requested;
extern uint8_t g_active_axes;
extern uint8_t g_active_coupled;
extern uint8_t g_active_aux;

    /* 参数更新 */
    {
        int new_speed = (int)mb_regs[MB_REG_POS_SPEED];
        if (new_speed >= 1 && new_speed <= 3000 && new_speed != g_move_rpm) {
            g_move_rpm = new_speed;
            for (int ax = 0; ax < NUM_AXES; ax++)
                if (!axis[ax].homing) Axis_SetMoveSpeed(ax, g_move_rpm);            // 更新全局定位速度
        }
    }

    /* ── ② 速度命令: 单轴/多轴连续旋转 ──
     * SPEED_RPM=0 → 停止, SPEED_RPM≠0 → 按指定方向旋转
     * SPEED_AXIS: 0~7=指定轴, 8=全部轴 */
    if (mb_regs[MB_REG_SPEED_TRIG] == 1)
    {
        rpm = (int16)mb_regs[MB_REG_SPEED_RPM];                       // 目标转速（有符号）     
        uint16 axis_sel = mb_regs[MB_REG_SPEED_AXIS];                 // 选哪个轴          
        int ax_s = 0, ax_e = NUM_AXES - 1;
        if (axis_sel < (uint16)NUM_AXES) { ax_s = axis_sel; ax_e = axis_sel; }

        if (rpm == 0) {
            for (int ax = ax_s; ax <= ax_e; ax++)
                if (ax + 1 <= ec_slavecount) Axis_StopOne(ax);
        } else {
            sign = (rpm > 0) ? 1 : -1;
            int abs_rpm = (rpm > 0) ? rpm : -rpm;
            if (abs_rpm > 3000) abs_rpm = 3000;
            for (int ax = ax_s; ax <= ax_e; ax++)
                if (ax + 1 <= ec_slavecount) Axis_ApplyVelocity(ax, abs_rpm, sign);
        }
        mb_regs[MB_REG_SPEED_TRIG] = 0;
    }

    /* ── ③ 定位命令: 单轴绝对定位 ──
     * 优先使用 mm 目标 (Float), 自动换算为脉冲
     * X/Y/Z: mm→脉冲 (导程×eppr), R轴: deg→脉冲 (最短路径旋转) */
    if (mb_regs[MB_REG_POS_TRIG] == 1)
    {
        uint16 axis_sel = mb_regs[MB_REG_POS_AXIS];
        int ax_s = 0, ax_e = NUM_AXES - 1;
        if (axis_sel < (uint16)NUM_AXES) { ax_s = axis_sel; ax_e = axis_sel; }

        for (int ax = ax_s; ax <= ax_e; ax++) {
            if (ax + 1 <= ec_slavecount) {
                /* 优先读取 mm 目标 (Float), 自动换算脉冲 */
                union { float f; uint32_t u32; uint16_t u16[2]; } fc;
                fc.u16[0] = mb_regs[MB_REG_POS_TARGET_MM_LO];
                fc.u16[1] = mb_regs[MB_REG_POS_TARGET_MM_HI];
                float target = fc.f;  /* X/Y/Z=mm, R=deg */

                if (axis[ax].axis_type == 1) {
                    /* 旋转模式: 度→脉冲, 使用 eppr 统一坐标空间 */
                    int32 eppr = axis[ax].effective_ppr;
                    int32 cur  = axis[ax].actual_pos;
                    int32 deg_enc = (int32)(target * eppr / 360.0f);          // 目标角度对应的脉冲  
                    while (deg_enc >= eppr) deg_enc -= eppr;
                    while (deg_enc < 0) deg_enc += eppr;
                    int32 rev = (cur / eppr) * eppr;                    // 当前圈数对应的脉冲 (整圈)
                    int32 t1 = rev + deg_enc;                           // 最近的脉冲位置
                    int32 t2 = t1 + eppr;                               
                    int32 t3 = t1 - eppr;
                    int32 d1 = t1 - cur; if (d1 < 0) d1 = -d1;
                    int32 d2 = t2 - cur; if (d2 < 0) d2 = -d2;
                    int32 d3 = t3 - cur; if (d3 < 0) d3 = -d3;
                    int32 best_enc = t1;                                
                    if (d2 < d1) { best_enc = t2; d1 = d2; }            // 选择最小增量
                    if (d3 < d1) { best_enc = t3; }                     
                    pos = best_enc;
                } else {
                    /* X/Y/Z: mm → 脉冲 */
                    int32 pitch = Param_Get(AXIS_PARAM(PARAM_AXIS_PITCH, ax));
                    if (pitch <= 0) pitch = 5000;
                    int32 eppr = axis[ax].effective_ppr;
                    if (eppr <= 0) eppr = GetAxisPPR(ax);
                    pos = (int32)(target * (float)eppr * 1000.0f / (float)pitch);
                }
                /* 回写脉冲值到 HMI 脉冲框 */
                mb_regs[MB_REG_POS_PULSES_LO] = (uint16)(pos & 0xFFFF);
                mb_regs[MB_REG_POS_PULSES_HI] = (uint16)((pos >> 16) & 0xFFFF);

                int32 speed = (int16)mb_regs[MB_REG_EXT_DEFAULT_VEL];
                if (speed <= 0) speed = 200;
                g_move_rpm = speed;
                Axis_ApplyPosition(ax, pos);
            }
        }
        mb_regs[MB_REG_POS_TRIG] = 0;
    }

    /* ── ④ 圈数命令: 按编码器圈数增量定位 ──
     * pos = actual_pos + revs × effective_ppr (带符号) */
    if (mb_regs[MB_REG_REVS_TRIG] == 1)
    {
        int16 revs = (int16)mb_regs[MB_REG_REVS];
        uint16 axis_sel = mb_regs[MB_REG_REVS_AXIS];
        int ax_s = 0, ax_e = NUM_AXES - 1;
        if (axis_sel < (uint16)NUM_AXES) { ax_s = axis_sel; ax_e = axis_sel; }

        for (int ax = ax_s; ax <= ax_e; ax++) 
        {
            if (ax + 1 <= ec_slavecount) 
            {
                pos = (int32)((int64)axis[ax].actual_pos + (int64)revs * (int64)axis[ax].effective_ppr);
                Axis_ApplyPosition(ax, pos);
            }
        }
        mb_regs[MB_REG_REVS_TRIG] = 0;
    }

    /* ── ⑤ 系统命令: STOP(1) / RESET(2) / ZERO 回零(3) ──
     * STOP: 停止指定轴, 同时清除点动
     * RESET: CiA 402 故障复位 (ControlWord=0x0080)
     * ZERO: 两段式回零 (快速逼近→蠕动精停) */
    if (mb_regs[MB_REG_SYS_TRIG] == 1)
    {
        uint16 cmd  = mb_regs[MB_REG_SYS_CMD];
        uint16 axis_sel = mb_regs[MB_REG_SYS_AXIS];
        int ax_s = 0, ax_e = NUM_AXES - 1;
        if (axis_sel < (uint16)NUM_AXES) { ax_s = axis_sel; ax_e = axis_sel; }

        if (cmd == 1) 
        {             // 停止
            for (int ax = ax_s; ax <= ax_e; ax++)
                if (ax + 1 <= ec_slavecount) Axis_StopOne(ax);
            mb_regs[MB_REG_JOG_TRIG]   = 0;
            mb_regs[MB_REG_JOG_STATUS] = 0;
        }
        else if (cmd == 2) {        // 复位
            int need_reset = 0;
            for (int ax = ax_s; ax <= ax_e; ax++) {
                if (ax + 1 <= ec_slavecount) {
                    if (axis[ax].pdo_in->StatusWord & 0x0008) 
                    {
                        axis[ax].pdo_out->ControlWord = 0x0080;         // CiA 402 故障复位命令
                        axis[ax].fault_reset_step = 1;
                        need_reset = 1;
                    } 
                    else if (!axis[ax].enabled)         
                    {
                        axis[ax].fault_reset_step = 3;              // 尝试重新使能
                        need_reset = 1;
                    }
                }
            }
            if (need_reset) fault_reset_requested = 1;
        }
        else if (cmd == 3) 
        {                    // 回零
            /* ZERO: 两段式回零 */
            int32 home_rpm = (int16)mb_regs[MB_REG_EXT_HOMING_VEL];
            if (home_rpm <= 0) home_rpm = 200;
            g_home_rpm = home_rpm;
            int32 creep_rpm = home_rpm / 5;                 // 慢段速度 = 快段的 1/5
            if (creep_rpm < 5) creep_rpm = 5;               // 最慢不低于 5 RPM
            #if printf_cmd
            printf(">>> ZERO: fast=%d RPM, creep=%d RPM <<<\r\n",
                   (int)home_rpm, (int)creep_rpm);
            #endif
            for (int ax = ax_s; ax <= ax_e; ax++) {
                if (ax + 1 <= ec_slavecount) {
                    Axis_CancelHoming(ax);
                    int32 cur = axis[ax].actual_pos;
                    int32 creep_dist = (int32)((int64)RPM_to_CycleVel(creep_rpm, ax) * 60);       // 蠕动段距离 = 蠕动速度 × 60s
                    int32 approach = (cur > 0) ? creep_dist : -creep_dist;                      // 快段目标位置 = 当前脉冲 ± 蠕动段距离
                    int32 dist = (cur > 0) ? cur : -cur;                                        // 当前脉冲绝对值
                    if (dist <= creep_dist * 2) {
                        approach = 0;
                        creep_rpm = home_rpm / 10;
                        if (creep_rpm < 2) creep_rpm = 2;
                    }
                    axis[ax].mode = AXIS_MODE_POS;                                              // 切换定位模式
                    axis[ax].cmd_target_pos = approach;                                         // 目标位置 = approach 点
                    axis[ax].homing = 1;                                                        
                    axis[ax].homing_phase = (approach != 0) ? 1 : 2;
                    axis[ax].homing_creep_rpm = creep_rpm;
                    Axis_SetMoveSpeed(ax, (approach != 0) ? home_rpm : creep_rpm);                  
                    {
                        int32 max_vel = axis[ax].pos_step;
                        int32 acc_cap = Param_Get(AXIS_PARAM(PARAM_AXIS_MAX_ACCEL, ax));
                        int32 max_acc, jerk;
                        SCurve_AutoAccJerk(max_vel, acc_cap, &max_acc, &jerk);                  // 自动算加速度和Jerk         
                        SCurve_Plan(&axis[ax].splanner, cur, approach,                             
                                    max_vel, max_acc, jerk);
                        axis[ax].target_pos = cur;
                        Axis_WriteTargetPos(ax, cur);
                    }
                }
            }
            homing_active = 1;
        }
        mb_regs[MB_REG_SYS_TRIG] = 0;
    }

    /* ── ⑥ 多轴插补命令: 直线(1)/顺圆CW(2)/逆圆CCW(3) ──
     * 先同步 mm/deg 目标 → 脉冲 (HMI 可能在同一帧内写入目标和触发)
     * 再读取脉冲目标 → Interp_Line / Interp_ArcXY */
    if (mb_regs[MB_REG_MULTI_TRIG] == 1)
    {
        uint16 cmd = mb_regs[MB_REG_MULTI_CMD];

        /* 【mm/deg → 脉冲同步】
         * HMI 在同一 MODBUS 写周期内可能先写入插补目标 mm 值,
         * 再写入 MULTI_TRIG=1。此段检测 mm 值与脉冲值是否一致,
         * 不一致则自动用 mm 换算覆盖脉冲, 确保 HMI 无需分两帧操作 */
        {
            union { float f; uint32_t u32; uint16_t u16[2]; } fc;
            for (int ax = 0; ax < MAX_AXES; ax++) {
                if (ax >= g_active_axes) break;

                uint16 pulse_lo, mm_lo;
                if (ax < 4) {
                    pulse_lo = (uint16)(MB_REG_MULTI_POS_X_LO + ax * 2);
                    mm_lo    = (uint16)(MB_REG_INTERP_X_MM_LO  + ax * 2);
                } else {
                    pulse_lo = (uint16)(MB_REG_MULTI_POS_U_LO + (ax - 4) * 2);
                    mm_lo    = (uint16)(MB_REG_INTERP_U_MM_LO  + (ax - 4) * 2);
                }
                if (ax + 1 > ec_slavecount) continue;

                fc.u16[0] = mb_regs[mm_lo];
                fc.u16[1] = mb_regs[mm_lo + 1];

                int32 cur_p = (int32)(((uint32)mb_regs[pulse_lo + 1] << 16) | mb_regs[pulse_lo]);   // 当前脉冲值
                int32 eppr = (axis[ax].effective_ppr > 0)
                           ? axis[ax].effective_ppr : GetAxisPPR(ax);                        // 当前轴有效脉冲数
                int32 expected_p;                                                                   // 期望脉冲值 (由 mm/deg 换算而来)
                if (axis[ax].axis_type == 1) {
                    /* 旋转模式: deg → 脉冲 */
                    expected_p = (int32)(fc.f * (float)eppr / 360.0f);
                } 
                else
                {
                    /* 直线轴: mm → 脉冲 */
                    int32 pitch = Param_Get(AXIS_PARAM(PARAM_AXIS_PITCH, ax));
                    if (pitch <= 0) pitch = 5000;
                    expected_p = (int32)(fc.f * (float)eppr * 1000.0f / (float)pitch);
                }
                {
                    int32 diff = expected_p - cur_p;
                    if (diff > 1 || diff < -1) {
                        mb_regs[pulse_lo]     = (uint16)(expected_p & 0xFFFF);
                        mb_regs[pulse_lo + 1] = (uint16)((expected_p >> 16) & 0xFFFF);
                    }
                }
            }
        }

        int32 targets[MAX_AXES] = {0};
        /* 联动轴 (0~3) */
        targets[0] = (int32)(((uint32)mb_regs[MB_REG_MULTI_POS_X_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_POS_X_LO]);
        targets[1] = (int32)(((uint32)mb_regs[MB_REG_MULTI_POS_Y_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_POS_Y_LO]);
        targets[2] = (int32)(((uint32)mb_regs[MB_REG_MULTI_POS_Z_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_POS_Z_LO]);
        targets[3] = (int32)(((uint32)mb_regs[MB_REG_MULTI_POS_R_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_POS_R_LO]);
        /* 辅助轴 (4~7) */
        targets[4] = (int32)(((uint32)mb_regs[MB_REG_MULTI_POS_U_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_POS_U_LO]);
        targets[5] = (int32)(((uint32)mb_regs[MB_REG_MULTI_POS_V_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_POS_V_LO]);
        targets[6] = (int32)(((uint32)mb_regs[MB_REG_MULTI_POS_W_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_POS_W_LO]);
        targets[7] = (int32)(((uint32)mb_regs[MB_REG_MULTI_POS_S_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_POS_S_LO]);
        int32 feed = (int32)(((uint32)mb_regs[MB_REG_MULTI_FEED_HI] << 16)
                           | (uint32)mb_regs[MB_REG_MULTI_FEED_LO]);                // 进给速度 (RPM)
        if (feed <= 0) feed = g_move_rpm;

        if (cmd == 1) 
        {
            /* 直线插补 (支持 4~8轴) */
            Interp_Line(targets, feed);
            mb_regs[MB_REG_MULTI_STATUS] = 1;
            #if printf_cmd
            printf(">>> Interp Line: X=%d Y=%d Z=%d R=%d U=%d V=%d W=%d S=%d Feed=%d RPM <<<\r\n",
                   (int)targets[0], (int)targets[1], (int)targets[2], (int)targets[3],
                   (int)targets[4], (int)targets[5], (int)targets[6], (int)targets[7], (int)feed);
            #endif
        }
        else if (cmd == 2 || cmd == 3) {
            /* 圆弧插补 (XY 平面, 仅用前4轴) */
            Interp_ArcXY(targets[0], targets[1], targets[2], targets[3],
                         (uint8)(cmd - 2), feed);
            mb_regs[MB_REG_MULTI_STATUS] = 1;
            #if printf_cmd
            printf(">>> Interp Arc: C=(%d,%d) E=(%d,%d) %s <<<\r\n",
                   (int)targets[0], (int)targets[1],
                   (int)targets[2], (int)targets[3],
                   (cmd == 2) ? "CW" : "CCW");
            #endif
        }
        mb_regs[MB_REG_MULTI_TRIG] = 0;
    }

    /* ── ⑦ 点动命令 (上升沿/下降沿状态机) ──
     * 【模式 0 - 连续速度点动】
     *   JOG_TRIG 1→0 上升沿: 启动速度旋转
     *   JOG_TRIG 保持 1: 检测方向变更 (DIR 变化 → 立即反转)
     *   JOG_TRIG 1→0 下降沿: 停止
     * 【模式 1 - 步进点动】
     *   JOG_TRIG 0→1 上升沿: 走步进脉冲后自动清除 TRIG
     *   不需要手动写 0 停止 */
    {
        static uint16 prev_jog_trig = 0;   /**< 上一周期的 TRIG 值 (上升/下降沿检测) */
        static uint16 prev_jog_dir  = 0xFF; /**< 上一周期的 DIR (方向变更检测) */
        uint16 jog_trig = mb_regs[MB_REG_JOG_TRIG];
        uint16 axis_sel = mb_regs[MB_REG_JOG_AXIS];
        uint16 mode     = mb_regs[MB_REG_JOG_MODE];
        uint16 dir      = mb_regs[MB_REG_JOG_DIR];

        int ax_s = 0, ax_e = NUM_AXES - 1;
        if (axis_sel < (uint16)NUM_AXES) { ax_s = (int)axis_sel; ax_e = (int)axis_sel; }

        if (jog_trig == 1 && prev_jog_trig == 0) {
            /* 分支 A: 上升沿 (0→1) → 启动点动
             * 模式0 (连续): 启动速度旋转, 保持 JOG_TRIG=1
             * 模式1 (步进): 走步进后自动清 TRIG */
            int32 vel  = (int16)mb_regs[MB_REG_JOG_VEL];
            int32 step = (int32)(((uint32)mb_regs[MB_REG_JOG_STEP_HI] << 16)
                                | (uint32)mb_regs[MB_REG_JOG_STEP_LO]);

            if (vel <= 0) vel = Param_Get(PARAM_JOG_VEL_GLOBAL);
            if (vel <= 0) vel = 200;

            for (int ax = ax_s; ax <= ax_e; ax++) {
                if (ax + 1 > ec_slavecount) continue;
                if (mode == 0) {
                    /* 连续速度点动 */
                    int rpm_val = (int)vel;
                    if (dir == 1) rpm_val = -rpm_val;
                    Axis_JogVel(ax, rpm_val);
                } else {
                    /* 步进点动 */
                    int32 delta = (dir == 0) ? step : -step;
                    Axis_JogStep(ax, delta);
                }
            }
            mb_regs[MB_REG_JOG_STATUS] = 1;
            if (mode == 1) mb_regs[MB_REG_JOG_TRIG] = 0;
            prev_jog_dir = dir;
        }
        else if (jog_trig == 1 && prev_jog_trig == 1 &&
                 mode == 0 && dir != prev_jog_dir) {
            /* 分支 B: 连续模式中方向变更 → 立即反转速度
             * 不需要先停再启, 直接 ApplyVelocity 覆盖 */
            prev_jog_dir = dir;
            int32 vel = (int16)mb_regs[MB_REG_JOG_VEL];
            if (vel <= 0) vel = Param_Get(PARAM_JOG_VEL_GLOBAL);
            if (vel <= 0) vel = 200;

            for (int ax = ax_s; ax <= ax_e; ax++) {
                if (ax + 1 > ec_slavecount) continue;
                int rpm_val = (int)vel;
                if (dir == 1) rpm_val = -rpm_val;
                Axis_JogVel(ax, rpm_val);
            }
        }
        else if (jog_trig == 0 && prev_jog_trig == 1 && mode == 0) 
        {
            /* 分支 C: 下降沿 (1→0) + 连续模式 → 停止
             * 模式1(步进)不需要此分支, 上升沿已自动清除 TRIG */
            for (int ax = ax_s; ax <= ax_e; ax++) {
                if (ax + 1 <= ec_slavecount) Axis_StopOne(ax);
            }
            mb_regs[MB_REG_JOG_STATUS] = 0;
        }

        prev_jog_trig = jog_trig;
    }

    /* ── ⑧ 轴配置应用: 动态修改激活轴数量 ──
     * 前提: 所有轴必须停止 (有轴运动时拒绝)
     * ACTIVE_AXES: 激活轴总数 (1~8)
     * COUPLED_AXES: 联动轴数量 (X/Y/Z/R, 1~4)
     * AUX_AXES: 辅助轴数量 (U/V/W/S, 0~4) */
    if (mb_regs[MB_REG_CFG_APPLY] == 1) {
        uint8_t new_total = (uint8_t)mb_regs[MB_REG_CFG_ACTIVE_AXES];
        if (new_total < 1 || new_total > MAX_AXES) {
            mb_regs[MB_REG_CFG_STATUS] = 3;
            mb_regs[MB_REG_CFG_APPLY]  = 0;
        } else {
            /* 检查是否有轴在运动 */
            uint8_t busy = 0;
            for (int ax = 0; ax < MAX_AXES; ax++) {
                if (ax + 1 > ec_slavecount) continue;
                if (axis[ax].motion_busy) { busy = 1; break; }
            }
            if (busy) 
            {
                mb_regs[MB_REG_CFG_STATUS] = 3;  /* 失败: 需停机 */
                mb_regs[MB_REG_CFG_APPLY]  = 0;
            } 
            else 
            {
                g_active_axes    = new_total;
                g_active_coupled = (uint8_t)mb_regs[MB_REG_CFG_COUPLED_AXES];
                g_active_aux     = (uint8_t)mb_regs[MB_REG_CFG_AUX_AXES];
                mb_regs[MB_REG_CFG_STATUS] = 2;  /* 成功 */
                mb_regs[MB_REG_CFG_APPLY]  = 0;
            }
        }
    }
}
