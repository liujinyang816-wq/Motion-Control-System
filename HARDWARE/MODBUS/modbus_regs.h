/**
 ******************************************************************************
 * @file    modbus_regs.h
 * @brief   MODBUS RTU 从站寄存器地址映射表 (0-based PDU 地址)
 *          STM32H7 ↔ IT7100E-G HMI (多轴 EtherCAT 伺服控制系统)
 *
 *          【布局原则】只读(RO) → 只写(WO) → 读写(RW) 三区连续
 *          总容量 MB_REG_COUNT = 512
 *          实际使用 PDU 0~326, 预留 327~509
 ******************************************************************************
 */

#ifndef __MODBUS_REGS_H
#define __MODBUS_REGS_H

#define MB_REG_COUNT            512

/* ================================================================
   一、只读寄存器 (RO) — 仅 FC03 — PDU 0~95
      STM32 写入, HMI 只读; 共 96 个寄存器
   ================================================================ */

/* ── A. 联动轴 X/Y/Z/R 状态 — PDU 0~28 ────────────────────────────
   每轴 5 个寄存器: 位置(Int32, 2字) + RPM(1) + StatusWord(1) + ErrorCode(1)
   外加 9 个系统信息寄存器 */

/* 编码器位置 Int32 — 每轴占2个连续寄存器 (LO在前, HI在后) */
#define MB_REG_POS_LO_X         0    /* X轴编码器位置 低16位 */
#define MB_REG_POS_HI_X         1    /* X轴编码器位置 高16位 */
#define MB_REG_POS_LO_Y         2    /* Y轴编码器位置 低16位 */
#define MB_REG_POS_HI_Y         3    /* Y轴编码器位置 高16位 */
#define MB_REG_POS_LO_Z         4    /* Z轴编码器位置 低16位 */
#define MB_REG_POS_HI_Z         5    /* Z轴编码器位置 高16位 */
#define MB_REG_POS_LO_R         6    /* R轴编码器位置 低16位 */
#define MB_REG_POS_HI_R         7    /* R轴编码器位置 高16位 */

/* 转速 Int16 */
#define MB_REG_X_RPM            8    /* X轴当前转速 (RPM) */
#define MB_REG_Y_RPM            9    /* Y轴当前转速 (RPM) */
#define MB_REG_Z_RPM            10   /* Z轴当前转速 (RPM) */
#define MB_REG_R_RPM            11   /* R轴当前转速 (RPM) */

/* StatusWord UInt16 — 伺服驱动器标准状态字 CiA402 (0x6041) */
#define MB_REG_X_STATUS_WORD    12   /* X轴 CiA402 StatusWord */
#define MB_REG_Y_STATUS_WORD    13   /* Y轴 CiA402 StatusWord */
#define MB_REG_Z_STATUS_WORD    14   /* Z轴 CiA402 StatusWord */
#define MB_REG_R_STATUS_WORD    15   /* R轴 CiA402 StatusWord */

/* ErrorCode UInt16 — 伺服驱动器错误码 (0x603F) */
#define MB_REG_X_ERRCODE        16   /* X轴 CiA402 ErrorCode */
#define MB_REG_Y_ERRCODE        17   /* Y轴 CiA402 ErrorCode */
#define MB_REG_Z_ERRCODE        18   /* Z轴 CiA402 ErrorCode */
#define MB_REG_R_ERRCODE        19   /* R轴 CiA402 ErrorCode */

/* 系统状态 */
#define MB_REG_SYS_STATE        20   /* 系统运行状态: 0=初始化 1=启动中 2=运行 */
#define MB_REG_FAULT_CODE       21   /* 系统级故障码 (任意轴故障时汇总) */
#define MB_REG_BRAKE_STATUS     22   /* 抱闸状态: 0=锁紧, 1=释放 */
#define MB_REG_HOMING_ACTIVE    23   /* 回零激活状态: 0=空闲, 1=回零进行中 */

/* 系统信息 */
#define MB_REG_SLAVE_COUNT      24   /* EtherCAT在线从站数量 */
#define MB_REG_FW_VERSION       25   /* 固件版本号 (0x0100=v1.0) */
#define MB_REG_UPTIME_LO        26   /* 系统运行时间 低16位(秒) */
#define MB_REG_UPTIME_HI        27   /* 系统运行时间 高16位(秒) */

/* 当前定位速度回显 */
#define MB_REG_MOVE_SPEED_RD    28   /* 当前定位速度回显 (RPM) */


/* ── B. 画面2 轴调试映射区 — PDU 29~31 ────────────────────────────
   STM32 根据 AXIS_SELECT 自动拷贝选中轴的数据到此区 */

#define MB_REG_DISP_POS_LO      29   /*  选中轴位置 低16位 */
#define MB_REG_DISP_POS_HI      30   /*  选中轴位置 高16位 */
#define MB_REG_DISP_RPM         31   /*  选中轴转速 (RPM) */


/* ── C. 轴状态解析 + 系统标志 — PDU 32~37 ─────────────────────────
   STM32 从 StatusWord 拆位后直接写入，HMI 无需脚本解析 */

#define MB_REG_AXIS_STATE_X      32   /* X轴状态: 0=离线, 1=就绪, 2=运动中, 3=故障 */
#define MB_REG_AXIS_STATE_Y      33   /* Y轴状态: 0=离线, 1=就绪, 2=运动中, 3=故障 */
#define MB_REG_AXIS_STATE_Z      34   /* Z轴状态: 0=离线, 1=就绪, 2=运动中, 3=故障 */
#define MB_REG_AXIS_STATE_R      35   /* R轴状态: 0=离线, 1=就绪, 2=运动中, 3=故障 */
#define MB_REG_SYS_FLAGS         36   /* 系统标志位: bit0=故障汇总 bit1=全部就绪 bit2=抱闸 bit3=回零中 */
#define MB_REG_SYS_ALARM_COUNT   37   /* 当前活跃报警数量 */


/* ── D. 联动轴 MCS 位置 mm/deg Float — PDU 38~45 ──────────────────
   脉冲×导程换算为 IEEE 754 Float, 每个 Float 占 2 个寄存器 (低字在前) */

#define MB_REG_MCS_X_MM_LO     38   /* X轴 MCS 位置 mm 低16位 (Float) */
#define MB_REG_MCS_X_MM_HI     39   /* X轴 MCS 位置 mm 高16位 (Float) */
#define MB_REG_MCS_Y_MM_LO     40   /* Y轴 MCS 位置 mm 低16位 (Float) */
#define MB_REG_MCS_Y_MM_HI     41   /* Y轴 MCS 位置 mm 高16位 (Float) */
#define MB_REG_MCS_Z_MM_LO     42   /* Z轴 MCS 位置 mm 低16位 (Float) */
#define MB_REG_MCS_Z_MM_HI     43   /* Z轴 MCS 位置 mm 高16位 (Float) */
#define MB_REG_MCS_R_DEG_LO    44   /* R轴 MCS 角度 deg 低16位 (Float) */
#define MB_REG_MCS_R_DEG_HI    45   /* R轴 MCS 角度 deg 高16位 (Float) */

#define MB_REG_FLOAT_COUNT     8     /* [数量] Float寄存器个数 (仅联动轴 MCS), 非地址 */


/* ── E. 系统布尔标志 — PDU 46~47 ───────────────────────────────── */

#define MB_REG_SYS_FAULT        46   /* 系统故障汇总: 0=正常, 1=有故障 */
#define MB_REG_SYS_ALL_READY    47   /* 全部轴就绪: 0=未就绪, 1=全部就绪 */


/* ── F. 插补/点动 状态回显 — PDU 48~49 ─────────────────────────── */

#define MB_REG_MULTI_STATUS      48   /* 插补运行状态: 0=空闲, 1=运行中 */
#define MB_REG_JOG_STATUS        49   /* 点动状态: 0=空闲, 1=运行中 */


/* ── G. 辅助轴 U/V/W/S 状态 — PDU 50~81 ───────────────────────────
   每轴 8 个连续寄存器: 位置(2) + RPM(1) + SW(1) + ErrorCode(1) + 预留(3)
   布局: U=50~57, V=58~65, W=66~73, S=74~81 */

#define MB_REG_POS_LO_U             50   /* U轴编码器位置 低16位 */
#define MB_REG_POS_HI_U             51   /* U轴编码器位置 高16位 */
#define MB_REG_U_RPM                52   /* U轴当前转速 (RPM) */
#define MB_REG_U_STATUS_WORD        53   /* U轴 CiA402 StatusWord */
#define MB_REG_U_ERRCODE            54   /* U轴 CiA402 ErrorCode */
/* 55~57: U轴预留 */

#define MB_REG_POS_LO_V             58   /* V轴编码器位置 低16位 */
#define MB_REG_POS_HI_V             59   /* V轴编码器位置 高16位 */
#define MB_REG_V_RPM                60   /* V轴当前转速 (RPM) */
#define MB_REG_V_STATUS_WORD        61   /* V轴 CiA402 StatusWord */
#define MB_REG_V_ERRCODE            62   /* V轴 CiA402 ErrorCode */
/* 63~65: V轴预留 */

#define MB_REG_POS_LO_W             66   /* W轴编码器位置 低16位 */
#define MB_REG_POS_HI_W             67   /* W轴编码器位置 高16位 */
#define MB_REG_W_RPM                68   /* W轴当前转速 (RPM) */
#define MB_REG_W_STATUS_WORD        69   /* W轴 CiA402 StatusWord */
#define MB_REG_W_ERRCODE            70   /* W轴 CiA402 ErrorCode */
/* 71~73: W轴预留 */

#define MB_REG_POS_LO_S             74   /* S轴编码器位置 低16位 */
#define MB_REG_POS_HI_S             75   /* S轴编码器位置 高16位 */
#define MB_REG_S_RPM                76   /* S轴当前转速 (RPM) */
#define MB_REG_S_STATUS_WORD        77   /* S轴 CiA402 StatusWord */
#define MB_REG_S_ERRCODE            78   /* S轴 CiA402 ErrorCode */
/* 79~81: S轴预留 */


/* ── H. 辅助轴 MCS mm/deg Float — PDU 82~89 ───────────────────────
   每轴 2 个寄存器 (IEEE 754 single, 低字在前) */

#define MB_REG_MCS_U_MM_LO          82   /* U轴 MCS 位置 mm 低16位 (Float) */
#define MB_REG_MCS_U_MM_HI          83   /* U轴 MCS 位置 mm 高16位 (Float) */
#define MB_REG_MCS_V_MM_LO          84   /* V轴 MCS 位置 mm 低16位 (Float) */
#define MB_REG_MCS_V_MM_HI          85   /* V轴 MCS 位置 mm 高16位 (Float) */
#define MB_REG_MCS_W_MM_LO          86   /* W轴 MCS 位置 mm 低16位 (Float) */
#define MB_REG_MCS_W_MM_HI          87   /* W轴 MCS 位置 mm 高16位 (Float) */
#define MB_REG_MCS_S_DEG_LO         88   /* S轴 MCS 角度 deg 低16位 (Float) */
#define MB_REG_MCS_S_DEG_HI         89   /* S轴 MCS 角度 deg 高16位 (Float) */


/* ── I. 辅助轴状态解析 — PDU 90~93 ──────────────────────────────
   0=离线, 1=就绪, 2=运动中, 3=故障 */

#define MB_REG_AXIS_STATE_U         90  /* U轴状态 */
#define MB_REG_AXIS_STATE_V         91  /* V轴状态 */
#define MB_REG_AXIS_STATE_W         92  /* W轴状态 */
#define MB_REG_AXIS_STATE_S         93  /* S轴状态 */


/* ── J. 配置状态 + 辅助轴标志 — PDU 94~95 ─────────────────────── */

#define MB_REG_CFG_STATUS           94   /* 轴配置状态: 0=空闲 1=忙 2=成功 3=失败 */
#define MB_REG_SYS_AUX_FLAGS        95   /* 辅助轴标志: bit0=U故障 bit1=V故障 bit2=W故障 bit3=S故障 */


/* ================================================================
   二、只写寄存器 (WO) — FC06/FC16 — PDU 96~128
      HMI 写入命令/触发, STM32 消费; 共 33 个寄存器
   ================================================================ */

/* ── A. 速度命令 — PDU 96~98 ──────────────────────────────────── */

#define MB_REG_SPEED_RPM        96   /* 速度命令 RPM 设定值 (-3000~3000) */
#define MB_REG_SPEED_AXIS       97   /* 速度命令轴选择: 0=X,1=Y,2=Z,3=R,4=全部 */
#define MB_REG_SPEED_TRIG       98   /* 速度命令触发: 写1触发, 执行后自动清0 */


/* ── B. 脉冲定位命令 — PDU 99~102 ──────────────────────────────── */

#define MB_REG_POS_PULSES_LO    99   /* 脉冲定位目标 低16位 */
#define MB_REG_POS_PULSES_HI    100  /* 脉冲定位目标 高16位 */
#define MB_REG_POS_AXIS         101  /* 脉冲定位轴选择: 0=X,1=Y,2=Z,3=R */
#define MB_REG_POS_TRIG         102  /* 脉冲定位触发: 写1触发, 执行后自动清0 */


/* ── C. 圈数命令 — PDU 103~105 ──────────────────────────────────── */

#define MB_REG_REVS             103  /* 圈数定位目标 (圈, 正=正转/负=反转) */
#define MB_REG_REVS_AXIS        104  /* 圈数轴选择: 0=X,1=Y,2=Z,3=R */
#define MB_REG_REVS_TRIG        105  /* 圈数触发: 写1触发, 执行后自动清0 */


/* ── D. 系统命令 — PDU 106~108 ──────────────────────────────────── */

#define MB_REG_SYS_CMD          106  /* 系统命令: 1=STOP, 2=RESET, 3=ZERO回零 */
#define MB_REG_SYS_AXIS         107  /* 系统命令轴选择: 0=X,1=Y,2=Z,3=R,4=全部 */
#define MB_REG_SYS_TRIG         108  /* 系统命令触发: 写1触发, 执行后自动清0 */


/* ── E. 多轴插补命令 + 脉冲目标 — PDU 109~120 ─────────────────────
   联动轴 X/Y/Z/R 的插补脉冲目标, HMI 写入脉冲值或通过 mm 换算区间接写入 */

#define MB_REG_MULTI_CMD         109  /* 插补命令类型: 1=直线, 2=顺圆弧CW, 3=逆圆弧CCW */
#define MB_REG_MULTI_TRIG        110  /* 插补触发: 写1触发, 执行后自动清0 */
#define MB_REG_MULTI_POS_X_LO    111  /* 插补 X轴目标 低16位(脉冲) */
#define MB_REG_MULTI_POS_X_HI    112  /* 插补 X轴目标 高16位(脉冲) */
#define MB_REG_MULTI_POS_Y_LO    113  /* 插补 Y轴目标 低16位(脉冲) */
#define MB_REG_MULTI_POS_Y_HI    114  /* 插补 Y轴目标 高16位(脉冲) */
#define MB_REG_MULTI_POS_Z_LO    115  /* 插补 Z轴目标 低16位(脉冲) */
#define MB_REG_MULTI_POS_Z_HI    116  /* 插补 Z轴目标 高16位(脉冲) */
#define MB_REG_MULTI_POS_R_LO    117  /* 插补 R轴目标 低16位(脉冲) */
#define MB_REG_MULTI_POS_R_HI    118  /* 插补 R轴目标 高16位(脉冲) */
#define MB_REG_MULTI_FEED_LO     119  /* 插补进给速度 低16位(RPM) */
#define MB_REG_MULTI_FEED_HI     120  /* 插补进给速度 高16位(RPM) */


/* ── F. 点动 (Jog) 控制 — PDU 121~127 ─────────────────────────────
   连续速度或步进模式点动, HMI 写 JOG_TRIG=1 启动/0 停止 */

#define MB_REG_JOG_AXIS          121  /* 点动轴选择: 0=X,1=Y,2=Z,3=R,4=全部 */
#define MB_REG_JOG_MODE          122  /* 点动模式: 0=连续速度, 1=步进 */
#define MB_REG_JOG_VEL           123  /* 点动速度 (RPM) */
#define MB_REG_JOG_STEP_LO       124  /* 步进脉冲量 低16位 */
#define MB_REG_JOG_STEP_HI       125  /* 步进脉冲量 高16位 */
#define MB_REG_JOG_DIR           126  /* 点动方向: 0=正转, 1=反转 */
#define MB_REG_JOG_TRIG          127  /* 点动触发: 1=启动, 0=停止 */


/* ── G. 轴配置应用触发 — PDU 128 ────────────────────────────────── */

#define MB_REG_CFG_APPLY          128  /* 写1=应用轴配置 (需停机), 执行后自动清0 */


/* ================================================================
   三、读写寄存器 (RW) — FC03/FC06/FC16 — PDU 129~326
      HMI 和 STM32 均可读写; 共 198 个寄存器
   ================================================================ */

/* ── A. 全局参数 — PDU 129~131 ──────────────────────────────────── */

#define MB_REG_POS_SPEED        129  /* 全局定位速度 (100~3000 RPM) */
#define MB_REG_JOG_SPEED        130  /* 全局点动速度 (50~500 RPM) */
#define MB_REG_AXIS_SELECT      131  /* 当前选中轴: 0=X,1=Y,2=Z,3=R (影响镜像区路由) */


/* ── B. 联动轴 X/Y/Z/R 参数映射 — PDU 132~195 ─────────────────────
   64 个寄存器映射 param_ram[0..63], 每轴 16 个参数 (定义见 param_defs.h)
   ┌──────────┬───────────┬──────────────────────────────────┐
   │ PDU      │ param_ram │ 内容                             │
   ├──────────┼───────────┼──────────────────────────────────┤
   │ 132~147  │ 0~15      │ X轴: 最大转速/加速度/Jerk/回零偏置│
   │          │           │      软限位×2/间隙/导程/齿轮比×2 │
   │          │           │      反转方向/点动速度/回零速度   │
   │          │           │      回零加速度/到位窗口/跟随误差 │
   │ 148~163  │ 16~31     │ Y轴: 同上 16 个参数              │
   │ 164~179  │ 32~47     │ Z轴: 同上 16 个参数              │
   │ 180~195  │ 48~63     │ R轴: 同上 16 个参数              │
   └──────────┴───────────┴──────────────────────────────────┘ */

#define MB_REG_PARAM_BASE        132  /* 联动轴参数映射区起始地址 */
#define MB_REG_PARAM_COUNT       64   /* [数量] 联动轴参数寄存器个数, 非地址 */


/* ── C. 伺服驱动器参数 — PDU 196~198 ────────────────────────────── */

#define MB_REG_SERVO_GAIN        196  /* 伺服位置环增益 */
#define MB_REG_SERVO_FILTER      197  /* 伺服速度环滤波 */
#define MB_REG_SERVO_INERTIA     198  /* 伺服转动惯量比 */


/* ── D. 扩展参数区-全局 — PDU 199~207 ──────────────────────────────
   映射 param_ram[128..136], 每参数占 1 个 16bit 寄存器 */

#define MB_REG_EXT_GLOBAL_BASE      199  /* 扩展参数区起始地址 */
#define MB_REG_EXT_DEFAULT_VEL      199  /* 全局默认定位速度 (RPM, param_ram[128]) */
#define MB_REG_EXT_JOG_VEL          200  /* 全局点动速度 (RPM, param_ram[129]) */
#define MB_REG_EXT_HOMING_VEL       201  /* 全局回零速度 (RPM, param_ram[130]) */
#define MB_REG_EXT_HOMING_METHOD    202  /* CiA402回零方式 (1~35, param_ram[131]) */
#define MB_REG_EXT_HOMING_ACCEL     203  /* 回零加速度 (cts/T², param_ram[132]) */
#define MB_REG_EXT_SMOOTH_STOP      204  /* 平滑停止使能: 0=梯形, 1=S曲线 (param_ram[133]) */
#define MB_REG_EXT_FEED_OVERRIDE    205  /* 进给倍率 10~200% (param_ram[134]) */
#define MB_REG_EXT_RAPID_OVERRIDE   206  /* 快进倍率 10~100% (param_ram[135]) */
#define MB_REG_EXT_SPINDLE_OVERRIDE 207  /* 主轴倍率 50~150% (param_ram[136]) */
#define MB_REG_EXT_GLOBAL_COUNT     9    /* [数量] 扩展全局参数寄存器个数, 非地址 */


/* ── E. 定位/点动 mm 目标 — PDU 208~211 ───────────────────────────
   HMI 写入 mm 值 (Float), STM32 按选中轴导程自动换算为脉冲
   每个 Float 占 2 个寄存器 (IEEE 754 single, 低字在前) */

#define MB_REG_POS_TARGET_MM_LO  208  /* 单轴定位目标 mm 低16位 (Float) */
#define MB_REG_POS_TARGET_MM_HI  209  /* 单轴定位目标 mm 高16位 (Float) */
#define MB_REG_JOG_STEP_MM_LO    210  /* 点动步进步长 mm 低16位 (Float) */
#define MB_REG_JOG_STEP_MM_HI    211  /* 点动步进步长 mm 高16位 (Float) */


/* ── F. 多轴插补目标 mm/deg — PDU 212~223 ─────────────────────────
   HMI 写入 mm/deg 值 (Float), STM32 自动换算为脉冲并同步到脉冲命令区
   每个 Float 占 2 个寄存器 (IEEE 754 single, 低字在前) */

#define MB_REG_INTERP_X_MM_LO     212  /* 直线插补 X 目标 mm 低16位 (Float) */
#define MB_REG_INTERP_X_MM_HI     213  /* 直线插补 X 目标 mm 高16位 (Float) */
#define MB_REG_INTERP_Y_MM_LO     214  /* 直线插补 Y 目标 mm 低16位 (Float) */
#define MB_REG_INTERP_Y_MM_HI     215  /* 直线插补 Y 目标 mm 高16位 (Float) */
#define MB_REG_INTERP_Z_MM_LO     216  /* 直线插补 Z 目标 mm 低16位 (Float) */
#define MB_REG_INTERP_Z_MM_HI     217  /* 直线插补 Z 目标 mm 高16位 (Float) */
#define MB_REG_INTERP_R_DEG_LO    218  /* 直线插补 R 目标 deg 低16位 (Float) */
#define MB_REG_INTERP_R_DEG_HI    219  /* 直线插补 R 目标 deg 高16位 (Float) */
#define MB_REG_INTERP_FEED_MM_LO  220  /* 插补进给速度 mm/min 低16位 (Float) */
#define MB_REG_INTERP_FEED_MM_HI  221  /* 插补进给速度 mm/min 高16位 (Float) */
#define MB_REG_INTERP_ARC_Y_MM_LO 222  /* 圆弧插补终点 Y mm 低16位 (Float) */
#define MB_REG_INTERP_ARC_Y_MM_HI 223  /* 圆弧插补终点 Y mm 高16位 (Float) */


/* ── G. 轴参数镜像区 — PDU 224~242 ─────────────────────────────────
   STM32 根据 AXIS_SELECT 自动路由到对应轴的 param_ram
   参数画面只需一套轴面板, 绑定此镜像区即可切换 X/Y/Z/R
   Int16 参数 13 个 (224~236) + Int32 参数 3 个×2 字 (237~242) = 19 寄存器 */

/* Int16 参数镜像 */
#define MB_REG_MIRROR_BASE              224  /* 镜像区起始地址 */
/* 224: 最大转速 RPM        (PARAM_AXIS_MAX_RPM)       */
/* 225: 最大加速度 cts/T²  (PARAM_AXIS_MAX_ACCEL)     */
/* 226: 最大Jerk cts/T³    (PARAM_AXIS_MAX_JERK)      */
/* 227: 反向间隙 脉冲       (PARAM_AXIS_BACKLASH)      */
/* 228: 丝杠导程 um/rev    (PARAM_AXIS_PITCH)         */
/* 229: 齿轮比分子          (PARAM_AXIS_GEAR_NUM)      */
/* 230: 齿轮比分母          (PARAM_AXIS_GEAR_DEN)      */
/* 231: 反转方向 0=正/1=反 (PARAM_AXIS_INVERT_DIR)    */
/* 232: 点动速度 RPM       (PARAM_AXIS_JOG_VEL)       */
/* 233: 回零速度 RPM       (PARAM_AXIS_HOMING_VEL)    */
/* 234: 回零加速度 cts/T²  (PARAM_AXIS_HOMING_ACC)    */
/* 235: 到位窗口 脉冲       (PARAM_AXIS_POS_ARRIVE_WIN) */
/* 236: 最大跟随误差 脉冲    (PARAM_AXIS_MAX_FOLLOW_ERR) */
#define MB_REG_MIRROR_COUNT             13   /* [数量] Int16镜像参数个数 (13个参数), 非地址 */

/* Int32 参数镜像 */
#define MB_REG_MIRROR_HOME_OFF_LO       237  /* 选中轴回零偏置 低16位 */
#define MB_REG_MIRROR_HOME_OFF_HI       238  /* 选中轴回零偏置 高16位 */
#define MB_REG_MIRROR_SOFT_PLUS_LO      239  /* 选中轴正软限位 低16位 */
#define MB_REG_MIRROR_SOFT_PLUS_HI      240  /* 选中轴正软限位 高16位 */
#define MB_REG_MIRROR_SOFT_MINUS_LO     241  /* 选中轴负软限位 低16位 */
#define MB_REG_MIRROR_SOFT_MINUS_HI     242  /* 选中轴负软限位 高16位 */
#define MB_REG_MIRROR_INT32_COUNT       6    /* [数量] Int32镜像参数占用的寄存器个数 (3参数×2字), 非地址 */


/* ── H. 辅助轴插补目标 — PDU 243~258 ──────────────────────────────
   mm/deg Float 目标(8) + 脉冲目标(8) = 16 寄存器 */

/* mm/deg Float 目标 */
#define MB_REG_INTERP_U_MM_LO       243  /* U 插补目标 mm 低16位 (Float) */
#define MB_REG_INTERP_U_MM_HI       244  /* U 插补目标 mm 高16位 (Float) */
#define MB_REG_INTERP_V_MM_LO       245  /* V 插补目标 mm 低16位 (Float) */
#define MB_REG_INTERP_V_MM_HI       246  /* V 插补目标 mm 高16位 (Float) */
#define MB_REG_INTERP_W_MM_LO       247  /* W 插补目标 mm 低16位 (Float) */
#define MB_REG_INTERP_W_MM_HI       248  /* W 插补目标 mm 高16位 (Float) */
#define MB_REG_INTERP_S_DEG_LO      249  /* S 插补目标 deg 低16位 (Float) */
#define MB_REG_INTERP_S_DEG_HI      250  /* S 插补目标 deg 高16位 (Float) */

/* 脉冲目标 (HMI 写入或 STM32 由 mm→脉冲换算写入) */
#define MB_REG_MULTI_POS_U_LO       251  /* U 插补脉冲目标 低16位 */
#define MB_REG_MULTI_POS_U_HI       252  /* U 插补脉冲目标 高16位 */
#define MB_REG_MULTI_POS_V_LO       253  /* V 插补脉冲目标 低16位 */
#define MB_REG_MULTI_POS_V_HI       254  /* V 插补脉冲目标 高16位 */
#define MB_REG_MULTI_POS_W_LO       255  /* W 插补脉冲目标 低16位 */
#define MB_REG_MULTI_POS_W_HI       256  /* W 插补脉冲目标 高16位 */
#define MB_REG_MULTI_POS_S_LO       257  /* S 插补脉冲目标 低16位 */
#define MB_REG_MULTI_POS_S_HI       258  /* S 插补脉冲目标 高16位 */


/* ── I. 系统配置参数 — PDU 259~262 ────────────────────────────────
   HMI 动态轴选择的核心接口 (需停机应用) */

#define MB_REG_CFG_ACTIVE_AXES      259  /* 激活轴总数 (1~8, 默认4) */
#define MB_REG_CFG_COUPLED_AXES     260  /* 联动轴数量 (1~4, 默认4) */
#define MB_REG_CFG_AUX_AXES         261  /* 辅助轴数量 (0~4, 默认0) */
#define MB_REG_CFG_AXIS_ENABLE      262  /* 轴使能位图 bit0=X...bit7=S */


/* ── J. 辅助轴 U/V/W/S 参数映射 — PDU 263~326 ─────────────────────
   64 个寄存器映射 param_ram[64..127], 每轴 16 槽位 (定义见 param_defs.h)
   注意: 辅助轴参数布局各不相同, 不像联动轴那样完全对称
   ┌──────────┬───────────┬──────────────────────────────────┐
   │ PDU      │ param_ram │ 内容                             │
   ├──────────┼───────────┼──────────────────────────────────┤
   │ 263~278  │ 64~79     │ S轴(刀库): 分度速度/加速度/偏置  │
   │          │           │      齿轮比×2/反转/点动/到位/回零│
   │ 279~294  │ 80~95     │ W轴(升降): 升降速度/加速度/偏置  │
   │          │           │      软限位×2/导程/齿轮比×2/反转 │
   │          │           │      点动速度/回零速度/到位窗口   │
   │ 295~310  │ 96~111    │ U轴(摇动): 振荡速度/振幅/导程    │
   │          │           │      齿轮比×2/反转方向           │
   │ 311~326  │ 112~127   │ V轴(摇动): 与U轴对称布局         │
   └──────────┴───────────┴──────────────────────────────────┘ */

#define MB_REG_AUX_PARAM_BASE       263  /* 辅助轴参数区起始 */
#define MB_REG_AUX_PARAM_COUNT      64   /* [数量] 辅助轴参数寄存器个数, 非地址 */

/* ── K. 轴类型模式 (PDU 327~331) ──
   R/U/V/W/S 的工作模式: 0=直线(50000ppr, 显示mm), 1=旋转(36000ppr, 显示deg)
   X/Y/Z 不支持切换, 固定为直线模式 */
#define MB_REG_AXIS_TYPE_R       327
#define MB_REG_AXIS_TYPE_U       328
#define MB_REG_AXIS_TYPE_V       329
#define MB_REG_AXIS_TYPE_W       330
#define MB_REG_AXIS_TYPE_S       331

/* 332~509: 预留 (178 个寄存器) */

#endif /* __MODBUS_REGS_H__ */
