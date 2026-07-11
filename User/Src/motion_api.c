/**
 ******************************************************************************
 * @file           : motion_api.c
 * @brief          : 单轴/多轴运动控制API + DDA插补实现
 * ----------------------------------------------------------------------------
 * 封装现有的 Axis_ApplyVelocity / Axis_ApplyPosition 逻辑,
 * 挂接 S曲线规划器, 提供统一的运动控制接口
 *
 * 【架构】
 *   HMI/串口 → motion_api → SCurvePlanner → Axis_ControlCycle → PDO输出
 ******************************************************************************
 */
#include "motor_axis.h"
#include "motion_api.h"
#include "param_defs.h"
#include "ethercatcoe.h"
#include "ethercat_slave.h"
#include "main.h"
#include "stdio.h"
#include <string.h>
#include <math.h>

/* ── 外部全局变量引用 ── */
extern int32 param_ram[PARAM_COUNT];

/* ── 辅助: 轴名称 ── */
static const char *axis_name[MAX_AXES] = {"X","Y","Z","R","U","V","W","S"};

// ── 全局运动控制变量 ── 
int   g_move_rpm = MOVE_RPM;       /**< 当前定位速度 (RPM) */
int   g_home_rpm = HOMING_RPM;     /**< 当前回零速度 (RPM) */
uint8 homing_active = 0;           /**< 1=开机回初始位置进行中 */
uint8 zero_requested = 0;          /**< 回零请求标志 */


/**
 * @brief  使能指定轴 (遵循 CiA 402 使能序列)
 * @param  ax  轴索引 (0=X, 1=Y, 2=Z, 3=R)
 * @retval 1=成功, 0=失败(轴索引非法或从站不存在)
 */
int Axis_Enable(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return 0;
    if (ax + 1 > ec_slavecount)   return 0;

    /* CiA 402 使能序列: Shutdown → SwitchOn → EnableOp */
    uint16 ctrl_seq[] = {0x0006, 0x0007, 0x000F};

    for (int step = 0; step < 3; step++) {
        axis[ax].pdo_out->ControlWord = ctrl_seq[step];
        axis[ax].pdo_out->TargetMode  = 8;

        HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

        HAL_Delay(30);
        ec_receive_processdata(EC_TIMEOUTRET);
    }

    /* 验证 */
    uint16 sw = axis[ax].pdo_in->StatusWord;
    if ((sw & 0x006F) == 0x0027) {
        axis[ax].enabled    = 1;
        axis[ax].actual_pos = axis[ax].pdo_in->CurrentPosition;
        axis[ax].target_pos = axis[ax].actual_pos;
        axis[ax].cmd_target_pos = axis[ax].actual_pos;
        axis[ax].pdo_out->TargetPos = axis[ax].target_pos;
        #if printf_cmd
        printf("%s: ENABLED\r\n", axis_name[ax]);
        #endif
        return 1;
    }

    /* 强制使能 */
    #if printf_cmd
    printf("%s: force-enable (SW=0x%04x)\r\n", axis_name[ax], sw);
    #endif
    axis[ax].enabled    = 1;
    axis[ax].actual_pos = axis[ax].pdo_in->CurrentPosition;
    axis[ax].target_pos = axis[ax].actual_pos;
    axis[ax].cmd_target_pos = axis[ax].actual_pos;
    axis[ax].pdo_out->TargetPos = axis[ax].target_pos;
    return 1;
}

/**
 * @brief  禁用指定轴
 * @param  ax  轴索引
 * @retval 1=成功, 0=失败
 */
int Axis_Disable(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return 0;
    axis[ax].pdo_out->ControlWord = 0x0000;  /* Disable Voltage */
    axis[ax].enabled = 0;
    axis[ax].mode    = AXIS_MODE_HOLD;
    #if printf_cmd
    printf("%s: DISABLED\r\n", axis_name[ax]);
    #endif
    return 1;
}

/**
 * @brief  查询轴是否正在运动中
 * @param  ax  轴索引
 * @retval 1=运动中, 0=已停止
 */
int Axis_IsMoving(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return 0;
    return axis[ax].motion_busy;
}

/**
 * @brief  查询轴是否处于故障状态
 * @param  ax  轴索引
 * @retval 1=故障, 0=正常
 */
int Axis_IsFault(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return 0;
    return (axis[ax].status_word & 0x0008) ? 1 : 0;
}

/**
 * @brief  查询轴是否已使能
 * @param  ax  轴索引
 * @retval 1=已使能, 0=未使能
 */
int Axis_IsEnabled(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return 0;
    return axis[ax].enabled;
}

/**
 * @brief  获取轴当前实际位置 (脉冲单位)
 * @param  ax  轴索引
 * @retval 当前位置 (用户坐标系脉冲)
 */
int32 Axis_GetPos(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return 0;
    return axis[ax].actual_pos;
}

/**
 * @brief  获取轴当前实际位置 (脉冲单位)
 * @param  ax  轴索引
 * @retval 当前位置 (用户坐标系脉冲)
 */
int32 Axis_GetVel(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return 0;
    /* 速度单位: RPM */
    return CycleVel_to_RPM(axis[ax].velocity, ax);
}



/* ══════════════════════════════════════════════════════════════════
 * L2: 单轴运动
 * ══════════════════════════════════════════════════════════════════ */
/**
 * @brief  点动 - 速度模式 (持续转动)
 * @param  ax   轴索引
 * @param  rpm  目标转速，正负表示方向，0=停止
 * @retval 1=成功, 0=失败
 * @note   如果 rpm=0 使用默认点动速度；转速限幅 3000RPM
 */
int Axis_JogVel(int ax, int32 rpm)
{
    if (ax < 0 || ax >= NUM_AXES)     return 0;
    if (ax + 1 > ec_slavecount)       return 0;
    if (!axis[ax].enabled)            return 0;

    /* 使用轴参数中的 Jog Vel */
    int32 jog_vel = Param_Get(AXIS_PARAM(PARAM_AXIS_JOG_VEL, ax));
    if (jog_vel <= 0) jog_vel = Param_Get(PARAM_JOG_VEL_GLOBAL);
    if (jog_vel <= 0) jog_vel = 200;

    int abs_rpm = (rpm >= 0) ? (int)rpm : (int)(-rpm);
    if (abs_rpm == 0) abs_rpm = jog_vel;
    if (abs_rpm > 3000) abs_rpm = 3000;

    int sign = (rpm >= 0) ? 1 : -1;
    Axis_ApplyVelocity(ax, abs_rpm, sign);
    return 1;
}

/**
 * @brief  点动 - 增量步进模式 (走指定脉冲数后停止)
 * @param  ax    轴索引
 * @param  delta 增量(脉冲)，正负表示方向
 * @retval 1=成功, 0=失败
 * @note   自动检查软限位并钳位；R轴不钳位，自动旋转归一化
 */
int Axis_JogStep(int ax, int32 delta)
{
    if (ax < 0 || ax >= NUM_AXES)     return 0;
    if (ax + 1 > ec_slavecount)       return 0;
    if (!axis[ax].enabled)            return 0;

    int32 target = (int32)((int64)axis[ax].actual_pos + (int64)delta);

    /* 检查软限位 (R轴由 Axis_ApplyPosition 做旋转归一化, 跳过钳位) */
    if (ax != 3) {
        int32 soft_p = Param_Get(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, ax));
        int32 soft_n = Param_Get(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, ax));
        if (soft_p > soft_n) {  /* 限位有效 */
            if (target > soft_p) target = soft_p;
            if (target < soft_n) target = soft_n;
        }
    }

    Axis_ApplyPosition(ax, target);  /* 内部已完成 SCurve_Plan */

    return 1;
}

/**
 * @brief  绝对位置定位 (走到指定绝对坐标)
 * @param  ax   轴索引
 * @param  pos  目标绝对位置 (脉冲)
 * @retval 1=成功, 0=失败
 * @note   使用 S 曲线规划；自动软限位检查
 */
int Axis_MoveAbs(int ax, int32 pos)
{
    if (ax < 0 || ax >= NUM_AXES)     return 0;
    if (ax + 1 > ec_slavecount)       return 0;
    if (!axis[ax].enabled)            return 0;

    /* 软限位校验 (R轴由 Axis_ApplyPosition 做旋转归一化, 跳过钳位) */
    if (ax != 3) {
        int32 soft_p = Param_Get(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, ax));
        int32 soft_n = Param_Get(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, ax));
        if (soft_p > soft_n) {
            if (pos > soft_p) pos = soft_p;
            if (pos < soft_n) pos = soft_n;
        }
    }

    Axis_ApplyPosition(ax, pos);  /* 内部已完成 SCurve_Plan */

    return 1;
}

/**
 * @brief  相对位置移动 (从当前位置走增量)
 * @param  ax    轴索引
 * @param  delta 增量(脉冲)
 * @retval 1=成功, 0=失败
 */
int Axis_MoveRel(int ax, int32 delta)
{
    int32 target = (int32)((int64)axis[ax].actual_pos + (int64)delta);
    return Axis_MoveAbs(ax, target);
}

/* ══════════════════════════════════════════════════════════════════
 * L3: 回零
 * ══════════════════════════════════════════════════════════════════ */

 /**
 * @brief  启动单轴回零
 * @param  ax      轴索引
 * @param  method  回零方法 (35=CiA 402 协议回零，其他=软件回零到偏移位置)
 * @retval 1=成功启动, 0=失败
 */
int Axis_Home(int ax, uint8 method)
{
    if (ax < 0 || ax >= NUM_AXES)     return 0;
    if (ax + 1 > ec_slavecount)       return 0;
    if (!axis[ax].enabled)            return 0;

    Axis_CancelHoming(ax);

    int32 home_offset = Param_Get(AXIS_PARAM(PARAM_AXIS_HOME_OFFSET, ax));
    int32 home_vel    = Param_Get(AXIS_PARAM(PARAM_AXIS_HOMING_VEL, ax));
    if (home_vel <= 0) home_vel = Param_Get(PARAM_HOMING_VEL_GLOBAL);
    if (home_vel <= 0) home_vel = HOMING_RPM;

    if (method == 35) {
        /* Method 35: 当前位置归零 (CiA 402 Homing)
         * 通过 SDO 写入 Homing Method + 启动 */
        uint8 hm = 35;
        ec_SDOwrite(axis[ax].slave_idx, 0x6098, 0x00, FALSE,
                    sizeof(hm), &hm, EC_TIMEOUTRXM);
        axis[ax].pdo_out->TargetMode  = 6;      /* Homing mode */
        axis[ax].pdo_out->ControlWord = 0x001F;  /* bit4=1: start homing */
        axis[ax].zero_step = 1;
        zero_requested = 1;
    } else {
        /* 通用 CSP 回零: 移动到 home_offset 位置 */
        axis[ax].mode = AXIS_MODE_POS;
        axis[ax].cmd_target_pos = home_offset;
        axis[ax].homing = 1;
        Axis_SetMoveSpeed(ax, (int)home_vel);
    }

    homing_active = 1;
    #if printf_cmd
    printf("%s: homing (method=%d, offset=%d)...\r\n",
           axis_name[ax], method, (int)home_offset);
    #endif
    return 1;
}

/**
 * @brief  对所有有效轴依次启动回零
 * @retval 全部成功返回1，任意失败返回0
 */
int Axis_HomeAll(void)
{
    int ok = 1;
    for (int ax = 0; ax < NUM_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        if (!Axis_Home(ax, (uint8)Param_Get(PARAM_HOMING_METHOD)))
            ok = 0;
    }
    return ok;
}

/* ══════════════════════════════════════════════════════════════════
 * L4: 多轴插补 (主从式 DDA, 单 S 曲线)
 *
 *  原理: 只用最长轴 (master) 建立一个 S 曲线规划器,
 *        其余从轴按比例跟随 → 保证所有轴严格同步启停。
 * ══════════════════════════════════════════════════════════════════ */

 /**
 * @brief  多轴直线插补 - 规划直线路径到目标点
 * @param  targets  各轴目标位置数组 [X,Y,Z,R]
 * @param  feedrate 进给速度 RPM，0=使用全局默认速度
 * @retval 1=成功, 0=失败
 * @note   采用主从 DDA 算法：选行程最长轴作主主轴规划 S 曲线，其余轴按比例跟随
 *         保证所有轴严格同步启停，形成直线
 */
int Interp_Line(int32 *targets, int32 feedrate)
{
    if (!targets) return 0;

    /* ── ① 计算各轴行程 + 选最长轴为主轴 ── */
    int32 delta[MAX_AXES];
    int32 max_delta = 0;
    int   master_ax = 0;

    for (int i = 0; i < NUM_AXES; i++) {
        if (i + 1 > ec_slavecount) { delta[i] = 0; continue; }
        delta[i] = targets[i] - axis[i].actual_pos;
        int32 abs_d = (delta[i] >= 0) ? delta[i] : -delta[i];
        if (abs_d > max_delta) { max_delta = abs_d; master_ax = i; }
    }
    if (max_delta <= 0) return 0;

    /* ── ② 主轴: 加速度随速度自动缩放, 建唯一 S 曲线规划器 ── */
    int32 base_step = (feedrate > 0) ? RPM_to_CycleVel(feedrate, master_ax)
                      : RPM_to_CycleVel(g_move_rpm, master_ax);
    int32 acc_cap = Param_Get(AXIS_PARAM(PARAM_AXIS_MAX_ACCEL, master_ax));
    int32 max_acc, jerk;
    SCurve_AutoAccJerk(base_step, acc_cap, &max_acc, &jerk);

    /* 避免主轴进入短行程模式导致轮廓变形: 若不满足则自动降速
     * 必须用与SCurve_Plan一致的Tj(max_acc与jerk可能被Tj约束修正) */
    {
        int32 Tj_ck = max_acc / jerk;
        if (Tj_ck < 1) Tj_ck = 1;
        /* Tj约束: 与SCurve_Plan保持同步, 否则降速判断偏保守 */
        while (Tj_ck > 1 &&
               (int64)(max_acc + jerk) * (int64)Tj_ck > (int64)base_step) {
            Tj_ck--;
        }
        int32 max_acc_ck = Tj_ck * jerk;  /* 修正后的max_acc */
        /* 离散公式: v_at_Tj = jerk·Tj·(Tj+1)/2 */
        int32 v_tj = jerk * Tj_ck * (Tj_ck + 1) / 2;
        int32 vel = base_step;
        int32 ta  = (vel > v_tj) ? ((vel - v_tj) / max_acc_ck) : 0;
        int32 s_acc = vel * (int32)(Tj_ck + ta);
        if ((int64)max_delta < (int64)2 * s_acc) {
            int32 safe_vel = vel;
            while (safe_vel > 0 &&
                   (int64)max_delta < (int64)2 * safe_vel *
                   (int32)(Tj_ck + ((safe_vel > v_tj) ? ((safe_vel - v_tj) / max_acc_ck) : 0)))
                safe_vel -= base_step / 20;
            if (safe_vel < base_step / 4) safe_vel = base_step / 4;
            base_step = safe_vel;
            /* 降速后重新计算 acc/jerk */
            SCurve_AutoAccJerk(base_step, acc_cap, &max_acc, &jerk);
        }
    }

    SCurve_Plan(&axis[master_ax].splanner,
                axis[master_ax].actual_pos, targets[master_ax],
                base_step, max_acc, jerk);
    axis[master_ax].mode       = AXIS_MODE_POS;
    axis[master_ax].cmd_target_pos = targets[master_ax];
    axis[master_ax].motion_busy    = 1;
    axis[master_ax].dda_master     = -1;  /* 主轴标记 */

    /* ── ③ 从轴: 记录 DDA 参数, 跟随 ── */
    int32 master_delta_abs = (delta[master_ax] >= 0)
                           ? delta[master_ax] : -delta[master_ax];
    for (int i = 0; i < NUM_AXES; i++) {
        if (i == master_ax || i + 1 > ec_slavecount) continue;
        axis[i].mode            = AXIS_MODE_DDA;
        axis[i].cmd_target_pos  = targets[i];
        axis[i].target_pos      = axis[i].actual_pos;  /* 累加起点, 每周期增量补足 */
        axis[i].dda_master      = (int8)master_ax;
        axis[i].dda_start       = axis[i].actual_pos;
        axis[i].dda_delta       = delta[i];
        axis[i].dda_master_start = axis[master_ax].splanner.start_pos;
        axis[i].dda_master_delta = master_delta_abs;
        axis[i].motion_busy     = 1;
    }

    /*
    #if printf_cmd
    printf("  DDA: master=%s vel=%d max_d=%d  ",
           axis_name[master_ax], (int)base_step, (int)max_delta);
    #endif
    for (int i = 0; i < NUM_AXES; i++)
        if (i + 1 <= ec_slavecount)
            #if printf_cmd
            printf("%s:%d(%d) ", axis_name[i],
                   (int)targets[i], (int)delta[i]);
            #endif
    #if printf_cmd
    printf("\r\n");
    #endif
*/
    return 1;
}

/* ══════════════════════════════════════════════════════════════════
 * 连续圆弧插补状态 (绕过运动队列, 直接驱动 XY 轴)
 * ══════════════════════════════════════════════════════════════════ */
typedef struct {
    uint8  active;          /**< 1=圆弧运行中 */
    int32  cx, cy;          /**< 圆心坐标 (MCS 脉冲) */
    int32  ex, ey;          /**< 终点坐标 (MCS 脉冲) */
    double radius_d;        /**< 半径 (double 精度) */
    double start_angle;     /**< 起始角度 (rad) */
    double sweep;           /**< 扫掠角 (rad, >0) */
    int32  arc_len;         /**< 弧长 (脉冲) */
    SCurvePlanner_t splanner; /**< S 曲线规划器 (控制沿弧线方向的速度) */
} ArcMotion_t;

static ArcMotion_t g_arc = {0};

/**
 * @brief  XY平面圆弧插补 - 启动圆弧运动
 * @param  cx        圆心 X 坐标 (脉冲)
 * @param  cy        圆心 Y 坐标 (脉冲)
 * @param  ex        终点 X 坐标 (脉冲)
 * @param  ey        终点 Y 坐标 (脉冲)
 * @param  dir       方向: 0=顺圆CW, 1=逆圆CCW
 * @param  feedrate  进给速度 RPM，0=使用全局默认
 * @retval 1=成功, 0=失败(半径不一致或弧长为0)
 * @note   使用 S 曲线控制沿弧线方向的进给速度，每周期重新计算 XY 坐标
 */
int Interp_ArcXY(int32 cx, int32 cy, int32 ex, int32 ey,
                 uint8 dir, int32 feedrate)
{
    /* ── ① 计算起点半径 ── */
    int64 dx_s = axis[0].actual_pos - cx;
    int64 dy_s = axis[1].actual_pos - cy;
    int64 r_sq = dx_s * dx_s + dy_s * dy_s;
    int32 radius = (int32)sqrt((double)r_sq);
    if (radius <= 0) return 0;

    /* ── ② 校验终点半径 ── */
    int64 dx_e = ex - cx;
    int64 dy_e = ey - cy;
    int64 r_e_sq = dx_e * dx_e + dy_e * dy_e;
    int32 radius_e = (int32)sqrt((double)r_e_sq);
    int32 r_diff = (radius_e > radius) ? (radius_e - radius)
                                       : (radius - radius_e);
    int32 r_tol = radius / 200;
    if (r_tol < 50) r_tol = 50;
    if (r_diff > r_tol) {
        #if printf_cmd
        printf("ARC ERR: radius mismatch (start=%ld end=%ld diff=%ld tol=%ld)\r\n",
               (long)radius, (long)radius_e, (long)r_diff, (long)r_tol);
        #endif
        return 0;
    }

    /* ── ③ 计算起止角度与扫掠角 (有符号: +CCW, -CW) ── */
    double start_angle = atan2((double)dy_s, (double)dx_s);
    double end_angle   = atan2((double)dy_e, (double)dx_e);
    double sweep = end_angle - start_angle;
    if (dir == 0) { /* CW: 角度减小 → sweep 必须 ≤ 0 */
        if (sweep > 0.0) sweep -= 2.0 * 3.141592653589793;
        if (sweep == 0.0) sweep = -2.0 * 3.141592653589793;  /* 全圆 */
    } else { /* CCW: 角度增大 → sweep 必须 ≥ 0 */
        if (sweep < 0.0) sweep += 2.0 * 3.141592653589793;
        if (sweep == 0.0) sweep =  2.0 * 3.141592653589793;  /* 全圆 */
    }

    /* ── ④ 计算弧长 (用 |sweep|) ── */
    double abs_sweep = (sweep >= 0.0) ? sweep : -sweep;
    int32 arc_len = (int32)((double)radius * abs_sweep);
    if (arc_len <= 0) return 0;

    /* ── ⑤ 中止之前的圆弧, 初始化全局状态 ── */
    Arc_Abort();

    g_arc.active      = 1;
    g_arc.cx          = cx;
    g_arc.cy          = cy;
    g_arc.ex          = ex;
    g_arc.ey          = ey;
    g_arc.radius_d    = (double)radius;
    g_arc.start_angle = start_angle;
    g_arc.sweep       = sweep;
    g_arc.arc_len     = arc_len;

    /* ── ⑥ 计算 S 曲线速度参数 (与 Interp_Line 保持一致) ── */
    int32 base_step = (feedrate > 0) ? RPM_to_CycleVel(feedrate, 0)
                      : RPM_to_CycleVel(g_move_rpm, 0);
    int32 max_acc, jerk;
    SCurve_AutoAccJerk(base_step, 0, &max_acc, &jerk);

    /* ── ⑦ 规划 S 曲线: 从 0 → arc_len, 控制沿弧线的进给速度 ── */
    SCurve_Plan(&g_arc.splanner, 0, arc_len, base_step, max_acc, jerk);

    /* ── ⑧ 标记 XY 轴进入圆弧模式 (ARC 模式: 不由 SCurve_Step 驱动) ── */
    axis[0].mode        = AXIS_MODE_ARC;
    axis[1].mode        = AXIS_MODE_ARC;
    axis[0].motion_busy = 1;
    axis[1].motion_busy = 1;

    return 1;
}

/* ── 每周期由 vEtherCAT_Task 调用: 从 S 曲线计算角度→XY→写 PDO ── */
/**
 * @brief  圆弧插补每周期步进处理 (由 EtherCAT 任务调用)
 * @note   根据 S 曲线当前进度计算角度，转换成 XY 坐标输出到 PDO
 */
void Arc_ProcessStep(void)
{
    if (!g_arc.active) return;

    /* 步进弧长方向的 S 曲线 */
    int32 s = SCurve_Step(&g_arc.splanner);

    /* 弧长 → 角度 (线性映射) */
    double angle = g_arc.start_angle
                 + ((double)s / (double)g_arc.arc_len) * g_arc.sweep;

    /* 角度 → XY 坐标 */
    int32 nx = g_arc.cx + (int32)(g_arc.radius_d * cos(angle));
    int32 ny = g_arc.cy + (int32)(g_arc.radius_d * sin(angle));

    /* 写入目标位置 (下一帧 PDO 发送) */
    axis[0].target_pos = nx;
    axis[1].target_pos = ny;
    axis[0].pdo_out->TargetPos = nx;
    axis[1].pdo_out->TargetPos = ny;

    /* Z/R 轴保持原位 */
    axis[2].target_pos = axis[2].actual_pos;
    axis[3].target_pos = axis[3].actual_pos;

    /* ── 各轴实际速度 (从弧线几何微分计算) ──
     *   dθ/dt = sweep / arc_len × current_vel (rad/周期)
     *   Vx = -r·sin(θ)·dθ/dt,  Vy = r·cos(θ)·dθ/dt     */
    {
        double dtheta_dt = g_arc.sweep * (double)g_arc.splanner.current_vel
                         / (double)g_arc.arc_len;
        int32 vx = (int32)(-g_arc.radius_d * sin(angle) * dtheta_dt);
        int32 vy = (int32)( g_arc.radius_d * cos(angle) * dtheta_dt);
        axis[0].velocity  = (vx >= 0) ?  vx : -vx;
        axis[1].velocity  = (vy >= 0) ?  vy : -vy;
        axis[0].direction = (vx >= 0) ? 1 : -1;
        axis[1].direction = (vy >= 0) ? 1 : -1;
    }

    /* ── 完成检查 ── */
    if (SCurve_IsDone(&g_arc.splanner)) {
        /* 精确对齐到终点 */
        axis[0].target_pos = g_arc.ex;
        axis[1].target_pos = g_arc.ey;
        axis[0].pdo_out->TargetPos = g_arc.ex;
        axis[1].pdo_out->TargetPos = g_arc.ey;

        axis[0].velocity    = 0;
        axis[1].velocity    = 0;
        axis[0].motion_busy = 0;
        axis[1].motion_busy = 0;
        axis[0].mode        = AXIS_MODE_HOLD;
        axis[1].mode        = AXIS_MODE_HOLD;

        g_arc.active = 0;
    }
}

/**
 * @brief  中止当前圆弧运动，清理状态
 */
void Arc_Abort(void)
{
    if (!g_arc.active) return;
    g_arc.active = 0;

    /* 将 XY 轴切回 HOLD 模式 */
    if (axis[0].mode == AXIS_MODE_ARC) {
        axis[0].mode        = AXIS_MODE_HOLD;
        axis[0].velocity    = 0;
        axis[0].motion_busy = 0;
    }
    if (axis[1].mode == AXIS_MODE_ARC) {
        axis[1].mode        = AXIS_MODE_HOLD;
        axis[1].velocity    = 0;
        axis[1].motion_busy = 0;
    }
}

/**
 * @brief  查询是否有圆弧正在运行
 * @retval 1=圆弧运行中, 0=空闲
 */
int Arc_IsActive(void)
{
    return g_arc.active;
}

/* ══════════════════════════════════════════════════════════════════
 * L5: 运动队列
 * ══════════════════════════════════════════════════════════════════ */

static MotionSegment_t motion_queue[SEGMENT_QUEUE_SIZE];
static int motion_queue_head = 0;
static int motion_queue_tail = 0;
static int motion_queue_count = 0;
static int motion_paused = 0;

/**
 * @brief  入队一个直线段到运动队列
 * @param  targets  各轴目标位置 [X,Y,Z,R]
 * @param  feedrate 该段进给速度
 * @retval 1=入队成功, 0=队列满
 */
int Motion_QueueSegment(int32 *targets, int32 feedrate)
{
    if (motion_queue_count >= SEGMENT_QUEUE_SIZE)
        return 0;  /* 队列满 */

    MotionSegment_t *seg = &motion_queue[motion_queue_tail];
    for (int i = 0; i < MAX_AXES; i++)
        seg->pos[i] = targets[i];
    seg->feedrate = feedrate;
    seg->type = 0;  /* 直线 */

    motion_queue_tail = (motion_queue_tail + 1) % SEGMENT_QUEUE_SIZE;
    motion_queue_count++;
    return 1;
}

/**
 * @brief  清空运动队列，复位队列状态
 * @note   同时清暂停标志
 */
void Motion_FlushQueue(void)
{
    motion_queue_head = 0;
    motion_queue_tail = 0;
    motion_queue_count = 0;
    motion_paused = 0;
}

/**
 * @brief  暂停运动队列执行
 */
void Motion_Pause(void)   { motion_paused = 1; }

/**
 * @brief  恢复运动队列执行
 */
void Motion_Resume(void)  { motion_paused = 0; }

/**
 * @brief  中止所有运动：清空队列 + 中止圆弧 + 停止所有轴
 */
void Motion_Abort(void)
{
    Motion_FlushQueue();
    Arc_Abort();  /* 中止正在运行的圆弧 */
    for (int ax = 0; ax < NUM_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        Axis_StopOne(ax);  /* 统一由 Axis_StopOne 处理 S曲线停止 / 简单停止 */
    }
}

/**
 * @brief  获取队列中待处理段数
 * @retval 队列中剩余段数
 */
int Motion_QueueCount(void) { return motion_queue_count; }

/* ── 每周期从队列弹出段并启动插补 ──
 *   由主循环在 pdoTimeFlag 处理中调用
 */
/**
 * @brief  运动队列处理 - 每周期调用，弹出已完成段后启动下一段
 * @note   只有当所有轴都完成当前段且圆弧空闲时才派发下一段
 */
void Motion_ProcessQueue(void)
{
    if (motion_paused || motion_queue_count == 0) return;

    /* 圆弧运行中: 不派发队列段 (由 Arc_ProcessStep 驱动) */
    if (g_arc.active) return;

    /* 检查当前运动是否完成 */
    int all_done = 1;
    for (int ax = 0; ax < NUM_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        if (axis[ax].motion_busy && !SCurve_IsDone(&axis[ax].splanner))
            { all_done = 0; break; }
    }

    if (!all_done) return;

    /* 弹出下一段 */
    MotionSegment_t *seg = &motion_queue[motion_queue_head];
    Interp_Line(seg->pos, seg->feedrate);

    motion_queue_head = (motion_queue_head + 1) % SEGMENT_QUEUE_SIZE;
    motion_queue_count--;
}

/* ══════════════════════════════════════════════════════════════════
 * L6: 安全停止
 * ══════════════════════════════════════════════════════════════════ */
/**
 * @brief  快速停止指定轴 (CiA 402 Quick Stop)
 * @param  ax  轴索引
 * @retval 1=成功, 0=失败
 */
int Axis_QuickStop(int ax)
{
    if (ax < 0 || ax >= NUM_AXES)     return 0;
    if (ax + 1 > ec_slavecount)       return 0;

    /* CiA 402 Quick Stop: ControlWord = 0x000B */
    axis[ax].pdo_out->ControlWord = 0x000B;
    axis[ax].mode      = AXIS_MODE_HOLD;
    axis[ax].direction = 0;
    axis[ax].motion_busy = 0;
    axis[ax].cmd_target_pos = axis[ax].actual_pos;

    #if printf_cmd
    printf("%s: QUICK STOP\r\n", axis_name[ax]);
    #endif
    return 1;
}

/**
 * @brief  紧急停止所有轴
 * @retval 总是返回1
 */
int Axis_EStopAll(void)
{
    Motion_Abort();

    for (int ax = 0; ax < NUM_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        axis[ax].pdo_out->ControlWord = 0x0000;  /* Disable Voltage */
        axis[ax].enabled    = 0;
        axis[ax].mode       = AXIS_MODE_HOLD;
        axis[ax].direction  = 0;
        axis[ax].velocity   = 0;
        axis[ax].motion_busy = 0;
        axis[ax].cmd_target_pos = axis[ax].actual_pos;
        axis[ax].pdo_out->TargetPos = axis[ax].actual_pos;
    }

    #if printf_cmd
    printf(">>> EMERGENCY STOP ALL AXES <<<\r\n");
    #endif
    return 1;
}

/**
 * @brief  紧急停止单个轴
 * @param  ax  轴索引
 * @retval 1=成功, 0=失败
 * @note   关使能，直接断电
 */
int Axis_EStop(int ax)
{
    if (ax < 0 || ax >= NUM_AXES)     return 0;
    if (ax + 1 > ec_slavecount)       return 0;

    axis[ax].pdo_out->ControlWord = 0x0000;
    axis[ax].enabled    = 0;
    axis[ax].mode       = AXIS_MODE_HOLD;
    axis[ax].direction  = 0;
    axis[ax].velocity   = 0;
    axis[ax].motion_busy = 0;
    axis[ax].cmd_target_pos = axis[ax].actual_pos;
    axis[ax].pdo_out->TargetPos = axis[ax].actual_pos;

    #if printf_cmd
    printf("%s: ESTOP\r\n", axis_name[ax]);
    #endif
    return 1;
}

/* ══════════════════════════════════════════════════════════════════
   轴控核心函数 
   ══════════════════════════════════════════════════════════════════ */

   /**
 * @brief  获取轴开机回零目标位置 (编译时常数)
 * @param  ax  轴索引
 * @retval 回零目标位置
 */
static int32 Axis_GetHomePos(int ax)
{
    static const int32 home_pos[MAX_AXES] = {
        AXIS_HOME_POS_X, AXIS_HOME_POS_Y, AXIS_HOME_POS_Z, AXIS_HOME_POS_R,
        0, 0, 0, 0   /* U/V/W/S */
    };
    if (ax < 0 || ax >= MAX_AXES) return 0;
    return home_pos[ax];
}

/**
 * @brief  设置轴定位速度 (RPM)，并重新计算加速度
 * @param  ax   轴索引
 * @param  rpm  速度 RPM
 * @note   加速度按 pos_step / 25 估算
 */
void Axis_SetMoveSpeed(int ax, int rpm)
{
    axis[ax].pos_step = RPM_to_CycleVel(rpm, ax);
    axis[ax].accel    = axis[ax].pos_step / 25;
}

/**
 * @brief  更新全局回零活跃标志，检测是否有轴还在回零中
 */
static void Axis_UpdateHomingActive(void)
{
    homing_active = 0;
    for (int ax = 0; ax < NUM_AXES; ax++) {
        if (axis[ax].homing) {
            homing_active = 1;
            return;
        }
    }
}

/**
 * @brief  取消指定轴的回零状态
 * @param  ax  轴索引
 */
void Axis_CancelHoming(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return;
    if (axis[ax].homing) {
        axis[ax].homing = 0;
        axis[ax].homing_phase = 0;
        Axis_UpdateHomingActive();
    }
}

/**
 * @brief  平滑停止单个轴 (S曲线减速到零)
 * @param  ax  轴索引
 * @note   - 取消回零
 *         - 如果启用平滑停止，走完整 S 曲线减速
 *         - 同步目标位置到编码器实际位置，避免跟随误差
 */
void Axis_StopOne(int ax)
{
    if (ax < 0 || ax >= NUM_AXES) return;
    Axis_CancelHoming(ax);

    /* 停止时取消所有待重启标志 */
    axis[ax].jog_pending_rpm = 0;

    /* 停止前从编码器同步 target_pos，防止 VEL 模式软件累加值偏离实际位置，
     * 导致 TargetPos 冻结在旧值 → 伺服强行拽回 → 跟随误差报警 */
    axis[ax].target_pos = axis[ax].pdo_in->CurrentPosition;

    int smooth_stop_en = Param_Get(PARAM_SMOOTH_STOP_EN);

    int32 cur_vel = 0;
    int8  cur_dir = 1;
    int   splanner_active = 0;

    if (axis[ax].splanner.segment != SCURVE_DONE &&
        axis[ax].splanner.segment != SCURVE_IDLE &&
        axis[ax].splanner.current_vel > 0) {
        cur_vel = axis[ax].splanner.current_vel;
        cur_dir = axis[ax].splanner.direction;
        splanner_active = 1;
    } else if (axis[ax].velocity > 0) {
        cur_vel = axis[ax].velocity;
        cur_dir = axis[ax].direction;
        if (cur_dir == 0) cur_dir = 1;
        splanner_active = 0;
    }

    if (smooth_stop_en && cur_vel > 0) {
        int32 max_vel = axis[ax].pos_step;
        /* 使用固定 accel/jerk，与 Axis_ApplyVelocity 启停参数一致，
         * 避免 SCurve_AutoAccJerk 在典型 RPM 下算出 jerk=1 → 减速过慢 → 跟随误差 */
        int32 max_acc = max_vel / 80;
        if (max_acc < 50) max_acc = 50;
        int32 jerk = max_acc / 12;
        if (jerk < 1) jerk = 1;

        if (!splanner_active) {
            axis[ax].splanner.current_pos = axis[ax].target_pos;
            axis[ax].splanner.current_vel = cur_vel;
            axis[ax].splanner.current_acc = 0;
            axis[ax].splanner.direction   = cur_dir;
            axis[ax].splanner.start_pos   = axis[ax].target_pos;
            axis[ax].splanner.segment     = SCURVE_CONST_VEL;
        }

        axis[ax].splanner.max_vel = max_vel;
        axis[ax].splanner.max_acc = max_acc;
        axis[ax].splanner.jerk    = jerk;
        SCurve_Stop(&axis[ax].splanner);

        axis[ax].mode = AXIS_MODE_POS;
        axis[ax].motion_busy = 1;
        axis[ax].direction = 0;
        #if printf_cmd
        printf(">>> %s: SMOOTH STOP (S-Curve) v=%ld RPM <<<\r\n",
               axis_name[ax], (long)(CycleVel_to_RPM(cur_vel, ax)));
        #endif
    } else {
        /* 简单停止: 规划器活跃时仍走 S 曲线减速, 防止 POS→VEL 模式切换引入位置跳变 */
        if (splanner_active) {
            /* 规划器确实在运行 (cur_vel > 0 且 segment != DONE/IDLE):
             * S曲线减速到零 */
            int32 max_acc = axis[ax].pos_step / 80;
            if (max_acc < 50) max_acc = 50;
            int32 jerk = max_acc / 12;
            if (jerk < 1) jerk = 1;
            axis[ax].splanner.max_acc = max_acc;
            axis[ax].splanner.jerk    = jerk;
            SCurve_Stop(&axis[ax].splanner);
            axis[ax].mode = AXIS_MODE_POS;
            axis[ax].motion_busy = 1;
            axis[ax].direction = 0;
        } else if (axis[ax].mode == AXIS_MODE_POS) {
            /* POS模式但planner已停止(DONE/IDLE): 无需减速,
             * 直接清理状态。不设 motion_busy=1, 防止 SCurve_Stop()
             * 对DONE planner直接返回后 motion_busy 永久卡在1 */
            axis[ax].direction = 0;
            axis[ax].velocity = 0;
            axis[ax].motion_busy = 0;
            /* 保持 POS 模式, 避免模式切换引入位置跳变 */
        } else {
            axis[ax].direction = 0;
            axis[ax].mode = AXIS_MODE_VEL;
            axis[ax].cmd_target_pos = axis[ax].target_pos;
        }
        #if printf_cmd
        printf(">>> %s: STOP <<<\r\n", axis_name[ax]);
        #endif
    }
}

/**
 * @brief  速度控制 - 让轴以指定转速持续转动
 * @param  ax   轴索引
 * @param  rpm  转速大小 (正值，RPM)
 * @param  sign 方向: 1=正转, -1=反转
 * @note   rpm=0 则平滑停止；方向反转时先减速再反向启动
 */
void Axis_ApplyVelocity(int ax, int rpm, int sign)
{
    if (ax < 0 || ax >= NUM_AXES) return;

    Axis_CancelHoming(ax);

    if (rpm == 0) {
        /* 停止: 清除待重启标志, 走 S 曲线减速 */
        axis[ax].jog_pending_rpm = 0;
        if (axis[ax].splanner.segment != SCURVE_DONE &&
            axis[ax].splanner.segment != SCURVE_IDLE &&
            axis[ax].splanner.current_vel > 0) {
            SCurve_Stop(&axis[ax].splanner);
        }
        axis[ax].mode        = AXIS_MODE_POS;
        axis[ax].motion_busy = 1;
        axis[ax].direction   = 0;
        return;
    }

    if (rpm > 3000) rpm = 3000;

    int32 dir = (sign >= 0) ? 1 : -1;

    /* 方向反转检测: 规划器运行中且方向相反 → 先减速到零再自动反向 */
    if (axis[ax].direction != 0 && axis[ax].direction != dir &&
        axis[ax].splanner.segment != SCURVE_DONE &&
        axis[ax].splanner.segment != SCURVE_IDLE &&
        axis[ax].splanner.current_vel > 0) {
        SCurve_Stop(&axis[ax].splanner);
        axis[ax].jog_pending_rpm = dir * rpm;   /* 存待重启速度(含方向) */
        axis[ax].mode        = AXIS_MODE_POS;
        axis[ax].motion_busy = 1;
        axis[ax].direction   = 0;                /* 清方向, 防重复触发 */
        return;
    }

    int32 pos_step = RPM_to_CycleVel(rpm, ax);
    int32 max_acc  = pos_step / 80;
    if (max_acc < 50) max_acc = 50;
    int32 jerk = max_acc / 12;
    if (jerk < 1) jerk = 1;

    int32 start  = axis[ax].pdo_in->CurrentPosition;
    /* 远距离目标(约27秒巡航@200RPM, ~536mm), 防溢出 */
    int32 target;
    if (dir > 0) {
        target = (start > INT32_MAX - 900000000) ? INT32_MAX : start + 900000000;
    } else {
        target = (start < INT32_MIN + 900000000) ? INT32_MIN : start - 900000000;
    }

    SCurve_Plan(&axis[ax].splanner, start, target, pos_step, max_acc, jerk);
    axis[ax].pos_step    = pos_step;
    axis[ax].mode        = AXIS_MODE_POS;
    axis[ax].motion_busy = 1;
    axis[ax].direction   = (int8)sign;
    axis[ax].jog_pending_rpm = 0;                /* 正常启动, 清除待重启 */

    #if printf_cmd
    printf("%s: %d RPM %s (S-Curve)\r\n", axis_name[ax], rpm,
           sign > 0 ? "FWD" : "REV");
    #endif
}

/**
 * @brief  位置控制 - 走到指定绝对位置
 * @param  ax   轴索引
 * @param  pos  目标位置 (脉冲)
 * @note   R轴自动做旋转归一化，走最短路径；自动 S 曲线规划
 */
void Axis_ApplyPosition(int ax, int32 pos)
{
    if (ax < 0 || ax >= NUM_AXES) return;

    Axis_CancelHoming(ax);

    /* ── R轴旋转轴: 目标归一化到 [0, 360°) + 最短旋转路径 ── */
    if (ax == 3) 
    {
        int32 ppr = axis[ax].effective_ppr;
        if (ppr <= 0) ppr = GetAxisPPR(ax);

        /* 目标角度归一化到 [0, ppr) */
        int32 target_angle = pos % ppr;
        if (target_angle < 0) target_angle += ppr;

        /* 当前角度归一化 */
        int32 cur_angle = axis[ax].actual_pos % ppr;
        if (cur_angle < 0) cur_angle += ppr;

        /* 最短旋转路径: delta ∈ (-half, +half] */
        int32 delta = target_angle - cur_angle;
        int32 half = ppr / 2;
        if (delta > half) delta -= ppr;
        else if (delta < -half) delta += ppr;

        pos = axis[ax].actual_pos + delta;
    }

    axis[ax].mode = AXIS_MODE_POS;
    axis[ax].cmd_target_pos = pos;
    Axis_SetMoveSpeed(ax, g_move_rpm);
    axis[ax].motion_busy = 1;

    {
        int32 max_vel = axis[ax].pos_step;
        int32 acc_cap = Param_Get(AXIS_PARAM(PARAM_AXIS_MAX_ACCEL, ax));
        int32 max_acc, jerk;
        SCurve_AutoAccJerk(max_vel, acc_cap, &max_acc, &jerk);

        /* R轴短行程 → 梯形曲线: Jerk=Amax → Tj=1, S曲线退化为梯形 */
        if (ax == 3) {
            jerk = max_acc;
        }

        SCurve_Plan(&axis[ax].splanner,
                    axis[ax].actual_pos, pos,
                    max_vel, max_acc, jerk);
    }

    #if printf_cmd
    printf("%s: GOTO pos=%d (%d RPM) %s\r\n",
           axis_name[ax], (int)pos, g_move_rpm,
           (ax == 3) ? "Trapezoid" : "S-Curve");
    #endif
}

/**
 * @brief  启动多轴开机回零到原点位置（目前未调用，可以实现回到定义的原点，在main.h中AXIS_HOME_POS_X/Y/Z/R）
 */
static void Axis_StartHoming(void)
{
    homing_active = 1;
    #if printf_cmd
    printf("Homing to init position (%d RPM)...\r\n", g_home_rpm);
    #endif

    for (int ax = 0; ax < NUM_AXES; ax++) {
        axis[ax].mode = AXIS_MODE_POS;
        axis[ax].cmd_target_pos = Axis_GetHomePos(ax);
        axis[ax].homing = 1;
        axis[ax].direction = 0;
        Axis_SetMoveSpeed(ax, g_home_rpm);
    }
}

/**
 * @brief  检查轴是否已经到达目标位置
 * @param  ax  轴索引
 * @retval 1=到位且速度为零, 0=未到位
 */
static int Axis_IsAtTarget(int ax)
{
    int32 err = axis[ax].cmd_target_pos - axis[ax].actual_pos;
    if (err < 0) err = -err;
    return (err <= POS_ARRIVE_THRESH) && (axis[ax].velocity == 0);
}

/**
 * @brief  回零完成检查 - 分段回零处理，每周期调用
 * @note   处理两段式回零：快速逼近 → 蠕动精停，所有轴到位后结束回零
 */
void Axis_CheckHomingDone(void)
{
    if (!homing_active) return;

    /* 两段式回零: 第一段(快速逼近)完成后启动第二段(蠕动精停) */
    for (int ax = 0; ax < NUM_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        if (!axis[ax].homing) continue;
        if (axis[ax].homing_phase == 1 && Axis_IsAtTarget(ax)) {
            int32 creep = axis[ax].homing_creep_rpm;
            if (creep <= 0) creep = 5;

            axis[ax].homing_phase = 2;
            axis[ax].cmd_target_pos = 0;
            Axis_SetMoveSpeed(ax, creep);
            int32 max_vel = axis[ax].pos_step;
            int32 acc_cap = Param_Get(AXIS_PARAM(PARAM_AXIS_MAX_ACCEL, ax));
            int32 max_acc, jerk;
            SCurve_AutoAccJerk(max_vel, acc_cap, &max_acc, &jerk);
            SCurve_Plan(&axis[ax].splanner, axis[ax].actual_pos, 0,
                        max_vel, max_acc, jerk);
            /* 消除阶段切换时的目标位置跳变:
             * SCurve_Plan 用 actual_pos 重算了 current_pos,
             * 但 target_pos 仍是 phase 1 旧值 (相差可达数百脉冲),
             * 伺服在下一帧收到跳变 → 过冲震动 → 同步到 actual_pos 保持连续 */
            axis[ax].target_pos = axis[ax].actual_pos;
            Axis_WriteTargetPos(ax, axis[ax].target_pos);
        }
    }

    for (int ax = 0; ax < NUM_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;
        if (!axis[ax].homing) continue;
        if (!Axis_IsAtTarget(ax)) return;
    }

    homing_active = 0;
    for (int ax = 0; ax < NUM_AXES; ax++) {
        if (axis[ax].homing) {
            axis[ax].homing = 0;
            axis[ax].homing_phase = 0;
            if (axis[ax].mode == AXIS_MODE_POS)
                Axis_SetMoveSpeed(ax, g_move_rpm);
        }
    }

    #if printf_cmd
    printf("\r\n=== Homing Done ===\r\n");
    #endif
    #if printf_cmd
    printf("SPEED:N | SPEED:v1,v2,v3,v4 | POS:v1,v2,v3,v4 | NUM:v1,v2,v3,v4\r\n");
    #endif
    #if printf_cmd
    printf("X:RPM | POS-X:pos | NUM-X:+/-rev | ZERO | ZERO:N | STOP | RESET\r\n");
    #endif
    #if printf_cmd
    printf("Move speed: %d RPM\r\n", g_move_rpm);
    #endif
}

/* ================================================================
   Axis_Init() — 单轴初始化
   ================================================================ */
void Axis_Init(int ax, uint16 slave)
{
    axis[ax].slave_idx   = slave;
    axis[ax].pdo_out     = (PDO_Output *)ec_slave[slave].outputs;
    axis[ax].pdo_in      = (PDO_Input  *)ec_slave[slave].inputs;
    axis[ax].enabled     = 0;
    axis[ax].velocity    = 0;
    axis[ax].direction   = 0;
    axis[ax].mode        = AXIS_MODE_HOLD;
    Axis_SetMoveSpeed(ax, g_move_rpm);
    axis[ax].target_pos     = 0;
    axis[ax].cmd_target_pos = 0;
    axis[ax].actual_pos     = 0;
    axis[ax].pdo_out->ControlWord = 0x0000;
    axis[ax].pdo_out->TargetMode  = 8;
    axis[ax].status_word    = 0;
    axis[ax].homing         = 0;
    axis[ax].homing_phase   = 0;
    axis[ax].homing_creep_rpm = 0;
    axis[ax].fault_reset_step = 0;
    axis[ax].zero_step      = 0;
    SCurve_Init(&axis[ax].splanner);
    axis[ax].dda_master     = -1;
    axis[ax].motion_busy    = 0;
    axis[ax].soft_limit_plus  = Param_Get(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, ax));
    axis[ax].soft_limit_minus = Param_Get(AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, ax));
    axis[ax].backlash_comp    = Param_Get(AXIS_PARAM(PARAM_AXIS_BACKLASH, ax));
    axis[ax].landing_step     = 0;
    axis[ax].effective_ppr    = GetAxisPPR(ax);  /* 默认1:1, RefreshAxisGearRatio 中更新 */
}

/**
 * @brief  所有轴目标位置同步到编码器当前值
 * @note   开机后调用，保证 TargetPos 与实际位置对齐
 */
void Axis_SyncAllTargetsFromEncoder(void)
{
    for (int ax = 0; ax < NUM_AXES; ax++)
    {
        if (ax + 1 > ec_slavecount) continue;

        int32 cur = axis[ax].pdo_in->CurrentPosition;
        axis[ax].actual_pos     = cur;
        axis[ax].target_pos     = cur;
        axis[ax].cmd_target_pos = cur;
        axis[ax].pdo_out->TargetPos = cur;
        axis[ax].pdo_out->TargetMode  = 8;
    }
}

/**
 * @brief  批量使能所有存在的轴
 * @retval 全部成功返回1，任意失败返回0
 * @note   执行 CiA 402 标准使能序列
 */
int Axis_EnableAll(void)
{
    uint16 ctrl_seq[] = {0x0006, 0x0007, 0x000F};
    //const char *step_name[] = {"Shutdown(0x06)", "SwitchOn(0x07)", "EnableOp(0x0F)"};

    for (int step = 0; step < 3; step++)
    {
        for (int ax = 0; ax < NUM_AXES; ax++)
        {
            if (ax + 1 > ec_slavecount) continue;
            /* 每步使能前同步目标位置, 防止 CSP 模式跳变 */
            int32 cur = axis[ax].pdo_in->CurrentPosition;
            axis[ax].pdo_out->TargetPos = cur;
            axis[ax].pdo_out->ControlWord = ctrl_seq[step];
            axis[ax].pdo_out->TargetMode  = 8;
        }

        HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

        HAL_Delay(30);

        ec_receive_processdata(EC_TIMEOUTRET);

        int all_ok = 1;
        for (int ax = 0; ax < NUM_AXES; ax++)
        {
            axis[ax].status_word = axis[ax].pdo_in->StatusWord;
            if (axis[ax].status_word == 0) { all_ok = 0; continue; }
            #if printf_cmd
            printf("  %s step%d: StatusWord=0x%04x\r\n",
                   axis_name[ax], step + 1, axis[ax].status_word);
            #endif
        }
        if (!all_ok) {
            #if printf_cmd
            printf("  WARNING: some axes not responding at step %d\r\n", step + 1);
            #endif
        }

        /* 最后一步 EnableOp(0x0F): 额外加一个确认周期
         * SV660N 在 CSP 模式下有时需要连续两帧 0x0F 才进入 Operation Enabled */
        if (step == 2)
        {
            HAL_Delay(50);
            HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
            ec_send_processdata();                     // 再发一帧 0x0F
            ec_receive_processdata(EC_TIMEOUTRET);
            HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
            HAL_Delay(30);
            ec_receive_processdata(EC_TIMEOUTRET);
        }
    }

    int all_enabled = 1;
    for (int ax = 0; ax < NUM_AXES; ax++)
    {
        axis[ax].status_word = axis[ax].pdo_in->StatusWord;
        if ((axis[ax].status_word & 0x006F) == 0x0027)
        {
            axis[ax].enabled    = 1;
            axis[ax].actual_pos = axis[ax].pdo_in->CurrentPosition;
            axis[ax].target_pos = axis[ax].actual_pos;
            axis[ax].cmd_target_pos = axis[ax].actual_pos;
            axis[ax].pdo_out->TargetPos = axis[ax].target_pos;
            #if printf_cmd
            printf("%s: ENABLED (pos=%d)\r\n", axis_name[ax], (int)axis[ax].target_pos);
            #endif
        }
        else
        {
            all_enabled = 0;
            #if printf_cmd
            printf("%s: enable FAILED StatusWord=0x%04x, force-enabling\r\n",
                   axis_name[ax], axis[ax].status_word);
            #endif
            axis[ax].enabled    = 1;
            axis[ax].actual_pos = axis[ax].pdo_in->CurrentPosition;
            axis[ax].target_pos = axis[ax].actual_pos;
            axis[ax].cmd_target_pos = axis[ax].actual_pos;
            axis[ax].pdo_out->TargetPos = axis[ax].target_pos;
        }
    }

    return all_enabled;
}

/**
 * @brief  单轴控制周期 - 每 EtherCAT 周期调用一次
 * @param  ax  轴索引
 * @note   根据轴模式处理：
 *         - POS 位置模式: S 曲线步进，更新目标位置
 *         - VEL 速度模式: 速度模式累加位置
 *         - DDA 插补跟随: 按主轴进度比例跟随
 *         - DDA 圆弧: 不在这里处理，由 Arc_ProcessStep 处理
 *         最后更新实际位置、检查软限位
 */
void Axis_ControlCycle(int ax)
{
    if (axis[ax].enabled == 0) return;

    axis[ax].actual_pos  = axis[ax].pdo_in->CurrentPosition;
    axis[ax].status_word = axis[ax].pdo_in->StatusWord;

    /* DEBUG: 每秒打印一次 PDO 反馈位置 (200周期 × 5ms = 1s) */
    {
        static uint32 dbg_tick = 0;
        if (++dbg_tick >= 200) {
            dbg_tick = 0;
            #if printf_cmd
            printf("%c: rawPDO=%ld enc=%ld target=%ld mode=%d\r\n",
                   "XYZRUVWS"[ax],
                   (long)axis[ax].pdo_in->CurrentPosition,
                   (long)axis[ax].actual_pos,
                   (long)axis[ax].target_pos,
                   (int)axis[ax].mode);
            #endif
        }
    }

    if (axis[ax].status_word & 0x0008)
    {
        /* 只在故障首次出现时打印，防止每周期重复打印导致串口洪水
         * (4轴 × 12行 × 每5ms = CPU被vEtherCAT_Task独占，vCmd_Task饿死) */
        if (!axis[ax].fault_reported) {
            axis[ax].fault_reported = 1;
            uint16 err_code = axis[ax].pdo_in->ErrorCode;
            #if printf_cmd
            printf("%s: FAULT! StatusWord=0x%04x ErrorCode=0x%04x\r\n",
                   axis_name[ax], axis[ax].status_word, err_code);
            #endif

            switch (err_code) {
                #if printf_cmd
                case 0x0000: printf("  -> no error (returned to 0 after reset)\r\n"); break;
                #endif
                #if printf_cmd
                case 0x1000: printf("  -> General error\r\n"); break;
                #endif
                #if printf_cmd
                case 0x2300: printf("  -> Overcurrent\r\n"); break;
                #endif
                #if printf_cmd
                case 0x3210: printf("  -> Overvoltage\r\n"); break;
                #endif
                #if printf_cmd
                case 0x4210: printf("  -> Undervoltage\r\n"); break;
                #endif
                #if printf_cmd
                case 0x5280: printf("  -> Encoder Error\r\n"); break;
                #endif
                #if printf_cmd
                case 0x5441: printf("  -> Emergency Stop\r\n"); break;
                #endif
                #if printf_cmd
                case 0x7122: printf("  -> Motor Stalled\r\n"); break;
                #endif
                #if printf_cmd
                case 0x8611: printf("  -> Following Error\r\n"); break;
                #endif
                #if printf_cmd
                default:     printf("  -> Unknown fault, see SV660N manual\r\n"); break;
                #endif
            }
            #if printf_cmd
            printf("  -> Send 'RESET' or 'X:RESET' to clear fault\r\n");
            #endif
        }

        axis[ax].mode = AXIS_MODE_HOLD;
        axis[ax].direction = 0;
        axis[ax].cmd_target_pos = axis[ax].actual_pos;
        /* 故障时同步 TargetPos 到实际位置，防止 PDO 输出旧值引起跳变 */
        axis[ax].target_pos = axis[ax].actual_pos;
        axis[ax].velocity   = 0;
        axis[ax].motion_busy = 0;
    }
    else
    {
        /* 故障清除后重置标志，下次再出现故障时可以再次打印 */
        axis[ax].fault_reported = 0;
    }

    if (axis[ax].mode == AXIS_MODE_VEL)
    {
        if (axis[ax].direction == 0)
        {
            if (axis[ax].velocity > 0)
            {
                axis[ax].velocity -= axis[ax].accel;
                if (axis[ax].velocity < 0) axis[ax].velocity = 0;
            }
        }
        else
        {
            if (axis[ax].velocity < axis[ax].pos_step)
            {
                axis[ax].velocity += axis[ax].accel;
                if (axis[ax].velocity > axis[ax].pos_step)
                    axis[ax].velocity = axis[ax].pos_step;
            }
            else if (axis[ax].velocity > axis[ax].pos_step)
            {
                axis[ax].velocity -= axis[ax].accel;
                if (axis[ax].velocity < axis[ax].pos_step)
                    axis[ax].velocity = axis[ax].pos_step;
            }
            axis[ax].target_pos += axis[ax].velocity * axis[ax].direction;
        }
    }
    else
    {
        /* ── HOLD模式: 减速到零 ── */
        if (axis[ax].mode == AXIS_MODE_HOLD)
        {
            if (axis[ax].velocity > 0)
            {
                axis[ax].velocity -= axis[ax].accel;
                if (axis[ax].velocity < 0) axis[ax].velocity = 0;
            }
        }
        /* ── 位置模式 + S曲线 ── */
        else if (axis[ax].mode == AXIS_MODE_POS)
        {
            SCurveSegment_t prev_seg = axis[ax].splanner.segment;

            int32 new_pos = SCurve_Step(&axis[ax].splanner);
            axis[ax].target_pos = new_pos;
            axis[ax].velocity   = axis[ax].splanner.current_vel;

            SCurveSegment_t cur_seg = axis[ax].splanner.segment;


            if (cur_seg == SCURVE_DONE && prev_seg != SCURVE_DONE)
            {
                axis[ax].velocity    = 0;
                axis[ax].motion_busy = 0;

                /* DEBUG: 到位时打印目标位置 vs 实际位置 */
                #if printf_cmd
                printf("  [DBG] %s: DONE  spl_pos=%ld  cmd_target=%ld  actual=%ld\r\n",
                       axis_name[ax],
                       (long)axis[ax].splanner.current_pos,
                       (long)axis[ax].cmd_target_pos,
                       (long)axis[ax].actual_pos);
                #endif

                /* S曲线离散化残余误差修正:
                 * 短行程下Tv修正粒度(max_vel/周期)过大无法收敛,
                 * 到位时强制 target_pos 对齐 cmd_target_pos。
                 * 阈值 = pos_step (1个Tv周期的位移量), 残余总小于此值.
                 * 适用于回零和所有定位模式 */
                {
                    int32 diff = axis[ax].cmd_target_pos - axis[ax].splanner.current_pos;
                    if (diff < 0) diff = -diff;
                    if (diff <= axis[ax].pos_step) {
                        axis[ax].target_pos = axis[ax].cmd_target_pos;
                        axis[ax].splanner.current_pos = axis[ax].cmd_target_pos;
                    }
                }

                /* 方向反转减速到零后自动反向启动 */
                if (axis[ax].jog_pending_rpm != 0) {
                    int32 pending = axis[ax].jog_pending_rpm;
                    int sign = (pending >= 0) ? 1 : -1;
                    int rpm  = (pending >= 0) ? (int)pending : (int)(-pending);
                    Axis_ApplyVelocity(ax, rpm, sign);
                }
                /* 点动巡航到头: 自动续程保持连续运动 */
                else if (axis[ax].direction != 0 && axis[ax].mode == AXIS_MODE_POS) {
                    int32 rpm = CycleVel_to_RPM(axis[ax].pos_step, ax);
                    if (rpm > 0) Axis_ApplyVelocity(ax, rpm, axis[ax].direction);
                }
            }
        }
        /* ── DDA从轴: 跟随主轴S曲线 ──
         * 用四舍五入 + 逐周期增量方式, 替代截断除法 + 末端强制跳变。
         * expected = round(master_progress * slave_abs / master_delta)
         * 每周期只补足不足的脉冲 (to_emit = expected - emitted),
         * 保证从轴脉冲均匀分布在运动全程, 真正实现直线插补。 */
        else if (axis[ax].mode == AXIS_MODE_DDA)
        {
            int master = axis[ax].dda_master;
            if (master < 0 || master >= NUM_AXES) {
                axis[ax].mode = AXIS_MODE_HOLD;
                axis[ax].motion_busy = 0;
            } else {
                int32 master_pos = axis[master].splanner.current_pos;
                int32 master_progress = master_pos - axis[ax].dda_master_start;
                if (master_progress < 0) master_progress = 0;

                int32 slave_dir = (axis[ax].dda_delta >= 0) ? 1 : -1;
                int32 slave_abs = (axis[ax].dda_delta >= 0)
                                ? axis[ax].dda_delta : -axis[ax].dda_delta;

                /* 已发出的从轴脉冲数 */
                int32 emitted = (axis[ax].target_pos - axis[ax].dda_start) * slave_dir;

                /* 理论上应发出的脉冲数 (四舍五入, 误差 ≤0.5 脉冲) */
                int64 num = (int64)master_progress * slave_abs;
                int32 expected = (int32)(
                    (num + (int64)axis[ax].dda_master_delta / 2)
                    / (int64)axis[ax].dda_master_delta);

                /* 逐周期补充不足的脉冲, 每次最多补 expected-emitted 个 */
                while (expected > emitted) {
                    axis[ax].target_pos += slave_dir;
                    emitted++;
                }

                /* 从轴速度 = 主轴速度 × 行程比 (用于监控显示) */
                if (axis[ax].dda_master_delta > 0) {
                    axis[ax].velocity = (int32)(
                        (int64)axis[master].splanner.current_vel * slave_abs
                        / axis[ax].dda_master_delta);
                }

                if (SCurve_IsDone(&axis[master].splanner)) {
                    /* 主轴到位后, 残余 ≤1 脉冲直接收尾 (不上报跟随误差) */
                    if (axis[ax].target_pos != axis[ax].cmd_target_pos) {
                        int32 diff = axis[ax].cmd_target_pos - axis[ax].target_pos;
                        if (diff == 1 || diff == -1) {
                            axis[ax].target_pos = axis[ax].cmd_target_pos;
                        }
                    }
                    axis[ax].velocity   = 0;
                    axis[ax].motion_busy = 0;
                    axis[ax].mode = AXIS_MODE_HOLD;
                }
            }
        }
        /* ── 圆弧模式: target_pos/velocity/motion_busy 由 Arc_ProcessStep 设置 ── */
        else if (axis[ax].mode == AXIS_MODE_ARC)
        {
        }
    }

    Axis_WriteTargetPos(ax, axis[ax].target_pos);
}

/**
 * @brief  写入目标位置到 PDO
 * @param  ax       轴索引
 * @param  pos_enc  目标位置 (编码器单位)
 * @note   单位转换后写入 PDO，供下一次 EtherCAT 发送
 */
void Axis_WriteTargetPos(int ax, int32 pos_enc)
{
    if (ax < 0 || ax >= NUM_AXES || ax + 1 > ec_slavecount) return;
    axis[ax].pdo_out->TargetPos = pos_enc;
}

/* ── S曲线加速度平方根缩放 ──
 *  a = max(K_acc × √v,  S_ACCEL_MIN)
 *  jerk = max(a / S_JERK_CYCLES, S_JERK_MIN)
 *  Tj   = a / jerk
 *
 *  基于 PULSES_PER_REV=50000, SCURVE_STOP_THRESH=4:
 *  √v 在 3000RPM 时仅为 √12000≈110, 永远小于 S_ACCEL_MIN=400
 *  → 实际 a 恒 = S_ACCEL_MIN=400, j 恒 = 1 (除 acc_cap 外)
 *  → Tj 恒 = 400 周期 = 2s, S 曲线加加速度时间固定
 *  → 加速度上限由轴参数 PARAM_AXIS_MAX_ACCEL 控制 */
void SCurve_AutoAccJerk(int32 max_vel, int32 acc_cap,
                                       int32 *acc_out, int32 *jerk_out)
{
    /* a = K * sqrt(v),  cap at acc_cap */
    float kv = (float)S_ACCEL_K_NUM / (float)S_ACCEL_K_DEN;
    int32 a = (int32)(kv * sqrtf((float)max_vel));
    if (a < S_ACCEL_MIN) a = S_ACCEL_MIN;
    if (acc_cap > 0 && a > acc_cap) a = acc_cap;
    int32 j = a / S_JERK_CYCLES;
    if (j < S_JERK_MIN) j = S_JERK_MIN;
    *acc_out = a;
    *jerk_out = j;
}

