/**
 ******************************************************************************
 * @file           : motor_axis.h
 * @brief          : MotorAxis_t 共享结构体定义
 * ----------------------------------------------------------------------------
 * 从 main.c 提取出来, 供 motion_api.c 等文件使用
 ******************************************************************************
 */
#ifndef _MOTOR_AXIS_H_
#define _MOTOR_AXIS_H_

#include "osal.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "motion_profile.h"

/* ── 轴控制模式 ── */
typedef enum {
    AXIS_MODE_HOLD = 0,        /**< 停止/保持 */
    AXIS_MODE_VEL  = 1,        /**< 速度模式: 连续旋转 */
    AXIS_MODE_POS  = 2,        /**< 位置模式: 定位到 cmd_target_pos */
    AXIS_MODE_DDA  = 3,        /**< DDA从轴: 跟随主轴的S曲线轮廓 */
    AXIS_MODE_ARC  = 4,        /**< 圆弧模式: 目标位置由 Arc_ProcessStep 外部设置 */
} AxisMode_t;

/* ── MotorAxis_t — 单轴伺服控制结构体 ── */
typedef struct {
    PDO_Output *pdo_out;       /**< 指向 IOmap 中该轴输出区 */
    PDO_Input  *pdo_in;        /**< 指向 IOmap 中该轴输入区 */
    uint16 slave_idx;          /**< EtherCAT 从站索引号 (1~4) */
    uint8  enabled;            /**< CiA 402 使能标志 */
    int32  target_pos;         /**< 目标位置 (0x607A) */
    int32  cmd_target_pos;     /**< 串口命令目标位置 */
    int32  actual_pos;         /**< 实际位置 (0x6064) */
    int32  velocity;           /**< 当前速度 (cts/周期) */
    int32  pos_step;           /**< 目标最大速度 (cts/周期) */
    int32  accel;              /**< 每周期加速度 */
    uint16 status_word;        /**< 状态字 (0x6041) */
    int8   direction;          /**< 速度模式方向: 0=停止, 1=正转, -1=反转 */
    uint8  mode;               /**< 控制模式: AXIS_MODE_HOLD/VEL/POS */
    uint8  homing;             /**< 1=开机回初始位置中 */
    uint8  fault_reset_step;   /**< 故障复位步序 */
    uint8  fault_reported;     /**< 故障已上报标志: 1=已打印, 0=未打印 (防洪水) */
    uint8  zero_step;          /**< 回零步序 */
    uint8  homing_phase;       /**< 两段式回零: 1=快速逼近 2=蠕动精停 0=无 */
    int32  homing_creep_rpm;   /**< 蠕动速度 RPM */
    /* ── S曲线 (始终启用) ── */
    SCurvePlanner_t splanner;  /**< S曲线7段规划器 (单轴:自身, DDA:主轴共用) */
    uint8  motion_busy;        /**< 1=运动进行中 */
    /* ── DDA 从轴参数 ── */
    int8   dda_master;         /**< DDA: 主轴编号, -1=非从轴 */
    int32  dda_start;          /**< DDA: 起始位置 */
    int32  dda_delta;          /**< DDA: 从轴行程 */
    int32  dda_master_delta;   /**< DDA: 主轴行程 (abs) */
    int32  dda_master_start;   /**< DDA: 主轴起始位置 */
    /* ── 扩展控制参数 ── */
    int32  soft_limit_plus;    /**< 正软限位 */
    int32  soft_limit_minus;   /**< 负软限位 */
    int32  backlash_comp;      /**< 反向间隙补偿 */
    /* ── 终点软着陆 ── */
    int32  landing_step;       /**< 剩余过渡周期数 (0=非着陆) */
    int32  landing_start;      /**< 着陆起始位置 */
    int32  landing_target;     /**< 着陆目标位置 */
    int32  jog_pending_rpm;    /**< 待自动重启点动速度(含符号), 0=无 */
    int32  effective_ppr;       /**< 有效每圈脉冲数 = GetAxisPPR(ax) * gear_num / gear_den (原为 PULSES_PER_REV 硬编码) */
    int32  servo_gear_num;      /**< 伺服驱动器电子齿轮比分子 (0x6091:01) */
    int32  servo_gear_den;      /**< 伺服驱动器电子齿轮比分母 (0x6091:02) */
    uint8  axis_type;           /**< 轴工作模式: 0=直线(50000ppr→mm), 1=旋转(36000ppr→deg)
                                     仅 R/U/V/W/S 有效, X/Y/Z 固定为 0 */
} MotorAxis_t;

/* ── 全局轴数组 (必须在內联函数之前声明) ── */
extern MotorAxis_t axis[MAX_AXES];

/* ══════════════════════════════════════════════════════════════════
   轴模式辅助函数 — PPR 获取 & RPM ↔ 脉冲 换算
   ══════════════════════════════════════════════════════════════════ */

/**
 * @brief  获取指定轴的模式 PPR（每圈脉冲数基准值）
 * @note   X/Y/Z 固定 50000；R/U/V/W/S 根据 axis_type 返回 50000 或 36000
 */
static inline int32 GetAxisPPR(int ax)
{
    if (ax < 0 || ax >= MAX_AXES)
        return PULSES_PER_REV;
    if (ax == 0 || ax == 1 || ax == 2)          /* X/Y/Z 始终直线 */
        return PULSES_PER_REV;
    return (axis[ax].axis_type == 1) ? PULSES_PER_REV_ROTARY : PULSES_PER_REV;
}

/**
 * @brief  RPM → 每周期脉冲数（替代 rpm * POS_STEP_PER_RPM）
 * @note   四舍五入: (abs_rpm * ppr + 6000) / 12000
 *         直线 50000 ppr: 50000/12000=4.17 → "+6000" 后取整 = 4（误差 0.16%）
 *         旋转 36000 ppr: 36000/12000=3.00 → 精确
 */
static inline int32 RPM_to_CycleVel(int32 rpm, int ax)
{
    int32 ppr = GetAxisPPR(ax);
    int32 abs_rpm = (rpm > 0) ? rpm : -rpm;
    int32 step = (int32)(((int64)abs_rpm * (int64)ppr + 6000LL) / 12000LL);
    return (rpm > 0) ? step : -step;
}

/**
 * @brief  每周期脉冲数 → RPM（替代 vel / POS_STEP_PER_RPM）
 * @note   四舍五入: (abs_vel * 12000 + ppr/2) / ppr
 */
static inline int32 CycleVel_to_RPM(int32 vel, int ax)
{
    if (vel == 0) return 0;
    int32 ppr = GetAxisPPR(ax);
    int32 abs_vel = (vel > 0) ? vel : -vel;
    int32 rpm = (int32)(((int64)abs_vel * 12000LL + (int64)ppr / 2) / (int64)ppr);
    return (vel > 0) ? rpm : -rpm;
}

#endif /* _MOTOR_AXIS_H_ */
