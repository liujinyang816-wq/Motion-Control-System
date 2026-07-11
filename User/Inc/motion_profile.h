/**
 ******************************************************************************
 * @file           : motion_profile.h
 * @brief          : S曲线7段加减速规划器接口
 * ----------------------------------------------------------------------------
 * 实现 CiA 402 CSP 模式下基于 Jerk 限制的 S 曲线速度规划
 *
 * 【7段S曲线结构】
 *   加加速(Tj) → 匀加速(Ta) → 减加速(Tj) → 匀速(Tv) →
 *   加减速(Tj) → 匀减速(Td) → 减减速(Tj) → 完成
 *
 * 【适用场景】
 *   - 工业机床精加工 (低 Jerk → 小冲击)
 *   - 高速定位 (高 Jerk → 快响应)
 *   - 易碎/精密工件搬运 (极低 Jerk)
 *
 * 【性能要求】
 *   - SCurve_Step() 每5ms调用一次, 必须 <50us
 *   - 全定点整数运算, 不使用浮点数
 ******************************************************************************
 */
#ifndef _MOTION_PROFILE_H_
#define _MOTION_PROFILE_H_

#include "osal.h"

/* ── S曲线段枚举 ── */
typedef enum {
    SCURVE_IDLE          = -1,  /**< 空闲 (无规划) */
    SCURVE_ACC_JERK_UP   = 0,   /**< T1: 加加速段 a=+Jt */
    SCURVE_ACC_CONST     = 1,   /**< T2: 匀加速段 a=Amax */
    SCURVE_ACC_JERK_DOWN = 2,   /**< T3: 减加速段 a=Amax-Jt */
    SCURVE_CONST_VEL     = 3,   /**< T4: 匀速段 v=Vmax, a=0 */
    SCURVE_DEC_JERK_UP   = 4,   /**< T5: 加减速段 a=-Jt */
    SCURVE_DEC_CONST     = 5,   /**< T6: 匀减速段 a=-Amax */
    SCURVE_DEC_JERK_DOWN = 6,   /**< T7: 减减速段 a=-Amax+Jt */
    SCURVE_CREEP         = 7,   /**< T8: 低速精确逼近目标 */
    SCURVE_DONE          = 8    /**< 规划完成 */
} SCurveSegment_t;

/* ── S曲线规划器 ── */
typedef struct {
    /* ── 用户设定参数 (调用 SCurve_Plan 时写入) ── */
    int32  max_vel;           /**< 最大速度 (cts/周期) */
    int32  max_acc;           /**< 最大加速度 (cts/周期²) */
    int32  jerk;              /**< Jerk 加加速度 (cts/周期³) */
    int32  start_pos;         /**< 起始位置 (编码器脉冲) */
    int32  target_pos;        /**< 目标位置 (编码器脉冲) */

    /* ── 运行时状态 (每周期 SCurve_Step 更新) ── */
    int32  current_pos;       /**< 当前位置输出 */
    int32  current_vel;       /**< 当前速度 (cts/周期) */
    int32  current_acc;       /**< 当前加速度 (cts/周期²) */
    int8   direction;         /**< 运动方向: 1=正向, -1=反向 */
    SCurveSegment_t segment;  /**< 当前S曲线段 */
    uint32 seg_elapsed;       /**< 当前段已执行周期数 */

    /* ── 各段时间 (周期数, SCurve_Plan 时计算) ── */
    uint32 Tj;                /**< 加加速段周期数 (= Amax/J) */
    uint32 Ta;                /**< 匀加速段周期数 */
    uint32 Tv;                /**< 匀速段周期数 */
    uint32 Td;                /**< 匀减速段周期数 */
    uint32 Tj2;               /**< 减速加加速段 (对称, =Tj) */

    /* ── 段边界速度 (用于重建状态) ── */
    int32  v_at_const_start;  /**< 匀加速段起始速度 */
    int32  s_at_const_start;  /**< 匀加速段起始位移 */
    int32  v_at_vel_start;    /**< 匀速段起始速度 (=Vmax) */
    int32  s_at_vel_start;    /**< 匀速段起始位移 */
    int32  v_at_dec_start;    /**< 减速段起始速度 (=Vmax) */
    int32  s_at_dec_start;    /**< 减速段起始位移 */
    int32  decel_dist;        /**< 精确减速距离 (离散模拟) */
} SCurvePlanner_t;

/* ── 预定义 Jerk 等级 (cts/周期³) ──
 *   基于: 5ms周期, PULSES_PER_REV=50000, SCURVE_STOP_THRESH=4
 *   典型 Amax=pos_step/25, 200RPM时 pos_step=800, Amax≈32
 * ── */
#define SCURVE_JERK_ULTRA     1    /**< 超低冲击: 镜面加工/脆性材料 */
#define SCURVE_JERK_LOW       3    /**< 低冲击:   精加工 */
#define SCURVE_JERK_MEDIUM    8    /**< 中等:     半精加工 (默认) */
#define SCURVE_JERK_HIGH      20   /**< 高响应:   粗加工/快速定位 */
#define SCURVE_JERK_MAX       40   /**< 极限:     空载快速移动 */

/* ── 预定义加速度等级 (cts/周期²) ── */
#define SCURVE_ACC_SLOW       5    /**< 慢加速 */
#define SCURVE_ACC_NORMAL     16   /**< 正常加速 (~pos_step/50) */
#define SCURVE_ACC_FAST       33   /**< 快速加速 (~pos_step/25) */
#define SCURVE_ACC_AGGRESSIVE 66   /**< 猛烈加速 */

/* ── API ── */

/**
 * @brief  初始化S曲线规划器
 * @param  p : 规划器指针
 */
void  SCurve_Init(SCurvePlanner_t *p);

/**
 * @brief  规划S曲线运动
 * @param  p        : 规划器指针
 * @param  start    : 起始位置 (编码器脉冲)
 * @param  target   : 目标位置 (编码器脉冲)
 * @param  max_vel  : 最大速度 (cts/周期)
 * @param  max_acc  : 最大加速度 (cts/周期²)
 * @param  jerk     : Jerk (cts/周期³)
 * @note   规划完成后, 每周期调用 SCurve_Step() 获取目标位置
 */
void  SCurve_Plan(SCurvePlanner_t *p,
                  int32 start, int32 target,
                  int32 max_vel, int32 max_acc, int32 jerk);

/**
 * @brief  每周期步进 — 返回当前目标位置
 * @param  p : 规划器指针
 * @retval 当前周期的目标位置 (编码器脉冲)
 * @note   必须在每个控制周期 (5ms) 调用一次
 */
int32 SCurve_Step(SCurvePlanner_t *p);

/**
 * @brief  查询规划是否完成
 * @retval 1=已完成, 0=进行中
 */
int   SCurve_IsDone(SCurvePlanner_t *p);

/**
 * @brief  平滑停止 — 从当前速度开始生成S曲线减速
 * @param  p : 规划器指针
 * @note   将当前位置设为目标, 从当前速度/加速度开始S减速到0
 */
void  SCurve_Stop(SCurvePlanner_t *p);

/**
 * @brief  获取当前规划进度百分比
 * @retval 0~1000 (0.0%~100.0%)
 */
int32 SCurve_Progress(SCurvePlanner_t *p);
int32 icbrt(int64 x);           /**< 整数立方根 (供圆弧插补用) */

/**
 * @brief  重新规划目标 (on-the-fly 目标更新)
 * @param  p       : 规划器指针
 * @param  new_target : 新目标位置
 * @note   在运动中修改目标, 保持 jerk 连贯性
 */
void  SCurve_Replan(SCurvePlanner_t *p, int32 new_target);

#endif /* _MOTION_PROFILE_H_ */
