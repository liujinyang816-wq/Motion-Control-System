/**
 ******************************************************************************
 * @file           : motion_api.h
 * @brief          : 单轴/多轴运动控制API + 插补引擎
 * ----------------------------------------------------------------------------
 * 【API 分层】
 *   L1 - 轴使能/状态:   Axis_Enable, Axis_Disable, Axis_IsMoving, ...
 *   L2 - 单轴运动:      Axis_JogVel, Axis_JogStep, Axis_MoveAbs, Axis_MoveRel
 *   L3 - 回零:          Axis_Home, Axis_HomeAll
 *   L4 - 多轴插补:      Interp_Line, Interp_ArcXY
 *   L5 - 运动队列:      Motion_QueueSegment, Motion_Pause, Motion_Resume
 *   L6 - 安全:          Axis_QuickStop, Axis_EStop, Axis_EStopAll
 *
 * 【坐标约定】
 *   所有位置参数使用 MCS (Machine Coordinate System) 编码器脉冲
 *   所有位置参数使用编码器脉冲, 通过轴参数 (丝杠导程/齿轮比) 转换坐标
 ******************************************************************************
 */
#ifndef _MOTION_API_H_
#define _MOTION_API_H_

#include "osal.h"
#include "main.h"

#define S_ACCEL_K_NUM       10    /**< K_acc 分子 (1.0) */
#define S_ACCEL_K_DEN       10    /**< K_acc 分母 */
#define S_JERK_CYCLES       400   /**< Jerk 基准 2000ms (400周期) */
#define S_ACCEL_MIN         400   /**< 编码器脉冲数每控制周期平方，最低加速度 = 保证 Tj≥400 (2s) */
#define S_JERK_MIN          1     /**< 最低 Jerk */


/* ── 运动段 (前瞻队列元素) ── */
typedef struct {
    int32 pos[MAX_AXES];     /**< 8轴目标位置 (编码器脉冲) */
    int32 feedrate;          /**< 进给速度 (编码器脉冲/周期) */
    uint8 type;              /**< 0=直线 1=圆弧CW 2=圆弧CCW */
    uint8 reserved[3];
} MotionSegment_t;

#define SEGMENT_QUEUE_SIZE   64    /**< 前瞻队列深度 */
#define MAX_FEEDRATE_MMPM    50000 /**< 最大进给速度 mm/min */

/* ══════════════════════════════════════════════════════════════════
 * L1: 轴使能与状态
 * ══════════════════════════════════════════════════════════════════ */

/**
 * @brief  使能单个轴 (CiA 402: Shutdown → SwitchOn → EnableOp)
 * @param  ax : 轴索引 0=X 1=Y 2=Z 3=R
 * @retval 1=成功 0=失败
 */
int  Axis_Enable(int ax);

/**
 * @brief  去使能单个轴 (CiA 402: Disable Voltage)
 * @param  ax : 轴索引
 * @retval 1=成功
 */
int  Axis_Disable(int ax);

/**
 * @brief  查询轴是否在运动中
 * @retval 1=运动中 0=静止
 */
int   Axis_IsMoving(int ax);

/**
 * @brief  查询轴是否有故障
 * @retval 1=故障 0=正常
 */
int   Axis_IsFault(int ax);

/**
 * @brief  查询轴是否已使能
 * @retval 1=已使能 0=未使能
 */
int   Axis_IsEnabled(int ax);

/**
 * @brief  获取轴当前位置 (MCS)
 * @retval 编码器脉冲
 */
int32 Axis_GetPos(int ax);

/**
 * @brief  获取轴当前速度
 * @retval RPM
 */
int32 Axis_GetVel(int ax);

/* ══════════════════════════════════════════════════════════════════
 * L2: 单轴运动
 * ══════════════════════════════════════════════════════════════════ */

/**
 * @brief  速度点动 — 轴以指定速度连续旋转
 * @param  ax  : 轴索引
 * @param  rpm : 转速 (+正转/-反转/0停止)
 * @note   使用轴参数中的 jerk/acc 设置进行S曲线加速
 */
int  Axis_JogVel(int ax, int32 rpm);

/**
 * @brief  步进点动 — 轴增量移动指定脉冲数
 * @param  ax    : 轴索引
 * @param  delta : 增量 (编码器脉冲, +正/-负)
 * @note   使用 S曲线 或梯形 (根据 PARAM_S_CURVE_EN)
 */
int  Axis_JogStep(int ax, int32 delta);

/**
 * @brief  绝对定位 — 轴移动到 MCS 绝对位置
 * @param  ax  : 轴索引
 * @param  pos : 目标位置 (MCS编码器脉冲)
 */
int  Axis_MoveAbs(int ax, int32 pos);

/**
 * @brief  相对定位 — 轴增量移动
 * @param  ax    : 轴索引
 * @param  delta : 增量 (编码器脉冲)
 */
int  Axis_MoveRel(int ax, int32 delta);

/* ══════════════════════════════════════════════════════════════════
 * L3: 回零
 * ══════════════════════════════════════════════════════════════════ */

/**
 * @brief  单轴回零
 * @param  ax     : 轴索引
 * @param  method : CiA 402 回零方式 (1~35, 默认35=当前位置归零)
 * @retval 1=回零启动成功 0=失败
 */
int  Axis_Home(int ax, uint8 method);

/**
 * @brief  所有轴同时回零
 */
int  Axis_HomeAll(void);

/* ══════════════════════════════════════════════════════════════════
 * L4: 多轴插补
 * ══════════════════════════════════════════════════════════════════ */

/**
 * @brief  4轴直线插补 — 所有轴同时出发, 同时到达
 * @param  targets  : 4轴目标位置数组 (MCS编码器脉冲)
 * @param  feedrate : 进给速度 (RPM, 沿路径方向)
 * @retval 1=成功 0=参数无效
 * @note   使用 DDA (Digital Differential Analyzer) 算法
 */
int  Interp_Line(int32 *targets, int32 feedrate);

/**
 * @brief  XY平面圆弧插补
 * @param  cx, cy  : 圆心坐标 (MCS)
 * @param  ex, ey  : 终点坐标 (MCS)
 * @param  dir     : 0=顺时针(CW) 1=逆时针(CCW)
 * @param  feedrate: 进给速度 (RPM)
 */
int  Interp_ArcXY(int32 cx, int32 cy, int32 ex, int32 ey,
                  uint8 dir, int32 feedrate);

/**
 * @brief  每周期圆弧轨迹步进 (由 EtherCAT 任务调用)
 * @note   从全局圆弧状态的 S 曲线规划器计算当前角度 → XY 位置 → 写入 PDO
 */
void Arc_ProcessStep(void);

/**
 * @brief  中止圆弧运动 (清状态 + 停止相关轴)
 */
void Arc_Abort(void);

/**
 * @brief  查询圆弧是否正在运行
 * @retval 1=运行中 0=空闲
 */
int  Arc_IsActive(void);

/* ══════════════════════════════════════════════════════════════════
 * L5: 运动队列 (Look-Ahead)
 * ══════════════════════════════════════════════════════════════════ */

/**
 * @brief  将运动段加入前瞻队列
 * @param  targets  : 4轴终点
 * @param  feedrate : 进给速度
 * @retval 1=入队成功 0=队列满
 */
int  Motion_QueueSegment(int32 *targets, int32 feedrate);

/**
 * @brief  清空运动队列
 */
void Motion_FlushQueue(void);

/**
 * @brief  暂停运动 (保持当前位置)
 */
void Motion_Pause(void);

/**
 * @brief  恢复暂停的运动
 */
void Motion_Resume(void);

/**
 * @brief  取消所有运动 (清队列 + 平滑停止)
 */
void Motion_Abort(void);

/**
 * @brief  查询队列空满状态
 * @retval 已用槽数
 */
int  Motion_QueueCount(void);

/* ══════════════════════════════════════════════════════════════════
 * L6: 安全停止
 * ══════════════════════════════════════════════════════════════════ */

/**
 * @brief  单轴快停 (CiA 402 Quick Stop)
 * @param  ax : 轴索引
 * @note   以快停减速度减速到0, 保持使能
 */
int  Axis_QuickStop(int ax);

/**
 * @brief  紧急停止 (断开使能)
 * @note   所有轴立即断电
 */
int  Axis_EStopAll(void);

/**
 * @brief  单轴紧急停止
 */
int  Axis_EStop(int ax);

/* ══════════════════════════════════════════════════════════════════
   轴控核心函数 (从 main.c 迁移)
   ══════════════════════════════════════════════════════════════════ */

void Axis_Init(int ax, uint16 slave);
void Axis_SyncAllTargetsFromEncoder(void);
int  Axis_EnableAll(void);
void Axis_ControlCycle(int ax);
void Axis_StopOne(int ax);
void Axis_ApplyVelocity(int ax, int rpm, int sign);
void Axis_ApplyPosition(int ax, int32 pos);
void Axis_SetMoveSpeed(int ax, int rpm);
void Axis_CancelHoming(int ax);
void Axis_CheckHomingDone(void);
void Axis_WriteTargetPos(int ax, int32 pos_enc);
void SCurve_AutoAccJerk(int32 max_vel, int32 acc_cap,int32 *acc_out, int32 *jerk_out);


/* ── 全局运动控制变量 ── */
extern int   g_move_rpm;
extern int   g_home_rpm;
extern uint8 homing_active;
extern uint8 zero_requested;

#endif /* _MOTION_API_H_ */
