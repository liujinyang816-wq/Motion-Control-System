/**
 ******************************************************************************
 * @file           : param_defs.h
 * @brief          : 工业机床参数系统 — 参数ID枚举 (0~255)
 * ----------------------------------------------------------------------------
 * 【参数布局 — 8轴扩展版】
 *   0~15   : X轴参数 (联动, 16个)
 *   16~31  : Y轴参数 (联动, 16个)
 *   32~47  : Z轴参数 (联动, 16个)
 *   48~63  : R轴参数 (联动, 16个)
 *   64~79  : S轴参数 (刀库旋转, 9参数 + 7保留)
 *   80~95  : W轴参数 (升降抓电极, 12参数 + 4保留)
 *   96~111 : U轴参数 (摇动光洁度, 6参数 + 10保留)
 *   112~127: V轴参数 (摇动光洁度, 6参数 + 10保留)
 *   128~137: 运动控制全局参数 (后移)
 *   144~151: 轴位置持久化 (后移, 8轴)
 *   152~255: 预留
 *
 * 【存储】
 *   param_ram[256] 直接按ID索引, Param_Set/Param_Get 直接访问数组
 *   Modbus PDU 150~213 映射 param_ram[0]~param_ram[63] (联动轴 X/Y/Z/R)
 *   Modbus PDU 281~344 映射 param_ram[64]~param_ram[127] (辅助轴 U/V/W/S)
 *   Modbus PDU 217~225 映射 param_ram[128]~param_ram[136] (全局参数)
 *
 * 【串口命令】
 *   PARAM:十进制ID,值    如 PARAM:128,1  → 全局默认速度
 *   PARAM:0,3000           → X轴最大转速3000RPM
 ******************************************************************************
 */
#ifndef _PARAM_DEFS_H_
#define _PARAM_DEFS_H_

#include "osal.h"

/* 1=禁用 Flash 参数/掉电保存 (仅用 RAM 默认值, 避免擦写 Flash 干扰调试) */
#define FLASH_STORAGE_DISABLED  1

/* ── 每轴参数个数 ── */
#define AXIS_PARAM_COUNT    16

/* ── 参数ID枚举 (紧凑索引 0~255) ── */
typedef enum {
    /* ================================================================
     * 联动轴参数 (X/Y/Z/R): 0~63
     * X: 0~15, Y: 16~31, Z: 32~47, R: 48~63
     * ================================================================ */
    PARAM_AXIS_MAX_RPM       = 0,   /**< 最大转速 RPM */
    PARAM_AXIS_MAX_ACCEL     = 1,   /**< 最大加速度 cts/T² */
    PARAM_AXIS_MAX_JERK      = 2,   /**< 最大 Jerk cts/T³ */
    PARAM_AXIS_HOME_OFFSET   = 3,   /**< 回零偏置 (脉冲) — Int32 */
    PARAM_AXIS_SOFT_LIMIT_P  = 4,   /**< 正软限位 (脉冲) — Int32 */
    PARAM_AXIS_SOFT_LIMIT_N  = 5,   /**< 负软限位 (脉冲) — Int32 */
    PARAM_AXIS_BACKLASH      = 6,   /**< 反向间隙 (脉冲) */
    PARAM_AXIS_PITCH         = 7,   /**< 丝杠导程 (um/rev) */
    PARAM_AXIS_GEAR_NUM      = 8,   /**< 齿轮比分子 */
    PARAM_AXIS_GEAR_DEN      = 9,   /**< 齿轮比分母 */
    PARAM_AXIS_INVERT_DIR    = 10,  /**< 反转方向 */
    PARAM_AXIS_JOG_VEL       = 11,  /**< 点动速度 RPM */
    PARAM_AXIS_HOMING_VEL    = 12,  /**< 回零速度 RPM */
    PARAM_AXIS_HOMING_ACC    = 13,  /**< 回零加速度 */
    PARAM_AXIS_POS_ARRIVE_WIN = 14, /**< 到位窗口 (脉冲) */
    PARAM_AXIS_MAX_FOLLOW_ERR = 15, /**< 最大跟随误差 (脉冲) */

    AXIS_PARAM_END = 15,

    /* ================================================================
     * S轴参数 — 刀库旋转 (64~79, 9参数 + 7保留)
     * ================================================================ */
    PARAM_S_MAX_RPM       = 64,  /**< 分度旋转速度 RPM */
    PARAM_S_MAX_ACCEL     = 65,  /**< 分度加速度 cts/T² */
    PARAM_S_HOME_OFFSET   = 66,  /**< 回零偏置 (脉冲) — Int32 */
    PARAM_S_GEAR_NUM      = 67,  /**< 齿轮比分子 */
    PARAM_S_GEAR_DEN      = 68,  /**< 齿轮比分母 */
    PARAM_S_INVERT_DIR    = 69,  /**< 反转方向 0=正/1=反 */
    PARAM_S_JOG_SPEED     = 70,  /**< 点动速度 RPM */
    PARAM_S_ARRIVE_WIN    = 71,  /**< 到位窗口 (脉冲) */
    PARAM_S_HOMING_SPEED  = 72,  /**< 回零速度 RPM */
    /* 73~79: 保留 */

    /* ================================================================
     * W轴参数 — 升降抓电极 (80~95, 12参数 + 4保留)
     * ================================================================ */
    PARAM_W_MAX_RPM       = 80,  /**< 升降速度 RPM */
    PARAM_W_MAX_ACCEL     = 81,  /**< 升降加速度 cts/T² */
    PARAM_W_HOME_OFFSET   = 82,  /**< 回零偏置 (脉冲) — Int32 */
    PARAM_W_SOFT_LIMIT_P  = 83,  /**< 上限位 (脉冲) — Int32 */
    PARAM_W_SOFT_LIMIT_N  = 84,  /**< 下限位 (脉冲) — Int32 */
    PARAM_W_PITCH         = 85,  /**< 丝杠导程 um/rev */
    PARAM_W_GEAR_NUM      = 86,  /**< 齿轮比分子 */
    PARAM_W_GEAR_DEN      = 87,  /**< 齿轮比分母 */
    PARAM_W_INVERT_DIR    = 88,  /**< 反转方向 0=正/1=反 */
    PARAM_W_JOG_SPEED     = 89,  /**< 点动速度 RPM */
    PARAM_W_HOMING_SPEED  = 90,  /**< 回零速度 RPM */
    PARAM_W_ARRIVE_WIN    = 91,  /**< 到位窗口 (脉冲) */
    /* 92~95: 保留 */

    /* ================================================================
     * U/V轴参数 — 摇动光洁度 (U:96~111, V:112~127, 各6参数)
     * ================================================================ */
    /* U轴: 96~101 */
    PARAM_UV_OSC_SPEED    = 96,  /**< 振荡速度 RPM */
    PARAM_UV_AMPLITUDE    = 97,  /**< 振幅 (脉冲) */
    PARAM_UV_PITCH        = 98,  /**< 丝杠导程 um/rev (振幅mm显示用) */
    PARAM_UV_GEAR_NUM     = 99,  /**< 齿轮比分子 */
    PARAM_UV_GEAR_DEN     = 100, /**< 齿轮比分母 */
    PARAM_UV_INVERT_DIR   = 101, /**< 反转方向 0=正/1=反 */
    /* 102~111: 保留 */

    /* V轴: 112~127 — 与U轴参数对称, 偏移 +16 */

    /* ================================================================
     * 运动控制全局参数: 128~137 (原64~72后移)
     * ================================================================ */
    PARAM_DEFAULT_VEL        = 128, /**< 默认定位速度 RPM */
    PARAM_JOG_VEL_GLOBAL     = 129, /**< 全局点动速度 RPM */
    PARAM_HOMING_VEL_GLOBAL  = 130, /**< 全局回零速度 RPM */
    PARAM_HOMING_METHOD      = 131, /**< CiA 402 回零方式 (1~35) */
    PARAM_HOMING_ACCEL       = 132, /**< 回零加速度 */
    PARAM_SMOOTH_STOP_EN     = 133, /**< 平滑停止使能 */
    PARAM_FEED_OVERRIDE      = 134, /**< 进给倍率 10~200% */
    PARAM_RAPID_OVERRIDE     = 135, /**< 快进倍率 10~100% */
    PARAM_SPINDLE_OVERRIDE   = 136, /**< 主轴倍率 50~150% */
    /* 137: 预留 */

    /* ================================================================
     * 轴位置持久化: 144~151 (8轴)
     * ================================================================ */
    PARAM_SAVED_POS_VALID    = 144, /**< 位置有效标志: 0x50534F4B("PSOK")=有效 */
    PARAM_SAVED_POS_X        = 145, /**< X轴保存位置 (脉冲) */
    PARAM_SAVED_POS_Y        = 146, /**< Y轴保存位置 (脉冲) */
    PARAM_SAVED_POS_Z        = 147, /**< Z轴保存位置 (脉冲) */
    PARAM_SAVED_POS_R        = 148, /**< R轴保存位置 (脉冲) */
    PARAM_SAVED_POS_U        = 149, /**< U轴保存位置 (脉冲) */
    PARAM_SAVED_POS_V        = 150, /**< V轴保存位置 (脉冲) */
    PARAM_SAVED_POS_W        = 151, /**< W轴保存位置 (脉冲) */
    PARAM_SAVED_POS_S        = 152, /**< S轴保存位置 (脉冲) */

    PARAM_ID_COUNT = 256     /**< 参数总数 */
} ParamId_t;

/* PARAM_COUNT 宏 (main.h也有定义, 用 #ifndef 避免冲突) */
#ifndef PARAM_COUNT
#define PARAM_COUNT  PARAM_ID_COUNT
#endif

/* ── 轴参数偏移宏 ──
 *   例: AXIS_PARAM(PARAM_AXIS_MAX_RPM, 2) → Z轴(索引2)的最大转速 = 0 + 2*16 = 32
 * ── */
#define AXIS_PARAM(base, ax) ((ParamId_t)((int)(base) + (ax) * AXIS_PARAM_COUNT))

/* ── UV轴参数偏移宏 (U/V对称, 各16槽位) ──
 *   例: UV_AXIS_PARAM(PARAM_UV_OSC_SPEED, 0) → U轴振荡速度 = 96
 *       UV_AXIS_PARAM(PARAM_UV_OSC_SPEED, 1) → V轴振荡速度 = 112
 * ── */
#define UV_AXIS_PARAM(base, uv_axis) \
    ((ParamId_t)((int)(base) + (uv_axis) * AXIS_PARAM_COUNT))

/* ── 参数描述符 (运行时只读) ── */
typedef struct {
    ParamId_t id;
    const char *name;
    const char *unit;
    int32  default_val;
    int32  min_val;
    int32  max_val;
    uint16 modbus_reg;
    uint8  need_save;
    uint8  need_reinit;
} ParamDesc_t;

extern const ParamDesc_t g_param_desc_table[];
extern const uint16      g_param_desc_count;
extern int32             param_ram[PARAM_COUNT];  /**< 运行时参数缓存 (定义于 param_store.c) */

/* ── 辅助宏 ── */
#define PARAM_VALID(p, v)    ((v) >= g_param_desc_table[p].min_val \
                                  && (v) <= g_param_desc_table[p].max_val)
#define PARAM_DEFAULT(p)     g_param_desc_table[p].default_val

/* ── 参数存储/读取 API ── */
void   Param_LoadAll(void);
int    Param_SaveAll(void);
int    Param_ResetFactory(void);
int    Param_Set(ParamId_t id, int32 value);
int32  Param_Get(ParamId_t id);
void   Param_EnsureDefaults(void);
uint32 Param_GetVersion(void);

/* ── 轴位置持久化 API ──
 * 掉电保存槽: PVD 检测电压跌落 → ISR 中写入当前位置到固定 Flash 槽位
 * 正常运行时 Flash 零写入, 彻底解决磨损问题。
 * 上电时若编码器返回0, 从掉电槽/param_ram 恢复上次保存的位置 */
void   Position_PowerFailInit(void);      /**< 上电: 读掉电保存槽 + param_ram 兜底 */
void   Position_PowerFailSave(void);      /**< ISR安全: 掉电瞬间写单个Flash Word (~100μs) */
void   Position_BackupToRam(void);        /**< 复制当前轴位置到 param_ram (RAM操作, 供 Param_SaveAll 保存) */
void   Position_RestoreIfNeeded(void);    /**< 检查编码器位置, 若为0则从掉电槽/param_ram恢复 */

#endif /* _PARAM_DEFS_H_ */
