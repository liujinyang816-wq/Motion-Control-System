/**
 ******************************************************************************
 * @file           : ethercat_slave.h
 * @brief          : SV660N EtherCAT 从站 PDO 映射配置
 * ----------------------------------------------------------------------------
 * CiA 402 CSP (Cyclic Synchronous Position) 模式 PDO 映射
 ******************************************************************************
 */
#ifndef _ETHERCAT_SLAVE_H_
#define _ETHERCAT_SLAVE_H_

#include "osal.h"
#include "ethercattype.h"
#include "ethercatmain.h"

/* ── 从站 PDO 配置 ── */

/**
 * @brief  配置 SV660N 从站 PDO 映射 (RxPDO + TxPDO) 并设为 CSP 模式
 * @param  slave : 从站索引号 (1-based, 对应 ec_slave[slave])
 * @retval 1 : 配置成功 (返回值忽略累计 SDO 错误数)
 * @note   在 ec_config_init() → ec_config_map() 阶段被 SOEM 回调
 *         RxPDO: ControlWord(0x6040) + TargetPos(0x607A) + Mode(0x6060)
 *         TxPDO: StatusWord(0x6041) + ActualPos(0x6064) + Velocity(0x606C)
 *                + ErrorCode(0x603F) + ModeDisplay(0x6061)
 */
int Servosetup(uint16 slave);

/**
 * @brief  恢复 EtherCAT 通信到 OP 状态 (Flash 擦除/通信中断后调用)
 * @retval 1=恢复成功, 0=失败 (无法到达目标状态)
 * @note   流程: 禁用TIM1 ISR → 重初始化DM9000 → INIT → PREOP → DC配置
 *         → IO映射 → SAFEOP → 恢复齿轮比 → OP → 重新使能所有轴
 *         内部会短暂禁用 TIM1_UP_IRQn, 调用者无需额外处理
 */
int EtherCAT_RecoverOP(void);

/**
 * @brief  从参数表读取各轴齿轮比, 计算有效每圈脉冲数 effective_ppr
 * @note   effective_ppr = PULSES_PER_REV × gear_num / gear_den
 *         用于补偿机械传动比 (电机转 gear_num 圈 = 输出轴转 gear_den 圈)
 *         参数变更后需调用此函数刷新
 */
void RefreshAxisGearRatio(void);

/**
 * @brief  从丝杠导程计算伺服电子齿轮比 (已弃用, 保留空函数体)
 * @param  ax : 轴索引 (未使用)
 * @note   当前齿轮比固定为 524288:3125 (50000/rev),
 *         μm↔脉冲转换在 coord_sys.c 完成
 */
void CalcGearRatioFromPitch(int ax);

/**
 * @brief  通过 SDO 将电子齿轮比写入单个伺服驱动器
 * @param  slave : EtherCAT 从站索引 (1-based)
 * @param  ax    : 轴索引
 * @retval 1=写入成功, 0=SDO 写入失败
 * @note   写入 0x6091:01 (分子) 和 0x6091:02 (分母)
 *         调用前从站需在 PREOP/SAFEOP/OP 状态 (SDO 可用)
 */
int  Servo_WriteGearRatio(uint16 slave, int ax);

/**
 * @brief  根据轴模式更新 servo_gear_num/den 字段
 * @param  ax   : 轴索引 (3~7)
 * @param  type : 0=直线(50000ppr), 1=旋转(36000ppr)
 * @note   仅更新结构体字段, SDO 写入由调用者通过 Servo_WriteGearRatio() 完成
 */
void Axis_UpdateModeGearRatio(int ax, uint8_t type);

/**
 * @brief  将齿轮比写入所有 SV660N 从站 (直线:524288:3125→50000/rev, 旋转:262144:1125→36000/rev)
 * @note   流程: 禁用ISR → SDO写入所有轴 → SDO读回验证 → PDO交换
 *         → 位置预同步 (TargetPos ← CurrentPosition, 零跳变) → 恢复ISR
 *         根据各轴 axis_type 自动选择直线或旋转齿轮比
 */
void Servo_WriteAllGearRatios(void);

/**
 * @brief  MCU 内部位置 → PDO 目标位置 (写入路径, 当前为恒等透传)
 * @param  ax      : 轴索引 (保留兼容, 未使用)
 * @param  enc_pos : MCU 内部位置 (与 PDO 相同单位)
 * @retval PDO 目标位置值 (直接透传, 不做缩放)
 * @note   伺服齿轮比已在驱动器端配置, MCU 不做额外转换
 */
int32 Servo_EncToUser(int ax, int32 enc_pos);

/**
 * @brief  PDO 反馈值 → MCU 内部位置 (读取路径, 当前为恒等透传)
 * @param  ax       : 轴索引 (保留兼容, 未使用)
 * @param  user_pos : PDO 反馈位置值 (与 MCU 内部相同单位)
 * @retval MCU 内部位置 (直接透传, 不做缩放)
 * @note   伺服齿轮比已在驱动器端配置, MCU 不做额外转换
 */
int32 Servo_UserToEnc(int ax, int32 user_pos);

#endif /* _ETHERCAT_SLAVE_H_ */
