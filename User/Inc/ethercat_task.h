/**
 ******************************************************************************
 * @file           : ethercat_task.h
 * @brief          : EtherCAT 实时控制任务
 * ----------------------------------------------------------------------------
 * vEtherCAT_Task — 5ms DC 同步周期实时任务
 *   PDO 交换 → 轴控 → 回零检查 → 故障处理 → ModBus 轮询
 ******************************************************************************
 */
#ifndef _ETHERCAT_TASK_H_
#define _ETHERCAT_TASK_H_

void vEtherCAT_Task(void *pvParameters);

#endif /* _ETHERCAT_TASK_H_ */
