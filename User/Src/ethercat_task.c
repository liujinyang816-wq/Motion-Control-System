/**
 ******************************************************************************
 * @file           : ethercat_task.c
 * @brief          : EtherCAT 实时控制任务 (FreeRTOS)
 * ----------------------------------------------------------------------------
 * 优先级: 6 (最高用户任务)
 * 触发源: TIM1 中断 (5ms DC SYNC0) → vTaskNotifyGiveFromISR
 *
 * 每 5ms 执行:
 *   1. 接收 PDO (ec_receive_processdata)
 *   2. 圆弧插补步进 (Arc_ProcessStep)
 *   3. 主轴(POS/VEL/HOLD)轴控 (SCurve_Step步进)
 *   4. DDA从轴控制 (读取主轴位置, 按比例跟随)
 *   5. ARC轴控制
 *   6. 回零检查
 *   7. 运动队列处理
 *   8. 故障复位序列
 *   9. 回零序列
 *   10. 周期性位置上报
 *   11. ModBus 轮询 (每 4 周期 = 20ms)
 ******************************************************************************
 */
#include "ethercat_task.h"
#include "motor_axis.h"
#include "motion_api.h"
#include "param_defs.h"
#include "modbus_slave.h"
#include "ethercatmain.h"
#include "ethercatcoe.h"
#include "ethercat_slave.h"
#include "main.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"

/* ── 外部引用 (来自 main.c) ── */
extern MotorAxis_t axis[MAX_AXES];
extern int dorun;
extern uint8 fault_reset_requested;
extern uint8 zero_requested;
extern char   IOmap[];


/* ── 轴名称映射 ── */
static const char *axis_name[MAX_AXES] = {"X","Y","Z","R","U","V","W","S"};

/* ================================================================
   vEtherCAT_Task() — EtherCAT 实时控制任务
   ================================================================ */
void vEtherCAT_Task(void *pvParameters)
{
    (void)pvParameters;

    #if printf_cmd
    printf("EtherCAT task started\r\n");
    #endif

    for (;;)
    {
        /* 阻塞等待 TIM1 ISR 的任务通知 (5ms DC SYNC0) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* ── ModBus 轮询 (每4周期 = 20ms) ──
         * 放在 dorun 检查之前，确保初始化期间 HMI 也能获取位置数据 */
        {
            static uint8 mb_counter = 0;
            mb_counter++;
            if (mb_counter >= 4)
            {
                mb_counter = 0;
                Modbus_UpdateRegs();
                Modbus_Poll();
            }
        }

        if (dorun != 1) continue;

        /* ── 启动后一次性栈快照: 首轮循环完成后打印各任务栈高水位 ── */
        {
            static uint8 stack_snap_done = 0;
            if (!stack_snap_done) {
                stack_snap_done = 1;
                TaskHandle_t h;
                #if printf_cmd
                printf("--- Startup Stack Snapshot (words free) ---\r\n");
                h = xTaskGetHandle("EtherCAT");
                if (h) printf("  EtherCAT  (1280): %lu\r\n", uxTaskGetStackHighWaterMark(h));
                h = xTaskGetHandle("CmdParse");
                if (h) printf("  CmdParse  (1280): %lu\r\n", uxTaskGetStackHighWaterMark(h));
                h = xTaskGetHandle("ModbusDef");
                if (h) printf("  ModbusDef  (640): %lu\r\n", uxTaskGetStackHighWaterMark(h));
                printf("  (this task)     : %lu\r\n", uxTaskGetStackHighWaterMark(NULL));
                #endif
            }
        }

        /* ── 接收 PDO ── */
        ec_receive_processdata(EC_TIMEOUTRET);

        /* ── 圆弧插补：在轴控之前生成当前周期的 XY 目标位置 ── */
        Arc_ProcessStep();

        /* ── 先处理主轴(POS/VEL/HOLD), 再处理DDA从轴 ──
         * DDA从轴只读取已步进的主轴位置, 不调用SCurve_Step
         * 避免主轴被多个从轴重复步进导致加速/减速时间缩水
         * ARC轴也跳过 — 它们的目标位置由 Arc_ProcessStep 设置 */
        for (int ax = 0; ax < NUM_AXES; ax++)
        {
            if (ax + 1 > ec_slavecount) continue;
            if (axis[ax].mode == AXIS_MODE_DDA) continue;
            if (axis[ax].mode == AXIS_MODE_ARC) continue;
            Axis_ControlCycle(ax);
        }
        for (int ax = 0; ax < NUM_AXES; ax++)
        {
            if (ax + 1 > ec_slavecount) continue;
            if (axis[ax].mode != AXIS_MODE_DDA) continue;
            Axis_ControlCycle(ax);
        }
        for (int ax = 0; ax < NUM_AXES; ax++)
        {
            if (ax + 1 > ec_slavecount) continue;
            if (axis[ax].mode != AXIS_MODE_ARC) continue;
            Axis_ControlCycle(ax);
        }

        Axis_CheckHomingDone();

        /* ── 运动队列处理 (前瞻) ── */
        Motion_ProcessQueue();

        /* ── 故障复位序列处理 ── */
        if (fault_reset_requested)
        {
            int all_done = 1;
            for (int ax = 0; ax < NUM_AXES; ax++)
            {
                if (ax + 1 > ec_slavecount) continue;

                uint8 step = axis[ax].fault_reset_step;
                if (step == 0) continue;
                all_done = 0;

                uint16 sw = axis[ax].pdo_in->StatusWord;

                if (sw == 0x0000) {
                    axis[ax].pdo_out->ControlWord = 0x0080;
                    #if printf_cmd
                    printf("%s: no response (SW=0), retrying...\r\n", axis_name[ax]);
                    #endif
                    continue;
                }

                /* 复位序列期间持续将 TargetPos 同步到当前编码器位置，
                 * 防止使能瞬间因旧 TargetPos 值导致伺服跳变 (180°→122°同类) */
                {
                    int32 cur = axis[ax].pdo_in->CurrentPosition;
                    axis[ax].actual_pos = cur;
                    axis[ax].target_pos = cur;
                    axis[ax].pdo_out->TargetPos = cur;
                }

                switch (step) {
                case 1:
                    if ((sw & 0x0008) == 0) {
                        #if printf_cmd
                        printf("%s: fault cleared (SW=0x%04x), disabling voltage...\r\n",
                               axis_name[ax], sw);
                        #endif
                        axis[ax].pdo_out->ControlWord = 0x0000;
                        axis[ax].fault_reset_step = 2;
                    } else {
                        axis[ax].pdo_out->ControlWord = 0x0080;
                    }
                    break;
                case 2:
                    if ((sw & 0x004F) == 0x0040 || (sw & 0x004F) == 0x0000) {
                        #if printf_cmd
                        printf("%s: switch on disabled (SW=0x%04x), shutdown...\r\n",
                               axis_name[ax], sw);
                        #endif
                        axis[ax].pdo_out->ControlWord = 0x0006;
                        axis[ax].fault_reset_step = 3;
                    } else {
                        axis[ax].pdo_out->ControlWord = 0x0000;
                        if ((sw & 0x0020)) {
                            #if printf_cmd
                            printf("%s: waiting Quick Stop -> Disabled (SW=0x%04x)...\r\n",
                                   axis_name[ax], sw);
                            #endif
                        }
                    }
                    break;
                case 3:
                    if ((sw & 0x006F) == 0x0021) {
                        #if printf_cmd
                        printf("%s: ready (SW=0x%04x), switch on...\r\n",
                               axis_name[ax], sw);
                        #endif
                        axis[ax].pdo_out->ControlWord = 0x0007;
                        axis[ax].fault_reset_step = 4;
                    } else {
                        axis[ax].pdo_out->ControlWord = 0x0006;
                    }
                    break;
                case 4:
                    if ((sw & 0x006F) == 0x0023) {
                        #if printf_cmd
                        printf("%s: switched on (SW=0x%04x), enable op...\r\n",
                               axis_name[ax], sw);
                        #endif
                        /* 先同步 TargetPos 到当前实际位置，再发送 0x000F，
                         * 消除使能瞬间的 1 周期竞态：伺服进入 OP 时已读到正确位置 */
                        {
                            int32 cur = axis[ax].pdo_in->CurrentPosition;
                            axis[ax].actual_pos = cur;
                            axis[ax].target_pos = cur;
                            axis[ax].pdo_out->TargetPos = cur;
                        }
                        axis[ax].pdo_out->ControlWord = 0x000F;
                        axis[ax].fault_reset_step = 5;
                    } else {
                        axis[ax].pdo_out->ControlWord = 0x0007;
                    }
                    break;
                case 5:
                    if ((sw & 0x006F) == 0x0027) {
                        axis[ax].pdo_out->TargetMode = 8;
                        axis[ax].enabled    = 1;
                        axis[ax].actual_pos = axis[ax].pdo_in->CurrentPosition;
                        axis[ax].target_pos = axis[ax].actual_pos;
                        axis[ax].cmd_target_pos = axis[ax].actual_pos;
                        axis[ax].pdo_out->TargetPos = axis[ax].target_pos;
                        /* 清理所有运动状态，防止旧状态导致使能后异常运动 */
                        axis[ax].velocity    = 0;
                        axis[ax].direction   = 0;
                        axis[ax].mode        = AXIS_MODE_HOLD;
                        axis[ax].motion_busy = 0;
                        axis[ax].jog_pending_rpm = 0;
                        SCurve_Init(&axis[ax].splanner);
                        axis[ax].fault_reset_step = 0;
                        #if printf_cmd
                        printf("%s: RE-ENABLED (pos=%d, mode=CSP)\r\n",
                               axis_name[ax], (int)axis[ax].target_pos);
                        #endif
                    } else {
                        axis[ax].pdo_out->ControlWord = 0x000F;
                        axis[ax].pdo_out->TargetMode = 8;
                    }
                    break;
                }
            }
            if (all_done)
                fault_reset_requested = 0;
        }

        /* ── 回零 (ZERO) 序列处理 ── */
        if (zero_requested)
        {
            int all_done = 1;
            for (int ax = 0; ax < NUM_AXES; ax++)
            {
                if (ax + 1 > ec_slavecount) continue;

                uint8 step = axis[ax].zero_step;
                if (step == 0) continue;
                all_done = 0;

                uint16 sw = axis[ax].pdo_in->StatusWord;

                switch (step) {
                case 1:
                    {
                        uint8 method = 35;
                        ec_SDOwrite(axis[ax].slave_idx, 0x6098, 0x00, FALSE,
                                    sizeof(method), &method, EC_TIMEOUTRXM);
                        #if printf_cmd
                        printf("%s: Homing method 35 written, starting homing...\r\n",
                               axis_name[ax]);
                        #endif
                        axis[ax].pdo_out->TargetMode  = 6;
                        axis[ax].pdo_out->ControlWord = 0x001F;
                        axis[ax].zero_step = 2;
                    }
                    break;
                case 2:
                    if ((sw & 0x1000) && (sw & 0x0400))
                    {
                        #if printf_cmd
                        printf("%s: homing complete, restoring CSP mode...\r\n",
                               axis_name[ax]);
                        #endif
                        axis[ax].pdo_out->TargetMode  = 8;
                        axis[ax].pdo_out->ControlWord = 0x000F;
                        axis[ax].zero_step = 3;
                    }
                    break;
                case 3:
                    if ((sw & 0x006F) == 0x0027)
                    {
                        axis[ax].actual_pos     = axis[ax].pdo_in->CurrentPosition;
                        axis[ax].target_pos     = axis[ax].actual_pos;
                        axis[ax].cmd_target_pos = axis[ax].actual_pos;
                        axis[ax].direction      = 0;
                        axis[ax].velocity        = 0;
                        axis[ax].mode            = AXIS_MODE_HOLD;
                        axis[ax].zero_step       = 0;
                        #if printf_cmd
                        printf("%s: ZERO DONE (pos=%d) <<<\r\n",
                               axis_name[ax], (int)axis[ax].actual_pos);
                        #endif
                    }
                    break;
                }
            }
            if (all_done)
                zero_requested = 0;
        }

//        /* ── 每50ms打印最大速度轴的实际速度 (编码器50ms累计差分求平均) ── */
//        {
//            static int32  snap_pos[NUM_AXES] = {0};   /* 窗口起始位置快照 */
//            static uint8  snap_valid[NUM_AXES] = {0};
//            static uint32 vel_report_counter = 0;

//            vel_report_counter++;
//            if (vel_report_counter >= 10)   /* 10周期 × 5ms = 50ms */
//            {
//                vel_report_counter = 0;

//                int32  max_vel = 0;
//                int    max_ax  = -1;
//                for (int ax = 0; ax < NUM_AXES; ax++)
//                {
//                    if (ax + 1 > ec_slavecount) continue;
//                    int32 cur = axis[ax].pdo_in->CurrentPosition;
//                    if (snap_valid[ax])
//                    {
//                        /* 50ms 内总位移 → 平均每周期速度 (cts/cycle) */
//                        int32 total = cur - snap_pos[ax];
//                        if (total < 0) total = -total;
//                        int32 avg = total / 10;       /* 10周期平均 */
//                        if (avg > max_vel)
//                        {
//                            max_vel = avg;
//                            max_ax  = ax;
//                        }
//                    }
//                    snap_pos[ax]  = cur;
//                    snap_valid[ax] = 1;
//                }
//                if (max_ax >= 0 && max_vel > 0)
//                {
//                    int32 rpm = max_vel / POS_STEP_PER_RPM;
//                    #if printf_cmd
//                    printf("[MAX_VEL] %s: %d RPM (%d cts/cycle)\r\n",
//                           axis_name[max_ax], (int)rpm, (int)max_vel);
//                    #endif
//                }
//            }
//        }

        /* ── 周期位置上报 (仅 X/Y 脉冲位置) ── */
        static uint32_t pos_report_counter = 0;
        pos_report_counter++;
        {
            uint32 interval = 20;
            if (pos_report_counter >= interval)
            {
                pos_report_counter = 0;

                int32 pos_x = (0 < ec_slavecount) ? axis[0].pdo_in->CurrentPosition : 0;
                int32 pos_y = (1 < ec_slavecount) ? axis[1].pdo_in->CurrentPosition : 0;
				int32 pos_z = (2 < ec_slavecount) ? axis[2].pdo_in->CurrentPosition : 0;
                int32 pos_r = (3 < ec_slavecount) ? axis[3].pdo_in->CurrentPosition : 0;
                int32 pos_u = (4 < ec_slavecount) ? axis[4].pdo_in->CurrentPosition : 0;
                int32 pos_v = (5 < ec_slavecount) ? axis[5].pdo_in->CurrentPosition : 0;
                int32 pos_w = (6 < ec_slavecount) ? axis[6].pdo_in->CurrentPosition : 0;
                int32 pos_s = (7 < ec_slavecount) ? axis[7].pdo_in->CurrentPosition : 0;
                printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n", (int)pos_x, (int)pos_y, (int)pos_z, (int)pos_r,
                       (int)pos_u, (int)pos_v, (int)pos_w, (int)pos_s);
            }
        }

        /* ── 位置持久化 (Flash 禁用时跳过) ── */
#if !FLASH_STORAGE_DISABLED
        {
            static uint32_t pos_save_counter = 0;
            pos_save_counter++;
            /* 6000 周期 = 30秒 */
            if (pos_save_counter >= 6000)
            {
                pos_save_counter = 0;
                Position_BackupToRam();

                /* ── 栈监控: 每30秒打印各任务历史最小剩余栈空间 ── */
                {
                    TaskHandle_t h;
                    #if printf_cmd
                    printf("[Stack] ");
                    #endif
                    h = xTaskGetHandle("EtherCAT");
                    #if printf_cmd
                    if (h) printf("Eth:%lu ", uxTaskGetStackHighWaterMark(h));
                    #endif
                    h = xTaskGetHandle("CmdParse");
                    #if printf_cmd
                    if (h) printf("Cmd:%lu ", uxTaskGetStackHighWaterMark(h));
                    #endif
                    h = xTaskGetHandle("ModbusDef");
                    #if printf_cmd
                    if (h) printf("Mod:%lu ", uxTaskGetStackHighWaterMark(h));
                    #endif
                    #if printf_cmd
                    printf("\r\n");
                    #endif
                }
            }
        }
#endif
    }
}
