/**
 ******************************************************************************
 * @file           : serial_cmd.c
 * @brief          : 串口命令解析 + 命令任务
 * ----------------------------------------------------------------------------
 * 【串口命令格式】
 *   速度: SPEED:30 (全局) / SPEED:300,200,100,50 (分轴X,Y,Z,R)
 *         X:500 / Y:-300 (单轴)
 *   位置: POS:10000,-5000,0,8388 (多轴脉冲)
 *         POS-X:50000 (单轴)
 *   圈数: NUM:2,-1,0,3 (多轴圈数) / NUM-X:+2 (单轴)
 *   回零: ZERO / ZERO:N / X:ZERO
 *   系统: STOP / RESET / SPEED(查询) / STATUS / WHERE
 ******************************************************************************
 */
#include "serial_cmd.h"
#include "motor_axis.h"
#include "motion_api.h"
#include "param_defs.h"
#include "modbus_slave.h"
#include "ethercatmain.h"
#include "ethercat_slave.h"
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

/* ── 命令缓冲区 ── */
char   cmd_buf[64] = {0};
uint8  cmd_idx = 0;
uint8  cmd_ready = 0;

/* ── 命令任务句柄 ── */
TaskHandle_t xCmdTaskHandle = NULL;

/* ── 外部引用 (来自 motion_api.c) ── */
extern int   g_move_rpm;
extern int   g_home_rpm;
extern uint8 homing_active;
extern uint8 fault_reset_requested;

/* ── 外部引用 (来自 main.c) ── */
extern MotorAxis_t axis[MAX_AXES];
extern int dorun;
extern uint8 Disable_Output_DI;
extern uint8 Disable_Output_RS485;

/* ── 外部引用 (来自其他模块) ── */
extern int32 param_ram[PARAM_COUNT];

/* ── 轴名称映射 ── */
static const char *axis_name[MAX_AXES] = {"X","Y","Z","R","U","V","W","S"};

/* ================================================================
   辅助函数
   ================================================================ */

static char Cmd_ToUpper(char c)
{
    if (c >= 'a' && c <= 'z') return (char)(c - 32);
    return c;
}

static int Cmd_CharIEq(char a, char b)
{
    return Cmd_ToUpper(a) == Cmd_ToUpper(b);
}

int Axis_StrIEq(const char *a, const char *b)
{
    while (*a && *b) {
        if (!Cmd_CharIEq(*a, *b)) return 0;
        a++; b++;
    }
    return *a == *b;
}

int Axis_NameToIndex(char c)
{
    switch (Cmd_ToUpper(c)) {
        case 'X': return 0;
        case 'Y': return 1;
        case 'Z': return 2;
        case 'R': return 3;
        case 'U': return 4;
        case 'V': return 5;
        case 'W': return 6;
        case 'S': return 7;
        default:  return -1;
    }
}

void Axis_TrimSpaces(char *s)
{
    char *dst = s;
    while (*s == ' ' || *s == '\t') s++;
    if (s != dst) {
        while (*s) *dst++ = *s++;
        *dst = '\0';
    } else {
        dst = s + strlen(s);
    }
    while (dst > s && (dst[-1] == ' ' || dst[-1] == '\t'))
        *--dst = '\0';
}

int32 Axis_ParsePosition(const char *cmd, int *ok)
{
    int sign = 1;
    long long val = 0;

    *ok = 0;
    if (cmd[0] == '-') { sign = -1; cmd++; }
    else if (cmd[0] == '+') { cmd++; }

    if (cmd[0] == '\0') return 0;

    for (int i = 0; cmd[i] != '\0'; i++) {
        if (cmd[i] >= '0' && cmd[i] <= '9')
            val = val * 10 + (cmd[i] - '0');
        else
            return 0;
    }

    *ok = 1;
    return (int32)(val * sign);
}

int Axis_ParseRpm(const char *cmd, int *rpm, int *sign, int *ok)
{
    *ok = 0;
    *sign = 1;
    if (cmd[0] == '-') { *sign = -1; cmd++; }
    else if (cmd[0] == '+') { cmd++; }

    if (cmd[0] == '\0') return 0;

    int val = 0;
    for (int i = 0; cmd[i] != '\0'; i++) {
        if (cmd[i] >= '0' && cmd[i] <= '9')
            val = val * 10 + (cmd[i] - '0');
        else
            return 0;
    }

    *rpm = val;
    *ok = 1;
    return 1;
}

int ParseCSV(const char *s, int32 *out, int max)
{
    int count = 0;
    while (*s && count < max) {
        while (*s == ' ') s++;
        if (*s == '\0') break;
        char tmp[16];
        int i = 0;
        while (*s && *s != ',' && i < 15)
            tmp[i++] = *s++;
        tmp[i] = '\0';
        int ok;
        int32 val = Axis_ParsePosition(tmp, &ok);
        if (!ok) break;
        out[count++] = val;
        if (*s == ',') s++;
    }
    return count;
}

/* ================================================================
   ParseOneSegment() — 解析单个逗号分隔的命令段
   ================================================================ */
static int ParseOneSegment(char *seg)
{
    int parse_ok;
    int rpm, sign;

    Axis_TrimSpaces(seg);
    if (seg[0] == '\0') return 0;

    /* POS-X:1000 / pos-y:-3000 */
    if ((seg[0] == 'P' || seg[0] == 'p')
        && (seg[1] == 'O' || seg[1] == 'o')
        && (seg[2] == 'S' || seg[2] == 's')
        && seg[3] == '-')
    {
        int ax = Axis_NameToIndex(seg[4]);
        if (ax < 0 || ax >= NUM_AXES || seg[5] != ':') return 0;

        int32 pos = Axis_ParsePosition(seg + 6, &parse_ok);
        if (!parse_ok) return 0;
        Axis_ApplyPosition(ax, pos);
        return 1;
    }

    /* NUM-X:+2 / NUM-Y:-5 → 按圈数移动 */
    if ((seg[0] == 'N' || seg[0] == 'n')
        && (seg[1] == 'U' || seg[1] == 'u')
        && (seg[2] == 'M' || seg[2] == 'm')
        && seg[3] == '-')
    {
        int ax = Axis_NameToIndex(seg[4]);
        if (ax < 0 || ax >= NUM_AXES || seg[5] != ':') return 0;

        int32 revs = Axis_ParsePosition(seg + 6, &parse_ok);
        if (!parse_ok) return 0;

        int32 ppr = axis[ax].effective_ppr;
        if (ppr <= 0) ppr = GetAxisPPR(ax);
        int32 pos = (int32)((int64)axis[ax].actual_pos + (int64)revs * (int64)ppr);
        Axis_ApplyPosition(ax, pos);
        
        
        printf("  (%+d revs = %+d pulses)\r\n", (int)revs, (int)(revs * ppr));
        
        
        return 1;
    }

    /* X:500 / Y:-300 / X:RESET / X:ZERO */
    {
        int ax = Axis_NameToIndex(seg[0]);
        if (ax < 0 || ax >= NUM_AXES || seg[1] != ':') return 0;

        if (Axis_StrIEq(seg + 2, "RESET")) {
            
            
            printf(">>> %s: FAULT RESET <<<\r\n", axis_name[ax]);
            
            
            axis[ax].pdo_out->ControlWord = 0x0080;
            axis[ax].fault_reset_step = 1;
            Axis_StopOne(ax);
            fault_reset_requested = 1;
            return 1;
        }

        if (Axis_StrIEq(seg + 2, "ZERO")) {
            
            
            printf(">>> %s: ZERO (moving to 0 with S-Curve) <<<\r\n", axis_name[ax]);
            
            
            Axis_CancelHoming(ax);
            axis[ax].mode = AXIS_MODE_POS;
            axis[ax].cmd_target_pos = 0;
            axis[ax].homing = 1;
            Axis_SetMoveSpeed(ax, g_home_rpm);
            {
                int32 max_vel = axis[ax].pos_step;
                int32 acc_cap = Param_Get(AXIS_PARAM(PARAM_AXIS_MAX_ACCEL, ax));
                int32 max_acc, jerk;
                SCurve_AutoAccJerk(max_vel, acc_cap, &max_acc, &jerk);
                SCurve_Plan(&axis[ax].splanner,
                            axis[ax].actual_pos, 0,
                            max_vel, max_acc, jerk);
            }
            homing_active = 1;
            return 1;
        }

        if (!Axis_ParseRpm(seg + 2, &rpm, &sign, &parse_ok) || !parse_ok) return 0;
        Axis_ApplyVelocity(ax, rpm, sign);
        return 1;
    }
}

/* ================================================================
   ParseCommand() — 串口命令解析器 (由 vCmd_Task 调用)
   ================================================================ */
static void ParseCommand(void)
{
    char work[64];
    int any_ok = 0;

    strncpy(work, cmd_buf, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';
    Axis_TrimSpaces(work);

    if (work[0] == '\0') return;

    if (Axis_StrIEq(work, "STOP")) {
        Motion_Abort();
        return;
    }

    /* ALL:300 / ALL:-200 → 所有轴同一转速旋转 */
    if ((work[0] == 'A' || work[0] == 'a')
        && (work[1] == 'L' || work[1] == 'l')
        && (work[2] == 'L' || work[2] == 'l')
        && work[3] == ':')
    {
        int rpm, sign, ok;
        if (Axis_ParseRpm(work + 4, &rpm, &sign, &ok) && ok) {
            if (rpm > 3000) rpm = 3000;

            for (int ax = 0; ax < NUM_AXES; ax++) {
                if (ax + 1 > ec_slavecount) continue;
                if (rpm == 0) {
                    Axis_StopOne(ax);
                } else {
                    Axis_ApplyVelocity(ax, rpm, sign);
                }
            }
        }  
        return;
    }

        if (Axis_StrIEq(work, "RESET")) {
        
        
        printf(">>> System RESET: fault reset all axes <<<\r\n");
        
        
        for (int ax = 0; ax < NUM_AXES; ax++) {
            if (ax + 1 > ec_slavecount) continue;
            axis[ax].pdo_out->ControlWord = 0x0080;
            axis[ax].fault_reset_step = 1;
            Axis_StopOne(ax);
        }
        fault_reset_requested = 1;
        return;
    }

    if (Axis_StrIEq(work, "SPEED")) {
        
        
        printf("Current move speed: %d RPM\r\n", g_move_rpm);
        
        
        return;
    }

    /* SPEED:N → 全局定位速度, SPEED:v1,v2,v3,v4 → 分轴速度+旋转 */
    if ((work[0] == 'S' || work[0] == 's')
        && (work[1] == 'P' || work[1] == 'p')
        && (work[2] == 'E' || work[2] == 'e')
        && (work[3] == 'E' || work[3] == 'e')
        && (work[4] == 'D' || work[4] == 'd')
        && work[5] == ':')
    {
        int32 csv[MAX_AXES];
        int n = ParseCSV(work + 6, csv, MAX_AXES);
        if (n == 1 && csv[0] > 0 && csv[0] <= 999) {
            g_move_rpm = (int)csv[0];
            for (int ax = 0; ax < NUM_AXES; ax++) {
                if (!axis[ax].homing)
                    Axis_SetMoveSpeed(ax, g_move_rpm);
            }
            
        } else if (n > 1) {
            for (int i = 0; i < n && i < NUM_AXES; i++) {
                int rpm = (int)csv[i];
                if (rpm > 3000)  rpm = 3000;
                if (rpm < -3000) rpm = -3000;
                if (!axis[i].homing) {
                    Axis_CancelHoming(i);
                    if (rpm == 0) {
                        Axis_StopOne(i);
                        
                        
                        printf("%s=STOP ", axis_name[i]);
                        
                        
                    } else {
                        int sign = (rpm > 0) ? 1 : -1;
                        int abs_rpm = rpm > 0 ? rpm : -rpm;
                        Axis_ApplyVelocity(i, abs_rpm, sign);
                        
                        
                        printf("%s=%+dRPM ", axis_name[i], rpm);
                        
                        
                    }
                }
            }

            printf("<<<\r\n");

        } else {
            printf("? Invalid speed: %s\r\n", work + 6);

        }
        return;
    }

    /* POS:v1,v2,...,v8 → 多轴同时定位 (最多8轴) */
    if ((work[0] == 'P' || work[0] == 'p')
        && (work[1] == 'O' || work[1] == 'o')
        && (work[2] == 'S' || work[2] == 's')
        && work[3] == ':')
    {
        int32 csv[MAX_AXES];
        int n = ParseCSV(work + 4, csv, MAX_AXES);
        if (n >= 1) {
            int32 targets[MAX_AXES];
            for (int i = 0; i < MAX_AXES; i++) {
                targets[i] = (i < n) ? csv[i] : axis[i].actual_pos;
            }
            Interp_Line(targets, g_move_rpm);
            #if printf_cmd
            printf(">>> Interp: ");
            for (int i = 0; i < n && i < MAX_AXES; i++)
                printf("%s=%d ", axis_name[i], (int)csv[i]);
            printf("<<<\r\n");
            #endif
        } else {
            #if printf_cmd
            printf("? %s\r\n", cmd_buf);
            #endif
        }
        return;
    }

    /* NUM:v1,v2,...,v8 → 多轴按圈数移动 (最多8轴) */
    if ((work[0] == 'N' || work[0] == 'n')
        && (work[1] == 'U' || work[1] == 'u')
        && (work[2] == 'M' || work[2] == 'm')
        && work[3] == ':')
    {
        int32 csv[MAX_AXES];
        int n = ParseCSV(work + 4, csv, MAX_AXES);
        if (n >= 1) {
            int32 targets[MAX_AXES];
            for (int i = 0; i < MAX_AXES; i++) {
                int32 ppr = axis[i].effective_ppr;
                if (ppr <= 0) ppr = GetAxisPPR(i);
                if (i < n)
                    targets[i] = (int32)((int64)axis[i].actual_pos
                                + (int64)csv[i] * (int64)ppr);
                else
                    targets[i] = axis[i].actual_pos;
            }
            Interp_Line(targets, g_move_rpm);
            #if printf_cmd
            printf(">>> Interp revs: ");
            for (int i = 0; i < n && i < MAX_AXES; i++)
                printf("%s=%+drev ", axis_name[i], (int)csv[i]);
            printf("<<<\r\n");
            #endif
        } else {
            #if printf_cmd
            printf("? %s\r\n", cmd_buf);
            #endif
        }
        return;
    }

    /* ZERO:N → 设置 S 曲线最大回零速度 */
    if ((work[0] == 'Z' || work[0] == 'z')
        && (work[1] == 'E' || work[1] == 'e')
        && (work[2] == 'R' || work[2] == 'r')
        && (work[3] == 'O' || work[3] == 'o')
        && work[4] == ':')
    {
        int32 csv[MAX_AXES];
        int n = ParseCSV(work + 5, csv, MAX_AXES);
        if (n == 1 && csv[0] > 0 && csv[0] <= 999) {
            g_home_rpm = (int)csv[0];
            
            
            printf(">>> ZERO speed set to: %d RPM <<<\r\n", g_home_rpm);
            
            
        } else {
            
            
            printf("? Invalid ZERO speed: %s (use ZERO:N, N=1~999)\r\n", work + 5);
            
            
        }
        return;
    }

    /* ZERO — 所有轴回零 */
    if (Axis_StrIEq(work, "ZERO")) {
        
        
        printf(">>> ZERO: returning to origin at %d RPM <<<\r\n", g_home_rpm);
        
        
        for (int ax = 0; ax < NUM_AXES; ax++) {
            if (ax + 1 > ec_slavecount) continue;
            if (!axis[ax].enabled) continue;
            Axis_CancelHoming(ax);
            axis[ax].mode = AXIS_MODE_POS;
            axis[ax].cmd_target_pos = 0;
            axis[ax].homing = 1;
            Axis_SetMoveSpeed(ax, g_home_rpm);
            {
                int32 max_vel = axis[ax].pos_step;
                int32 acc_cap = Param_Get(AXIS_PARAM(PARAM_AXIS_MAX_ACCEL, ax));
                int32 max_acc, jerk;
                SCurve_AutoAccJerk(max_vel, acc_cap, &max_acc, &jerk);
                SCurve_Plan(&axis[ax].splanner,
                            axis[ax].actual_pos, 0,
                            max_vel, max_acc, jerk);
            }
            
            
            printf("%s: homing to 0...\r\n", axis_name[ax]);
            
            
        }
        homing_active = 1;
        return;
    }

    /* PARAM:SAVE — 掉电保存: 先禁 TIM1 ISR + 切 INIT, 保存后恢复 OP */
    if (Axis_StrIEq(work, "PARAM:SAVE")) {
#if FLASH_STORAGE_DISABLED
        printf(">>> PARAM:SAVE skipped (Flash storage disabled) <<<\r\n");
        return;
#else
        
        printf(">>> Preparing Flash save, disabling TIM1 ISR... <<<\r\n");
        HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);

        /* 切换到 INIT 状态, 防止 Flash 擦除期间 (CPU停顿1~2秒)
         * 从站 SM 看门狗超时报 EE08.6 */
        
        
        printf(">>> Switching EtherCAT to INIT... <<<\r\n");
        
        
        ec_slave[0].state = EC_STATE_INIT;
        ec_writestate(0);
        ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE);

        if(ec_slave[0].state == EC_STATE_INIT) {
            
            
            printf("  EtherCAT in INIT state, safe to erase Flash.\r\n");
            
            
        } else {
            
            
            printf("  WARNING: cannot reach INIT (state=0x%04x), saving anyway.\r\n",
                   ec_slave[0].state);
            
            
        }

        
        
        printf("  [DBG] Entering Param_SaveAll()...\r\n");
        
        
        if (Param_SaveAll()) {
            
            
            printf(">>> Params saved to Flash <<<\r\n");
            
            
        } else {
            
            
            printf("!!! Save FAILED !!!\r\n");
            
            
        }
        
        
        printf("  [DBG] Param_SaveAll() returned, entering EtherCAT_RecoverOP()...\r\n");
        
        

        /* 恢复 EtherCAT 通信到 OP 并重新使能所有轴
         * EtherCAT_RecoverOP() 内部会重新使能 TIM1 ISR */
        
        
        printf(">>> Recovering EtherCAT communication... <<<\r\n");
        
        
        if (EtherCAT_RecoverOP()) {
            
            
            printf(">>> Communication restored <<<\r\n");
            
            

            /* EtherCAT 状态机恢复后, 从站可能还锁存了 EE08.6 故障,
             * 需要对所有轴执行 CiA 402 故障复位 (与 RESET 命令相同逻辑) */
            
            
            printf(">>> Fault reset all axes... <<<\r\n");
            
            
            for (int ax = 0; ax < NUM_AXES; ax++) {
                if (ax + 1 > ec_slavecount) continue;
                axis[ax].pdo_out->ControlWord = 0x0080;
                axis[ax].fault_reset_step = 1;
                Axis_StopOne(ax);
            }
            fault_reset_requested = 1;
            
            
            printf(">>> Params saved OK <<<\r\n");
            
            
        } else {
            
            
            printf(">>> Recovery FAILED! Power cycle required <<<\r\n");
            
            
        }

        return;
#endif /* FLASH_STORAGE_DISABLED */
    }
	
    if (Axis_StrIEq(work, "PARAM:RESET")) {
#if FLASH_STORAGE_DISABLED
        HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
        Param_ResetFactory();
        RefreshAxisGearRatio();
        HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
        printf(">>> Factory reset (RAM only, Flash disabled) <<<\r\n");
        return;
#else
        /* 先禁用 TIM1 ISR, 防止 Flash 擦除期间 ISR 发帧导致从站看门狗超时 */
        
        
        printf(">>> Factory reset: disabling TIM1 ISR, erasing Flash... <<<\r\n");
        
        
        HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
        Param_ResetFactory();
        
        
        printf(">>> Factory reset done, rebooting... <<<\r\n");
        
        
        HAL_Delay(100);
        NVIC_SystemReset();
        return;
#endif /* FLASH_STORAGE_DISABLED */
    }

    /* PARAM:ID,VAL → 读写参数 */
    if ((work[0] == 'P' || work[0] == 'p')
        && (work[1] == 'A' || work[1] == 'a')
        && (work[2] == 'R' || work[2] == 'r')
        && (work[3] == 'A' || work[3] == 'a')
        && (work[4] == 'M' || work[4] == 'm')
        && work[5] == ':')
    {
        char *comma2 = strchr(work + 6, ',');
        if (comma2) {
            *comma2 = '\0';
            int ok1, ok2;
            int32 id, val;
            id  = Axis_ParsePosition(work + 6, &ok1);
            val = Axis_ParsePosition(comma2 + 1, &ok2);
            if (ok1 && ok2 && (int)id >= 0 && (int)id < PARAM_COUNT) {
                if (Param_Set((ParamId_t)(int)id, val))
                    
                    
                    printf("Param[%d] = %d\r\n", (int)id, (int)val);
                    
                    
                else
                    
                    
                    printf("? Param[%d] out of range: %d\r\n", (int)id, (int)val);
                    
                    
            } else {
                
                
                printf("? Invalid param: %s\r\n", work + 6);
                
                
            }
        } else {
            int ok;
            int32 id = Axis_ParsePosition(work + 6, &ok);
            if (ok && (int)id >= 0 && (int)id < PARAM_COUNT) {
                int32 val = Param_Get((ParamId_t)(int)id);
                const ParamDesc_t *desc = &g_param_desc_table[(int)id];
                
                
                printf("Param[%d] %-16s = %d %s\r\n",
                       (int)id, desc->name, (int)val, desc->unit);
                
                
            } else {
                
                
                printf("? Invalid param id: %s\r\n", work + 6);
                
                
            }
        }
        return;
    }

    /* MOV:X,Y,Z,R → mm move (使用脉冲定位) */
    if ((work[0] == 'M' || work[0] == 'm')
        && (work[1] == 'O' || work[1] == 'o')
        && (work[2] == 'V' || work[2] == 'v')
        && work[3] == ':')
    {
        float vals[MAX_AXES] = {0};
        char *p = work + 4;
        for (int i = 0; i < MAX_AXES; i++) {
            while (*p == ' ') p++;
            if (*p == '\0') break;
            int sign = 1;
            if (*p == '-') { sign = -1; p++; }
            else if (*p == '+') { p++; }
            float f = 0.0f;
            while (*p >= '0' && *p <= '9') { f = f * 10.0f + (float)(*p - '0'); p++; }
            if (*p == '.') {
                p++;
                float frac = 0.1f;
                while (*p >= '0' && *p <= '9') {
                    f += frac * (float)(*p - '0');
                    frac *= 0.1f;
                    p++;
                }
            }
            vals[i] = (float)sign * f;
            if (*p == ',') p++;
        }
        int32 targets[MAX_AXES];
        for (int i = 0; i < MAX_AXES; i++) {
            int32 pitch = Param_Get(AXIS_PARAM(PARAM_AXIS_PITCH, i));
            if (pitch <= 0) pitch = 5000;
            int32 eppr = axis[i].effective_ppr;
            if (eppr <= 0) eppr = GetAxisPPR(i);
            int32 delta = (int32)(vals[i] * (float)eppr * 1000.0f / (float)pitch);
            targets[i] = (int32)((int64)axis[i].actual_pos + (int64)delta);
        }
        Interp_Line(targets, g_move_rpm);

        #if printf_cmd
        printf("MOV: X=%.1f Y=%.1f Z=%.1f R=%.1f U=%.1f V=%.1f W=%.1f S=%.1f mm\r\n",
               (double)vals[0], (double)vals[1], (double)vals[2], (double)vals[3],
               (double)vals[4], (double)vals[5], (double)vals[6], (double)vals[7]);
        #endif


        return;
    }

    /* WHERE → 查询当前位置 */
    if (Axis_StrIEq(work, "WHERE")) {


        printf("--- Position ---\r\n");


        for (int ax = 0; ax < NUM_AXES; ax++) {
            if (ax + 1 > ec_slavecount) continue;
            int32 pitch = Param_Get(AXIS_PARAM(PARAM_AXIS_PITCH, ax));
            if (pitch <= 0) pitch = 5000;
            int32 eppr = axis[ax].effective_ppr;
            if (eppr <= 0) eppr = GetAxisPPR(ax);
            float mm = (float)axis[ax].actual_pos * (float)pitch / (1000.0f * (float)eppr);


            printf("  %s: pos=%d (%.1fmm)\r\n",
                   axis_name[ax],
                   (int)axis[ax].actual_pos,
                   (double)mm);
        }


        printf("  Speed: %d RPM (S-Curve)\r\n", g_move_rpm);


        return;
    }

    /* STATUS → 系统状态总览 */
    if (Axis_StrIEq(work, "STATUS")) {
        extern uint32_t g_uptime_seconds;

        printf("=== System Status ===\r\n");

        printf("Slaves: %d  FW: v%d.%d  Uptime: %us\r\n",
               ec_slavecount, FW_VERSION >> 8, FW_VERSION & 0xFF,
               (unsigned int)g_uptime_seconds);
        
        
        for (int ax = 0; ax < NUM_AXES; ax++) {
            if (ax + 1 > ec_slavecount) continue;
            
            
            printf("%s: enabled=%d fault=%d busy=%d pos=%d vel=%dRPM\r\n",
                   axis_name[ax],
                   axis[ax].enabled,
                   (axis[ax].status_word & 0x0008) ? 1 : 0,
                   axis[ax].motion_busy,
                   (int)axis[ax].actual_pos,
                   (int)(CycleVel_to_RPM(axis[ax].velocity, ax)));
            
            
        }
        return;
    }

    /* STACKSPACE → 查看各任务栈历史最小剩余空间 (单位: word, 1word=4bytes) */
    if (Axis_StrIEq(work, "STACKSPACE")) {
        TaskHandle_t h;
        
        
        printf("--- Stack High Water Mark (words free, 1word=4B) ---\r\n");
        
        

        h = xTaskGetHandle("EtherCAT");
        
        
        if (h) printf("  EtherCAT  (1280): %lu\r\n", uxTaskGetStackHighWaterMark(h));
        
        

        h = xTaskGetHandle("CmdParse");
        
        
        if (h) printf("  CmdParse  (1280): %lu\r\n", uxTaskGetStackHighWaterMark(h));
        
        

        h = xTaskGetHandle("ModbusDef");
        
        
        if (h) printf("  ModbusDef  (640): %lu\r\n", uxTaskGetStackHighWaterMark(h));
        
        

        /* 当前任务 = 发送本条命令的任务 (CmdParse) */
        
        
        printf("  (this task)     : %lu\r\n", uxTaskGetStackHighWaterMark(NULL));
        
        
        return;
    }

    char *seg = work;
    while (seg && *seg) {
        char *comma = strchr(seg, ',');
        if (comma) *comma = '\0';

        if (ParseOneSegment(seg))
            any_ok = 1;

        seg = comma ? comma + 1 : NULL;
    }
	
    if (!any_ok)
       
        printf("? %s\r\n", cmd_buf);
     
        
}

/* ================================================================
   vCmd_Task() — 串口命令解析任务
   ================================================================
 * 优先级: 3
 * 触发源: USART1 中断 → vTaskNotifyGiveFromISR
 *
 * 等待 USART1 ISR 收到完整命令后唤醒，调用 ParseCommand()
 */
void vCmd_Task(void *pvParameters)
{
    (void)pvParameters;

    /* 获取 xEtherCATTaskHandle (由 main.c 创建) */
    extern TaskHandle_t xEtherCATTaskHandle;

    
    
    printf("Cmd task started\r\n");
    

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        
        if (cmd_ready)
        {
            cmd_ready = 0;
            vTaskSuspend(xEtherCATTaskHandle);
            ParseCommand();
            vTaskResume(xEtherCATTaskHandle);
        }
        
        cmd_ready = 0;
        
    }
}
