/**
 ******************************************************************************
 * @file           : motion_profile.c
 * @brief          : S曲线7段加减速规划器实现 — 全定点整数运算
 * ----------------------------------------------------------------------------
 * 【数学模型】
 *   加加速度 Jerk J 保持恒定, 加速度 a(t) 线性变化:
 *     a(t) = ∫J dt = a₀ + J·t        (线性斜坡)
 *     v(t) = ∫a(t) dt = v₀ + a₀·t + ½·J·t²
 *     s(t) = ∫v(t) dt = s₀ + v₀·t + ½·a₀·t² + ⅙·J·t³
 *
 *   但为避免累积分误差, 本实现使用增量式:
 *     a[n+1] = a[n] + J
 *     v[n+1] = v[n] + a[n]
 *     s[n+1] = s[n] + v[n]
 *
 * 【7段有限状态机】
 *   T1(加加速): j>0, a↑ → T2(匀加速): j=0, a=Amax →
 *   T3(减加速): j<0, a↓ → T4(匀速): j=0, a=0 →
 *   T5(加减速): j<0, |a|↑ → T6(匀减速): j=0, a=-Amax →
 *   T7(减减速): j>0, |a|↓ → DONE
 *
 * 【短行程处理】
 *   当位移不足以达到 Vmax 时, 自动缩短为三角形/梯形:
 *   - 位移 < 2·Vmax·Ta: 跳过匀速段, Vpeak < Vmax
 ******************************************************************************
 */
#include "motion_profile.h"
#include "main.h"
#include <string.h>

/* ── 离散模拟器: 与SCurve_Step规则完全一致, 含所有钳位 ── */
static int32 SCurve_SimulateDisp(SCurvePlanner_t *p)
{
    int32 pos = 0, vel = 0, acc = 0;   /* 位置/速度/加速度 增量累加器 */
    int32 mv = p->max_vel, ma = p->max_acc; /* 局部副本, 避免重复解引用 */
    uint32 t;                            /* 段内周期计数器 */
    /* 加速: jerk up */
    for (t = 0; t < p->Tj; t++) {
        acc += p->jerk; if (acc > ma) acc = ma;
        vel += acc; if (vel > mv) vel = mv;
        pos += vel;
    }
    /* 加速: const accel */
    for (t = 0; t < p->Ta; t++) {
        vel += acc; if (vel > mv) vel = mv;
        pos += vel;
    }
    /* 加速: jerk down */
    for (t = 0; t < p->Tj; t++) {
        acc -= p->jerk; if (acc < 0) acc = 0;
        vel += acc; if (vel > mv) vel = mv;
        pos += vel;
    }
    /* 匀速 */
    for (t = 0; t < p->Tv; t++) { pos += vel; }
    /* 减速: jerk up */
    for (t = 0; t < p->Tj2; t++) {
        acc -= p->jerk; if (acc < -ma) acc = -ma;
        vel += acc; if (vel < 0) vel = 0;
        pos += vel;
    }
    /* 减速: const decel */
    for (t = 0; t < p->Td; t++) {
        vel += acc; if (vel < 0) vel = 0;
        pos += vel;
    }
    /* 减速: jerk down */
    for (t = 0; t < p->Tj2; t++) {
        acc += p->jerk; if (acc > 0) acc = 0;
        vel += acc; if (vel < 0) vel = 0;
        pos += vel;
    }
    return pos;  /* 返回模拟的总位移 (脉冲) */
}

/* ── 整数立方根近似 (Newton-Raphson, 用于短行程 Vpeak 计算) ── */
int32 icbrt(int64 x)
{
    if (x <= 0) return 0;
    int32 r = 1;
    /* 第1步: 倍增找上界 — 找到最小的2^k使得(2^k)³ ≥ x */
    while ((int64)r * r * r < x) r <<= 1;
    /* 回退一倍作为初始猜测 (介于真实根值的 0.5~1.0 倍之间) */
    r >>= 1;
    if (r < 1) r = 1;
    /* 第2步: Newton迭代 — r_{n+1} = (2r_n + x/r_n²) / 3, 最多5次 */
    for (int i = 0; i < 5; i++) {
        int64 r2 = (int64)r * r;    /* r², 避免溢出 */
        if (r2 == 0) break;         /* r=0 时防除零 */
        int32 next = (int32)(((int64)2 * r + x / r2) / 3);
        if (next == r) break;       /* 收敛: 连续两次结果一致 */
        r = next;
    }
    return r;  /* 返回 floor(∛x) */
}

/* ── 离散加速段模拟: 返回 {位移, 末速度} ──
 * J+(Tj) → Ta → J-(Tj), 与 SCurve_Step 钳位规则完全一致 */
static void SimAccelPhase(uint32 Tj, uint32 Ta, int32 amax, int32 jerk, int32 vmax,
                          int32 *out_disp, int32 *out_vel)
{
    int32 pos = 0, vel = 0, acc = 0;
    uint32 t;
    for (t = 0; t < Tj; t++) {           /* J+: a=0→Amax */
        acc += jerk; if (acc > amax) acc = amax;
        vel += acc;  if (vel > vmax) vel = vmax;
        pos += vel;
    }
    for (t = 0; t < Ta; t++) {           /* Ta: a=Amax 恒定 */
        vel += acc;  if (vel > vmax) vel = vmax;
        pos += vel;
    }
    for (t = 0; t < Tj; t++) {           /* J-: a=Amax→0 */
        acc -= jerk; if (acc < 0)   acc = 0;
        vel += acc;  if (vel > vmax) vel = vmax;
        pos += vel;
    }
    *out_disp = pos;
    *out_vel  = vel;
}

/* ── 离散减速段模拟: 从 cruise_vel 减速到0, 返回减速距离 ──
 * DJ+(Tj2) → Td → DJ-(Tj2), 与 SCurve_Step 钳位规则完全一致 */
static int32 SimDecelDist(int32 cruise_vel, uint32 Tj2, uint32 Td,
                          int32 amax, int32 jerk)
{
    int32 vel = cruise_vel, acc = 0, dist = 0;
    uint32 t;
    for (t = 0; t < Tj2; t++) {          /* DJ+: a=0→-Amax */
        acc -= jerk; if (acc < -amax) acc = -amax;
        vel += acc;  if (vel <= 0)  { vel = 0; break; }
        dist += vel;
    }
    for (t = 0; t < Td;  t++) {          /* Td:  a=-Amax 恒定 */
        vel += acc;  if (vel <= 0)  { vel = 0; break; }
        dist += vel;
    }
    for (t = 0; t < Tj2; t++) {          /* DJ-: a=-Amax→0 */
        acc += jerk; if (acc > 0)   acc = 0;
        vel += acc;  if (vel <= 0)  { vel = 0; break; }
        dist += vel;
    }
    return dist;
}

/* ── 初始化 ──
 * 将规划器全部字段清零, 设为"已完成"状态。
 * direction 默认为正向 (+1), 后续 Plan() 会根据目标位置自动修正。 */
void SCurve_Init(SCurvePlanner_t *p)
{
    memset(p, 0, sizeof(*p));
    p->segment = SCURVE_DONE;     /* 初始状态: 无运动指令 */
    p->direction = 1;             /* 默认正向运动 */
}

/* ── 规划 ── */
void SCurve_Plan(SCurvePlanner_t *p,
                 int32 start, int32 target,
                 int32 max_vel, int32 max_acc, int32 jerk)
{
    memset(p, 0, sizeof(*p));

    p->start_pos   = start;
    p->target_pos  = target;
    p->current_pos = start;
    p->current_vel = 0;
    p->current_acc = 0;
    p->max_vel     = max_vel;
    p->max_acc     = max_acc;
    p->jerk        = (jerk > 0) ? jerk : 1;

    /* ── 1. 方向 & 位移 ── */
    int32 disp = target - start;
    if (disp < 0) {
        p->direction = -1;
        disp = -disp;
    } else if (disp > 0) {
        p->direction = 1;
    } else {
        p->segment = SCURVE_DONE;
        p->current_pos = target;
        return;
    }

    /* ================================================================
     * 2. 计算 Tj (Jerk时间) — 纯离散公式
     * ================================================================
     * Tj = Amax / J  (整数截断)
     * 约束: J+ 终点速度 ≤ Vmax → J·Tj·(Tj+1)/2 ≤ Vmax
     * ================================================================ */
    p->Tj = max_acc / p->jerk;
    if (p->Tj < 1) p->Tj = 1;

    while (p->Tj > 1 &&
           (int64)p->jerk * p->Tj * (p->Tj + 1) / 2 > (int64)max_vel) {
        p->Tj--;
    }
    p->Tj2 = p->Tj;  /* 对称: 离散域加减速 Jerk 时间相等 */

    /* 实际 Amax = Tj·J (整数截断后的真实值, ≤ 用户设定) */
    int32 amax = (int32)p->Tj * p->jerk;

    /* ================================================================
     * 3. 计算 Ta / Td — 离散闭式公式
     * ================================================================
     * 加速段速度增量:
     *   J+ :  Δv₁ = J·Tj·(Tj+1)/2
     *   Ta :  Δv₂ = Amax·Ta
     *   J- :  Δv₃ = J·Tj·(Tj-1)/2
     *   总和: Vmax = Amax·(Tj+Ta)  →  Ta = Vmax/Amax - Tj
     *
     * 减速段对称性 (离散推导):
     *   DJ+ 速度损失 = J·Tj·(Tj+1)/2
     *   DJ- 需要初速 = J·Tj·(Tj-1)/2
     *   →  Td = Vmax/Amax - Tj = Ta  (时间对称, 位移不对称)
     * ================================================================ */
    int32 ta = 0;
    if (amax > 0) {
        ta = max_vel / amax - (int32)p->Tj;
        if (ta < 0) ta = 0;
    }
    /* Td = Ta: 离散域时间参数严格对称, 位移不对称由下方模拟体现 */
    int32 td = ta;

    /* ================================================================
     * 4. 离散模拟: 计算加速位移、巡航速度、减速距离
     * ================================================================
     * 位移不能用闭式 (涉及 J±/DJ± 钳位, 公式过于复杂),
     * 改为离散模拟 — 与 SCurve_Step 逐周期行为完全一致。
     * ================================================================ */
    int32 accel_disp, cruise_vel;
    SimAccelPhase(p->Tj, (uint32)ta, amax, p->jerk, max_vel,
                  &accel_disp, &cruise_vel);

    int32 decel_disp = SimDecelDist(cruise_vel, p->Tj2, (uint32)td,
                                    amax, p->jerk);

    /* ================================================================
     * 5. 判断 7段 / 短行程
     * ================================================================ */
    int32 min_disp = accel_disp + decel_disp;

    if (disp >= min_disp && ta > 0) {
        /* ── 完整7段: 有匀速段 ── */
        p->Ta = (uint32)ta;
        p->Td = (uint32)td;
        p->Tv = (uint32)(((int64)disp - min_disp) / cruise_vel);
        p->max_acc = amax;
        /* max_vel 保持用户设定 (用于钳位, 实际巡航速度 ≤ max_vel) */
        p->max_vel = max_vel;

    } else {
        /* ── 短行程: 位移不足以达到 Vmax → 三角形 S 曲线 ── */
        p->Ta = 0;
        p->Td = 0;
        p->Tv = 0;

        /* icbrt 提供初始 Tj 猜测 (纯整数, 避免从1扫描) */
        p->Tj = (uint32)icbrt(disp / (int64)(2 * p->jerk));
        if (p->Tj < 1) p->Tj = 1;
        p->Tj2 = p->Tj;
        p->max_acc = (int32)p->Tj * p->jerk;
        /* 短行程峰值速度 ≤ 用户 Vmax (由 SimulateDisp 内钳位保证) */
        p->max_vel = max_vel;

        /* 搜索 Tj: 递增到位移足够, 再递减消除过冲 */
        {
            int32 sim_d = SCurve_SimulateDisp(p);
            while (sim_d < disp) {
                p->Tj++;
                p->Tj2 = p->Tj;
                p->max_acc = (int32)p->Tj * p->jerk;
                sim_d = SCurve_SimulateDisp(p);
            }
            while (sim_d > disp && p->Tj > 1) {
                p->Tj--;
                p->Tj2 = p->Tj;
                p->max_acc = (int32)p->Tj * p->jerk;
                sim_d = SCurve_SimulateDisp(p);
            }
        }

        /* 短行程减速距离: 重新模拟 (Tj 已变) */
        {
            int32 dv;
            SimAccelPhase(p->Tj, 0, p->max_acc, p->jerk, max_vel,
                          &accel_disp, &dv);
            decel_disp = SimDecelDist(dv, p->Tj2, 0,
                                      p->max_acc, p->jerk);
        }
    }

    /* ================================================================
     * 6. 离散验证 & 微量修正 (处理整数舍入)
     * ================================================================
     * 用 SCurve_SimulateDisp 验证总位移, 通过 ±Tv 微调,
     * 残余 |error| < max_vel 交由 SCurve_Step 末尾到位收尾处理。
     * ================================================================ */
    {
        int32 sim_disp = SCurve_SimulateDisp(p);
        int32 error    = disp - sim_disp;

        /* 位移不足 → 加 Tv 周期 */
        if (error > 0 && p->max_vel > 0) {
            uint32 tv_add = (uint32)(error / (int32)p->max_vel);
            if (tv_add > 0) {
                p->Tv += tv_add;
                sim_disp = SCurve_SimulateDisp(p);
                error = disp - sim_disp;
            }
        }

        /* 位移过大 (罕见: 仅短行程可能出现) → 缩 Tj */
        if (error < 0 && p->Tj > 1 && p->Ta == 0) {
            int loops = 0;
            while (error < 0 && p->Tj > 1 && loops < 6) {
                p->Tj--;
                p->Tj2 = p->Tj;
                p->max_acc = (int32)p->Tj * p->jerk;
                sim_disp = SCurve_SimulateDisp(p);
                error = disp - sim_disp;
                loops++;
            }
        }
    }

    /* ================================================================
     * 7. 存储减速距离 (用于 CONST_VEL 位置触发)
     * ================================================================
     * 使用已计算的 decel_disp; 若短行程 Tj 变化后未更新, 重新模拟。
     * ================================================================ */
    p->decel_dist = decel_disp;

    /* ── 启动状态机: 从 T1 加加速段开始 ── */
    p->segment     = SCURVE_ACC_JERK_UP;
    p->seg_elapsed = 0;
}

/* ── 每周期步进 ── */
int32 SCurve_Step(SCurvePlanner_t *p)
{
    if (p->segment == SCURVE_DONE || p->segment == SCURVE_IDLE)
        return p->current_pos;

    p->seg_elapsed++;

    /* ── 有限状态机 ── */
    switch (p->segment) {

    case SCURVE_ACC_JERK_UP: {
        /* T1: 加加速段 — a(t)=J·t, a从0线性增大到Amax
         * 增量: a+=J, v+=a, s+=v (离散积分, 避免累积误差) */
        p->current_acc += p->jerk;
        if (p->current_acc > p->max_acc)
            p->current_acc = p->max_acc;    /* 钳位到最大加速度 */
        p->current_vel += p->current_acc;
        if (p->current_vel > p->max_vel)
            p->current_vel = p->max_vel;    /* 钳位到最大速度 */

        if (p->seg_elapsed >= p->Tj) {
            /* T1到期: 保存边界状态 → 判断下一段 */
            p->v_at_const_start = p->current_vel;
            p->s_at_const_start = p->current_pos;

            if (p->Ta > 0) {
                p->segment = SCURVE_ACC_CONST;     /* → T2 匀加速 */
            } else {
                /* Ta=0 (短行程): 跳过匀加速段
                 * 仅当速度已到 max_vel 才进匀速段 (Tv>0不构成跳过J-的理由) */
                if (p->current_vel >= p->max_vel)
                    p->segment = SCURVE_CONST_VEL; /* → T4 匀速 */
                else
                    p->segment = SCURVE_ACC_JERK_DOWN; /* → T3 减加速 */
            }
            p->seg_elapsed = 0;
        }
        break;
    }

    case SCURVE_ACC_CONST: {
        /* T2: 匀加速段 — a=Amax恒定, v线性增大
         * 增量: v += Amax (a不变) */
        p->current_vel += p->max_acc;
        if (p->current_vel > p->max_vel)
            p->current_vel = p->max_vel;

        if (p->seg_elapsed >= p->Ta) {
            /* T2到期 → T3 减加速段 (即使速度已达Vmax也不跳过) */
            p->segment = SCURVE_ACC_JERK_DOWN;
            p->seg_elapsed = 0;
        }
        break;
    }

    case SCURVE_ACC_JERK_DOWN: {
        /* T3: 减加速段 — a从Amax线性减小到0, v继续增大但增速放缓
         * 增量: a -= J, v += a (a>0所以v仍在增加) */
        p->current_acc -= p->jerk;
        if (p->current_acc < 0) p->current_acc = 0;   /* a钳位到0 (不低于0) */
        p->current_vel += p->current_acc;
        if (p->current_vel > p->max_vel)
            p->current_vel = p->max_vel;

        /* 退出条件: Tj周期到期 或 加速度已归零 (短行程可能提前结束) */
        if (p->seg_elapsed >= p->Tj || p->current_acc <= 0) {
            p->current_acc = 0;   /* 强制归零, 避免积分漂移 */
            p->v_at_vel_start = p->current_vel;
            p->s_at_vel_start = p->current_pos;

            if (p->Tv > 0)
                p->segment = SCURVE_CONST_VEL;     /* → T4 匀速 */
            else {
                /* 无匀速段: 记录减速起始状态, 直接进入减速 */
                p->v_at_dec_start = p->current_vel;
                p->s_at_dec_start = p->current_pos;
                p->segment = SCURVE_DEC_JERK_UP;   /* → T5 加减速 */
            }
            p->seg_elapsed = 0;
        }
        break;
    }

    case SCURVE_CONST_VEL: {
        /* T4: 匀速段 — a=0, v=Vmax恒定
         * 位置触发减速: 预扣本周期行程 (pos在switch后才+=vel)
         * → 触发时实际剩余 = remain - vel, 提前一个周期进入减速,
         *   确保减速距离精确匹配, 不会冲过目标。 */
        p->current_acc = 0;
        /* 计算到目标的剩余距离 (方向感知) */
        int32 remain = (p->direction > 0)
            ? (p->target_pos - p->current_pos)
            : (p->current_pos - p->target_pos);
        if (remain - p->current_vel <= p->decel_dist) {
            /* 剩余距离 ≤ 减速距离 → 触发减速 */
            p->v_at_dec_start = p->current_vel;
            p->s_at_dec_start = p->current_pos;
            p->segment = SCURVE_DEC_JERK_UP;   /* → T5 加减速 */
            p->seg_elapsed = 0;
        }
        break;
    }

    case SCURVE_DEC_JERK_UP: {
        /* T5: 加减速段 — a从0线性降到-Amax, v开始减小
         * a(t) = -J·t → 负向增大
         * v(t) = Vcruise - ½·J·t² */
        p->current_acc -= p->jerk;
        if (p->current_acc < -p->max_acc)
            p->current_acc = -p->max_acc;     /* 钳位到负最大加速度 */
        p->current_vel += p->current_acc;      /* a<0 → v减小 */
        if (p->current_vel < 0) p->current_vel = 0;

        if (p->seg_elapsed >= p->Tj2) {
            if (p->Td > 0)
                p->segment = SCURVE_DEC_CONST;      /* → T6 匀减速 */
            else
                p->segment = SCURVE_DEC_JERK_DOWN;  /* → T7 减减速 (短行程跳过T6) */
            p->seg_elapsed = 0;
        }
        break;
    }

    case SCURVE_DEC_CONST: {
        /* T6: 匀减速段 — a=-Amax恒定, v线性减小
         * v -= Amax (直接减, 与加速段对称) */
        p->current_vel -= p->max_acc;
        if (p->current_vel < 0) p->current_vel = 0;

        if (p->seg_elapsed >= p->Td) {
            p->segment = SCURVE_DEC_JERK_DOWN;  /* → T7 减减速 */
            p->seg_elapsed = 0;
        }
        break;
    }

    case SCURVE_DEC_JERK_DOWN: {
        /* T7: 减减速段 — a从-Amax线性回到0, v→0
         * a += J (由负向零恢复), v += a (a<0时v继续减小, a→0时v→0) */
        p->current_acc += p->jerk;
        if (p->current_acc > 0) p->current_acc = 0;   /* a钳位到0 (不高于0) */
        p->current_vel += p->current_acc;
        if (p->current_vel < 0) p->current_vel = 0;

        /* 退出条件: Tj2周期到期 或 速度已降到Jerk量级
         * vel≤jerk替代vel≤0: 整数运算下速度可能停在1~jerk-1无法归零,
         * acc已被clamp到0, 速度不再变化, 提前退出避免长时间低速蠕动 */
        if (p->seg_elapsed >= p->Tj2 || p->current_vel <= p->jerk) {
            p->current_vel = 0;
            p->current_acc = 0;
            p->segment = SCURVE_DONE;
            p->seg_elapsed = 0;
            /* 位置积分已前移, 减速距离精确, DJ-自然停在目标 ±1脉冲 */
        }
        break;
    }

    default:
        break;
    }

    /* ── 位置积分 (段逻辑之后, 使用本周期已更新的 vel) ──
     * 在 switch 外统一执行: 所有状态的位置更新都走这条路。
     * direction 决定了运动方向 (±1), 负方向时位置递减。 */
    p->current_pos += p->current_vel * p->direction;

    /* ── 到位收尾: 仅处理整数截断导致的微小残余(≤1~2个脉冲)
     *   ⚠️ 绝对不能做大幅跳变, SV660N 会检测单周期位置增量并报 0x6320。
     *   CONST_VEL 修复已确保减速距离精确, 此处只兜底最后的整数舍入误差。 */
    {
        int32 remain = p->direction > 0 ? (p->target_pos - p->current_pos)
                      : (p->current_pos - p->target_pos);
        if (remain < 0) remain = -remain;

        /* 仅当 S 曲线已自然降到极低速 (≈1 RPM 级别) 且残余 ≤ 2 个脉冲时才收尾 */
        if (p->current_vel <= SCURVE_STOP_THRESH &&
            remain <= 2 &&
            p->segment != SCURVE_DONE) {
            /* 残余 1~2 脉冲, 直接到位 (SV660N 不会报警) */
            p->current_pos = p->target_pos;
            p->current_vel = 0;
            p->current_acc = 0;
            p->segment = SCURVE_DONE;
        }
        /* 残余 > 2 脉冲: 不干预, 让 S 曲线自然走完。
         * 最终位置可能有数个脉冲的偏差 (亚微米级), 下一条指令会从实际位置起算,
         * 不影响绝对精度。 */
    }

    return p->current_pos;
}

/* ── 查询完成 ──
 * 返回 1: 规划器空闲, 运动已完成或未启动
 * 返回 0: 运动中 */
int SCurve_IsDone(SCurvePlanner_t *p)
{
    return (p->segment == SCURVE_DONE) ? 1 : 0;
}

/* ── 平滑停止 ──
 * 从当前速度/加速度状态生成S曲线减速到0, 不依赖之前的SCurve_Plan参数。
 * 通过离散模拟精确计算Tj2/Td/stop_dist, 保证与SCurve_Step行为逐周期一致。
 *
 * 调用时机: 外部急停/终止指令, 在任何运动段中均可触发。
 * 核心步骤:
 *   ① 计算Tj2 = Amax/J, 检查退化条件 (v≤0 等直接置DONE)
 *   ② 离散模拟整条减速曲线 (J+ → Td → J-), 得到精确 stop_dist
 *   ③ 记录减速起始边界, 置 segment = DEC_JERK_UP 进入减速 */
void SCurve_Stop(SCurvePlanner_t *p)
{
    if (p->segment == SCURVE_DONE || p->segment == SCURVE_IDLE)
        return;

    /* 已处于减速段 (DEC_JERK_UP / DEC_CONST / DEC_JERK_DOWN):
     * 无需重复触发, 直接返回。否则会重置 current_acc=0 导致减速进度丢失,
     * 以及目标位置重新计算可能造成 planner 状态混乱 */
    if (p->segment == SCURVE_DEC_JERK_UP ||
        p->segment == SCURVE_DEC_CONST ||
        p->segment == SCURVE_DEC_JERK_DOWN)
        return;

    int32 v  = p->current_vel;
    int32 ma = p->max_acc;
    int32 J  = p->jerk;

    if (v <= 0 || ma <= 0 || J <= 0) {
        /* 退化条件: 已停止或参数无效 → 直接完成 */
        p->segment    = SCURVE_DONE;
        p->current_acc = 0;
        p->current_vel = 0;
        return;
    }

    /* ── 1. 计算 Jerk 段时间: Tj2 = Amax / J ── */
    p->Tj2 = ma / J;
    if (p->Tj2 < 1) p->Tj2 = 1;

    /* ── 2. 离散模拟整条减速曲线, 计算精确 stop_dist 和 Td ──
     *     模拟规则与 SCurve_Step 完全一致 (含所有钳位)
     *     Phase1: 加减速 (a: 0 → -Amax,  jerk=-J)
     *     Phase2: 匀减速 (a = -Amax)
     *     Phase3: 减减速 (a: -Amax → 0, jerk=+J)
     * ── */
    {
        int32 sv = v, sa = 0;     /* 模拟速度/加速度 */
        int32 dist = 0;            /* 累计减速距离 */
        int32 v_before_const;      /* 匀减速段起始速度 */
        uint32 t;

        /* Phase 1: DEC_JERK_UP */
        for (t = 0; t < p->Tj2; t++) {
            sa -= J;  if (sa < -ma) sa = -ma;
            sv += sa; if (sv <= 0) { sv = 0; break; }
            dist += sv;
        }
        v_before_const = sv;

        /* Phase 2: DEC_CONST 需要的 Td
         * DJ-段完整走完Tj2周期需要的初速度:
         *   v_entry = Σ(ma - i*J)  i=0..Tj2-1
         *           = ma*Tj2 - J*Tj2*(Tj2-1)/2
         */
        int32 v_djd_entry = ma * (int32)p->Tj2
                          - J * (int32)p->Tj2 * (int32)(p->Tj2 - 1) / 2;
        int32 v_budget = v_before_const - v_djd_entry;
        if (v_budget < 0) v_budget = 0;
        p->Td = (uint32)(v_budget / ma);

        /* Phase 2: DEC_CONST */
        for (t = 0; t < p->Td; t++) {
            sv -= ma;  if (sv <= 0) { sv = 0; break; }
            dist += sv;
        }

        /* Phase 3: DEC_JERK_DOWN */
        for (t = 0; t < p->Tj2; t++) {
            sa += J;  if (sa > 0) sa = 0;
            sv += sa; if (sv <= 0) { sv = 0; break; }
            dist += sv;
        }

        p->decel_dist = dist;      /* 精确减速距离, 供后续位置触发使用 */

        /* 设置目标位置: current_pos ± dist (方向感知)
         * 停止后位置 = 当前位置 + 减速所需的位移 */
        if (p->direction > 0)
            p->target_pos = p->current_pos + dist;
        else
            p->target_pos = p->current_pos - dist;
    }

    /* ── 3. 记录减速起始边界 ──
     * 保存当前状态快照, 供进度计算和调试使用 */
    p->start_pos       = p->current_pos;
    p->v_at_dec_start  = v;              /* 减速起始速度 */
    p->s_at_dec_start  = p->current_pos; /* 减速起始位置 */

    /* ── 4. 进入减速段 ──
     * 从 T5 (DEC_JERK_UP) 开始, 与实际 S 曲线减速入口一致 */
    p->segment     = SCURVE_DEC_JERK_UP;
    p->seg_elapsed = 0;
    p->current_acc = 0;  /* 从 a=0 开始减速, 符合 DJ+ 入口条件 */
}

/* ── 进度百分比 ──
 * 返回千分比 (0~1000), 避免浮点运算。
 * 1000 = 100.0% (已完成), 0 = 0.0% (刚开始)。
 * 基于位置计算: done/total × 1000, 方向无关 (均取绝对值)。 */
int32 SCurve_Progress(SCurvePlanner_t *p)
{
    if (p->segment == SCURVE_DONE) return 1000;       /* 已完成 */
    if (p->start_pos == p->target_pos) return 1000;   /* 零行程, 视为完成 */

    int32 total = p->target_pos - p->start_pos;       /* 总行程 (带符号) */
    if (total < 0) total = -total;                    /* 取绝对值 */
    int32 done  = p->current_pos - p->start_pos;      /* 已完成行程 */
    if (done < 0) done = -done;

    if (total == 0) return 0;                         /* 防除零 */
    int32 prog = (int32)((int64)done * 1000 / total); /* 千分比, int64防溢出 */
    return (prog > 1000) ? 1000 : (prog < 0) ? 0 : prog;  /* 钳位 [0, 1000] */
}

/* ── 在线重规划 ──
 * 运动中动态变更目标位置 (On-The-Fly)。
 * 从当前运动状态出发, 以 current_pos 为新起点重新规划到 new_target。
 *
 * 当前实现: 简单重规划 (Plan 会清零 vel/acc, 从静止开始重新规划)。
 * 生产增强方向: 保留当前 vel/acc 作为 SCurve_Plan 的初速/初加速输入,
 *   实现速度前馈 (velocity feed-forward), 使重规划后运动更平滑。 */
void SCurve_Replan(SCurvePlanner_t *p, int32 new_target)
{
    SCurve_Plan(p, p->current_pos, new_target,
                p->max_vel, p->max_acc, p->jerk);
}
