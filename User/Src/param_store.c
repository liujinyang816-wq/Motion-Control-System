/**
 ******************************************************************************
 * @file           : param_store.c
 * @brief          : 参数持久化存储 — STM32H7 内部Flash 擦除+写入
 * ----------------------------------------------------------------------------
 * 【存储方案】
 *   使用 STM32H7 Bank2 最后 128KB Sector (Sector7, 0x081E0000)
 *   单槽存储: 上电读取 → 校验 → 加载 / 失败则用默认值
 *
 * 【数据格式】
 *   [0]: Magic (0x5041524D "PARM")
 *   [1]: Version (递增计数器)
 *   [2]: Data CRC32
 *   [3]: Parameter Count
 *   [4..N]: int32 values[PARAM_COUNT]
 *
 *   Slot大小: 544B (528B有效 + 16B对齐) = 17×32B Flash Word 对齐
 *
 * 【Flash 操作时序】
 *   擦除 128KB 扇区约需 1~2 秒。
 *   ⚠️ 调用 Param_SaveAll() 之前，调用者必须先断开 EtherCAT (进入 INIT 状态)，
 *   写入完成后再恢复 EtherCAT。否则擦除期间的 CPU 停顿会导致从站看门狗超时。
 *
 * 【Flash 位置选择】
 *   使用 Bank2 (0x08100000), 与代码所在的 Bank1 (0x08000000) 分离。
 *   STM32H7 双 Bank 支持 RWW: Bank2 擦写时 CPU 从 Bank1 正常取指令。
 *
 * 【电源要求】
 *   ⚠️ Flash 擦除需要 VDD 稳定 (纹波 < 50mV)。
 *   板载 24V→3.3V 开关电源纹波过大会导致 Flash 电荷泵工作异常、擦除卡死。
 *   调试时仅使用 JLINK USB 供电即可。
 *
 * 【Flash 寿命】
 *   STM32H7 内部Flash 擦写次数: ~10,000 次
 *   每次 PARAM:SAVE 擦除1次, 按每天10次保存计算:
 *   10,000÷10÷365 ≈ 2.7年 → 原型阶段可接受
 ******************************************************************************
 */
#include "main.h"
#include "param_defs.h"
#include "ethercat_slave.h"
#include "motor_axis.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ── Flash 地址 (Bank2, Sector7, 128KB) ── */
#define FLASH_SECTOR_PARAM    FLASH_SECTOR_7
#define FLASH_PARAM_BANK      FLASH_BANK_2     /**< Bank2 独立擦写, RWW 不阻塞 Bank1 */
#define FLASH_PARAM_BASE      0x081E0000UL     /**< Bank2 Sector7 起始地址 */
#define FLASH_SLOT_OFFSET     0x000UL          /**< 单槽: 偏移0 */

#define PARAM_MAGIC           0x5041524DUL    /**< "PARM" */

/* ── 存储结构 (544B = 17×32B, Flash Word 对齐) ── */
typedef struct {
    uint32 magic;             /**< 魔数 */
    uint32 version;           /**< 版本号 (递增) */
    uint32 crc;               /**< 数据区 CRC32 */
    uint32 count;             /**< 参数个数 */
    int32  values[PARAM_COUNT]; /**< 参数值数组 (512B) */
    uint32 _pad[4];           /**< 补齐到 544B (17×32), Flash Word 对齐 */
} ParamSlot_t;

/* ── 运行时参数缓存 ── */
int32 param_ram[PARAM_COUNT];

/* ── 当前存储版本号 (0=从未保存, 使用默认值) ── */
static uint32 g_store_version = 0;

/* ── 掉电保存槽运行时状态 ──
 * g_pf_saved_pos[] : 从掉电槽读取的各轴位置 (上电恢复用)
 * g_pf_has_valid   : 1=掉电槽校验通过, 数据有效; 0=无效/空
 * g_pf_counter     : 掉电写入序号 (递增, 用于调试确认是新数据)
 * ⚠️ Param_SaveAll() 擦除扇区后会将 has_valid/counter 清零,
 *    因为掉电槽也被一同擦除了 */
static int32  g_pf_saved_pos[MAX_AXES];
static uint8  g_pf_has_valid = 0;
static uint32 g_pf_counter   = 0;

/* ── CRC32 表 (IEEE 802.3 多项式 0xEDB88320, 等效 0x04C11DB7) ──
 * 查表法: 每次处理 1 字节, 8 次查表移位, O(n) 时间复杂度
 * 用于校验 Flash 中存储的参数数据完整性 */
static const uint32 crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
    0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
    0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
    0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
    0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
    0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
    0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
    0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
    0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
    0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
    0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
    0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
    0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
    0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
    0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
    0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
    0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
    0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
    0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
    0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
    0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB30A04,
    0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
    0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
    0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
    0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
    0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
    0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
    0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/* ── CRC32 计算 (IEEE 802.3, 查表法) ──
 * 输入: data=数据指针, len=字节数
 * 输出: CRC32 校验值 (32位)
 * 初始值 0xFFFFFFFF, 结果异或 0xFFFFFFFF (标准后处理)
 * 用于校验 Flash 参数槽数据完整性 */
static uint32 CRC32_Calc(const uint8 *data, uint32 len)
{
    uint32 crc = 0xFFFFFFFF;                     /* 初始值全1 */
    for (uint32 i = 0; i < len; i++)
        crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFF];  /* 逐字节查表 */
    return crc ^ 0xFFFFFFFF;                     /* 最终异或 (标准要求) */
}

/* ── 从 Flash 读取参数槽, 校验合法性 ──
 * 返回 1=有效, 0=无效 (魔数/数量/CRC 任一不通过) */
static int ParamSlot_Read(uint32 offset, ParamSlot_t *slot)
{
    uint32 addr = FLASH_PARAM_BASE + offset;
    /* 直接从 Flash 地址 memcpy (Bank2 可读, 不需要特殊的 Flash 读 API) */
    memcpy(slot, (void*)addr, sizeof(ParamSlot_t));

    /* 三重校验: 魔数 → 数量 → CRC32 */
    if (slot->magic != PARAM_MAGIC) return 0;   /* 未写入过或数据损坏 */
    if (slot->count > PARAM_COUNT)  return 0;   /* 跨版本不兼容 */

    uint32 calc_crc = CRC32_Calc((uint8*)slot->values,
                                 slot->count * sizeof(int32));
    if (calc_crc != slot->crc) return 0;        /* 数据位翻转 */

    return 1;
}

/* ── 擦除扇区 + 写入槽 ──
 * ⚠️ 耗时约 1~2 秒，调用前必须断开 EtherCAT！
 * ⚠️ 需要 VDD 稳定 (纹波 < 50mV)，否则 Flash 电荷泵可能卡死 */
static int ParamSlot_EraseAndWrite(uint32 offset, ParamSlot_t *slot)
{
    uint32 addr = FLASH_PARAM_BASE + offset;
    uint32 errors;

    /* ═══════════════════════════════════════════════════════════
     * 【关键】所有操作使用直接寄存器访问, 不调用任何 HAL 函数。
     * HAL Flash 函数可能在 Bank2, 擦除 Bank2 期间无法访问。
     * param_store.o 本身在 Bank1, 直接寄存器操作不受影响。
     * ═══════════════════════════════════════════════════════════ */

    /* ── Step 1: 解锁 Bank 2 ── */
    #if printf_cmd
    printf("  [DBG] Flash: unlocking Bank2...\r\n");
    #endif
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        FLASH->KEYR2 = 0x45670123U;
        FLASH->KEYR2 = 0xCDEF89ABU;
    }
    if (FLASH->CR2 & FLASH_CR_LOCK) {
        #if printf_cmd
        printf("  [Param] Flash unlock FAILED (CR2 still locked)\r\n");
        #endif
        return 0;
    }
    #if printf_cmd
    printf("  [DBG] Flash: unlocked, erasing sector...\r\n");
    #endif

    /* ── Step 2: 等待 Bank 2 空闲 + 清除残留错误标志 ── */
    {
        uint32 timeout = 100000000U;
        while (FLASH->SR2 & FLASH_SR_QW) {
            if (--timeout == 0U) {
                #if printf_cmd
                printf("  [Param] Flash busy timeout before erase\r\n");
                #endif
                FLASH->CR2 |= FLASH_CR_LOCK;
                return 0;
            }
        }
    }
    errors = FLASH->SR2 & 0x07FE0000U;  /* 所有错误位 (bit17~26) */
    if (errors) {
        FLASH->CCR2 = errors;           /* 写1清零 */
    }

    /* ── Step 3: 擦除 Sector 7 ──
     * CR2 bits: SER(bit2)=1, PSIZE(bit4-5)=0b10(VoltageRange3),
     *            SNB(bit8-10)=7, START(bit7)=1
     * ⚠️ CR2 START 写入后 Bank2 立即进入擦除, 此后直到 BSY 清零前
     *    不能调用任何函数 (包括 printf), 因为被调函数可能在 Bank2! */
    #if printf_cmd
    printf("  [DBG] Flash: SR2 before erase=0x%08lx, CR2=0x%08lx, starting...\r\n",
           (unsigned long)FLASH->SR2, (unsigned long)FLASH->CR2);
    #endif

    FLASH->CR2 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
    FLASH->CR2 |= (FLASH_CR_SER
                   | FLASH_CR_PSIZE_1            /* VoltageRange 3 = 32bit */
                   | (7U << FLASH_CR_SNB_Pos)    /* Sector 7 */
                   | FLASH_CR_START);            /* 启动擦除! Bank2 从此不可访问 */
    __DSB();
    __ISB();
    /* ═══════════════════════════════════════════════════════════
     * 从此处开始直到 BSY 清零, 禁止调用任何函数!
     * 包括 printf / HAL_Delay 等, 因为它们可能在 Bank2 中.
     * ═══════════════════════════════════════════════════════════ */

    /* ── Step 4: 等待擦除完成 (纯寄存器轮询, 零函数调用) ── */
    {
        uint32 timeout = 500000000U;
        while (FLASH->SR2 & FLASH_SR_BSY) {
            if (--timeout == 0U) {
                /* BSY 超时, 此时 Bank2 仍不可访问, 不能调用 printf */
                FLASH->CR2 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
                FLASH->CR2 |= FLASH_CR_LOCK;
                return 0;  /* 调用者会打印错误 */
            }
        }
    }

    /* ═══════════════════════════════════════════════════════════
     * BSY 已清零, Bank2 擦除完成, 可以安全调用函数了!
     * ═══════════════════════════════════════════════════════════ */

    /* ── Step 5: 检查擦除结果 ── */
    errors = FLASH->SR2 & 0x07FE0000U;
    #if printf_cmd
    printf("  [DBG] Flash: erase done, SR2=0x%08lx %s\r\n",
           (unsigned long)FLASH->SR2, errors ? "(ERROR!)" : "");
    #endif

    if (errors) {
        #if printf_cmd
        printf("  [Param] Flash erase ERROR: SR2=0x%08lx\r\n", errors);
        #endif
        FLASH->CCR2 = errors;             /* 清除错误标志 */
        FLASH->CR2 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
        FLASH->CR2 |= FLASH_CR_LOCK;
        return 0;
    }

    /* 清除 EOP + 关闭 SER/SNB */
    FLASH->CCR2 = FLASH_SR_EOP;
    FLASH->CR2 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    SCB_CleanInvalidateDCache();
    #if printf_cmd
    printf("  [DBG] Flash: erase OK, programming...\r\n");
    #endif

    /* ── Step 6: 编程 Flash (256bit/次, 544B = 17×32B) ── */
    {
        uint32 *src = (uint32*)slot;
        uint32 words = sizeof(ParamSlot_t) / 4;  /* 544/4 = 136 */

        #if printf_cmd
        printf("  [DBG] Flash: programming %lu words (%lu bytes)...\r\n",
               (unsigned long)words, (unsigned long)sizeof(ParamSlot_t));
        #endif

        for (uint32 i = 0; i < words; i += 8)
        {
            /* 6a. 设 PG 位 (Bank2 此时已可访问, 但为安全仍只用寄存器) */
            FLASH->CR2 |= FLASH_CR_PG;
            __DSB();
            __ISB();

            /* 6b. 写 8 个 32bit 字 (256bit Flash Word) */
            {
                volatile uint32 *dst = (volatile uint32 *)(addr + i * 4);
                dst[0] = src[i + 0];
                dst[1] = src[i + 1];
                dst[2] = src[i + 2];
                dst[3] = src[i + 3];
                dst[4] = src[i + 4];
                dst[5] = src[i + 5];
                dst[6] = src[i + 6];
                dst[7] = src[i + 7];
            }
            __DSB();

            /* 6c. 等待编程完成 (纯寄存器轮询, 零函数调用) */
            {
                uint32 prog_timeout = 50000000U;
                while (FLASH->SR2 & FLASH_SR_BSY) {
                    if (--prog_timeout == 0U) {
                        FLASH->CR2 &= ~FLASH_CR_PG;
                        FLASH->CR2 |= FLASH_CR_LOCK;
                        return 0;
                    }
                }
            }

            /* 6d. 检查错误 */
            errors = FLASH->SR2 & 0x07FE0000U;
            if (errors) {
                FLASH->CCR2 = errors;
                FLASH->CR2 &= ~FLASH_CR_PG;
                FLASH->CR2 |= FLASH_CR_LOCK;
                return 0;
            }

            /* 6e. 清除 EOP + 关 PG 位 (此时 Bank2 可访问) */
            FLASH->CCR2 = FLASH_SR_EOP;
            FLASH->CR2 &= ~FLASH_CR_PG;
        }
    }

    #if printf_cmd
    printf("  [DBG] Flash: programming done, locking...\r\n");
    #endif
    SCB_CleanInvalidateDCache();

    /* ── Step 7: 锁定 Bank 2 ── */
    FLASH->CR2 |= FLASH_CR_LOCK;
    #if printf_cmd
    printf("  [DBG] Flash: locked, all done\r\n");
    #endif
    return 1;
}

/* ── 从参数描述表加载默认值到 param_ram[] ──
 * g_param_desc_table 中定义了每个参数的 .default_val,
 * 此函数遍历描述表, 将全部默认值写入 param_ram */
static void Param_LoadDefaults(void)
{
    for (uint16 i = 0; i < PARAM_COUNT; i++)
        param_ram[i] = PARAM_DEFAULT(i);
}

/* ── 上电加载 ──
 * 读取 Flash 扇区中的参数槽，校验通过则加载，否则使用默认值。
 * static 避免栈溢出 (ParamSlot_t = 544B, STM32H750默认栈仅1024B) */
void Param_LoadAll(void)
{
#if FLASH_STORAGE_DISABLED
    Param_LoadDefaults();
    g_store_version = 0;
    #if printf_cmd
    printf("  [Param] Flash disabled, using RAM defaults\r\n");
    #endif
    return;
#else
    static ParamSlot_t slot;

    if (ParamSlot_Read(FLASH_SLOT_OFFSET, &slot))
    {
        g_store_version = slot.version;
        memcpy(param_ram, slot.values, PARAM_COUNT * sizeof(int32));
        #if printf_cmd
        printf("  [Param] Loaded from Flash, ver=%lu\r\n", g_store_version);
        #endif
    }
    else
    {
        Param_LoadDefaults();
        g_store_version = 0;
        #if printf_cmd
        printf("  [Param] No valid data, using defaults\r\n");
        #endif
    }
#endif /* FLASH_STORAGE_DISABLED */
}

/* ── 保存参数到 Flash ──
 * ⚠️ 调用前必须断开 EtherCAT (进入 INIT 状态)!
 *    调用后必须恢复 EtherCAT (回到 OP 状态)!
 *    擦除约需 1~2 秒，如果在 EtherCAT 运行时调用会导致驱动器看门狗超时。
 *
 * static 避免栈溢出 */
int Param_SaveAll(void)
{
#if FLASH_STORAGE_DISABLED
    #if printf_cmd
    printf("  [Param] Save skipped (Flash storage disabled)\r\n");
    #endif
    (void)0;
    return 0;
#else
    static ParamSlot_t slot;

    #if printf_cmd
    printf("  [DBG] Param_SaveAll: start\r\n");
    #endif

    /* 0. 将当前轴位置同步到 param_ram, 确保随参数一同保存 */
    #if printf_cmd
    printf("  [DBG] Param_SaveAll: calling Position_BackupToRam()...\r\n");
    #endif
    Position_BackupToRam();
    #if printf_cmd
    printf("  [DBG] Param_SaveAll: Position_BackupToRam() done\r\n");
    #endif

    /* 1. 构造参数槽 */
    slot.magic   = PARAM_MAGIC;
    slot.version = g_store_version + 1;
    slot.count   = PARAM_COUNT;
    memcpy(slot.values, param_ram, PARAM_COUNT * sizeof(int32));
    slot.crc = CRC32_Calc((uint8*)slot.values, PARAM_COUNT * sizeof(int32));
    #if printf_cmd
    printf("  [DBG] Param_SaveAll: slot built, magic=0x%08lx ver=%lu, entering Flash erase/write...\r\n",
           (unsigned long)slot.magic, (unsigned long)slot.version);
    #endif

    /* 2. 擦除 + 写入 (~1~2秒) */
    if (!ParamSlot_EraseAndWrite(FLASH_SLOT_OFFSET, &slot)) {
        #if printf_cmd
        printf("  [DBG] Param_SaveAll: ParamSlot_EraseAndWrite FAILED\r\n");
        #endif
        return 0;
    }
    #if printf_cmd
    printf("  [DBG] Param_SaveAll: ParamSlot_EraseAndWrite OK\r\n");
    #endif

    /* 扇区擦除后, 掉电保存槽也被清空, 下次掉电才能写入 */
    g_pf_has_valid = 0;
    g_pf_counter   = 0;

    g_store_version = slot.version;
    #if printf_cmd
    printf("  [Param] Saved to Flash OK, ver=%lu\r\n", g_store_version);
    #endif
    return 1;
#endif /* FLASH_STORAGE_DISABLED */
}

/* ── 恢复出厂设置: 加载默认值并写入 Flash ──
 * 1. 从参数描述表加载默认值到 param_ram
 * 2. 构造参数槽 (版本号递增)
 * 3. 擦除 Flash 扇区 + 写入默认参数
 * 返回 1=成功, 0=Flash 写入失败 */
int Param_ResetFactory(void)
{
    /* Step 1: 用描述表中的 .default_val 填充 param_ram */
    Param_LoadDefaults();

#if FLASH_STORAGE_DISABLED
    g_store_version = 0;
    #if printf_cmd
    printf("  [Param] Factory reset (RAM only, Flash disabled)\r\n");
    #endif
    return 1;
#else
    static ParamSlot_t slot;

    /* Step 2: 构造 Flash 参数槽 */
    slot.magic   = PARAM_MAGIC;
    slot.version = g_store_version + 1;    /* 版本号递增 */
    slot.count   = PARAM_COUNT;
    memcpy(slot.values, param_ram, PARAM_COUNT * sizeof(int32));
    slot.crc = CRC32_Calc((uint8*)slot.values, PARAM_COUNT * sizeof(int32));

    /* Step 3: 擦除 + 写入 (~1~2秒, 需先断开 EtherCAT) */
    if (!ParamSlot_EraseAndWrite(FLASH_SLOT_OFFSET, &slot))
        return 0;

    g_store_version = slot.version;
    return 1;
#endif /* FLASH_STORAGE_DISABLED */
}

/* ── 读取单个参数值 ──
 * @param id  参数ID (0~PARAM_COUNT-1), 见 param_defs.h 枚举
 * @return    参数值 (int32); 若 ID 越界则返回 0
 * 直接从 param_ram[] 数组读取, O(1) */
int32 Param_Get(ParamId_t id)
{
    if ((int)id < 0 || (int)id >= PARAM_COUNT) return 0;  /* 越界保护 */
    return param_ram[(int)id];
}

/* ── 写入单个参数值 (带范围校验) ──
 * @param id    参数ID (0~PARAM_COUNT-1)
 * @param value 要写入的值
 * @return      1=成功, 0=失败 (ID越界 或 值超出 min/max 范围)
 * 范围校验通过 PARAM_VALID 宏查 g_param_desc_table 中的 min_val/max_val */
int Param_Set(ParamId_t id, int32 value)
{
    if ((int)id < 0 || (int)id >= PARAM_COUNT) return 0;       /* 越界 */
    if (!PARAM_VALID((int)id, value)) return 0;                 /* 范围校验失败 */
    param_ram[(int)id] = value;                                 /* 写入 RAM */
    return 1;
}

/* ── 获取 Flash 存储版本号 ──
 * @return 版本号 (递增计数器), 0=从未保存过 (使用默认值)
 * 每次 Param_SaveAll() 成功保存后版本号 +1 */
uint32 Param_GetVersion(void)
{
    return g_store_version;
}

/* ══════════════════════════════════════════════════════════════════
 * 参数默认值表
 * ══════════════════════════════════════════════════════════════════ */

/* ── 参数描述表: 每个条目的字段含义 ──
 * { id, "名称", "单位", 默认值, 最小值, 最大值, Modbus寄存器, 需保存, 需复位 }
 * 例如: [0]={0, "Max RPM", "RPM", 3000, 0, 3000, 64, 1, 0}
 *       → ID=0, 名称="Max RPM", 默认=3000, 范围0~3000, Modbus寄存器=64, 需保存, 不需要复位 */
const ParamDesc_t g_param_desc_table[PARAM_COUNT] = {
    /* ── X轴参数: 0~15 ── */
    [0]  = { 0,  "Max RPM",      "RPM",     3000,  0,    3000,     64, 1, 0 },
    [1]  = { 1,  "Max Accel",    "cts/T^2", 1000,   0,    20000,    65, 1, 0 },
    [2]  = { 2,  "Max Jerk",     "cts/T^3", 100,    0,    2000,     66, 1, 0 },
    [3]  = { 3,  "Home Offset",  "pls",     0,    -2000000000, 2000000000, 67, 1, 0 },
    [4]  = { 4,  "SoftLimit +",  "pls",     2000000000, 0, 2000000000, 68, 1, 0 },
    [5]  = { 5,  "SoftLimit -",  "pls",    -2000000000, -2000000000, 0, 69, 1, 0 },
    [6]  = { 6,  "Backlash",     "pls",     0,     0,    10000,    70, 1, 0 },
    [7]  = { 7,  "Screw Pitch",  "um/rev",  5000,  100,  50000,    71, 1, 0 },
    [8]  = { 8,  "Gear Num",     "",        1,      1,    1000000, 72, 1, 0 },
    [9]  = { 9,  "Gear Den",     "",        1,      1,    1000000, 73, 1, 0 },
    [10] = { 10, "Invert Dir",   "",        0,     0,    1,        74, 1, 1 },
    [11] = { 11, "Jog Speed",    "RPM",     200,   1,    500,      75, 1, 0 },
    [12] = { 12, "Homing Speed", "RPM",     50,    1,    200,      76, 1, 0 },
    [13] = { 13, "Homing Accel", "cts/T^2", 33,    1,    100,      77, 1, 0 },
    [14] = { 14, "Arrive Win",   "pls",     833,   10,   100000,   78, 1, 0 },
    [15] = { 15, "Follow Err",   "pls",     100000,1,    2000000000,79, 1, 0 },

    /* ── 运动控制全局参数: 128~136 (原64~72后移) ── */
    [128] = { 128, "Default Vel",  "RPM",     100,   1,    3000,     80, 1, 0 },
    [129] = { 129, "Global Jog",   "RPM",     200,   1,    500,      81, 1, 0 },
    [130] = { 130, "Global Home",  "RPM",     50,    1,    200,      82, 1, 0 },
    [131] = { 131, "Home Method",  "",        35,    1,    35,       83, 1, 0 },
    [132] = { 132, "Home Accel",   "cts/T^2", 33,    1,    100,      84, 1, 0 },
    [133] = { 133, "Smooth Stop",  "",        1,     0,    1,        85, 1, 0 },
    [134] = { 134, "Feed Override","%",       100,   10,   200,      86, 1, 0 },
    [135] = { 135, "Rapid Override","%",      100,   10,   100,      87, 1, 0 },
    [136] = { 136, "Spindle Override","%",    100,   50,   150,      88, 1, 0 },

    /* ── S轴参数 — 刀库旋转: 64~72 (9参数) ── */
    [64] = { 64, "S Max RPM",      "RPM",     300,   0,    3000,     284, 1, 0 },
    [65] = { 65, "S Max Accel",    "cts/T^2", 1000,  0,    20000,    285, 1, 0 },
    [70] = { 70, "S Jog Speed",    "RPM",     100,   1,    500,      290, 1, 0 },
    [71] = { 71, "S Arrive Win",   "pls",     833,   10,   100000,   291, 1, 0 },
    [72] = { 72, "S Homing Speed", "RPM",     50,    1,    200,      292, 1, 0 },

    /* ── W轴参数 — 升降抓电极: 80~91 (12参数) ── */
    [80] = { 80, "W Max RPM",      "RPM",     500,   0,    3000,     300, 1, 0 },
    [81] = { 81, "W Max Accel",    "cts/T^2", 1000,  0,    20000,    301, 1, 0 },
    [85] = { 85, "W Pitch",        "um/rev",  5000,  100,  50000,    305, 1, 0 },
    [89] = { 89, "W Jog Speed",    "RPM",     200,   1,    500,      309, 1, 0 },
    [90] = { 90, "W Homing Speed", "RPM",     50,    1,    200,      310, 1, 0 },
    [91] = { 91, "W Arrive Win",   "pls",     833,   10,   100000,   311, 1, 0 },

    /* ── U轴参数 — 摇动光洁度: 96~101 (6参数) ── */
    [96] = { 96, "U Osc Speed",    "RPM",     500,   0,    3000,     316, 1, 0 },
    [97] = { 97, "U Amplitude",    "pls",     1000,  0,    100000,   317, 1, 0 },
    [98] = { 98, "U Pitch",        "um/rev",  5000,  100,  50000,    318, 1, 0 },

};

const uint16 g_param_desc_count = sizeof(g_param_desc_table)
                                / sizeof(g_param_desc_table[0]);

/* ── 确保参数有默认值 (无论Flash是否有数据, 零值参数均补默认) ── */
void Param_EnsureDefaults(void)
{
    /* 填充默认值: 仅当参数当前值为0且有非零默认值时覆盖 */
    for (uint16 i = 0; i < g_param_desc_count; i++) {
        ParamDesc_t d = g_param_desc_table[i];
        if ((int)d.id >= 0 && (int)d.id < PARAM_COUNT && d.name[0] != '\0') {
            if (param_ram[(int)d.id] == 0 && d.default_val != 0) {
                param_ram[(int)d.id] = d.default_val;
            }
        }
    }
    /* 复制X轴默认值到联动轴 Y(16~31), Z(32~47), R(48~63) — 仅当目标为0时 */
    for (int ax = 1; ax < 4; ax++) {
        for (int p = 0; p < 16; p++) {
            if (param_ram[ax * 16 + p] == 0)
                param_ram[ax * 16 + p] = param_ram[p];
        }
    }
    /* 复制X轴基础参数默认值到辅助轴 S(64~79), W(80~95), U(96~111), V(112~127)
     * 仅填充通用参数 (齿轮比/方向/导程), 专用参数由描述表默认值覆盖 */
    for (int ax = 4; ax < MAX_AXES; ax++) {
        if (param_ram[ax * 16 + 8]  == 0) param_ram[ax * 16 + 8]  = param_ram[8];  /* Gear Num */
        if (param_ram[ax * 16 + 9]  == 0) param_ram[ax * 16 + 9]  = param_ram[9];  /* Gear Den */
        if (param_ram[ax * 16 + 10] == 0) param_ram[ax * 16 + 10] = param_ram[10]; /* Invert Dir */
        if (param_ram[ax * 16 + 7]  == 0) param_ram[ax * 16 + 7]  = param_ram[7];  /* Pitch */
    }

    /* ── R轴(旋转轴)特殊默认值: 范围 0 ~ 359.999° ── */
    /* SoftLimit+: pulses_per_rev - 1 (≈359.99996°) */
    if (param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, 3)] == 2000000000)
        param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, 3)] = GetAxisPPR(3) - 1;
    /* SoftLimit-: 0° (旋转轴起点, 不允许负角度) */
    if (param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, 3)] == -2000000000)
        param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, 3)] = 0;

    /* ── S轴(刀库旋转)特殊默认值: 范围 0 ~ 359.999° ── */
    if (param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, 7)] == 2000000000)
        param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_P, 7)] = GetAxisPPR(7) - 1;
    if (param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, 7)] == -2000000000)
        param_ram[AXIS_PARAM(PARAM_AXIS_SOFT_LIMIT_N, 7)] = 0;
}

/* ══════════════════════════════════════════════════════════════════
 * 轴位置持久化 — 掉电保存槽 (PVD 触发, 仅掉电时写入)
 * ══════════════════════════════════════════════════════════════════
 *
 * 【设计思路】
 *   STM32H7 PVD 检测 VDD 电压跌落 → 中断里紧急写入当前位置到 Flash。
 *   正常运行时 Flash 零写入, 彻底解决磨损问题。
 *   PARAM:SAVE 写入的 param_ram 位置数据作二次兜底。
 *
 * 【掉电保存槽布局】
 *   Flash 偏移 0x2000 (8KB), 固定一个 256bit Flash Word:
 *     [0]  magic      (0x50465752 "PFWR")
 *     [1]  pos_x      (脉冲)
 *     [2]  pos_y
 *     [3]  pos_z
 *     [4]  pos_r
 *     [5]  counter    (递增序号)
 *     [6]  tick_ms    (HAL_GetTick)
 *     [7]  xor_chk    (前7字的XOR ^ 0xA5A5A5A5)
 *
 *   ⚠️ 写入耗时 ~100μs, 需板载电容维持 VDD > 1.62V 至少 200μs。
 *   ⚠️ PARAM:SAVE 擦除扇区后此槽也被清空, 位置由 param_ram 兜底。
 *
 * 【恢复优先级】
 *   上电编码器返回0 →
 *     1. 掉电保存槽 (最新, 掉电瞬间写入)
 *     2. param_ram 保存值 (上次 PARAM:SAVE)
 */

/* ── 掉电保存槽 ── */
#define PF_SAVE_OFFSET         0x2000UL         /**< 在 Sector7 内的偏移 (8KB) */
#define PF_SAVE_ADDR           (FLASH_PARAM_BASE + PF_SAVE_OFFSET)
#define PF_SAVE_MAGIC          0x50465752UL     /**< "PFWR" */
#define POS_SAVED_VALID_MAGIC  0x584E3010UL     /**< param_ram[112] 有效标志 */

/* ── 外部引用 ── */
extern MotorAxis_t axis[MAX_AXES];
extern int32       param_ram[PARAM_COUNT];
extern int         ec_slavecount;    /* SOEM: int, 非 uint8 */

/* ── 检查 32 字节 Flash 区域是否全为 0xFF (已擦除, 未被编程) ──
 * Flash 编程只能将 bit 从 1→0, 不能 0→1 (需要擦除才能恢复为1)
 * 所以全 0xFF = 已擦除 = 可写入; 有非 0xFF = 已写过 = 不可再写
 * 返回 1=已擦除(可写), 0=已编程(不可写) */
static uint8 Flash_IsErased32(uint32 addr) {
    volatile uint32 *p = (volatile uint32 *)addr;
    for (int i = 0; i < 8; i++) {
        if (p[i] != 0xFFFFFFFFUL) return 0;
    }
    return 1;
}

/* ══════════════════════════════════════════════════════════════════
 * Position_PowerFailInit() — 上电: 读掉电保存槽 + param_ram
 * ══════════════════════════════════════════════════════════════════
 * 检查掉电保存槽有效性 (magic + XOR校验)
 * 同时读 param_ram 保存的位置作为兜底
 */
void Position_PowerFailInit(void)
{
#if FLASH_STORAGE_DISABLED
    g_pf_has_valid = 0;
    memset(g_pf_saved_pos, 0, sizeof(g_pf_saved_pos));
    return;
#else
    g_pf_has_valid = 0;
    memset(g_pf_saved_pos, 0, sizeof(g_pf_saved_pos));

    /* ── 读掉电保存槽 ── */
    volatile uint32 *pf = (volatile uint32 *)PF_SAVE_ADDR;

    if (!Flash_IsErased32(PF_SAVE_ADDR) && pf[0] == PF_SAVE_MAGIC) {
        /* XOR 校验: 前7字 XOR 再异或 0xA5A5A5A5 应等于第8字 */
        uint32 xsum = 0;
        for (int i = 0; i < 7; i++) xsum ^= pf[i];
        xsum ^= 0xA5A5A5A5UL;

        if (xsum == pf[7]) {
            g_pf_saved_pos[0] = (int32)pf[1];
            g_pf_saved_pos[1] = (int32)pf[2];
            g_pf_saved_pos[2] = (int32)pf[3];
            g_pf_saved_pos[3] = (int32)pf[4];
            /* U/V/W/S: 掉电保存槽仅4轴, 辅助轴位置由 param_ram 兜底 */
            g_pf_counter       = pf[5];
            g_pf_has_valid     = 1;

            #if printf_cmd
            printf("  [PF] Power-fail slot valid (#%lu): X=%ld Y=%ld Z=%ld R=%ld\r\n",
                   (unsigned long)g_pf_counter,
                   (long)g_pf_saved_pos[0], (long)g_pf_saved_pos[1],
                   (long)g_pf_saved_pos[2], (long)g_pf_saved_pos[3]);
            #endif
        } else {
            #if printf_cmd
            printf("  [PF] Power-fail slot: XOR mismatch, ignoring\r\n");
            #endif
        }
    } else {
        #if printf_cmd
        printf("  [PF] Power-fail slot: empty\r\n");
        #endif
    }

    /* ── 兜底: param_ram (上次 PARAM:SAVE 写入) ──
     * 掉电槽为空或损坏时, 用 param_ram 的数据填充 g_pf_saved_pos
     * (仅掉电槽无效的轴才用 param_ram, Position_RestoreIfNeeded 会择优使用) */
    if ((uint32)param_ram[PARAM_SAVED_POS_VALID] == POS_SAVED_VALID_MAGIC) {
        #if printf_cmd
        printf("  [PF] param_ram backup valid: X=%ld Y=%ld Z=%ld R=%ld\r\n",
               (long)param_ram[PARAM_SAVED_POS_X],
               (long)param_ram[PARAM_SAVED_POS_Y],
               (long)param_ram[PARAM_SAVED_POS_Z],
               (long)param_ram[PARAM_SAVED_POS_R]);
        #endif
    }
#endif /* FLASH_STORAGE_DISABLED */
}

/* ══════════════════════════════════════════════════════════════════
 * Position_PowerFailSave() — ISR 安全: 掉电瞬间紧急保存
 * ══════════════════════════════════════════════════════════════════
 * 仅从 PVD_AVD_IRQHandler 调用。
 * 已提前关全局中断, 使用 XOR 校验 (无表查找, 速度优先)。
 * 写入单个 Flash Word (~100μs), 完成后死等 BOR 复位。
 *
 * ⚠️ 此函数不应返回 (电压继续下降, 正常运行已不可能)
 */
void Position_PowerFailSave(void)
{
#if FLASH_STORAGE_DISABLED
    return;
#else
    uint32 addr = PF_SAVE_ADDR;

    /* 检查槽位是否可写 (参数保存擦除后重置为全0xFF) */
    if (!Flash_IsErased32(addr)) {
        /* 已被写入过 — 不应发生, PARAM:SAVE 会擦除整个扇区。
         * 如果走到这里说明上次掉电保存后没有做过 PARAM:SAVE。
         * 无法再次写入 (Flash 只能从 1→0 编程, 不能覆盖), 直接退出。 */
        return;
    }

    /* 构造条目 (栈上, 最小编码) */
    uint32 entry[8];
    entry[0] = PF_SAVE_MAGIC;
    entry[1] = (uint32)axis[0].actual_pos;
    entry[2] = (uint32)axis[1].actual_pos;
    entry[3] = (uint32)axis[2].actual_pos;
    entry[4] = (uint32)axis[3].actual_pos;
    entry[5] = g_pf_counter + 1;
    entry[6] = HAL_GetTick();

    /* XOR 校验 (比 CRC32 快 ~10x, ISR 中关键) */
    uint32 xsum = 0;
    for (int i = 0; i < 7; i++) xsum ^= entry[i];
    entry[7] = xsum ^ 0xA5A5A5A5UL;

    /* 写 Flash (1 个 Flash Word = 256bit = 32B) */
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32)(uintptr_t)entry);
    HAL_FLASH_Lock();

    /* 刷新 D-Cache, 确保写入完成 */
    SCB_CleanInvalidateDCache();
    __DSB();
    __ISB();
#endif /* FLASH_STORAGE_DISABLED */
}

/* ══════════════════════════════════════════════════════════════════
 * Position_BackupToRam() — 复制当前轴位置到 param_ram
 * ══════════════════════════════════════════════════════════════════
 * 定期调用 (RAM 操作, 零 Flash 磨损)。
 * PARAM:SAVE 时随参数一同写入 Flash, 作为掉电槽的兜底。
 */
void Position_BackupToRam(void)
{
#if FLASH_STORAGE_DISABLED
    return;
#else
    param_ram[PARAM_SAVED_POS_VALID] = (int32)POS_SAVED_VALID_MAGIC;
    param_ram[PARAM_SAVED_POS_X]     = (0 < ec_slavecount) ? axis[0].actual_pos : 0;
    param_ram[PARAM_SAVED_POS_Y]     = (1 < ec_slavecount) ? axis[1].actual_pos : 0;
    param_ram[PARAM_SAVED_POS_Z]     = (2 < ec_slavecount) ? axis[2].actual_pos : 0;
    param_ram[PARAM_SAVED_POS_R]     = (3 < ec_slavecount) ? axis[3].actual_pos : 0;
    param_ram[PARAM_SAVED_POS_U]     = (4 < ec_slavecount) ? axis[4].actual_pos : 0;
    param_ram[PARAM_SAVED_POS_V]     = (5 < ec_slavecount) ? axis[5].actual_pos : 0;
    param_ram[PARAM_SAVED_POS_W]     = (6 < ec_slavecount) ? axis[6].actual_pos : 0;
    param_ram[PARAM_SAVED_POS_S]     = (7 < ec_slavecount) ? axis[7].actual_pos : 0;
#endif /* FLASH_STORAGE_DISABLED */
}

/* ══════════════════════════════════════════════════════════════════
 * Position_RestoreIfNeeded() — 上电时检查编码器, 若为0则恢复
 * ══════════════════════════════════════════════════════════════════
 * 恢复优先级: 掉电保存槽 (最新) > param_ram (PARAM:SAVE 兜底)
 */
void Position_RestoreIfNeeded(void)
{
#if FLASH_STORAGE_DISABLED
    return;
#else
    int32 saved[MAX_AXES];
    uint8 has_saved[MAX_AXES] = {0};

    /* 第一优先: 掉电保存槽 (仅前4轴) */
    if (g_pf_has_valid) {
        for (int ax = 0; ax < 4; ax++) {
            saved[ax]    = g_pf_saved_pos[ax];
            has_saved[ax] = 1;
        }
    }

    /* 第二优先 (兜底): param_ram (支持8轴) */
    if ((uint32)param_ram[PARAM_SAVED_POS_VALID] == POS_SAVED_VALID_MAGIC) {
        for (int ax = 0; ax < MAX_AXES; ax++) {
            if (!has_saved[ax]) {
                saved[ax]    = param_ram[PARAM_SAVED_POS_X + ax];
                has_saved[ax] = 1;
            }
        }
    }

    /* ── 检查每个轴, 按需恢复 ── */
    static const char axis_names[8] = "XYZRUVWS";
    for (int ax = 0; ax < MAX_AXES; ax++) {
        if (ax + 1 > ec_slavecount) continue;

        int32 enc_pos = axis[ax].actual_pos;

        /* 编码器位置非0 → 正常, 不恢复 */
        if (enc_pos != 0) continue;

        /* 编码器为0 且有非零保存值 → 恢复 */
        if (has_saved[ax] && saved[ax] != 0) {
            axis[ax].actual_pos     = saved[ax];
            axis[ax].target_pos     = saved[ax];
            axis[ax].cmd_target_pos = saved[ax];
            axis[ax].pdo_out->TargetPos = Servo_EncToUser(ax, saved[ax]);

            #if printf_cmd
            printf("  %c: encoder=0, RESTORED from saved: pos=%ld\r\n",
                   axis_names[ax], (long)saved[ax]);
            #endif
        } else if (enc_pos == 0 && (!has_saved[ax] || saved[ax] == 0)) {
            #if printf_cmd
            printf("  %c: encoder=0, no saved position → keeping 0\r\n",
                   axis_names[ax]);
            #endif
        }
    }
#endif /* FLASH_STORAGE_DISABLED */
}
