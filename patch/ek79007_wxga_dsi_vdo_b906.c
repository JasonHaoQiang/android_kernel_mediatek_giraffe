/*
 * EK79007 WXGA (1280x800) DSI VDO -- b906 (MT6582) 专用 LCM 驱动
 *
 * 适配目标：DeVviL/android_kernel_mediatek_giraffe  (MT6582, kernel 3.10.108)
 * 机型：杂牌 MP4 / 学习平板 B906 (MT6582 + MT6323 + MT6627, eMMC, 屏 1280x800)
 *
 * ---------------------------------------------------------------------------
 * 一、初始化序列来源
 * ---------------------------------------------------------------------------
 * 下面的寄存器表不是抄来的，是从 b906 原厂 3.10.54 内核 Image 里反解出来的
 * （原厂 Image 见 F:\mbr\base_kernel.bin）：
 *
 *   原厂 struct LCM_setting_table 的 cmd 是 unsigned int，步长 72 字节：
 *       {0xF0, 5, {0x55,0xAA,0x52,0x08,0x00}}   <- 页选择 page0
 *       ...
 *   原厂用 cmd = 0xFFE 表示「延时 count 毫秒」、cmd = 0xFFF 表示「表结束」。
 *
 * ---------------------------------------------------------------------------
 * 二、为什么要做这层翻译（v23 卡死第一屏的根因）
 * ---------------------------------------------------------------------------
 * giraffe 3.10.108 树的 lcm_drv.h 里：
 *
 *     struct LCM_setting_table {
 *         unsigned char cmd;        <-- 只有 1 字节！
 *         unsigned char count;
 *         unsigned char para_list[64];
 *     };
 *
 * 若把原厂表原样搬过来：
 *     0xFFE -> 被截断成 0xFE
 *     count = 200 -> 越过只有 64 字节的 para_list，向 DSI 发出 200 字节越界数据
 * 结果：显示初始化阶段把乱码发给面板，DSI 传输阻塞，
 *       表现为「屏幕停在开机 logo、没有 USB、不重启」——正是之前 v23 的现象。
 *
 * 本文件的处理：
 *     0xFFE (count=N)  ->  B906_REGFLAG_DELAY (0xAB)，即延时 N 毫秒
 *     0xFFF           ->  B906_REGFLAG_END_TABLE (0xAA)，即表结束
 *     其余原样保留；注意 0xFF / 0xFE 在原厂表里是真实的寄存器写
 *     （{0xFF,4,{0xA5,0x5A,0x13,0x86}}、{0xFE,2,{0x01,0x54}}），
 *     所以这里自写 push_table，只把 0xAB/0xAA 当标记处理。
 *
 * ---------------------------------------------------------------------------
 * 三、DSI 时序（1280x800 @60, 4 lane, RGB888）
 * ---------------------------------------------------------------------------
 * 按 MTK 官方公式：
 *     BRPL(每 lane 比特率) = (v_total * h_total * 24bit * 60fps) / lane
 *                           = (3+10+5+800) * (20+20+20+1280) * 24 * 60 / 4
 *                           = 818 * 1340 * 24 * 60 / 4 = 394.6 Mbps
 *     DRPL = 2 * PLL_CLOCK  ->  PLL_CLOCK = 200 (DRPL = 400 Mbps) 余量约 1.4%
 *
 * 点不亮 / 花屏 / 画面偏移时，按顺序只动下面这几个：
 *     1) vertical_*  / horizontal_*   (垂直/水平前后沿与同步宽)
 *     2) PLL_CLOCK                     (200 -> 197 更贴 394.6M；上调可试 210/250)
 *     3) dsi.mode                      (SYNC_EVENT_VDO_MODE <-> SYNC_PULSE_VDO_MODE)
 *     4) LANE_NUM                      (LCM_FOUR_LANE <-> LCM_TWO_LANE)
 */

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif

/* ------------------------------------------------------------------------- */
/*  屏参数                                                                    */
/* ------------------------------------------------------------------------- */

#define FRAME_WIDTH   (1280)
#define FRAME_HEIGHT  (800)

/* 本驱动私有的表项标记：避开 lcm_drv.h 的同名宏，
 * 保证 0xFF / 0xFE 这两条真实寄存器写不会被当成标记。 */
#define B906_REGFLAG_DELAY      0xAB    /* = lcm_drv.h 的 REGFLAG_DELAY，延时 count ms */
#define B906_REGFLAG_END_TABLE  0xAA    /* = lcm_drv.h 的 REGFLAG_END_OF_TABLE */

/* DSI 时序 */
#define T_VSYNC   3
#define T_VBP     10
#define T_VFP     5
#define T_HSYNC   20
#define T_HBP     20
#define T_HFP     20
#define T_PLL     200

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)     lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)

/* ------------------------------------------------------------------------- */
/*  初始化序列（b906 原厂面板参数，逐字节反解自 3.10.54 原厂内核）              */
/* ------------------------------------------------------------------------- */

static struct LCM_setting_table lcm_initialization_setting[] = {
    {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00}},
    {0xB0, 4, {0x0F, 0x0F, 0x1E, 0x14}},
    {0xB2, 1, {0x00}},
    {0xB6, 1, {0x02}},
    {0xC0, 20, {0x03, 0x00, 0x06, 0x07, 0x08, 0x09, 0x00, 0x00, 0x00, 0x00,
                0x02, 0x00, 0x0A, 0x0B, 0x0C, 0x0D, 0x00, 0x00, 0x00, 0x00}},
    {0xC1, 16, {0x08, 0x00, 0x00, 0x01, 0x18, 0x00, 0x68, 0x85,
                0x08, 0x00, 0x00, 0x01, 0x18, 0x00, 0x68, 0x85}},
    {0xC2, 24, {0x03, 0x15, 0x1B, 0x00, 0x22, 0x30, 0x01, 0x15,
                0x1B, 0x00, 0x22, 0x30, 0x03, 0x15, 0x1B, 0x10,
                0x01, 0x10, 0x02, 0x15, 0x1B, 0x10, 0x01, 0x10}},
    {0xC3, 24, {0x02, 0x15, 0x1B, 0x00, 0x22, 0x10, 0x03, 0x15,
                0x1B, 0x00, 0x22, 0x10, 0x03, 0x15, 0x1B, 0x10,
                0x01, 0x10, 0x02, 0x15, 0x1B, 0x10, 0x01, 0x10}},
    {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x01}},
    {0xB5, 1, {0x1E}},
    {0xB6, 1, {0x2D}},
    {0xB7, 1, {0x04}},
    {0xB8, 1, {0x05}},
    {0xB9, 1, {0x04}},
    {0xBA, 1, {0x14}},
    {0xBB, 1, {0x2F}},
    {0xBE, 1, {0x12}},
    {0xC2, 3, {0x02, 0xF5, 0x07}},
    {0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x02}},
    {0xC1, 2, {0x02, 0xDD}},
    {0xC9, 1, {0x1E}},
    {0xD4, 3, {0x01, 0x04, 0x2C}},
    {0x8C, 3, {0x55, 0x49, 0x53}},
    {0xFF, 4, {0xA5, 0x5A, 0x13, 0x86}},
    {0xFE, 2, {0x01, 0x54}},
    {0x51, 1, {0x20}},
    {0x53, 1, {0x20}},
    {0x11, 1, {0x00}},
    {B906_REGFLAG_DELAY, 200, {0}},
    {0x29, 1, {0x00}},
    {B906_REGFLAG_DELAY, 50, {0}},
    {B906_REGFLAG_END_TABLE, 0, {0}}
};

/* suspend：原厂表第 32~36 项 */
static struct LCM_setting_table lcm_suspend_setting[] = {
    {0x28, 1, {0x00}},
    {B906_REGFLAG_DELAY, 20, {0}},
    {0x10, 1, {0x00}},
    {B906_REGFLAG_DELAY, 120, {0}},
    {B906_REGFLAG_END_TABLE, 0, {0}}
};

/* ------------------------------------------------------------------------- */
/*  本驱动自己的 push_table：只把 0xAB/0xAA 当标记，0xFF/0xFE 照常当寄存器写     */
/* ------------------------------------------------------------------------- */

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {
        unsigned int cmd = table[i].cmd;

        switch (cmd) {
        case B906_REGFLAG_DELAY:
            MDELAY(table[i].count);
            break;

        case B906_REGFLAG_END_TABLE:
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
            break;
        }
    }
}

/* ------------------------------------------------------------------------- */
/*  LCM_DRIVER 接口实现                                                        */
/* ------------------------------------------------------------------------- */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->physical_width  = 163;     /* mm，仅供上报 */
    params->physical_height = 102;     /* mm */

    params->dsi.mode   = SYNC_EVENT_VDO_MODE;

    params->dsi.LANE_NUM             = LCM_FOUR_LANE;
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    params->dsi.packet_size            = 256;
    params->dsi.intermediat_buffer_num = 2;
    params->dsi.PS                     = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active   = T_VSYNC;
    params->dsi.vertical_backporch     = T_VBP;
    params->dsi.vertical_frontporch    = T_VFP;
    params->dsi.vertical_active_line   = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = T_HSYNC;
    params->dsi.horizontal_backporch   = T_HBP;
    params->dsi.horizontal_frontporch  = T_HFP;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    /* DRPL = 2 * PLL_CLOCK；需求 BRPL ≈ 394.6 Mbps/lane */
    params->dsi.PLL_CLOCK = T_PLL;

    params->dsi.ssc_disable = 0;   /* 0: 使能展频 */
    params->dsi.ssc_range   = 5;
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);

    push_table(lcm_initialization_setting,
               sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(lcm_suspend_setting,
               sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_esd_check(void)
{
    /* 暂不做 DSI 读回检测：读回在 DSI 时序不匹配时会把显示线程阻塞住，
     * 先保证开机路径不被 ESD 检测拖死。 */
    return 1;
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();
    return 1;
}

static unsigned int lcm_compare_id(void)
{
    return 1;
}

LCM_DRIVER ek79007_wxga_dsi_vdo_b906_lcm_drv = {
    .name         = "ek79007_wxga_dsi_vdo_b906",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params   = lcm_get_params,
    .init         = lcm_init,
    .suspend      = lcm_suspend,
    .resume       = lcm_resume,
    .esd_check    = lcm_esd_check,
    .esd_recover  = lcm_esd_recover,
    .compare_id   = lcm_compare_id,
};
