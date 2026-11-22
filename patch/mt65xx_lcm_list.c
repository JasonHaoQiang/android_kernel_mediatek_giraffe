#include <lcm_drv.h>
#ifdef BUILD_LK
#include <platform/disp_drv_platform.h>
#else
#include <linux/delay.h>
#include <mach/mt_gpio.h>
#endif

/*
 * b906: 只挂一块屏（EK79007 720x1280）。
 * disp_hal.c 里 lcm_count == 1 时会自动选用唯一的那一个驱动，
 * 因此这里必须只保留 ek79007，避免其它面板参与自动识别。
 */
extern LCM_DRIVER ek79007_wxga_dsi_vdo_b906_lcm_drv;

LCM_DRIVER* lcm_driver_list[] = {
	&ek79007_wxga_dsi_vdo_b906_lcm_drv,
};

#define LCM_COMPILE_ASSERT(condition) LCM_COMPILE_ASSERT_X(condition, __LINE__)
#define LCM_COMPILE_ASSERT_X(condition, line) LCM_COMPILE_ASSERT_XX(condition, line)
#define LCM_COMPILE_ASSERT_XX(condition, line) char assertion_failed_at_line_##line[(condition)?1:-1]

unsigned int lcm_count = sizeof(lcm_driver_list)/sizeof(LCM_DRIVER*);
LCM_COMPILE_ASSERT(0 != sizeof(lcm_driver_list)/sizeof(LCM_DRIVER*));
