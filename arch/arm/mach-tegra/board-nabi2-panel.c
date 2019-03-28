/*
 * arch/arm/mach-tegra/board-kai-panel.c
 *
 * Copyright (c) 2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <linux/wlan_plat.h>
#include <linux/spi/spi.h>
#include <linux/ssd2825.h>
#include "board.h"
#include "board-nabi2.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra3_host1x_devices.h"
#include "clock.h"
#include <linux/clk.h>
#include "fuse.h"
/* kai default display board pins */
#define kai_lvds_avdd_en		TEGRA_GPIO_PH2
#define kai_lvds_shutdown	TEGRA_GPIO_PN6
#define kai_lvds_vdd			TEGRA_GPIO_PW1

/* common pins( backlight ) for all display boards */
#define kai_bl_enb			TEGRA_GPIO_PH3
#define kai_bl_pwm			TEGRA_GPIO_PH0
#define kai_hdmi_hpd			TEGRA_GPIO_PN7
#define DSI_PANEL_RESET 1
#define DC_CTRL_MODE  TEGRA_DC_OUT_CONTINUOUS_MODE

#define  LCD_VDDS_EN_GPIO   TEGRA_GPIO_PW1
#define  LCD_EN_3V3_GPIO     TEGRA_GPIO_PH2
#define  LCD_1V2_EN               TEGRA_GPIO_PP2

static struct ssd2828_data * mSsd2828_Data;

#ifdef CONFIG_TEGRA_DC
static struct regulator *kai_hdmi_reg;
static struct regulator *kai_hdmi_pll;
static struct regulator *kai_hdmi_vddio;
#endif
static atomic_t keenhi_release_bootloader_fb_flag = ATOMIC_INIT(0);
static atomic_t sd_brightness = ATOMIC_INIT(255);

static tegra_dc_bl_output kai_bl_output_measured = {
	0, 1, 2, 3, 4, 5, 6, 7,
	7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 20, 21,
	22, 23, 24, 25, 26, 27, 28, 29,
	30, 31, 32, 32, 34, 34, 36, 36,
	38, 39, 40, 40, 41, 42, 42, 43,
	44, 44, 45, 46, 46, 47, 48, 48,
	49, 50, 50, 51, 52, 53, 54, 54,
	55, 56, 57, 58, 58, 59, 60, 61,
	62, 63, 64, 65, 66, 67, 68, 69,
	70, 71, 72, 72, 73, 74, 75, 76,
	76, 77, 78, 79, 80, 81, 82, 83,
	85, 86, 87, 89, 90, 91, 92, 92,
	93, 94, 95, 96, 96, 97, 98, 99,
	100, 100, 101, 102, 103, 104, 104, 105,
	106, 107, 108, 108, 109, 110, 112, 114,
	116, 118, 120, 121, 122, 123, 124, 125,
	126, 127, 128, 129, 130, 131, 132, 133,
	134, 135, 136, 137, 138, 139, 140, 141,
	142, 143, 144, 145, 146, 147, 148, 149,
	150, 151, 151, 152, 153, 153, 154, 155,
	155, 156, 157, 157, 158, 159, 159, 160,
	162, 164, 166, 168, 170, 172, 174, 176,
	178, 180, 181, 181, 182, 183, 183, 184,
	185, 185, 186, 187, 187, 188, 189, 189,
	190, 191, 192, 193, 194, 195, 196, 197,
	198, 199, 200, 201, 201, 202, 203, 203,
	204, 205, 205, 206, 207, 207, 208, 209,
	209, 210, 211, 211, 212, 212, 213, 213,
	214, 215, 215, 216, 216, 217, 217, 218,
	219, 219, 220, 222, 226, 230, 232, 234,
	236, 238, 240, 244, 248, 251, 253, 255
};

static struct clk * usbd_emc=NULL;
static p_tegra_dc_bl_output bl_output;

static int kai_backlight_init(struct device *dev)
{
	int ret=0;

	bl_output = kai_bl_output_measured;

	if (WARN_ON(ARRAY_SIZE(kai_bl_output_measured) != 256))
		pr_err("bl_output array does not have 256 elements\n");

	ret = gpio_request(kai_bl_enb, "backlight_enb");
	if (ret < 0)
		return ret;

	ret = gpio_direction_output(kai_bl_enb, 1);
	if (ret < 0)
		gpio_free(kai_bl_enb);

	return ret;
};

static void kai_backlight_exit(struct device *dev)
{
	return;
}

#define MIN_BRIGHTNESS 147
#define MAX_BRIGHTNESS 23
static int kai_backlight_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

    brightness = (brightness * cur_sd_brightness) / 255;
    brightness = brightness * (MIN_BRIGHTNESS-MAX_BRIGHTNESS) / (255-25);

    return brightness;
}

static int kai_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data kai_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 1000000,
	.init		= kai_backlight_init,
	.exit		= kai_backlight_exit,
	.notify		= kai_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= kai_disp1_check_fb,
};

static struct platform_device kai_backlight_device = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &kai_backlight_data,
	},
};

static int kai_panel_postpoweron(void)
{
	printk("kai_panel_postpoweron\n");

	gpio_set_value(kai_lvds_avdd_en, 1);
	mdelay(5);

	gpio_set_value(kai_lvds_shutdown, 1);
	gpio_set_value(kai_lvds_vdd, 1);

	mdelay(250);

	gpio_set_value(kai_bl_enb, 1);

	return 0;
}

static int kai_panel_enable(void)
{

	if(!atomic_read(&keenhi_release_bootloader_fb_flag)) {
		tegra_release_bootloader_fb();
		atomic_set(&keenhi_release_bootloader_fb_flag, 1);
	}

	if(usbd_emc == NULL){
		usbd_emc =clk_get_sys("tegra-udc.0", "emc");
	}

	if(usbd_emc){
		printk("hold the usbd.emc for usb\n");
		clk_enable(usbd_emc);
	}

	pr_err("%s:====================>EXE\n",__func__);
	if(mSsd2828_Data&&mSsd2828_Data->ssd_init_callback){
		mSsd2828_Data->ssd_init_callback(mSsd2828_Data->spi);
	}
	return 0;
}

static int kai_panel_disable(void)
{
	if(usbd_emc==NULL){
		usbd_emc =clk_get_sys("tegra-udc.0", "emc");
	}
	if(usbd_emc){
		printk("release the usbd.emc for usb\n");
		clk_disable(usbd_emc);
	}

	pr_err("%s:====================>EXE\n",__func__);

	if(mSsd2828_Data&&mSsd2828_Data->ssd_deinit_callback){
		mSsd2828_Data->ssd_deinit_callback(mSsd2828_Data->spi);
	}
	return 0;
}

static int kai_panel_prepoweroff(void)
{

	//printk("kai_panel_prepoweroff\n");

	gpio_request(kai_bl_pwm, "disable_pwm");
	gpio_direction_output(kai_bl_pwm, 1);
	gpio_free(kai_bl_pwm);

	gpio_set_value(kai_lvds_vdd, 0);
	gpio_set_value(kai_lvds_shutdown, 0);
	mdelay(5);

	gpio_set_value(kai_lvds_avdd_en, 0);
	mdelay(5);

	gpio_set_value(kai_bl_enb, 0);

	return 0;
}

#ifdef CONFIG_TEGRA_DC
static int kai_hdmi_vddio_enable(void)
{
	int ret;
	if (!kai_hdmi_vddio) {
		kai_hdmi_vddio = regulator_get(NULL, "vdd_hdmi_con");
		if (IS_ERR_OR_NULL(kai_hdmi_vddio)) {
			ret = PTR_ERR(kai_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_con\n");
			kai_hdmi_vddio = NULL;
			return ret;
		}
	}
	ret = regulator_enable(kai_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_con\n");
		regulator_put(kai_hdmi_vddio);
		kai_hdmi_vddio = NULL;
		return ret;
	}
	return ret;
}

static int kai_hdmi_vddio_disable(void)
{
	if (kai_hdmi_vddio) {
		regulator_disable(kai_hdmi_vddio);
		regulator_put(kai_hdmi_vddio);
		kai_hdmi_vddio = NULL;
	}
	return 0;
}

static int kai_hdmi_enable(void)
{
	int ret;
	if (!kai_hdmi_reg) {
		kai_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(kai_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			kai_hdmi_reg = NULL;
			return PTR_ERR(kai_hdmi_reg);
		}
	}
	ret = regulator_enable(kai_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!kai_hdmi_pll) {
		kai_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(kai_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			kai_hdmi_pll = NULL;
			regulator_put(kai_hdmi_reg);
			kai_hdmi_reg = NULL;
			return PTR_ERR(kai_hdmi_pll);
		}
	}
	ret = regulator_enable(kai_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int kai_hdmi_disable(void)
{
	regulator_disable(kai_hdmi_reg);
	regulator_put(kai_hdmi_reg);
	kai_hdmi_reg = NULL;

	regulator_disable(kai_hdmi_pll);
	regulator_put(kai_hdmi_pll);
	kai_hdmi_pll = NULL;
	return 0;
}

static struct resource kai_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by kai_panel_init() */
		.end	= 0,	/* Filled in by kai_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource kai_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

#if defined(CONFIG_BRIDGE_SSD2828) && !defined(CONFIG_KEENHI_NV_DC_SSD2828_AUO)//is CPT panel
static struct tegra_dc_mode nabi2_xd_panel_modes[] = {
	{
		/* 1366x768@58Hz */
		.pclk = 70000000,//70mhz
		.h_ref_to_sync = 2,
		.v_ref_to_sync = 1,
		.h_sync_width = 2,
		.v_sync_width = 2,
		.h_active = 1366,
		.v_active = 768,
	       .h_back_porch = 46,
	       .v_back_porch = 14,
	       .h_front_porch = 108,
	       .v_front_porch = 26,
	},
};
#endif

#if defined(CONFIG_BRIDGE_SSD2828) && defined(CONFIG_KEENHI_NV_DC_SSD2828_AUO)//is AUO panel
static struct tegra_dc_mode nabi2_xd_panel_modes[] = {
	{
		/* 768x1024@60Hz */
		.pclk = 64800000,//64.8mhz
		.h_ref_to_sync = 2,
		.v_ref_to_sync = 1,
		.h_sync_width = 64,
		.v_sync_width = 50,
		.h_active = 768,
		.v_active = 1024,
	       .h_back_porch = 56,
	       .v_back_porch = 30,
	       .h_front_porch = 60,
	       .v_front_porch = 36,
	},
};
#endif

//#define __CHAGE_FREQ__
#ifdef CONFIG_TLC59116_LED
static struct tegra_dc_mode nabi2_3d_panel_modes[] = {
	{
#ifdef __CHAGE_FREQ__
		/* 1024x600@60Hz */
		.pclk = 66770000,//51206000,
#else
		.pclk = 81750000,//51206000,
#endif
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 64,
		.v_sync_width = 1,
		.h_back_porch = 128,
		.v_back_porch = 2,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = 64,
		.v_front_porch = 5,
	},
};
#else
static struct tegra_dc_mode nabi2_3d_panel_modes[] = {
	{
#ifdef __CHAGE_FREQ__
		/* 1024x600@60Hz */
		.pclk = 66770000,//51206000,
#else
		.pclk = 81750000,//51206000,
#endif
		.h_ref_to_sync = 1,
		.v_ref_to_sync = 1,
		.h_sync_width = 64,
		.v_sync_width = 1,
		.h_back_porch = 128,
		.v_back_porch = 2,
		.h_active = 800,
		.v_active = 1280,
		.h_front_porch = 64,
		.v_front_porch = 5,
	},
};
#endif

static struct tegra_dc_sd_settings kai_sd_settings = {
	.enable = 1, /* enabled by default. */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 1,
	.phase_in_adjustments = true,
	.use_vid_luma = false,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 74, 83},
				{93, 103, 114, 126},
				{138, 151, 165, 179},
				{194, 209, 225, 242},
			},
			{
				{58, 66, 75, 84},
				{94, 105, 116, 127},
				{140, 153, 166, 181},
				{196, 211, 227, 244},
			},
			{
				{60, 68, 77, 87},
				{97, 107, 119, 130},
				{143, 156, 170, 184},
				{199, 215, 231, 248},
			},
			{
				{64, 73, 82, 91},
				{102, 113, 124, 137},
				{149, 163, 177, 192},
				{207, 223, 240, 255},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{250, 250, 250},
				{194, 194, 194},
				{149, 149, 149},
				{113, 113, 113},
				{82, 82, 82},
				{56, 56, 56},
				{34, 34, 34},
				{15, 15, 15},
				{0, 0, 0},
			},
			{
				{246, 246, 246},
				{191, 191, 191},
				{147, 147, 147},
				{111, 111, 111},
				{80, 80, 80},
				{55, 55, 55},
				{33, 33, 33},
				{14, 14, 14},
				{0, 0, 0},
			},
			{
				{239, 239, 239},
				{185, 185, 185},
				{142, 142, 142},
				{107, 107, 107},
				{77, 77, 77},
				{52, 52, 52},
				{30, 30, 30},
				{12, 12, 12},
				{0, 0, 0},
			},
			{
				{224, 224, 224},
				{173, 173, 173},
				{133, 133, 133},
				{99, 99, 99},
				{70, 70, 70},
				{46, 46, 46},
				{25, 25, 25},
				{7, 7, 7},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = &kai_backlight_device,
};

#ifdef CONFIG_TEGRA_DC

#ifdef CONFIG_TLC59116_LED
static struct tegra_fb_data nabi2_3d_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 800,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};
#else
static struct tegra_fb_data nabi2_3d_fb_data = {
	.win		= 0,
	.xres		= 800,
	.yres		= 1280,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};
#endif

static struct tegra_fb_data kai_hdmi_fb_data = {
	.win		= 0,
	.xres		= 1024,
	.yres		= 600,
	.bits_per_pixel	= 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out kai_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,

	.dcc_bus	= 3,
	.hotplug_gpio	= kai_hdmi_hpd,

	.max_pixclock	= KHZ2PICOS(148500),

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= kai_hdmi_enable,
	.disable	= kai_hdmi_disable,

	.postsuspend	= kai_hdmi_vddio_disable,
	.hotplug_init	= kai_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data nabi2_3d_disp2_pdata = {
	.flags		= 0,
	.default_out	= &kai_disp2_out,
	.fb		= &kai_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};
#endif

#ifdef CONFIG_BRIDGE_SSD2828
static struct tegra_dc_out_pin kai_dc_out_pins[] = {
	{
		.name	= TEGRA_DC_OUT_PIN_H_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},

	{
		.name	= TEGRA_DC_OUT_PIN_V_SYNC,
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
	},

	{
		.name	= TEGRA_DC_OUT_PIN_PIXEL_CLOCK,
#if defined(CONFIG_KEENHI_NV_DC_SSD2828_AUO)//is AUO panel
		.pol	= TEGRA_DC_OUT_PIN_POL_LOW,
#else
		.pol	= TEGRA_DC_OUT_PIN_POL_HIGH,
#endif
	},
};
#endif

#ifdef CONFIG_BRIDGE_SSD2828
static struct tegra_dc_out nabi2_xd_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.sd_settings	= &kai_sd_settings,
	.parent_clk	= "pll_p",
	.parent_clk_backup = "pll_d2_out0",

	.type		= TEGRA_DC_OUT_RGB,
	.depth		= 19,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes		= nabi2_xd_panel_modes,
	.n_modes	= ARRAY_SIZE(nabi2_xd_panel_modes),
	.out_pins	= kai_dc_out_pins,
	.n_out_pins	= ARRAY_SIZE(kai_dc_out_pins),
	.enable		= kai_panel_enable,
	.postpoweron	= kai_panel_postpoweron,
	.prepoweroff	= kai_panel_prepoweroff,
	.disable	= kai_panel_disable,
};
#endif

#ifdef CONFIG_KEENHI_NV_DC_DSI_OUT
static struct tegra_dsi_cmd dsi_init_cmd[]= {

	DSI_CMD_SHORT(0x23, 0xF3, 0xA0),
	DSI_DLY_MS(100),

	DSI_CMD_SHORT(0x23, 0x0E, 0x02),
	DSI_DLY_MS(100),

	DSI_CMD_SHORT(0x03, 0x00, 0x00),
	DSI_DLY_MS(100),
};

struct tegra_dsi_out nabi2_xd_dsi = {
	.n_data_lanes = 2,
	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,

	.refresh_rate = 60,

	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.video_burst_mode= TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,

	.hs_cmd_mode_supported=false,
	.no_pkt_seq_eot=false,//only burst_mode can be used
	.enable_hs_clock_on_lp_cmd_mode=false, //if true force video clock mode entry  continuous mode
	.panel_send_dc_frames=false,

	.panel_has_frame_buffer = false,
	.dsi_instance = 0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.n_init_cmd = ARRAY_SIZE(dsi_init_cmd),
	.dsi_init_cmd = dsi_init_cmd,
	.video_clock_mode=TEGRA_DSI_VIDEO_CLOCK_CONTINUOUS,

	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
};
#endif

static struct tegra_dc_out nabi2_3d_disp1_out = {
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.sd_settings	= &kai_sd_settings,
	.parent_clk	= "pll_p",
	.parent_clk_backup = "pll_d2_out0",

	.type		= TEGRA_DC_OUT_RGB,
	.depth		= 18,
	.dither		= TEGRA_DC_ORDERED_DITHER,

	.modes		= nabi2_3d_panel_modes,
	.n_modes	= ARRAY_SIZE(nabi2_3d_panel_modes),

	.enable		= kai_panel_enable,
	.postpoweron	= kai_panel_postpoweron,
	.prepoweroff	= kai_panel_prepoweroff,
	.disable	= kai_panel_disable,

	.width		= 151,
	.height		= 94,
};

#ifdef CONFIG_TEGRA_DC

#ifdef CONFIG_BRIDGE_SSD2828
static struct tegra_dc_platform_data nabi2_xd_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &nabi2_xd_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &nabi2_xd_fb_data,
};
#endif
static struct tegra_dc_platform_data nabi2_3d_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &nabi2_3d_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &nabi2_3d_fb_data,
};

#ifdef CONFIG_BRIDGE_SSD2828
static struct nvhost_device nabi2_xd_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= kai_disp1_resources,
	.num_resources	= ARRAY_SIZE(kai_disp1_resources),
	.dev = {
		.platform_data = &nabi2_xd_disp1_pdata,
	},
};
#endif
static struct nvhost_device nabi2_3d_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= kai_disp1_resources,
	.num_resources	= ARRAY_SIZE(kai_disp1_resources),
	.dev = {
		.platform_data = &nabi2_3d_disp1_pdata,
	},
};

static int kai_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &nabi2_3d_disp1_device.dev;
}

static struct nvhost_device nabi2_3d_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= kai_disp2_resources,
	.num_resources	= ARRAY_SIZE(kai_disp2_resources),
	.dev = {
		.platform_data = &nabi2_3d_disp2_pdata,
	},
};

#else
static int kai_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return 0;
}
#endif

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout kai_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by kai_panel_init() */
		.size		= 0,	/* Filled in by kai_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data kai_nvmap_data = {
	.carveouts	= kai_carveouts,
	.nr_carveouts	= ARRAY_SIZE(kai_carveouts),
};

static struct platform_device kai_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &kai_nvmap_data,
	},
};
#endif

static struct platform_device *kai_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&kai_nvmap_device,
#endif
	&tegra_pwfm0_device,
	&kai_backlight_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend kai_panel_early_suspender;

static void kai_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);
	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_store_default_gov();
	cpufreq_change_gov(cpufreq_conservative_gov);
#endif
}

static void kai_panel_late_resume(struct early_suspend *h)
{
	unsigned i;
#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
	cpufreq_restore_default_gov();
#endif
	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
}
#endif

#ifdef CONFIG_BRIDGE_SSD2828
int register_resume_callback(struct ssd2828_data * data){
	pr_err("%s:=================>register lcd init\n",__func__);
	mSsd2828_Data = data;
	return 0;
}

static __initdata struct tegra_clk_init_table spi_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "sbc2",       "pll_p",        52000000,       true},
	{ NULL,         NULL,           0,              0},
};

static struct ssd2828_device_platform_data ssd2828_platformdata = {
	.register_resume = register_resume_callback,
	.mipi_shutdown_gpio = TEGRA_GPIO_PN6,
	.mipi_reset_gpio = TEGRA_GPIO_PP1,
	.lcd_en_3v3_gpio =TEGRA_GPIO_PH2, //TEGRA_GPIO_PX1,
	.lcd_vdds_en_gpio = TEGRA_GPIO_PH3,//TEGRA_GPIO_PW1 //if auo lcd bist
#ifdef CONFIG_KEENHI_NV_DC_SSD2828_AUO
	.lcd_1v2_gpio = -1,
#else
	.lcd_1v2_gpio = TEGRA_GPIO_PP2,
#endif
};
struct spi_board_info ssd_2825_spi_board[1] = {
	{
	 .modalias = "ssd_2825",
	 .bus_num = 1,
	 .chip_select = 1,
	 .max_speed_hz = 13 * 1000 * 1000,
	 .mode = SPI_MODE_0,
	 .platform_data = &ssd2828_platformdata,
	 },
};

int touch_init_ssd_lcd(void)
{
	int err = 0;
	spi_register_board_info(ssd_2825_spi_board,
				ARRAY_SIZE(ssd_2825_spi_board));
	pr_err("%s:===============>\n",__func__);
	return err;
}

static int __init kai_ssd_init(void)
{
	tegra_clk_init_from_table(spi_clk_init_table);
	touch_init_ssd_lcd();
	return 0;
}
#endif
int __init kai_panel_init(void)
{
	int err;
	struct resource __maybe_unused *res;

#if defined(CONFIG_TEGRA_NVMAP)
	kai_carveouts[1].base = tegra_carveout_start;
	kai_carveouts[1].size = tegra_carveout_size;
#endif

	gpio_request(kai_lvds_avdd_en, "lvds_avdd_en");
	gpio_direction_output(kai_lvds_avdd_en, 1);

	gpio_request(kai_lvds_vdd, "lvds_vdd_3v");
	gpio_direction_output(kai_lvds_vdd, 1);

	gpio_request(kai_lvds_shutdown, "lvds_shutdown");
	gpio_direction_output(kai_lvds_shutdown, 1);

	kai_backlight_data.dft_brightness = 30;
	kai_backlight_data.pwm_period_ns = 5000000;

	gpio_request(kai_hdmi_hpd, "hdmi_hpd");
	gpio_direction_input(kai_hdmi_hpd);

#ifdef CONFIG_HAS_EARLYSUSPEND
	kai_panel_early_suspender.suspend = kai_panel_early_suspend;
	kai_panel_early_suspender.resume = kai_panel_late_resume;
	kai_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&kai_panel_early_suspender);
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra3_register_host1x_devices();
	if (err)
		return err;
#endif

	err = platform_add_devices(kai_gfx_devices,
				ARRAY_SIZE(kai_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&nabi2_3d_disp1_device,
				 IORESOURCE_MEM, "fbmem");

	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
				min(tegra_fb_size, tegra_bootloader_fb_size));


#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
	err = nvhost_device_register(&nabi2_3d_disp1_device);

	res = nvhost_get_resource_byname(&nabi2_3d_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
	if (!err)
		err = nvhost_device_register(&nabi2_3d_disp2_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif
	return err;
}
