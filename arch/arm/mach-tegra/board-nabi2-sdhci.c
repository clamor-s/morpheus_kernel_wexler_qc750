/*
 * arch/arm/mach-tegra/board-kai-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2012 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/wl12xx.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/io_dpd.h>

#include "gpio-names.h"
#include "board.h"
#include "board-nabi2.h"
#include <linux/wlan_plat.h>

#define TI_WLAN_EN	TEGRA_GPIO_PD4
#define TI_WLAN_IRQ	TEGRA_GPIO_PV1

#define NABI2_SD_CD TEGRA_GPIO_PI5

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int nabi2_wifi_status_register(void (*callback)(int , void *), void *);

static int nabi2_wifi_power(int on);
static int nabi2_wifi_set_carddetect(int val);

static struct wl12xx_platform_data ti_wlan_data __initdata = {
	.irq = TEGRA_GPIO_TO_IRQ(TI_WLAN_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
	.set_power = nabi2_wifi_power,
	.set_carddetect = nabi2_wifi_set_carddetect,
};

static struct resource sdhci_resource0[] = {
	[0] = {
		.start	= INT_SDMMC1,
		.end	= INT_SDMMC1,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource2[] = {
	[0] = {
		.start	= INT_SDMMC3,
		.end	= INT_SDMMC3,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start	= INT_SDMMC4,
		.end	= INT_SDMMC4,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_sdhci_platform_data ti_sdhci_platform_data2 = {
	.mmc_data = {
		.register_status_notify	= nabi2_wifi_status_register,
		.built_in = 0,
		.ocr_mask = MMC_OCR_1V8_MASK,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
};


static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.cd_gpio = NABI2_SD_CD,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.built_in = 1,
	}
};

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device ti_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &ti_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int nabi2_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int nabi2_wifi_set_carddetect(int val)
{
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int nabi2_wifi_power(int on)
{
	/*
	 * FIXME : we need to revisit IO DPD code
	 * on how should multiple pins under DPD get controlled
	 *
	 * cardhu GPIO WLAN enable is part of SDMMC3 pin group
	 */
 	struct tegra_io_dpd *sd_dpd;
 	sd_dpd = tegra_io_dpd_get(&ti_sdhci_device2.dev);

	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_disable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}

	if (on) {
		gpio_set_value(TI_WLAN_EN, 1);
		mdelay(15);
		gpio_set_value(TI_WLAN_EN, 0);
		mdelay(1);
		gpio_set_value(TI_WLAN_EN, 1);
		mdelay(70);
	} else {
		gpio_set_value(TI_WLAN_EN, 0);
	}

	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_enable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}

	return 0;
}

static void ti_wifi_init(void)
{
	int rc;

	rc = gpio_request(TI_WLAN_EN, "wl12xx-power");
	if (rc)
		pr_err("WLAN_EN gpio request failed: %d\n", rc);

	rc = gpio_request(TI_WLAN_IRQ, "wl12xx");
	if (rc)
		pr_err("WLAN_IRQ gpio request failed: %d\n", rc);

	rc = gpio_direction_output(TI_WLAN_EN, 0);
	if (rc)
		pr_err("WLAN_EN gpio direction configuration failed: %d\n", rc);

	rc = gpio_direction_input(TI_WLAN_IRQ);
	if (rc)
		pr_err("WLAN_IRQ gpio direction configuration failed: %d\n", rc);

	if (wl12xx_set_platform_data(&ti_wlan_data))
		pr_err("Error setting wl12xx data\n");
}

int __init kai_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&ti_sdhci_device2);
	ti_wifi_init();
	platform_device_register(&tegra_sdhci_device0);
	return 0;
}
