/*
 * arch/arm/mach-tegra/board-kai.h
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

#ifndef _MACH_TEGRA_BOARD_KAI_H
#define _MACH_TEGRA_BOARD_KAI_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/max77663-core.h>
#include "gpio-names.h"

/* External peripheral act as gpio */
/* MAX77663 GPIO */
#define MAX77663_GPIO_BASE	TEGRA_NR_GPIOS

/* Audio-related GPIOs */
#define TEGRA_GPIO_RT5631_RST	TEGRA_GPIO_PX2
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PW2

#ifdef CONFIG_SND_SOC_TEGRA_SOC_TLV320AIC325X
#define TEGRA_GPIO_SPKR_EN		TEGRA_GPIO_PB1
#endif

/* Touch releated GPIO */
#define TEGRA_GPIO_TOUCH_DETECT     TEGRA_GPIO_PH4
#define TEGRA_GPIO_TOUCH_DETECT_0   TEGRA_GPIO_PH5
#define TEGRA_GPIO_TOUCH_DETECT_1   TEGRA_GPIO_PH6
#define TEGRA_GPIO_TOUCH_DETECT_2   TEGRA_GPIO_PH7

/* Add by vin for usi_3g_module */
#define USI_3G_PWR_EN_GPIO		TEGRA_GPIO_PY3
#define USI_3G_USB_EN_GPIO		TEGRA_GPIO_PY2
#define USI_3G_PWR_KEY_GPIO		TEGRA_GPIO_PY1

/* Stat LED GPIO */
#define TEGRA_GPIO_STAT_LED		(MAX77663_GPIO_BASE + MAX77663_GPIO7)

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* MAX77663 IRQs */
#define MAX77663_IRQ_BASE	TEGRA_NR_IRQS

/* UART port which is used by bluetooth*/
#define BLUETOOTH_UART_DEV_NAME "/dev/ttyHS2"

int kai_charge_init(void);
int kai_regulator_init(void);
int kai_suspend_init(void);
int kai_sdhci_init(void);
int kai_pinmux_init(void);
int kai_panel_init(void);
int kai_sensors_init(void);
int kai_keys_init(void);
int kai_pins_state_init(void);
int kai_emc_init(void);
int kai_edp_init(void);
void __init kai_tsensor_init(void);
int __init touch_init_synaptics_kai(void);

#define KAI_TEMP_ALERT_GPIO	TEGRA_GPIO_PS3

#define MPU_GYRO_BUS_NUM	0
#define KEENHI_COMPASS_BUS_NUM	0

#define TDIODE_OFFSET	(10000) /* in millicelsius */

#endif
