/*
 * arch/arm/mach-tegra/board-kai-sensors.c
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
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/nct1008.h>
#include <linux/err.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <mach/thermal.h>
#include "board.h"
#include "board-nabi2.h"
#include "cpu-tegra.h"
#include <linux/interrupt_bq27x00.h>
#include <linux/bq24160_charger.h>

#ifdef CONFIG_INPUT_KIONIX_ACCEL
#include <linux/input/kionix_accel.h>
#endif

#define DMARD06_IRQ_GPIO		TEGRA_GPIO_PX1

#define GC0308_POWER_RST_PIN TEGRA_GPIO_PBB0
#define GC0308_POWER_DWN_PIN TEGRA_GPIO_PBB5

#define T8EV5_POWER_RST_PIN TEGRA_GPIO_PBB4
#define T8EV5_POWER_DWN_PIN TEGRA_GPIO_PBB6

static struct regulator *kai_1v8_cam1;
static struct regulator *kai_vdd_cam1;

static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *thermal_device;

	thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!thermal_device) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	thermal_device->name = "nct72";
	thermal_device->data = data;
	thermal_device->id = THERMAL_DEVICE_ID_NCT_EXT;
	thermal_device->offset = TDIODE_OFFSET;
	thermal_device->get_temp = nct_get_temp;
	thermal_device->get_temp_low = nct_get_temp_low;
	thermal_device->set_limits = nct_set_limits;
	thermal_device->set_alert = nct_set_alert;
	thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(thermal_device);
}

static struct nct1008_platform_data kai_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x09, /* 0x09 corresponds to 32Hz conversion rate */
	.offset = 8, /* 4 * 2C. 1C for device accuracies */
	.probe_callback = nct1008_probe_callback,
};

static struct i2c_board_info kai_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4C),
		.platform_data = &kai_nct1008_pdata,
		.irq = -1,
	}
};

static int kai_nct1008_init(void)
{
	int ret = 0;

	/* FIXME: enable irq when throttling is supported */
	kai_i2c4_nct1008_board_info[0].irq =
		TEGRA_GPIO_TO_IRQ(KAI_TEMP_ALERT_GPIO);

	ret = gpio_request(KAI_TEMP_ALERT_GPIO, "temp_alert");
	if (ret < 0) {
		pr_err("%s: gpio_request failed\n", __func__);
		return ret;
	}

	ret = gpio_direction_input(KAI_TEMP_ALERT_GPIO);
	if (ret < 0) {
		pr_err("%s: set gpio to input failed\n", __func__);
		gpio_free(KAI_TEMP_ALERT_GPIO);
	}
	return ret;
}

struct yuv_sensor_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
};

#ifdef CONFIG_VIDEO_GC0308
static int kai_gc0308_power_on(void)
{
	pr_err("%s:==========>EXE\n",__func__);

       //first pull up 1.8 power
	if (kai_1v8_cam1 == NULL) {
		kai_1v8_cam1 = regulator_get(NULL, "vdd_1v8_cam1");
		if (WARN_ON(IS_ERR(kai_1v8_cam1))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam1 %d\n",	__func__, (int)PTR_ERR(kai_1v8_cam1));
			goto reg_get_vdd_1v8_cam3_fail;
		}
	}
	regulator_enable(kai_1v8_cam1);
	//second pull up 2.8 power
	if (kai_vdd_cam1 == NULL) {
		kai_vdd_cam1 = regulator_get(NULL, "vdd_cam1");//2.8v
		if (WARN_ON(IS_ERR(kai_vdd_cam1))) {
			pr_err("%s: couldn't get regulator vdd_cam1: %d\n",	__func__, (int)PTR_ERR(kai_vdd_cam1));
			goto reg_get_vdd_cam3_fail;
		}
	}
	regulator_enable(kai_vdd_cam1);
	mdelay(5);

	gpio_direction_output(GC0308_POWER_DWN_PIN, 0);
	mdelay(10);
	gpio_direction_output(GC0308_POWER_RST_PIN, 1);
	mdelay(10);
	return 0;

reg_get_vdd_1v8_cam3_fail:
	regulator_put(kai_1v8_cam1);
	kai_1v8_cam1 = NULL;
reg_get_vdd_cam3_fail:
	regulator_put(kai_vdd_cam1);
	kai_vdd_cam1 = NULL;

	return -ENODEV;
}

static int kai_gc0308_power_off(void)
{
	pr_err("%s:==========>EXE\n",__func__);
	gpio_direction_output(GC0308_POWER_DWN_PIN, 1);//old 1
	gpio_direction_output(GC0308_POWER_RST_PIN, 0);

	if (kai_1v8_cam1){
		regulator_disable(kai_1v8_cam1);
		regulator_put(kai_1v8_cam1);
		kai_1v8_cam1 =NULL;
	}
	if (kai_vdd_cam1){
		regulator_disable(kai_vdd_cam1);
		regulator_put(kai_vdd_cam1);
		kai_vdd_cam1 =NULL;
	}
	return 0;
}

struct yuv_sensor_platform_data kai_gc0308_data = {
	.power_on = kai_gc0308_power_on,
	.power_off = kai_gc0308_power_off,
};
#endif

#ifdef CONFIG_VIDEO_T8EV5
static int kai_t8ev5_power_on(void)
{
	pr_err("%s:==========>EXE\n",__func__);

	if (kai_1v8_cam1 == NULL) {
		kai_1v8_cam1 = regulator_get(NULL, "vdd_1v8_cam1");
		if (WARN_ON(IS_ERR(kai_1v8_cam1))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam1 %d\n",	__func__, (int)PTR_ERR(kai_1v8_cam1));
			goto reg_get_vdd_1v8_cam3_fail;
		}
	}
	regulator_enable(kai_1v8_cam1);

	if (kai_vdd_cam1 == NULL) {
		kai_vdd_cam1 = regulator_get(NULL, "vdd_cam1");
		if (WARN_ON(IS_ERR(kai_vdd_cam1))) {
			pr_err("%s: couldn't get regulator vdd_cam1: %d\n",	__func__, (int)PTR_ERR(kai_vdd_cam1));
			goto reg_get_vdd_cam3_fail;
		}
	}
	regulator_enable(kai_vdd_cam1);
	mdelay(5);

	gpio_direction_output(T8EV5_POWER_DWN_PIN, 1);
	mdelay(10);
	gpio_direction_output(T8EV5_POWER_RST_PIN, 1);
	mdelay(10);

	return 0;

reg_get_vdd_1v8_cam3_fail:
	regulator_put(kai_1v8_cam1);
	kai_1v8_cam1 = NULL;
reg_get_vdd_cam3_fail:
	regulator_put(kai_vdd_cam1);
	kai_vdd_cam1 = NULL;

	return -ENODEV;
}

static int kai_t8ev5_power_off(void)
{
	pr_err("%s:==========>EXE\n",__func__);
	gpio_direction_output(T8EV5_POWER_DWN_PIN, 0);//old 1
	gpio_direction_output(T8EV5_POWER_RST_PIN, 0);

	if (kai_1v8_cam1){
		regulator_disable(kai_1v8_cam1);
		regulator_put(kai_1v8_cam1);
		kai_1v8_cam1 = NULL;
	}

	if (kai_vdd_cam1){
		regulator_disable(kai_vdd_cam1);
		regulator_put(kai_vdd_cam1);
		kai_vdd_cam1 = NULL;
	}

	return 0;
}

struct yuv_sensor_platform_data kai_t8ev5_data = {
	.power_on = kai_t8ev5_power_on,
	.power_off = kai_t8ev5_power_off,
};
#endif

static struct i2c_board_info nabi2_gc0308_board_info[] = {
	{
		I2C_BOARD_INFO("gc0308", 0x21),
		.platform_data = &kai_gc0308_data,
	},
};

static struct i2c_board_info nabi2_t8ev5_i2c9_board_info[] = {
	{
		I2C_BOARD_INFO("t8ev5", 0x3c),
		.platform_data = &kai_t8ev5_data,
	},
};

static int kai_camera_init(void)
{
	int ret;

	//tegra_gpio_enable(GC0308_POWER_DWN_PIN);
	ret = gpio_request(GC0308_POWER_DWN_PIN, "gc0308_pwdn");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "GC0308_PWDN_GPIO");
	}
	gpio_direction_output(GC0308_POWER_DWN_PIN, 1);//old 1

	//tegra_gpio_enable(GC0308_POWER_RST_PIN);
	ret = gpio_request(GC0308_POWER_RST_PIN, "gc0308_reset");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "GC0308_RESET_GPIO");
	}
	gpio_direction_output(GC0308_POWER_RST_PIN, 0);

	//tegra_gpio_enable(T8EV5_POWER_DWN_PIN);
	ret = gpio_request(T8EV5_POWER_DWN_PIN, "t8ev5_pwdn");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "T8EV5_PWDN_GPIO");
	}
	gpio_direction_output(T8EV5_POWER_DWN_PIN, 0);//old 1

	//tegra_gpio_enable(T8EV5_POWER_RST_PIN);
	ret = gpio_request(T8EV5_POWER_RST_PIN, "t8ev5_reset");
	if (ret < 0) {
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "T8EV5_RESET_GPIO");
	}
	gpio_direction_output(T8EV5_POWER_RST_PIN, 0);

	return 0;
}

//add by vin for usi_3g_module
static int kai_usi_3g_module_init(void)
{
	int ret;

    ret = gpio_request(USI_3G_PWR_KEY_GPIO, "usi_3g_pwr_key");
    if (ret < 0) {
        pr_err("%s: gpio_request failed for gpio %s\n",
		    __func__, "USI_3G_PWR_KEY_GPIO");
    }
    gpio_direction_output(USI_3G_PWR_KEY_GPIO, 1);

    ret = gpio_request(USI_3G_PWR_EN_GPIO, "usi_3g_pwr_en");
    if (ret < 0) {
        pr_err("%s: gpio_request failed for gpio %s\n",
		    __func__, "USI_3G_PWR_EN_GPIO");
    }
    gpio_direction_output(USI_3G_PWR_EN_GPIO, 1);

	return 0;
}

void kai_usi_3g_usb_en_value(int value)
{
	//printk("======================set gpio to %d \n",value);
	gpio_direction_output(USI_3G_USB_EN_GPIO, value);
	msleep(10);
}
EXPORT_SYMBOL_GPL(kai_usi_3g_usb_en_value);


#ifdef CONFIG_BATTERY_BQ27x00_2
static struct interrupt_plug_ac ac_ok_int[] = {
	{
		.irq = TEGRA_MAX77663_TO_IRQ(MAX77663_IRQ_ONOFF_ACOK_FALLING),
		.active_low = 1,
	},
	{
		.irq = TEGRA_MAX77663_TO_IRQ(MAX77663_IRQ_ONOFF_ACOK_RISING),
		.active_low = 0,
	},
};

struct interrupt_bq27x00_platform_data kai_bq27x00_data = {
	.int_plug_ac = ac_ok_int,
	.nIrqs = ARRAY_SIZE(ac_ok_int),
};

static const struct i2c_board_info ventana_i2c1_board_info[] = {
	{
		I2C_BOARD_INFO("bq27500", 0x55),
		.platform_data = &kai_bq27x00_data,
	},
};
#endif

#ifdef CONFIG_INPUT_KIONIX_ACCEL
struct kionix_accel_platform_data nabi2_qc750_kionix_accel_pdata = {
	.min_interval = 5,
	.poll_interval = 200,
	.accel_direction = 8,
	.accel_irq_use_drdy = 0,
	.accel_res = KIONIX_ACCEL_RES_12BIT,
	.accel_g_range = KIONIX_ACCEL_G_2G,
};

static struct i2c_board_info __initdata nabi2_qc750_inv_mpu_i2c0_board_info[] = {
	{ I2C_BOARD_INFO("kionix_accel", KIONIX_ACCEL_I2C_ADDR),
		.platform_data = &nabi2_qc750_kionix_accel_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(DMARD06_IRQ_GPIO), // Replace with appropriate GPIO setup
	},
};
#endif /* CONFIG_INPUT_KIONIX_ACCEL */

#ifdef CONFIG_SENSORS_AK8975
static struct i2c_board_info __initdata keenhi_i2c0_board_info_copmass[] = {
		{
			I2C_BOARD_INFO("akm8975", 0x0c),
			.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PW0),
		},
};
#endif /* CONFIG_SENSORS_AK8975 */

#ifdef CONFIG_CHARGER_BQ24160
static struct regulator_consumer_supply bq24160_otg_vbus_supply[] = {
	REGULATOR_SUPPLY("usb_vbus_otg", NULL),
};

static struct bq24160_charger_platform_data bq24160_charger_pdata_xd = {
	.vbus_gpio = TEGRA_GPIO_PH1,
	.otg_consumer_supplies = bq24160_otg_vbus_supply,
	.num_otg_consumer_supplies = ARRAY_SIZE(bq24160_otg_vbus_supply),
	.bq24160_reg3 = 0x8E, // set charge regulation voltage 4.2v  Input Limit for   2.5A
	.bq24160_reg5 = 0xEE,//set changer current 0.1A , 2.7A
	.bq24160_reg5_susp = 0xEA
};

static struct i2c_board_info charger_board_info_xd[] = {
	{
		I2C_BOARD_INFO("bq24160", 0x6b),
		.irq		= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PK2),
		.platform_data = &bq24160_charger_pdata_xd,
	},
};
#endif

static void keenhi_camera_init(void)
{
	kai_camera_init();

	i2c_register_board_info(2, nabi2_gc0308_board_info,
		ARRAY_SIZE(nabi2_gc0308_board_info));
	i2c_register_board_info(2, nabi2_t8ev5_i2c9_board_info,
		ARRAY_SIZE(nabi2_t8ev5_i2c9_board_info));
}

static void keenhi_charge_init(void)
{
	int rc;

#ifdef CONFIG_BATTERY_BQ27x00_2
	i2c_register_board_info(1, ventana_i2c1_board_info,
		ARRAY_SIZE(ventana_i2c1_board_info));
#endif

#ifdef CONFIG_CHARGER_BQ24160
	rc = gpio_request(TEGRA_GPIO_PK2, "charge-bq24160");
	if (rc)
		pr_err("charge-bq24160 gpio request failed: %d\n", rc);

	rc = gpio_direction_input(TEGRA_GPIO_PK2);
	if (rc)
		pr_err("charge-bq24160 gpio direction configuration failed: %d\n", rc);

	i2c_register_board_info(4, charger_board_info_xd,
		ARRAY_SIZE(charger_board_info_xd));
#endif
}

int __init kai_sensors_init(void)
{
	int err;

	err = kai_nct1008_init();
	if (err)
		pr_err("%s: nct1008 init failed\n", __func__);
	else
		i2c_register_board_info(4, kai_i2c4_nct1008_board_info,
			ARRAY_SIZE(kai_i2c4_nct1008_board_info));

	kai_usi_3g_module_init();

	keenhi_camera_init();
	keenhi_charge_init();

#ifdef CONFIG_INPUT_KIONIX_ACCEL
	pr_info("*** sensor init ***\n");
	i2c_register_board_info(MPU_GYRO_BUS_NUM, nabi2_qc750_inv_mpu_i2c0_board_info,
		ARRAY_SIZE(nabi2_qc750_inv_mpu_i2c0_board_info));
#endif

#ifdef CONFIG_SENSORS_AK8975
	pr_info("*** compass init ***\n");
	i2c_register_board_info(KEENHI_COMPASS_BUS_NUM, keenhi_i2c0_board_info_copmass,
			ARRAY_SIZE(keenhi_i2c0_board_info_copmass));
#endif

	return 0;
}
