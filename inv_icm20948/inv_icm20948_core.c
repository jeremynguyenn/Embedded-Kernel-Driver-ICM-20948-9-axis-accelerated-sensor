// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2025 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/workqueue.h>

#include "inv_icm20948.h"

/* Trigger operation structure */
static const struct iio_trigger_ops inv_icm20948_trigger_ops = {
	.owner = THIS_MODULE,
};

/* Configuration and feature settings */
#define INV_ICM20948_WHO_AM_I_VAL     0xEA
#define INV_ICM20948_REG_WHO_AM_I     0x00  /* This is the WHO_AM_I register address in Bank 0 */

static const struct regmap_range inv_icm20948_readable_ranges[] = {
    // Bank 0
    regmap_reg_range(INV_ICM20948_REG_WHO_AM_I, INV_ICM20948_REG_WHO_AM_I),
    regmap_reg_range(INV_ICM20948_REG_USER_CTRL, INV_ICM20948_REG_PWR_MGMT_2),
    regmap_reg_range(INV_ICM20948_REG_INT_PIN_CFG, INV_ICM20948_REG_INT_ENABLE_3),
    regmap_reg_range(INV_ICM20948_REG_INT_STATUS, INV_ICM20948_REG_INT_STATUS_3),
    regmap_reg_range(INV_ICM20948_REG_ACCEL_XOUT_H, INV_ICM20948_REG_TEMP_OUT_L),
    regmap_reg_range(INV_ICM20948_REG_EXT_SLV_SENS_DATA_00, INV_ICM20948_REG_EXT_SLV_SENS_DATA_06),
    regmap_reg_range(INV_ICM20948_REG_FIFO_EN_1, INV_ICM20948_REG_FIFO_EN_2),
    regmap_reg_range(INV_ICM20948_REG_FIFO_RST, INV_ICM20948_REG_FIFO_RST),
    regmap_reg_range(INV_ICM20948_REG_FIFO_COUNT_H, INV_ICM20948_REG_FIFO_R_W),
    regmap_reg_range(INV_ICM20948_REG_BANK_SEL, INV_ICM20948_REG_BANK_SEL),

    // Bank 2
    regmap_reg_range(INV_ICM20948_REG_GYRO_SMPLRT_DIV, INV_ICM20948_REG_GYRO_CONFIG_2),
    regmap_reg_range(INV_ICM20948_REG_ACCEL_SMPLRT_DIV_1, INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2),
    regmap_reg_range(INV_ICM20948_REG_ACCEL_CONFIG, INV_ICM20948_REG_ACCEL_CONFIG_2),

    // Bank 3
    regmap_reg_range(INV_ICM20948_REG_I2C_MST_CTRL, INV_ICM20948_REG_I2C_SLV0_DO),
    regmap_reg_range(INV_ICM20948_REG_I2C_SLV0_CTRL, INV_ICM20948_REG_I2C_SLV0_CTRL),
};

const struct regmap_access_table inv_icm20948_readable_regs = {
	.yes_ranges = inv_icm20948_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(inv_icm20948_readable_ranges),
};

static const struct regmap_range inv_icm20948_writable_ranges[] = {
	// Bank 0
	regmap_reg_range(INV_ICM20948_REG_USER_CTRL, INV_ICM20948_REG_PWR_MGMT_2),
	regmap_reg_range(INV_ICM20948_REG_INT_PIN_CFG, INV_ICM20948_REG_INT_ENABLE_3),
	regmap_reg_range(INV_ICM20948_REG_FIFO_EN_1, INV_ICM20948_REG_FIFO_EN_2),
	regmap_reg_range(INV_ICM20948_REG_FIFO_RST, INV_ICM20948_REG_FIFO_RST),
	regmap_reg_range(INV_ICM20948_REG_FIFO_R_W, INV_ICM20948_REG_FIFO_R_W),
	regmap_reg_range(INV_ICM20948_REG_BANK_SEL, INV_ICM20948_REG_BANK_SEL),

	// Bank 2
	regmap_reg_range(INV_ICM20948_REG_GYRO_SMPLRT_DIV, INV_ICM20948_REG_GYRO_CONFIG_2),
	regmap_reg_range(INV_ICM20948_REG_ACCEL_SMPLRT_DIV_1, INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2),
	regmap_reg_range(INV_ICM20948_REG_ACCEL_CONFIG, INV_ICM20948_REG_ACCEL_CONFIG_2),

	// Bank 3 - I2C Master Control Registers
	regmap_reg_range(INV_ICM20948_REG_I2C_MST_EDGE, INV_ICM20948_REG_I2C_SLV0_DO),
	regmap_reg_range(INV_ICM20948_REG_I2C_SLV0_CTRL, INV_ICM20948_REG_I2C_SLV0_CTRL),
};

const struct regmap_access_table inv_icm20948_writable_regs = {
	.yes_ranges = inv_icm20948_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(inv_icm20948_writable_ranges),
};

const struct regmap_config inv_icm20948_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
	.rd_table = &inv_icm20948_readable_regs,
	.wr_table = &inv_icm20948_writable_regs,
	.cache_type = REGCACHE_NONE,
};
EXPORT_SYMBOL_GPL(inv_icm20948_regmap_config);

/**
 * inv_icm20948_set_bank - Set register bank
 * @st: Driver state
 * @bank: Bank to select (0-3)
 *
 * Select the register bank to access registers in different banks.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_bank(struct inv_icm20948_state *st, u8 bank)
{
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int read_val;

	/* Switch to target Bank */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, bank);
	if (ret) {
		dev_err(dev, "Switch to Bank 0x%02x failed: %d\n", bank, ret);
		return ret;
	}

	/* Use msleep instead of usleep_range to ensure longer delays */
	msleep(10);

	/* Verify that the bank switch is successful */
	ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &read_val);
	if (ret) {
		dev_err(dev, "Verification of bank switch failed: %d\n", ret);
		return ret;
	}

	if ((read_val & INV_ICM20948_BANK_SEL_MASK) != bank) {
		dev_err(dev, "Bank switch verification failed, expected: 0x%02x, actual: 0x%02x\n",
			bank, read_val & INV_ICM20948_BANK_SEL_MASK);
		return -EIO;
	}

	return 0;
}

/**
 * inv_icm20948_write_reg - Write to a register in a specific bank
 * @st: Driver state
 * @bank: Bank number (0-3)
 * @reg: Register address
 * @val: Value to write
 *
 * Select bank, write to register, and restore bank 0.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_write_reg(struct inv_icm20948_state *st, u8 bank, u8 reg, u8 val)
{
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int current_bank, read_val;

	/* First read the current Bank */
	ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &current_bank);
	if (ret) {
		dev_err(dev, "Failed to read the current bank: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Current bank original value: 0x%02x, After mask: 0x%02x, Target bank: 0x%02x\n",
		 current_bank, current_bank & INV_ICM20948_BANK_SEL_MASK, bank);

	if ((current_bank & INV_ICM20948_BANK_SEL_MASK) != bank) {
		/* Switch to target Bank */
		ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, bank);
		if (ret) {
			dev_err(dev, "Switch to bank 0x%02x failed: %d\n", bank, ret);
			return ret;
		}

		/* Use msleep instead of usleep_range to ensure longer delays */
		msleep(20);

		/* Verify that the bank switch is successful */
		ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &read_val);
		if (ret) {
			dev_err(dev, "Verification of bank switch failed: %d\n", ret);
			return ret;
		}

		dev_info(dev, "Original value after bank switching: 0x%02x, After mask: 0x%02x\n",
			 read_val, read_val & INV_ICM20948_BANK_SEL_MASK);

		if ((read_val & INV_ICM20948_BANK_SEL_MASK) != bank) {
			dev_err(dev, "Bank switch verification failed, expected: 0x%02x, actual: 0x%02x\n",
				bank, read_val & INV_ICM20948_BANK_SEL_MASK);
			return -EIO;
		}
	}

	/* Write register value */
	ret = regmap_write(st->map, reg, val);
	if (ret) {
		dev_err(dev, "Failed to write register 0x%02x: %d\n", reg, ret);
		/* No error is returned, try to restore Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		msleep(10);
		return ret;
	}

	/* Wait for the write to complete */
	msleep(5);

	/* Restore to Bank 0 */
	if (bank != INV_ICM20948_BANK_0) {
		ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		if (ret) {
			dev_err(dev, "Failed to restore to bank 0: %d\n", ret);
			return ret;
		}
		msleep(10);
	}

	return 0;
}

/**
 * inv_icm20948_read_reg - Read from a register in a specific bank
 * @st: Driver state
 * @bank: Bank number (0-3)
 * @reg: Register address
 * @val: Pointer to store read value
 *
 * Select bank, read from register, and restore bank 0.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_read_reg(struct inv_icm20948_state *st, u8 bank, u8 reg, u8 *val)
{
	unsigned int regval;
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int current_bank, read_val;

	/* First read the current Bank */
	ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &current_bank);
	if (ret) {
		dev_err(dev, "Failed to read the current bank: %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "Current bank: 0x%02x, Target bank: 0x%02x\n",
		current_bank & INV_ICM20948_BANK_SEL_MASK, bank);

	if ((current_bank & INV_ICM20948_BANK_SEL_MASK) != bank) {
		/* Switch to target Bank */
		ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, bank);
		if (ret) {
			dev_err(dev, "Switch to bank 0x%02x failed: %d\n", bank, ret);
			return ret;
		}

		/* Use msleep instead of usleep_range to ensure longer delays */
		msleep(10);

		/* Verify that the bank switch is successful */
		ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &read_val);
		if (ret) {
			dev_err(dev, "Verification of bank switch failed: %d\n", ret);
			return ret;
		}

		if ((read_val & INV_ICM20948_BANK_SEL_MASK) != bank) {
			dev_err(dev, "Bank switch verification failed, expected: 0x%02x, actual: 0x%02x\n",
				bank, read_val & INV_ICM20948_BANK_SEL_MASK);
			return -EIO;
		}
	}

	/* Read register value */
	ret = regmap_read(st->map, reg, &regval);
	if (ret) {
		dev_err(dev, "Failed to read register 0x%02x: %d\n", reg, ret);
		/* Try to restore Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		msleep(10);
		return ret;
	}

	*val = (u8)regval;

	/* Restore to Bank 0 */
	if (bank != INV_ICM20948_BANK_0) {
		ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		if (ret) {
			dev_err(dev, "Failed to restore to bank 0: %d\n", ret);
			return ret;
		}
		msleep(10);
	}

	return 0;
}

/**
 * inv_icm20948_set_gyro_fs - Set gyroscope full-scale range
 * @st: Driver state
 * @fs: Full-scale range index
 *
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_gyro_fs(struct inv_icm20948_state *st, int fs)
{
	u8 val;
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int regval;

	if (fs >= INV_ICM20948_GYRO_FS_NB) {
		dev_err(dev, "The gyroscope range setting value is invalid: %d\n", fs);
		return -EINVAL;
	}

	dev_info(dev, "Set the gyroscope full-scale range: %d\n", fs);

	/* Switch directly to Bank 2 */
	dev_info(dev, "Switch to register bank 2\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_2);
	if (ret) {
		dev_err(dev, "Unable to switch to register bank 2: %d\n", ret);
		return ret;
	}

	/* Waiting for register bank switching to complete */
	usleep_range(2000, 3000);

	/* Read current configuration */
	ret = regmap_read(st->map, INV_ICM20948_REG_GYRO_CONFIG_1, &regval);
	if (ret) {
		dev_err(dev, "Failed to read GYRO_CONFIG_1: %d\n", ret);
		/* Restore to Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		return ret;
	}
	val = (u8)regval;
	dev_info(dev, "GYRO_CONFIG_1 current value: 0x%02x\n", val);

	/* Clear and set the range bit */
	val &= ~INV_ICM20948_GYRO_FSR_MASK;
	val |= (fs << INV_ICM20948_GYRO_FSR_SHIFT) & INV_ICM20948_GYRO_FSR_MASK;
	dev_info(dev, "New GYRO_CONFIG_1 value: 0x%02x\n", val);

	/* Write new configuration */
	ret = regmap_write(st->map, INV_ICM20948_REG_GYRO_CONFIG_1, val);
	if (ret) {
		dev_err(dev, "Writing GYRO_CONFIG_1 failed: %d\n", ret);
		/* Restore to Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		return ret;
	}

	/* Wait for the write to complete */
	usleep_range(1000, 2000);

	/* Restore to Bank 0 */
	dev_info(dev, "Restore to register bank 0\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Unable to restore to register bank 0: %d\n", ret);
		return ret;
	}

	/* Waiting for register bank switching to complete */
	usleep_range(1000, 2000);

	dev_info(dev, "Gyroscope full-scale setting completed\n");
	return 0;
}

/**
 * inv_icm20948_set_accel_fs - Set accelerometer full-scale range
 * @st: Driver state
 * @fs: Full-scale range index
 *
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_accel_fs(struct inv_icm20948_state *st, int fs)
{
	u8 val;
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int regval;

	if (fs >= INV_ICM20948_ACCEL_FS_NB) {
		dev_err(dev, "The accelerometer range setting value is invalid: %d\n", fs);
		return -EINVAL;
	}

	dev_info(dev, "Set the accelerometer full-scale range: %d\n", fs);

	/* Switch directly to Bank 2 */
	dev_info(dev, "Switch to register bank 2\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_2);
	if (ret) {
		dev_err(dev, "Unable to switch to register bank 2: %d\n", ret);
		return ret;
	}

	/* Waiting for register bank switching to complete */
	usleep_range(2000, 3000);

	/* Read current configuration */
	ret = regmap_read(st->map, INV_ICM20948_REG_ACCEL_CONFIG, &regval);
	if (ret) {
		dev_err(dev, "Failed to read ACCEL_CONFIG: %d\n", ret);
		/* Restore to Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		return ret;
	}
	val = (u8)regval;
	dev_info(dev, "ACCEL_CONFIG current value: 0x%02x\n", val);

	/* Clear and set the range bit */
	val &= ~INV_ICM20948_ACCEL_FSR_MASK;
	val |= (fs << INV_ICM20948_ACCEL_FSR_SHIFT) & INV_ICM20948_ACCEL_FSR_MASK;
	dev_info(dev, "New ACCEL_CONFIG value: 0x%02x\n", val);

	/* Write new configuration */
	ret = regmap_write(st->map, INV_ICM20948_REG_ACCEL_CONFIG, val);
	if (ret) {
		dev_err(dev, "Writing ACCEL_CONFIG failed: %d\n", ret);
		/* Restore to Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		return ret;
	}

	/* Wait for the write to complete */
	usleep_range(1000, 2000);

	/* Restore to Bank 0 */
	dev_info(dev, "Restore to register bank 0\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Unable to restore to register bank 0: %d\n", ret);
		return ret;
	}

	/* Waiting for register bank switching to complete */
	usleep_range(1000, 2000);

	dev_info(dev, "Accelerometer full-scale setting completed\n");
	return 0;
}

/**
 * inv_icm20948_set_sensor_rate - Set sensor sample rate
 * @st: Driver state
 * @type: Sensor type (0 for gyro, 1 for accel)
 * @rate: Desired rate in Hz
 *
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_sensor_rate(struct inv_icm20948_state *st, int type, int rate)
{
	u8 div;
	int ret;
	unsigned int regval;
	struct device *dev = regmap_get_device(st->map);
	u8 reg;

	dev_info(dev, "Directly set the sensor sampling rate: type=%d, rate=%dHz\n", type, rate);

	/* Calculate the frequency division value */
	div = 1125 / rate - 1;  /* 1125Hz internal sampling rate */
	if (div > 255)
		div = 255;
	if (div < 0)
		div = 0;

	dev_info(dev, "Calculated frequency division value: %d (0x%02x)\n", div, div);

	if (type == 0) {  /* gyroscope */
		reg = INV_ICM20948_REG_GYRO_SMPLRT_DIV;
		dev_info(dev, "Set the gyroscope sampling rate, register: 0x%02x\n", reg);
	} else if (type == 1) {  /* accelerometer */
		reg = INV_ICM20948_REG_ACCEL_SMPLRT_DIV_1;
		dev_info(dev, "Set the accelerometer sampling rate, register: 0x%02x\n", reg);
	} else {
		dev_err(dev, "Unknown sensor type: %d\n", type);
		return -EINVAL;
	}

	/* Verify communication */
	/* 1. Switch to Bank 0 and read WHO_AM_I */
	dev_info(dev, "Switch to bank 0 to read WHO_AM_I\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
	if (ret) {
		dev_err(dev, "Switch to bank 0 failed: %d\n", ret);
		return ret;
	}
	msleep(30);

	ret = regmap_read(st->map, INV_ICM20948_REG_WHO_AM_I, &regval);
	if (ret) {
		dev_err(dev, "Failed to read WHO_AM_I: %d\n", ret);
		return ret;
	}
	dev_info(dev, "WHO_AM_I value: 0x%02x (Should be 0xEA)\n", regval);

	/* 2. Switch to Bank 2 */
	dev_info(dev, "Switch to bank 2\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x20);
	if (ret) {
		dev_err(dev, "Switch to bank 2 failed: %d\n", ret);
		return ret;
	}
	msleep(30);

	/* 3. Important: Correctly configure FCHOICE and DLPF_CFG */
	if (type == 0) {  /* gyroscope */
		ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &regval);
		if (ret) {
			dev_err(dev, "Failed to read BANK_SEL: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Current BANK_SEL: 0x%02x (expect 0x20)\n", regval);
		/* Read the existing GYRO_CONFIG_1 value */
		ret = regmap_read(st->map, INV_ICM20948_REG_GYRO_CONFIG_1, &regval);
		if (ret) {
			dev_err(dev, "Failed to read GYRO_CONFIG_1: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		dev_info(dev, "Current GYRO_CONFIG_1 value: 0x%02x\n", regval);

		/* Set FCHOICE=1 (bit0=0) so that SMPLRT_DIV will take effect */
		regval &= ~INV_ICM20948_GYRO_FCHOICE_MASK; /* Clear the FCHOICE bit */

		dev_info(dev, "Write GYRO_CONFIG_1: 0x%02x (keep FSR and set FCHOICE=1)\n", regval);
		ret = regmap_write(st->map, INV_ICM20948_REG_GYRO_CONFIG_1, regval);
		if (ret) {
			dev_err(dev, "Failed to set GYRO_CONFIG_1: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		msleep(30);

		/* Configure GYRO_CONFIG_2, set DLPF_CFG (should be between 1-6) */
		ret = regmap_read(st->map, INV_ICM20948_REG_GYRO_CONFIG_2, &regval);
		if (ret) {
			dev_err(dev, "Failed to read GYRO_CONFIG_2: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		dev_info(dev, "Current GYRO_CONFIG_2 value: 0x%02x\n", regval);

		/* Set DLPF_CFG to a value between 1-6 */
		regval &= ~INV_ICM20948_GYRO_LPF_MASK;
		regval |= 1 & INV_ICM20948_GYRO_LPF_MASK; /* Set DLPF_CFG=1 */

		dev_info(dev, "Write to GYRO_CONFIG_2: 0x%02x (set DLPF_CFG=1)\n", regval);
		ret = regmap_write(st->map, INV_ICM20948_REG_GYRO_CONFIG_2, regval);
		if (ret) {
			dev_err(dev, "Failed to set GYRO_CONFIG_2: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		msleep(30);
	} else if (type == 1) {  /* accelerometer */
		/* Read the existing ACCEL_CONFIG value */
		ret = regmap_read(st->map, INV_ICM20948_REG_ACCEL_CONFIG, &regval);
		if (ret) {
			dev_err(dev, "Failed to read ACCEL_CONFIG: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		dev_info(dev, "Current ACCEL_CONFIG value: 0x%02x\n", regval);

		/* Set ACCEL_FCHOICE=1 (bit0=0) so that SMPLRT_DIV will take effect */
		regval &= ~INV_ICM20948_ACCEL_FCHOICE_MASK;

		dev_info(dev, "Write to ACCEL_CONFIG: 0x%02x (keep FSR and set FCHOICE=1)\n", regval);
		ret = regmap_write(st->map, INV_ICM20948_REG_ACCEL_CONFIG, regval);
		if (ret) {
			dev_err(dev, "Failed to set ACCEL_CONFIG: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		msleep(30);

		/* Configure ACCEL_CONFIG_2, set ACCEL_DLPFCFG (should be between 1-6) */
		ret = regmap_read(st->map, INV_ICM20948_REG_ACCEL_CONFIG_2, &regval);
		if (ret) {
			dev_err(dev, "Failed to read ACCEL_CONFIG_2: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		dev_info(dev, "Current ACCEL_CONFIG_2 value: 0x%02x\n", regval);

		/* Set ACCEL_DLPFCFG to a value between 1-6 */
		regval &= ~INV_ICM20948_ACCEL_LPF_MASK;
		regval |= 1 & INV_ICM20948_ACCEL_LPF_MASK; /* Set DLPFCFG=1 */

		dev_info(dev, "Write to ACCEL_CONFIG_2: 0x%02x (set DLPFCFG=1)\n", regval);
		ret = regmap_write(st->map, INV_ICM20948_REG_ACCEL_CONFIG_2, regval);
		if (ret) {
			dev_err(dev, "Failed to set ACCEL_CONFIG_2: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		msleep(30);
	}

	/* 4. Write divider value */
	dev_info(dev, "Write divider value 0x%02x to register 0x%02x\n", div, reg);
	ret = regmap_write(st->map, reg, div);
	if (ret) {
		dev_err(dev, "Failed to write divider value: %d\n", ret);
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
		return ret;
	}

	/* Update sampling rate field in state structure */
	if (type == 0) {
		st->conf.gyro.odr = div;
		dev_info(dev, "Updated gyroscope sampling rate field: odr=%d (actual rate: %dHz)\n", div, 1125/(div+1));
	} else if (type == 1) {
		st->conf.accel.odr = div;
		dev_info(dev, "Updated accelerometer sampling rate field: odr=%d (actual rate: %dHz)\n", div, 1125/(div+1));
	}

	msleep(30);

	/* 5. Read back divider value for verification */
	ret = regmap_read(st->map, reg, &regval);
	if (ret) {
		dev_err(dev, "Failed to read back divider value: %d\n", ret);
	} else {
		dev_info(dev, "Read back divider value: 0x%02x (expected: 0x%02x)\n", regval, div);
	}

	/* 6. If accelerometer, write second register */
	if (type == 1) {
		dev_info(dev, "Write accelerometer divider low byte 0x00 to register 0x%02x\n",
			INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2);
		ret = regmap_write(st->map, INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0x00);
		if (ret) {
			dev_err(dev, "Failed to write accelerometer low byte divider value: %d\n", ret);
		} else {
			dev_info(dev, "Successfully wrote accelerometer low byte divider value\n");
		}
		msleep(30);
	}

	/* 7. Restore to Bank 0 */
	dev_info(dev, "Restore to bank 0\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
	if (ret) {
		dev_err(dev, "Failed to restore to bank 0: %d\n", ret);
		return ret;
	}
	msleep(30);

	dev_info(dev, "Sensor sampling rate setting completed\n");
	return 0;
}

/**
 * inv_icm20948_setup_mag - Setup magnetometer I2C master
 * @st: Driver state
 *
 * Configure I2C master to communicate with AK09916 magnetometer.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_setup_mag(struct inv_icm20948_state *st)
{
	int ret;
	u8 val;

	/* Configure I2C master clock frequency */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_MST_CTRL,
				    INV_ICM20948_I2C_MST_CLK_400KHZ);
	if (ret)
		return ret;

	/* Configure slave 0 - read from magnetometer */
	/* Set slave address with read bit */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_ADDR,
				    INV_ICM20948_MAG_I2C_ADDR | INV_ICM20948_I2C_SLV0_RNW);
	if (ret)
		return ret;

	/* Set register to read from - magnetometer data */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_REG,
				    INV_ICM20948_REG_MAG_ST1);
	if (ret)
		return ret;

	/* Enable reading from magnetometer, read 8 bytes */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_CTRL,
				    INV_ICM20948_I2C_SLV0_EN | 0x08);
	if (ret)
		return ret;

	/* Put magnetometer in continuous mode 4 (100Hz) */
	/* Set slave address for write */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_ADDR,
				    INV_ICM20948_MAG_I2C_ADDR);
	if (ret)
		return ret;

	/* Set register to write to */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_REG,
				    INV_ICM20948_REG_MAG_CNTL2);
	if (ret)
		return ret;

	/* Set value to write */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_DO,
				    INV_ICM20948_MAG_MODE_CONT4);
	if (ret)
		return ret;

	/* Enable write */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_CTRL,
				    INV_ICM20948_I2C_SLV0_EN | 0x01);
	if (ret)
		return ret;

	/* Wait for write to complete */
	msleep(10);

	/* Set back to read mode */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_ADDR,
				    INV_ICM20948_MAG_I2C_ADDR | INV_ICM20948_I2C_SLV0_RNW);
	if (ret)
		return ret;

	/* Set register to read from */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_REG,
				    INV_ICM20948_REG_MAG_ST1);
	if (ret)
		return ret;

	/* Enable reading, read 8 bytes */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_CTRL,
				    INV_ICM20948_I2C_SLV0_EN | 0x08);
	if (ret)
		return ret;

	st->mag_initialized = true;

	return 0;
}

/**
 * inv_icm20948_configure_fifo - Configure FIFO
 * @st: Driver state
 * @enable: Enable or disable FIFO
 *
 * Configure and enable/disable FIFO.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_configure_fifo(struct inv_icm20948_state *st, bool enable)
{
	int ret;

	if (!enable) {
		/* Disable FIFO */
		ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
					INV_ICM20948_BIT_FIFO_EN, 0);
		return ret;
	}

	/* Reset FIFO */
	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				INV_ICM20948_BIT_FIFO_RST,
				INV_ICM20948_BIT_FIFO_RST);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				INV_ICM20948_BIT_FIFO_RST, 0);
	if (ret)
		return ret;

	/* Configure FIFO to receive accel, gyro, and possibly mag data */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_0,
				    INV_ICM20948_REG_FIFO_EN_2,
				    INV_ICM20948_FIFO_GYRO_EN_ALL |
				    INV_ICM20948_BIT_ACCEL_FIFO_EN);
	if (ret)
		return ret;

	if (st->mag_initialized) {
		ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_0,
					    INV_ICM20948_REG_FIFO_EN_1,
					    INV_ICM20948_BIT_SLV_0_FIFO_EN);
		if (ret)
			return ret;
	}

	/* Enable FIFO */
	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				INV_ICM20948_BIT_FIFO_EN,
				INV_ICM20948_BIT_FIFO_EN);
	if (ret)
		return ret;

	st->use_fifo = true;

	return 0;
}

/**
 * inv_icm20948_set_power_mode - Set power mode for the ICM20948
 * @st: Driver state
 * @mode: Desired power mode
 *
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_power_mode(struct inv_icm20948_state *st,
				enum inv_icm20948_sensor_mode mode)
{
	int ret;
	unsigned int val, read_val;
	struct device *dev = regmap_get_device(st->map);

	/* Ensure operation in Bank 0 */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Unable to set register bank 0: %d\n", ret);
		return ret;
	}

	/* Add delay to ensure bank switch completion */
	usleep_range(1000, 2000);

	switch (mode) {
	case INV_ICM20948_SENSOR_MODE_OFF:
		val = INV_ICM20948_BIT_SLEEP;
		dev_info(dev, "Setting power mode: Sleep mode\n");
		break;
	case INV_ICM20948_SENSOR_MODE_STANDBY:
		val = 0;
		dev_info(dev, "Setting power mode: Standby mode\n");
		break;
	case INV_ICM20948_SENSOR_MODE_LOW_POWER:
		val = INV_ICM20948_BIT_LP_EN;
		dev_info(dev, "Setting power mode: Low power mode\n");
		break;
	case INV_ICM20948_SENSOR_MODE_LOW_NOISE:
		val = 0;
		dev_info(dev, "Setting power mode: Low noise mode\n");
		break;
	default:
		dev_err(dev, "Invalid power mode: %d\n", mode);
		return -EINVAL;
	}

	/* Read current value */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_1, &read_val);
	if (ret) {
		dev_err(dev, "Failed to read PWR_MGMT_1: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_1 current value: 0x%02x\n", read_val);

	/* Preserve clock source settings, modify only power management bits */
	read_val &= INV_ICM20948_BIT_CLKSEL_MASK;
	val |= read_val;

	/* Write register value directly */
	ret = regmap_write(st->map, INV_ICM20948_REG_PWR_MGMT_1, val);
	if (ret) {
		dev_err(dev, "Failed to set power mode: %d\n", ret);
		return ret;
	}

	/* Verify setting */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_1, &read_val);
	if (ret) {
		dev_err(dev, "Failed to verify PWR_MGMT_1: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_1 value after setting: 0x%02x\n", read_val);

	/* Wait for power mode to stabilize */
	msleep(100);

	return 0;
}

/**
 * inv_icm20948_set_clock_source - Set clock source to internal oscillator
 * @st: Driver state
 *
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_clock_source(struct inv_icm20948_state *st)
{
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int val;

	/* Ensure operation in Bank 0 */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Unable to set register bank 0: %d\n", ret);
		return ret;
	}

	/* Add delay to ensure bank switch completion */
	usleep_range(1000, 2000);

	/* Read current value */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_1, &val);
	if (ret) {
		dev_err(dev, "Failed to read PWR_MGMT_1: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_1 current value: 0x%02x\n", val);

	/* Clear current clock bits and set to internal oscillator */
	val &= ~INV_ICM20948_BIT_CLKSEL_MASK; /* Clear clock select bits */

	/* Write entire register directly */
	ret = regmap_write(st->map, INV_ICM20948_REG_PWR_MGMT_1, val);
	if (ret) {
		dev_err(dev, "Failed to set internal clock source: %d\n", ret);
		return ret;
	}

	/* Verify setting */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_1, &val);
	if (ret) {
		dev_err(dev, "Failed to verify PWR_MGMT_1: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_1 value after setting: 0x%02x\n", val);

	/* Add delay to ensure clock source stabilization */
	msleep(20);

	return 0;
}

/* Work queue handler - Process interrupts in non-interrupt context */
static void inv_icm20948_irq_work_fn(struct work_struct *work)
{
	struct inv_icm20948_state *st = container_of(work, struct inv_icm20948_state, irq_work);
	unsigned int status, status_1;
	int ret;
	struct device *dev = regmap_get_device(st->map);

	/* Ensure operation in Bank 0 */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Failed to switch to bank 0 in work queue: %d\n", ret);
		return;
	}
	msleep(10);  /* Wait sufficiently for bank switch completion */

	/* Read interrupt status register */
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS, &status);
	if (ret) {
		dev_err(dev, "Failed to read INT_STATUS in work queue: %d\n", ret);
		return;
	}

	/* Read INT_STATUS_1 register - Data ready interrupt is in this register */
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_1, &status_1);
	if (ret) {
		dev_err(dev, "Failed to read INT_STATUS_1 in work queue: %d\n", ret);
		return;
	}

	dev_info(dev, "Work queue handling interrupt: INT_STATUS=0x%02x, INT_STATUS_1=0x%02x\n", status, status_1);

	/* Check data ready interrupt flag */
	if (status_1 & INV_ICM20948_BIT_RAW_DATA_RDY_INT) {
		dev_info(dev, "Data ready interrupt detected in work queue\n");
		if (st->trig) {
			iio_trigger_poll(st->trig);
		}
	}
}

/* Handle the IRQ */
static irqreturn_t inv_icm20948_irq_handler(int irq, void *private)
{
	struct inv_icm20948_state *st = private;

	/* Schedule work to handle I2C operations in non-interrupt context */
	schedule_work(&st->irq_work);

	return IRQ_HANDLED;
}

/**
 * inv_icm20948_check_magn - Check for magnetometer presence
 * @st: Driver state
 *
 * Returns 0 if magnetometer is detected, a negative error code otherwise.
 */
int inv_icm20948_check_magn(struct inv_icm20948_state *st)
{
	u8 val1, val2;
	unsigned int regval1, regval2;
	int ret;
	struct device *dev = regmap_get_device(st->map);

	/* Check if magnetometer is present by reading its ID registers */
	/* First we need to configure I2C master */
	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				INV_ICM20948_BIT_I2C_MST_EN,
				INV_ICM20948_BIT_I2C_MST_EN);
	if (ret)
		return ret;
	dev_info(dev, "Preparing to write I2C_MST_CTRL, switching to bank 0x30\n");

	/* Configure I2C master clock frequency */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_MST_CTRL,
				    INV_ICM20948_I2C_MST_CLK_400KHZ);
	if (ret)
		return ret;

	/* Set slave address with read bit */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_ADDR,
				    INV_ICM20948_MAG_I2C_ADDR | INV_ICM20948_I2C_SLV0_RNW);
	if (ret)
		return ret;

	/* Set register to read from - WIA1 */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_REG,
				    INV_ICM20948_REG_MAG_WIA1);
	if (ret)
		return ret;

	/* Enable reading, read 2 bytes (WIA1 and WIA2) */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_CTRL,
				    INV_ICM20948_I2C_SLV0_EN | 0x02);
	if (ret)
		return ret;

	/* Wait for I2C transaction to complete */
	msleep(20);

	/* Read the values */
	ret = regmap_read(st->map, INV_ICM20948_REG_EXT_SLV_SENS_DATA_00, &regval1);
	if (ret)
		return ret;

	ret = regmap_read(st->map, INV_ICM20948_REG_EXT_SLV_SENS_DATA_01, &regval2);
	if (ret)
		return ret;

	val1 = (u8)regval1;
	val2 = (u8)regval2;

	if (val1 != INV_ICM20948_MAG_WIA1_VAL || val2 != INV_ICM20948_MAG_WIA2_VAL) {
		dev_info(dev, "Magnetometer not detected or unsupported (0x%02x, 0x%02x)\n",
			val1, val2);
		return -ENODEV;
	}

	return 0;
}

/**
 * inv_icm20948_clear_interrupts - Clear all interrupt flags
 * @st: Driver state
 *
 * Clears all interrupt flags by reading the INT_STATUS register.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_clear_interrupts(struct inv_icm20948_state *st)
{
	unsigned int status, status_1, status_2, status_3;
	int ret;
	struct device *dev = regmap_get_device(st->map);

	/* Ensure operation in Bank 0 */
	ret = regmap_write(st->map, INV_ICM20948_REG bank_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Unable to set register bank 0: %d\n", ret);
		return ret;
	}
	msleep(10);

	/* Read all interrupt status registers to clear interrupt flags */
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS, &status);
	if (ret) {
		dev_err(dev, "Failed to read INT_STATUS: %d\n", ret);
		return ret;
	}

	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_1, &status_1);
	if (ret) {
		dev_err(dev, "Failed to read INT_STATUS_1: %d\n", ret);
		return ret;
	}

	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_2, &status_2);
	if (ret) {
		dev_err(dev, "Failed to read INT_STATUS_2: %d\n", ret);
		return ret;
	}

	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_3, &status_3);
	if (ret) {
		dev_err(dev, "Failed to read INT_STATUS_3: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Cleared interrupt status: INT_STATUS=0x%02x, INT_STATUS_1=0x%02x, INT_STATUS_2=0x%02x, INT_STATUS_3=0x%02x\n",
		 status, status_1, status_2, status_3);
	return 0;
}

/**
 * inv_icm20948_init_device - Initialize the ICM20948 device
 * @st: Driver state
 *
 * Returns 0 on success, a negative error code otherwise.
 */
static int inv_icm20948_init_device(struct inv_icm20948_state *st)
{
	unsigned int val;
	int ret;
	struct device *dev = regmap_get_device(st->map);

	dev_info(dev, "Initializing ICM20948 device\n");

	/* Explicitly set register bank 0 to ensure correct initialization */
	dev_info(dev, "Setting initial register bank 0\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Failed to set register bank 0 directly: %d\n", ret);
		return ret;
	}

	/* Extra delay to ensure bank selection takes effect */
	usleep_range(1000, 2000);

	/* Manually read WHO_AM_I register to verify communication */
	ret = regmap_read(st->map, INV_ICM20948_REG_WHO_AM_I, &val);
	if (ret) {
		dev_err(dev, "Failed to read WHO_AM_I register before reset: %d\n", ret);
		return ret;
	}
	dev_info(dev, "WHO_AM_I register before reset: 0x%02x\n", val);

	/* Reset device */
	dev_info(dev, "Resetting device\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_PWR_MGMT_1,
			  INV_ICM20948_BIT_H_RESET);
	if (ret) {
		dev_err(dev, "Failed to reset device: %d\n", ret);
		return ret;
	}

	/* Wait for reset to complete */
	dev_info(dev, "Waiting for reset to complete (100ms)\n");
	msleep(100);

	/* Select register bank 0 again after reset, as reset restores default state */
	dev_info(dev, "Setting register bank 0 after reset\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Failed to set register bank 0 directly after reset: %d\n", ret);
		return ret;
	}

	/* Extra delay to ensure bank selection takes effect */
	usleep_range(1000, 2000);

	/* Verify device ID */
	dev_info(dev, "Verifying device ID\n");
	ret = regmap_read(st->map, INV_ICM20948_REG_WHO_AM_I, &val);
	if (ret) {
		dev_err(dev, "Failed to read WHO_AM_I register: %d\n", ret);
		return ret;
	}
	dev_info(dev, "WHO_AM_I register value: 0x%02x (expected: 0x%02x)\n",
		 val, INV_ICM20948_WHO_AM_I_VAL);

	if (val != INV_ICM20948_WHO_AM_I_VAL) {
		dev_err(dev, "Invalid WHO_AM_I value 0x%02x\n", val);
		return -ENODEV;
	}

	/* Set clock source */
	dev_info(dev, "Setting clock source\n");
	ret = inv_icm20948_set_clock_source(st);
	if (ret) {
		dev_err(dev, "Failed to set clock source\n");
		return ret;
	}

	/* Configure power management */
	ret = inv_icm20948_set_power_mode(st, INV_ICM20948_SENSOR_MODE_STANDBY);
	if (ret) {
		dev_err(dev, "Failed to set power mode: %d\n", ret);
		return ret;
	}

	/* Ensure register bank 0 is selected */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Unable to set register bank 0: %d\n", ret);
		return ret;
	}
	usleep_range(1000, 2000);

	/* Read current PWR_MGMT_2 value */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_2, &val);
	if (ret) {
		dev_err(dev, "Failed to read PWR_MGMT_2: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_2 current value: 0x%02x\n", val);

	/* Enable all sensor axes (disable accelerometer and gyroscope sleep mode) */
	dev_info(dev, "Enabling all sensor axes\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_PWR_MGMT_2, 0x00);
	if (ret) {
		dev_err(dev, "Failed to enable sensors: %d\n", ret);
		return ret;
	}

	/* Verify setting */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_2, &val);
	if (ret) {
		dev_err(dev, "Failed to verify PWR_MGMT_2: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_2 value after setting: 0x%02x\n", val);

	/* Set default sampling rates */
	/* Use direct register access to set sampling rates */
	dev_info(dev, "Setting sampling rates using enhanced register access\n");
	ret = inv_icm20948_set_sensor_rate(st, 0, 100);  /* Gyroscope set to 100Hz */
	if (ret)
		return ret;

	ret = inv_icm20948_set_sensor_rate(st, 1, 100);  /* Accelerometer set to 100Hz */
	if (ret)
		return ret;

	/* Set default full-scale ranges */
	ret = inv_icm20948_set_gyro_fs(st, INV_ICM20948_GYRO_FS_2000DPS);
	if (ret)
		return ret;

	ret = inv_icm20948_set_accel_fs(st, INV_ICM20948_ACCEL_FS_16G);
	if (ret)
		return ret;

	/* Enable temperature sensor */
	ret = regmap_update_bits(st->map, INV_ICM20948_REG_PWR_MGMT_1,
			       INV_ICM20948_BIT_TEMP_DIS, 0);
	if (ret)
		return ret;
	st->conf.temp_en = true;

	/* Configure interrupt pin - Use pulse mode (remove latching mode) */
	ret = regmap_write(st->map, INV_ICM20948_REG_INT_PIN_CFG,
			  INV_ICM20948_BIT_INT_OPEN_DRAIN);
	if (ret)
		return ret;

	/* Set INT_ENABLE (0x10) - Enable main interrupt */
	ret = regmap_write(st->map, INV_ICM20948_REG_INT_ENABLE, 0x01);
	if (ret)
		return ret;

	/* Set INT_ENABLE_1 (0x11) - Enable data ready interrupt */
	ret = regmap_write(st->map, INV_ICM20948_REG_INT_ENABLE_1,
			  INV_ICM20948_BIT_RAW_DATA_RDY_EN);
	if (ret)
		return ret;

	/* Read and print interrupt-related register values for debugging */
	unsigned int int_pin_cfg_val, int_enable_val, int_enable_1_val;
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_PIN_CFG, &int_pin_cfg_val);
	if (ret)
		dev_err(dev, "Failed to read INT_PIN_CFG: %d\n", ret);
	else
		dev_info(dev, "INT_PIN_CFG value after setting: 0x%02x\n", int_pin_cfg_val);

	ret = regmap_read(st->map, INV_ICM20948_REG_INT_ENABLE, &int_enable_val);
	if (ret)
		dev_err(dev, "Failed to read INT_ENABLE: %d\n", ret);
	else
		dev_info(dev, "INT_ENABLE value after setting: 0x%02x\n", int_enable_val);

	ret = regmap_read(st->map, INV_ICM20948_REG_INT_ENABLE_1, &int_enable_1_val);
	if (ret)
		dev_err(dev, "Failed to read INT_ENABLE_1: %d\n", ret);
	else
		dev_info(dev, "INT_ENABLE_1 value after setting: 0x%02x\n", int_enable_1_val);

	/* Clear any existing interrupt flags */
	ret = inv_icm20948_clear_interrupts(st);
	if (ret)
		return ret;

	/* Check for magnetometer */
	ret = inv_icm20948_check_magn(st);
	if (ret == 0) {
		/* Setup magnetometer */
		ret = inv_icm20948_setup_mag(st);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * inv_icm20948_core_probe - Probe function for ICM20948
 * @regmap: Register map for the device
 * @irq: IRQ number
 * @bus_setup: Bus-specific setup function
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_core_probe(struct regmap *regmap, int irq,
			   int (*bus_setup)(struct inv_icm20948_state *))
{
	struct inv_icm20948_state *st;
	struct device *dev = regmap_get_device(regmap);
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->map = regmap;
	st->name = "icm20948";

	mutex_init(&st->lock);

	/* Initialize work queue */
	INIT_WORK(&st->irq_work, inv_icm20948_irq_work_fn);

	dev_set_drvdata(dev, st);

	/* Setup bus specific configuration */
	if (bus_setup) {
		ret = bus_setup(st);
		if (ret)
			return ret;
	}

	/* Initialize device */
	ret = inv_icm20948_init_device(st);
	if (ret)
		return ret;

	/* Initialize debug support */
	ret = inv_icm20948_debug_init(st);
	if (ret) {
		dev_warn(dev, "Failed to initialize debug support\n");
		/* Non-fatal error, continue without debug */
	}

	/* Initialize sensors */
	st->indio_gyro = inv_icm20948_gyro_init(st);
	if (IS_ERR(st->indio_gyro))
		return PTR_ERR(st->indio_gyro);

	st->indio_accel = inv_icm20948_accel_init(st);
	if (IS_ERR(st->indio_accel))
		return PTR_ERR(st->indio_accel);

	if (st->mag_initialized) {
		st->indio_magn = inv_icm20948_magn_init(st);
		if (IS_ERR(st->indio_magn))
			st->indio_magn = NULL;
	}

	/* Initialize temperature sensor */
	if (st->conf.temp_en) {
		st->indio_temp = inv_icm20948_temp_init(st);
		if (IS_ERR(st->indio_temp))
			st->indio_temp = NULL;
	}

	/* Initialize IIO trigger */
	st->trig = devm_iio_trigger_alloc(dev, "%s-trigger", st->name);
	if (!st->trig)
		return -ENOMEM;

	st->trig->dev.parent = dev;
	st->trig->ops = &inv_icm20948_trigger_ops;
	iio_trigger_set_drvdata(st->trig, st);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret) {
		dev_err(dev, "Failed to register IIO trigger\n");
		return ret;
	}

	/* Associate trigger with each sensor device */
	st->indio_gyro->trig = st->trig;
	st->indio_accel->trig = st->trig;
	if (st->indio_magn)
		st->indio_magn->trig = st->trig;

	/* Setup trigger - Call before registering IIO devices */
	if (irq > 0) {
		dev_info(dev, "Setting up sensor trigger\n");
		ret = inv_icm20948_setup_trigger(st);
		if (ret) {
			dev_err(dev, "Failed to setup trigger: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Trigger setup successful\n");
	}

	/* Register IIO devices - Complete after setting up trigger */
	ret = devm_iio_device_register(dev, st->indio_gyro);
	if (ret) {
		dev_err(dev, "Failed to register gyro IIO device\n");
		return ret;
	}

	ret = devm_iio_device_register(dev, st->indio_accel);
	if (ret) {
		dev_err(dev, "Failed to register accel IIO device\n");
		return ret;
	}

	if (st->indio_magn) {
		ret = devm_iio_device_register(dev, st->indio_magn);
		if (ret) {
			dev_err(dev, "Failed to register magn IIO device\n");
			return ret;
		}
	}

	if (st->indio_temp) {
		ret = devm_iio_device_register(dev, st->indio_temp);
		if (ret) {
			dev_err(dev, "Failed to register temp IIO device\n");
			return ret;
		}
	}

	/* FIFO setup if needed - Complete before registering interrupt handler */
	if (irq > 0) {
		ret = inv_icm20948_configure_fifo(st, true);
		if (ret) {
			dev_err(dev, "Failed to configure FIFO\n");
			/* Non-fatal error, continue without FIFO */
			st->use_fifo = false;
		}
	}

	/* Finally, register interrupt handler after all initialization */
	if (irq > 0) {
		dev_info(dev, "Registering interrupt handler, IRQ: %d\n", irq);
		ret = devm_request_irq(dev, irq, inv_icm20948_irq_handler,
				      IRQF_TRIGGER_RISING, "inv_icm20948", st);
		if (ret) {
			dev_err(dev, "Failed to request IRQ %d\n", irq);
			return ret;
		}
		dev_info(dev, "Interrupt handler registered successfully\n");
	}

	dev_info(dev, "ICM-20948 initialized\n");

	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm20948_core_probe);

static int __maybe_unused inv_icm20948_resume(struct device *dev)
{
	struct inv_icm20948_state *st = dev_get_drvdata(dev);

	return inv_icm20948_init_device(st);
}

static int __maybe_unused inv_icm20948_suspend(struct device *dev)
{
	struct inv_icm20948_state *st = dev_get_drvdata(dev);

	return inv_icm20948_set_power_mode(st, INV_ICM20948_SENSOR_MODE_OFF);
}

const struct dev_pm_ops inv_icm20948_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(inv_icm20948_suspend, inv_icm20948_resume)
};
EXPORT_SYMBOL_GPL(inv_icm20948_pm_ops);

MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("Invensense ICM-20948 driver");
MODULE_LICENSE("GPL v2");
