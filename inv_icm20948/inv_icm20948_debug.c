// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "inv_icm20948.h"

#ifdef CONFIG_INV_ICM20948_DEBUG

struct inv_icm20948_debug {
	struct dentry *root;
	struct inv_icm20948_state *st;
};

/* Show all register values for debugging */
static int inv_icm20948_reg_dump(void *data, u64 *val)
{
	struct inv_icm20948_state *st = data;
	u8 reg_val;
	int ret, i, bank;

	for (bank = 0; bank < 4; bank++) {
		ret = inv_icm20948_set_bank(st, bank);
		if (ret)
			return ret;
		for (i = 0; i <= 0x7F; i++) {
			ret = regmap_read(st->map, i, &reg_val);
			if (!ret)
				dev_info(regmap_get_device(st->map), "Bank%d Reg[0x%02x] = 0x%02x\n", bank, i, reg_val);
		}
	}

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(inv_icm20948_reg_dump_fops, NULL, inv_icm20948_reg_dump, "%llu\n");

/* Write to a register for debugging */
static int inv_icm20948_reg_write(void *data, u64 val)
{
	struct inv_icm20948_state *st = data;
	u8 bank = (val >> 16) & 0xFF;
	u8 reg = (val >> 8) & 0xFF;
	u8 write_val = val & 0xFF;
	int ret;

	ret = inv_icm20948_write_reg(st, bank, reg, write_val);
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(inv_icm20948_reg_write_fops, NULL, inv_icm20948_reg_write, "%llu\n");

/* Sysfs attribute to show chip ID */
static ssize_t chip_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct inv_icm20948_state *st = dev_get_drvdata(dev);
	u8 val;
	int ret;

	mutex_lock(&st->lock);
	ret = inv_icm20948_read_reg(st, INV_ICM20948_BANK_0, INV_ICM20948_REG_WHO_AM_I, &val);
	mutex_unlock(&st->lock);

	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "ICM20948 WHO_AM_I: 0x%02x\n", val);
}
static DEVICE_ATTR_RO(chip_id);

/* Initialize debugfs and sysfs entries */
int inv_icm20948_debug_init(struct inv_icm20948_state *st)
{
	struct device *dev = regmap_get_device(st->map);
	struct inv_icm20948_debug *debug;
	int ret;

	debug = devm_kzalloc(dev, sizeof(*debug), GFP_KERNEL);
	if (!debug)
		return -ENOMEM;

	debug->st = st;
	debug->root = debugfs_create_dir("inv_icm20948", NULL);
	if (IS_ERR(debug->root))
		return PTR_ERR(debug->root);

	debugfs_create_file("reg_dump", 0444, debug->root, st, &inv_icm20948_reg_dump_fops);
	debugfs_create_file("reg_write", 0200, debug->root, st, &inv_icm20948_reg_write_fops);

	ret = device_create_file(dev, &dev_attr_chip_id);
	if (ret) {
		debugfs_remove_recursive(debug->root);
		return ret;
	}

	dev_info(dev, "Debug support initialized with debugfs and sysfs entries\n");
	return 0;
}

/* Cleanup debugfs and sysfs entries */
void inv_icm20948_debug_cleanup(struct inv_icm20948_state *st)
{
	struct device *dev = regmap_get_device(st->map);
	struct inv_icm20948_debug *debug = dev_get_drvdata(dev);

	if (debug && !IS_ERR_OR_NULL(debug->root))
		debugfs_remove_recursive(debug->root);

	device_remove_file(dev, &dev_attr_chip_id);
}

#endif /* CONFIG_INV_ICM20948_DEBUG */