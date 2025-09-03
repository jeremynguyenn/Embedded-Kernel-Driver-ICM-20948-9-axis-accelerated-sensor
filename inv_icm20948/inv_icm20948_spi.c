// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/acpi.h>
#include <linux/property.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>

#include "inv_icm20948.h"

/* SPI specific registers and values */
#define INV_ICM20948_SPI_READ_BIT 0x80

static int inv_icm20948_spi_bus_setup(struct inv_icm20948_state *st)
{
	/* Set I2C_IF_DIS bit to enable SPI interface */
	return regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				 INV_ICM20948_BIT_I2C_IF_DIS,
				 INV_ICM20948_BIT_I2C_IF_DIS);
}

/* SPI regmap configuration with the read bit included */
const struct regmap_config inv_icm20948_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
	.rd_table = &inv_icm20948_readable_regs,
	.wr_table = &inv_icm20948_writable_regs,
	.cache_type = REGCACHE_NONE,
	.read_flag_mask = INV_ICM20948_SPI_READ_BIT,
};
EXPORT_SYMBOL_GPL(inv_icm20948_spi_regmap_config);

static int inv_icm20948_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	int ret;

	/* Set SPI mode and bits per word */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi->max_speed_hz = 7000000;  /* Datasheet max SPI speed */
	ret = spi_setup(spi);
	if (ret) {
		dev_err(&spi->dev, "SPI setup failed: %d\n", ret);
		return ret;
	}

	regmap = devm_regmap_init_spi(spi, &inv_icm20948_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to initialize SPI regmap: %ld\n", PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	ret = inv_icm20948_core_probe(regmap, spi->irq, inv_icm20948_spi_bus_setup);
	if (ret) {
		dev_err(&spi->dev, "Core probe failed: %d\n", ret);
		return ret;
	}

	pm_runtime_set_active(&spi->dev);
	pm_runtime_enable(&spi->dev);

	return 0;
}

static void inv_icm20948_spi_remove(struct spi_device *spi)
{
	struct inv_icm20948_state *st = dev_get_drvdata(&spi->dev);

	if (st) {
		inv_icm20948_cleanup_buffer(st);
		inv_icm20948_debug_cleanup(st);
	}
	pm_runtime_disable(&spi->dev);
}

static const struct of_device_id inv_icm20948_spi_of_match[] = {
	{ .compatible = "invensense,icm20948" },
	{ }
};
MODULE_DEVICE_TABLE(of, inv_icm20948_spi_of_match);

static const struct spi_device_id inv_icm20948_spi_id[] = {
	{ "icm20948", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, inv_icm20948_spi_id);

static struct spi_driver inv_icm20948_spi_driver = {
	.driver = {
		.name = "inv_icm20948_spi",
		.pm = &inv_icm20948_pm_ops,
		.of_match_table = inv_icm20948_spi_of_match,
	},
	.probe = inv_icm20948_spi_probe,
	.remove = inv_icm20948_spi_remove,
	.id_table = inv_icm20948_spi_id,
};

static int __init inv_icm20948_spi_init(void)
{
	int ret = spi_register_driver(&inv_icm20948_spi_driver);
	if (ret)
		return ret;

	return inv_icm20948_add_sysfs_attrs(&inv_icm20948_spi_driver.driver);
}

static void __exit inv_icm20948_spi_exit(void)
{
	inv_icm20948_remove_sysfs_attrs(&inv_icm20948_spi_driver.driver);
	spi_unregister_driver(&inv_icm20948_spi_driver);
}

module_init(inv_icm20948_spi_init);
module_exit(inv_icm20948_spi_exit);

MODULE_AUTHOR("Invensense, Inc.");
MODULE_DESCRIPTION("Invensense ICM-20948 SPI driver");
MODULE_LICENSE("GPL v2");