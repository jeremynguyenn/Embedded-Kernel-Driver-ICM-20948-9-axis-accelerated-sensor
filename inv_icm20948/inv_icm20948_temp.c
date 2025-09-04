// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2025 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "inv_icm20948.h"

/* ICM-20948 temperature has a sensitivity of 333.87 LSB/°C
 * and an offset of 0°C = 21°C.
 */
#define INV_ICM20948_TEMP_SCALE 333870
#define INV_ICM20948_TEMP_OFFSET 21000

static const struct iio_chan_spec inv_icm20948_temp_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				     BIT(IIO_CHAN_INFO_SCALE) |
				     BIT(IIO_CHAN_INFO_OFFSET) |
				     BIT(IIO_CHAN_INFO_PROCESSED) |
				     BIT(IIO_CHAN_INFO_CALIBSCALE),
		.scan_index = -1,
	},
};

static int inv_icm20948_temp_read_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     int *val, int *val2, long mask)
{
	struct inv_icm20948_state *st = iio_device_get_drvdata(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	int ret;
	__be16 data;
	int raw_temp;

	if (!st || !st->map)
		return -EINVAL;

	pm_runtime_get_sync(dev);

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&st->lock);
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
		if (ret) {
			mutex_unlock(&st->lock);
			pm_runtime_put(dev);
			return ret;
		}
		ret = regmap_bulk_read(st->map, INV_ICM20948_REG_TEMP_OUT_H,
				      &data, sizeof(data));
		mutex_unlock(&st->lock);
		if (ret) {
			pm_runtime_put(dev);
			return ret;
		}
		raw_temp = be16_to_cpu(data);
		if (raw_temp < -32768 || raw_temp > 32767) {
			pm_runtime_put(dev);
			return -ERANGE;
		}
		*val = (raw_temp * 1000LL) / (INV_ICM20948_TEMP_SCALE / 1000) + INV_ICM20948_TEMP_OFFSET; // Improved calculation for milli-degrees
		pm_runtime_put(dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
		if (ret) {
			mutex_unlock(&st->lock);
			pm_runtime_put(dev);
			return ret;
		}
		ret = regmap_bulk_read(st->map, INV_ICM20948_REG_TEMP_OUT_H,
				      &data, sizeof(data));
		mutex_unlock(&st->lock);
		if (ret) {
			pm_runtime_put(dev);
			return ret;
		}
		*val = be16_to_cpu(data);
		pm_runtime_put(dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = INV_ICM20948_TEMP_SCALE;
		pm_runtime_put(dev);
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		*val = INV_ICM20948_TEMP_OFFSET;
		pm_runtime_put(dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_CALIBSCALE:
		*val = 1;  /* Placeholder for calibration scale */
		pm_runtime_put(dev);
		return IIO_VAL_INT;

	default:
		pm_runtime_put(dev);
		return -EINVAL;
	}
}

static const struct iio_info inv_icm20948_temp_info = {
	.read_raw = inv_icm20948_temp_read_raw,
};

struct iio_dev *inv_icm20948_temp_init(struct inv_icm20948_state *st)
{
	struct iio_dev *indio_dev;
	struct device *dev = regmap_get_device(st->map);

	if (!st || !st->map)
		return ERR_PTR(-EINVAL);

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);

	indio_dev->name = "icm20948_temp";
	indio_dev->info = &inv_icm20948_temp_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = inv_icm20948_temp_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm20948_temp_channels);
	iio_device_set_drvdata(indio_dev, st);

	return indio_dev;
}
EXPORT_SYMBOL_GPL(inv_icm20948_temp_init);

MODULE_ALIAS("iio:inv_icm20948_temp");
