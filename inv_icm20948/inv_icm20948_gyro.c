// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "inv_icm20948.h"

#define INV_ICM20948_GYRO_CHAN(_type, _axis, _index) {			\
	.type = _type,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			     BIT(IIO_CHAN_INFO_SCALE) |			\
			     BIT(IIO_CHAN_INFO_CALIBBIAS),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,						\
		.endianness = IIO_BE,					\
	},									\
}

static const struct iio_chan_spec inv_icm20948_gyro_channels[] = {
	INV_ICM20948_GYRO_CHAN(IIO_ANGL_VEL, X, 0),
	INV_ICM20948_GYRO_CHAN(IIO_ANGL_VEL, Y, 1),
	INV_ICM20948_GYRO_CHAN(IIO_ANGL_VEL, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const int inv_icm20948_gyro_fs_map[] = {
	[INV_ICM20948_GYRO_FS_250DPS]  = 131,
	[INV_ICM20948_GYRO_FS_500DPS]  = 65,
	[INV_ICM20948_GYRO_FS_1000DPS] = 33,
	[INV_ICM20948_GYRO_FS_2000DPS] = 16,
};

static int inv_icm20948_gyro_read_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     int *val, int *val2, long mask)
{
	struct inv_icm20948_state *st = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	int ret;
	__be16 data;

	if (!st || !st->map)
		return -EINVAL;

	pm_runtime_get_sync(dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
		if (ret) {
			mutex_unlock(&st->lock);
			pm_runtime_put(dev);
			return ret;
		}
		ret = regmap_bulk_read(st->map,
				      INV_ICM20948_REG_GYRO_XOUT_H + chan->scan_index * 2,
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
		*val2 = 1000000000 / inv_icm20948_gyro_fs_map[st->conf.gyro.fs];
		pm_runtime_put(dev);
		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->conf.gyro.odr;
		pm_runtime_put(dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_CALIBBIAS:
		mutex_lock(&st->lock);
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_1);
		if (ret) {
			mutex_unlock(&st->lock);
			pm_runtime_put(dev);
			return ret;
		}
		ret = regmap_read(st->map, INV_ICM20948_REG_SELF_TEST_X_GYRO + chan->scan_index, val);
		mutex_unlock(&st->lock);
		pm_runtime_put(dev);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	default:
		pm_runtime_put(dev);
		return -EINVAL;
	}
}

static int inv_icm20948_gyro_write_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int val, int val2, long mask)
{
	struct inv_icm20948_state *st = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	int ret, i;

	if (!st || !st->map)
		return -EINVAL;

	pm_runtime_get_sync(dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		for (i = 0; i < ARRAY_SIZE(inv_icm20948_gyro_fs_map); i++) {
			if (val == 0 && val2 == (1000000000 / inv_icm20948_gyro_fs_map[i])) {
				mutex_lock(&st->lock);
				ret = inv_icm20948_set_gyro_fs(st, i);
				if (!ret)
					st->conf.gyro.fs = i;
				mutex_unlock(&st->lock);
				pm_runtime_put(dev);
				return ret;
			}
		}
		pm_runtime_put(dev);
		return -EINVAL;

	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		ret = inv_icm20948_set_sensor_rate(st, 0, val);
		mutex_unlock(&st->lock);
		pm_runtime_put(dev);
		return ret;

	default:
		pm_runtime_put(dev);
		return -EINVAL;
	}
}

static int inv_icm20948_gyro_validate_trigger(struct iio_dev *indio_dev,
					     struct iio_trigger *trig)
{
	struct inv_icm20948_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_info inv_icm20948_gyro_info = {
	.read_raw = inv_icm20948_gyro_read_raw,
	.write_raw = inv_icm20948_gyro_write_raw,
	.validate_trigger = inv_icm20948_gyro_validate_trigger,
};

irqreturn_t inv_icm20948_gyro_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_icm20948_state *st = iio_priv(indio_dev);
	__be16 buffer[8]; /* 3 x 16-bit + timestamp */
	struct device *dev = regmap_get_device(st->map);
	int ret;
	u8 status;

	pm_runtime_get_sync(dev);

	mutex_lock(&st->lock);
	ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
	if (ret) {
		mutex_unlock(&st->lock);
		pm_runtime_put(dev);
		goto done;
	}
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_1, &status);
	if (ret || !(status & INV_ICM20948_BIT_RAW_DATA_RDY_INT)) {
		mutex_unlock(&st->lock);
		pm_runtime_put(dev);
		goto done;
	}
	ret = regmap_bulk_read(st->map, INV_ICM20948_REG_GYRO_XOUT_H,
			       buffer, 3 * sizeof(__be16));
	st->timestamp.gyro = iio_get_time_ns(indio_dev);
	mutex_unlock(&st->lock);
	pm_runtime_put(dev);
	if (ret)
		goto done;

	iio_push_to_buffers_with_timestamp(indio_dev, buffer, st->timestamp.gyro);

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

struct iio_dev *inv_icm20948_gyro_init(struct inv_icm20948_state *st)
{
	struct iio_dev *indio_dev;
	struct inv_icm20948_sensor_state *sensor;

	indio_dev = devm_iio_device_alloc(regmap_get_device(st->map), sizeof(*sensor));
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);

	sensor = iio_priv(indio_dev);
	sensor->scales = inv_icm20948_gyro_fs_map;
	sensor->scales_len = ARRAY_SIZE(inv_icm20948_gyro_fs_map);

	indio_dev->name = "icm20948_gyro";
	indio_dev->info = &inv_icm20948_gyro_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = inv_icm20948_gyro_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm20948_gyro_channels);
	iio_device_set_drvdata(indio_dev, st);
	if (st->trig)
		indio_dev->trig = st->trig;

	return indio_dev;
}
EXPORT_SYMBOL_GPL(inv_icm20948_gyro_init);

MODULE_ALIAS("iio:inv_icm20948_gyro");