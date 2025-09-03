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

/* AK09916 Registers */
#define INV_ICM20948_REG_MAG_WIA1		0x00
#define INV_ICM20948_REG_MAG_WIA2		0x01
#define INV_ICM20948_MAG_WIA1_VAL		0x48
#define INV_ICM20948_MAG_WIA2_VAL		0x09

#define INV_ICM20948_REG_MAG_ST1		0x10
#define INV_ICM20948_REG_MAG_HXL		0x11
#define INV_ICM20948_REG_MAG_HXH		0x12
#define INV_ICM20948_REG_MAG_HYL		0x13
#define INV_ICM20948_REG_MAG_HYH		0x14
#define INV_ICM20948_REG_MAG_HZL		0x15
#define INV_ICM20948_REG_MAG_HZH		0x16
#define INV_ICM20948_REG_MAG_ST2		0x18

#define INV_ICM20948_REG_MAG_CNTL2		0x31
#define INV_ICM20948_MAG_MODE_POWER_DOWN	0x00
#define INV_ICM20948_MAG_MODE_SINGLE		0x01
#define INV_ICM20948_MAG_MODE_CONT1		0x02
#define INV_ICM20948_MAG_MODE_CONT2		0x04
#define INV_ICM20948_MAG_MODE_CONT3		0x06
#define INV_ICM20948_MAG_MODE_CONT4		0x08
#define INV_ICM20948_MAG_MODE_SELF_TEST		0x10

#define INV_ICM20948_MAGN_CHAN(_type, _axis, _index) {			\
	.type = _type,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			     BIT(IIO_CHAN_INFO_SCALE) |			\
			     BIT(IIO_CHAN_INFO_CALIBSCALE),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,						\
		.endianness = IIO_LE,					\
	},									\
}

static const struct iio_chan_spec inv_icm20948_magn_channels[] = {
	INV_ICM20948_MAGN_CHAN(IIO_MAGN, X, 0),
	INV_ICM20948_MAGN_CHAN(IIO_MAGN, Y, 1),
	INV_ICM20948_MAGN_CHAN(IIO_MAGN, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

/* AK09916 magnetometer has a fixed scale of 0.15 μT/LSB */
static const int inv_icm20948_magn_scale = 150; /* 0.15 μT/LSB * 1000 */

static int inv_icm20948_magn_read_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     int *val, int *val2, long mask)
{
	struct inv_icm20948_state *st = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	int ret;
	__le16 data;
	u8 st2;

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
				      INV_ICM20948_REG_EXT_SLV_SENS_DATA_01 + chan->scan_index * 2,
				      &data, sizeof(data));
		if (ret) {
			mutex_unlock(&st->lock);
			pm_runtime_put(dev);
			return ret;
		}
		ret = regmap_read(st->map, INV_ICM20948_REG_MAG_ST2, &st2);
		mutex_unlock(&st->lock);
		if (ret) {
			pm_runtime_put(dev);
			return ret;
		}
		if (st2 & BIT(3)) {
			pm_runtime_put(dev);
			return -EOVERFLOW;  /* Magnetic overflow */
		}
		*val = le16_to_cpu(data);
		pm_runtime_put(dev);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = inv_icm20948_magn_scale;
		pm_runtime_put(dev);
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 100; // Fixed for AK09916 in cont mode 4
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

static int inv_icm20948_magn_write_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int val, int val2, long mask)
{
	if (mask == IIO_CHAN_INFO_SAMP_FREQ && val == 100)
		return 0; // Fixed rate

	return -EINVAL;
}

static int inv_icm20948_magn_validate_trigger(struct iio_dev *indio_dev,
                                             struct iio_trigger *trig)
{
	struct inv_icm20948_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_info inv_icm20948_magn_info = {
	.read_raw = inv_icm20948_magn_read_raw,
	.write_raw = inv_icm20948_magn_write_raw,
	.validate_trigger = inv_icm20948_magn_validate_trigger,
};

irqreturn_t inv_icm20948_magn_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_icm20948_state *st = iio_priv(indio_dev);
	__le16 buffer[8]; /* 3 x 16-bit + timestamp */
	struct device *dev = regmap_get_device(st->map);
	int ret;

	pm_runtime_get_sync(dev);

	mutex_lock(&st->lock);
	ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
	if (ret) {
		mutex_unlock(&st->lock);
		pm_runtime_put(dev);
		goto done;
	}
	ret = regmap_bulk_read(st->map, INV_ICM20948_REG_EXT_SLV_SENS_DATA_01,
			       buffer, 3 * sizeof(__le16));
	st->timestamp.magn = iio_get_time_ns(indio_dev);
	mutex_unlock(&st->lock);
	pm_runtime_put(dev);
	if (ret)
		goto done;

	iio_push_to_buffers_with_timestamp(indio_dev, buffer, st->timestamp.magn);

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

struct iio_dev *inv_icm20948_magn_init(struct inv_icm20948_state *st)
{
	struct iio_dev *indio_dev;
	struct inv_icm20948_sensor_state *sensor;

	indio_dev = devm_iio_device_alloc(regmap_get_device(st->map), sizeof(*sensor));
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);

	sensor = iio_priv(indio_dev);

	indio_dev->name = "icm20948_magn";
	indio_dev->info = &inv_icm20948_magn_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = inv_icm20948_magn_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm20948_magn_channels);
	iio_device_set_drvdata(indio_dev, st);
	if (st->trig)
		indio_dev->trig = st->trig;

	return indio_dev;
}
EXPORT_SYMBOL_GPL(inv_icm20948_magn_init);

MODULE_ALIAS("iio:inv_icm20948_magn");