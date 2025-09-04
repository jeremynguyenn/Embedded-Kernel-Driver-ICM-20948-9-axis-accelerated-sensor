// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2025 Invensense, Inc.
 * Author:  Nguyen nhan
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

#define INV_ICM20948_ACCEL_CHAN(_type, _axis, _index) {			\
	.type = _type,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			     BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,						\
		.endianness = IIO_BE,					\
	},									\
}

static const struct iio_chan_spec inv_icm20948_accel_channels[] = {
	INV_ICM20948_ACCEL_CHAN(IIO_ACCEL, X, 0),
	INV_ICM20948_ACCEL_CHAN(IIO_ACCEL, Y, 1),
	INV_ICM20948_ACCEL_CHAN(IIO_ACCEL, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const int inv_icm20948_accel_fs_map[] = {
	[INV_ICM20948_ACCEL_FS_2G]  = 16384,
	[INV_ICM20948_ACCEL_FS_4G]  = 8192,
	[INV_ICM20948_ACCEL_FS_8G]  = 4096,
	[INV_ICM20948_ACCEL_FS_16G] = 2048,
};

static int inv_icm20948_accel_read_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int *val, int *val2, long mask)
{
	struct inv_icm20948_state *st;
	struct device *dev;
	int ret;
	__be16 data;
	unsigned int bank_val;

	/* Check input parameters */
	if (!indio_dev)
		return -EINVAL;

	st = iio_device_get_drvdata(indio_dev);

	/* Safety check: ensure that the state structure and regmap are valid */
	if (!st) {
		pr_err("ICM20948: Accelerometer sensor status structure is empty\n");
		return -EINVAL;
	}

	if (!st->map) {
		pr_err("ICM20948: Accelerometer sensor regmap is empty\n");
		return -EINVAL;
	}

	/* Use pr_debug instead of dev_info to avoid relying on device pointers */
	pr_debug("ICM20948: Accelerometer sensor reading started\n");

	/* Make sure the map is valid before getting the device pointer */
	dev = regmap_get_device(st->map);
	if (!dev) {
		pr_err("ICM20948: Unable to obtain the accelerometer sensor device pointer\n");
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		dev_info(dev, "Accelerometer reading begins, prepare to acquire the mutex lock\n");
		mutex_lock(&st->lock);
		dev_info(dev, "Mutex lock acquired, ready to switch to Bank 0\n");

		/* Read the current Bank value */
		ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &bank_val);
		if (ret) {
			dev_err(dev, "Failed to read the current Bank: %d\n", ret);
			mutex_unlock(&st->lock);
			return ret;
		}
		dev_info(dev, "Current Bank value: 0x%02x\n", bank_val);

		/* Ensure we are in bank 0 */
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
		if (ret) {
			dev_err(dev, "Switch to Bank 0 failed: %d\n", ret);
			mutex_unlock(&st->lock);
			return ret;
		}

		/* Verify Bank Switch */
		ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &bank_val);
		if (ret) {
			dev_err(dev, "Failed to read Bank after switching: %d\n", ret);
			mutex_unlock(&st->lock);
			return ret;
		}
		dev_info(dev, "Bank value after switching: 0x%02x\n", bank_val);

		/* Add a delay to ensure that the bank switch is completed */
		msleep(10);
		dev_info(dev, "Bank switching delay is completed, ready to read accelerometer data\n");

		ret = regmap_bulk_read(st->map,
				      INV_ICM20948_REG_ACCEL_XOUT_H + chan->scan_index * 2,
				      &data, sizeof(data));
		dev_info(dev, "Accelerometer data reading is complete, ready to release the mutex lock\n");
		mutex_unlock(&st->lock);
		dev_info(dev, "Mutex released\n");

		if (ret) {
			dev_err(dev, "Accelerometer data reading failed: %d\n", ret);
			return ret;
		}

		*val = be16_to_cpu(data);
		dev_info(dev, "Accelerometer raw value: %d\n", *val);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = 1000000000 / inv_icm20948_accel_fs_map[st->conf.accel.fs];
		return IIO_VAL_INT_PLUS_NANO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		/* Returns the actual set sampling rate, not a hardcoded value */
		*val = 1125 / (st->conf.accel.odr + 1); /* Calculate the actual sampling rate based on the frequency division value */
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int inv_icm20948_accel_write_raw(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       int val, int val2, long mask)
{
	struct inv_icm20948_state *st = iio_device_get_drvdata(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	int ret, i;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		/* Support direct writing of G value */
		dev_info(dev, "Accelerometer range setting: val=%d, val2=%d\n", val, val2);
		
		/* Check whether the G value is written directly */
		if (val2 == 0) {
			/* Directly writing G value */
			switch (val) {
			case 2:
				i = INV_ICM20948_ACCEL_FS_2G;
				break;
			case 4:
				i = INV_ICM20948_ACCEL_FS_4G;
				break;
			case 8:
				i = INV_ICM20948_ACCEL_FS_8G;
				break;
			case 16:
				i = INV_ICM20948_ACCEL_FS_16G;
				break;
			default:
				dev_err(dev, "Unsupported accelerometer G-value: %d，Please use 2/4/8/16\n", val);
				return -EINVAL;
			}
			dev_info(dev, "Directly set the accelerometer range to %d G (index: %d)\n", val, i);
			mutex_lock(&st->lock);
			ret = inv_icm20948_set_accel_fs(st, i);
			if (!ret)
				st->conf.accel.fs = i;
			mutex_unlock(&st->lock);
			return ret;
		}
		
		/* Traditional method: set the range by scale value */
		for (i = 0; i < ARRAY_SIZE(inv_icm20948_accel_fs_map); i++) {
			int scale_val = 1000000000 / inv_icm20948_accel_fs_map[i];
			dev_info(dev, "Compare range[%d]: expected value=%d, Calculated value=%d\n", i, val2, scale_val);
			if (val == 0 && val2 == scale_val) {
				dev_info(dev, "Find a matching accelerometer scale: %d\n", i);
				mutex_lock(&st->lock);
				ret = inv_icm20948_set_accel_fs(st, i);
				if (!ret)
					st->conf.accel.fs = i;
				mutex_unlock(&st->lock);
				return ret;
			}
		}
		dev_err(dev, "No matching accelerometer range found\n");
		return -EINVAL;

	case IIO_CHAN_INFO_SAMP_FREQ:
		{
			struct device *dev = regmap_get_device(st->map);
			if (val <= 0)
				return -EINVAL;
			dev_info(dev, "Set the accelerometer sampling rate: %dHz\n", val);
			mutex_lock(&st->lock);
			ret = inv_icm20948_set_sensor_rate(st, 1, val);
			mutex_unlock(&st->lock);
			if (ret)
				dev_err(dev, "Failed to set accelerometer sampling rate: %d\n", ret);
			else
				dev_info(dev, "The accelerometer sampling rate was set successfully.: %dHz\n", val);
			return ret;
		}

	default:
		return -EINVAL;
	}
}

static int inv_icm20948_accel_validate_trigger(struct iio_dev *indio_dev,
                                            struct iio_trigger *trig)
{
	struct inv_icm20948_state *st = iio_device_get_drvdata(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_info inv_icm20948_accel_info = {
	.read_raw = inv_icm20948_accel_read_raw,
	.write_raw = inv_icm20948_accel_write_raw,
	.validate_trigger = inv_icm20948_accel_validate_trigger,
};

/* Change trigger_handler to a non-static function to make it visible in buffer.c */
irqreturn_t inv_icm20948_accel_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_icm20948_state *st = iio_device_get_drvdata(indio_dev);
	__be16 buffer[8]; /* 3 x 16-bit channels + timestamp */
	int ret;

	mutex_lock(&st->lock);
	
	/* Ensure we are in bank 0 */
	ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
	if (ret) {
		mutex_unlock(&st->lock);
		goto done;
	}

	/* Read all three acceleration channels at once */
	ret = regmap_bulk_read(st->map, INV_ICM20948_REG_ACCEL_XOUT_H,
	                      buffer, 3 * sizeof(__be16));
	if (ret) {
		mutex_unlock(&st->lock);
		goto done;
	}
	
	st->timestamp.accel = iio_get_time_ns(indio_dev);
	mutex_unlock(&st->lock);

	iio_push_to_buffers_with_timestamp(indio_dev, buffer, st->timestamp.accel);

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

struct iio_dev *inv_icm20948_accel_init(struct inv_icm20948_state *st)
{
	struct iio_dev *indio_dev;
	struct inv_icm20948_sensor_state *sensor;
	int ret;

	indio_dev = devm_iio_device_alloc(regmap_get_device(st->map),
					 sizeof(*sensor));
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);

	sensor = iio_priv(indio_dev);
	sensor->scales = inv_icm20948_accel_fs_map;
	sensor->scales_len = ARRAY_SIZE(inv_icm20948_accel_fs_map);

	indio_dev->name = "icm20948_accel";
	indio_dev->info = &inv_icm20948_accel_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = inv_icm20948_accel_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm20948_accel_channels);

	/* Store the same state pointer for all sensors */
	iio_device_set_drvdata(indio_dev, st);

	/* Only associate the trigger, do not set the trigger buffer, this is handled uniformly by inv_icm20948_setup_trigger */
	if (st->trig) {
		indio_dev->trig = st->trig;
	}

	return indio_dev;
}
EXPORT_SYMBOL_GPL(inv_icm20948_accel_init);
