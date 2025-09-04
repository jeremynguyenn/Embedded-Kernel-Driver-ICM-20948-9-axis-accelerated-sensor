// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2025 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/slab.h>

#include "inv_icm20948.h"

#define INV_ICM20948_FIFO_CHUNK_SIZE 16
#define INV_ICM20948_FIFO_MAX_SIZE 1024

extern irqreturn_t inv_icm20948_gyro_trigger_handler(int irq, void *p);
extern irqreturn_t inv_icm20948_accel_trigger_handler(int irq, void *p);
extern irqreturn_t inv_icm20948_magn_trigger_handler(int irq, void *p);

static void inv_icm20948_fifo_flush(struct inv_icm20948_state *st)
{
	mutex_lock(&st->lock);
	inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
	regmap_write(st->map, INV_ICM20948_REG_FIFO_RST, 0x1F);
	regmap_write(st->map, INV_ICM20948_REG_FIFO_RST, 0x00);
	mutex_unlock(&st->lock);
}

static int inv_icm20948_parse_fifo_gyro(struct iio_dev *indio_dev, uint8_t *data, int64_t timestamp)
{
	s16 x, y, z;
	s8 *buf = kzalloc(indio_dev->scan_bytes, GFP_KERNEL);
	int i;

	if (!buf)
		return -ENOMEM;

	x = (s16)(data[0] << 8 | data[1]);
	y = (s16)(data[2] << 8 | data[3]);
	z = (s16)(data[4] << 8 | data[5]);

	for (i = 0; i < 3; i++) {
		if (test_bit(i, indio_dev->active_scan_mask)) {
			s16 *dest = (s16 *)(buf + indio_dev->channels[i].scan_index * sizeof(s16));
			if (i == 0) *dest = x;
			else if (i == 1) *dest = y;
			else *dest = z;
		}
	}

	iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
	kfree(buf);
	return 0;
}

static int inv_icm20948_parse_fifo_accel(struct iio_dev *indio_dev, uint8_t *data, int64_t timestamp)
{
	s16 x, y, z;
	s8 *buf = kzalloc(indio_dev->scan_bytes, GFP_KERNEL);
	int i;

	if (!buf)
		return -ENOMEM;

	x = (s16)(data[0] << 8 | data[1]);
	y = (s16)(data[2] << 8 | data[3]);
	z = (s16)(data[4] << 8 | data[5]);

	for (i = 0; i < 3; i++) {
		if (test_bit(i, indio_dev->active_scan_mask)) {
			s16 *dest = (s16 *)(buf + indio_dev->channels[i].scan_index * sizeof(s16));
			if (i == 0) *dest = x;
			else if (i == 1) *dest = y;
			else *dest = z;
		}
	}

	iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
	kfree(buf);
	return 0;
}

static int inv_icm20948_parse_fifo_magn(struct iio_dev *indio_dev, uint8_t *data, int64_t timestamp)
{
	s16 x, y, z;
	s8 *buf = kzalloc(indio_dev->scan_bytes, GFP_KERNEL);
	int i;

	if (!buf)
		return -ENOMEM;

	x = (s16)(data[1] << 8 | data[0]);
	y = (s16)(data[3] << 8 | data[2]);
	z = (s16)(data[5] << 8 | data[4]);

	for (i = 0; i < 3; i++) {
		if (test_bit(i, indio_dev->active_scan_mask)) {
			s16 *dest = (s16 *)(buf + indio_dev->channels[i].scan_index * sizeof(s16));
			if (i == 0) *dest = x;
			else if (i == 1) *dest = y;
			else *dest = z;
		}
	}

	iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
	kfree(buf);
	return 0;
}

int inv_icm20948_read_fifo(struct inv_icm20948_state *st)
{
	u8 data[INV_ICM20948_FIFO_MAX_SIZE];
	__be16 fifo_count;
	u16 word;
	u8 *ptr;
	bool has_accel = true, has_gyro = true, has_magn = st->mag_initialized;
	bool accel_done = false, gyro_done = false, magn_done = false;
	int64_t timestamp = iio_get_time_ns(st->indio_accel); // Use accel as reference
	int ret;
	u8 status;

	mutex_lock(&st->lock);
	ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	ret = regmap_read(st->map, INV_ICM20948_REG_FIFO_WM_INT_STATUS, &status);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}
	if (!(status & BIT(0))) {
		mutex_unlock(&st->lock);
		return -EAGAIN;  /* No watermark, partial read or skip */
	}

	ret = regmap_bulk_read(st->map, INV_ICM20948_REG_FIFO_COUNT_H, &fifo_count, sizeof(fifo_count));
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	word = be16_to_cpu(fifo_count);
	if (!word || word > INV_ICM20948_FIFO_MAX_SIZE) {
		inv_icm20948_fifo_flush(st);
		mutex_unlock(&st->lock);
		return -EIO;
	}

	ret = regmap_bulk_read(st->map, INV_ICM20948_REG_FIFO_R_W, data, word);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;

	ptr = data;
	while (ptr < data + word) {
		if (has_accel && !accel_done) {
			ret = inv_icm20948_parse_fifo_accel(st->indio_accel, ptr, timestamp);
			ptr += 6;
			accel_done = true;
		}
		if (has_gyro && !gyro_done) {
			ret = inv_icm20948_parse_fifo_gyro(st->indio_gyro, ptr, timestamp);
			ptr += 6;
			gyro_done = true;
		}
		if (has_magn && !magn_done && st->indio_magn) {
			ret = inv_icm20948_parse_fifo_magn(st->indio_magn, ptr, timestamp);
			ptr += 6;
			magn_done = true;
		}
		if ((accel_done || !has_accel) && (gyro_done || !has_gyro) && (magn_done || !has_magn || !st->indio_magn)) {
			accel_done = gyro_done = magn_done = false;
		}
		if (ret)
			return ret;
	}

	return 0;
}

int inv_icm20948_configure_fifo(struct inv_icm20948_state *st, bool enable)
{
	int ret;
	u8 val = 0;

	mutex_lock(&st->lock);
	ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	if (enable) {
		val = INV_ICM20948_BIT_FIFO_EN;
	}

	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL, INV_ICM20948_BIT_FIFO_EN, val);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	if (enable) {
		ret = regmap_write(st->map, INV_ICM20948_REG_FIFO_EN_2, 0xF8); // Enable all sensors FIFO
		if (ret) {
			mutex_unlock(&st->lock);
			return ret;
		}
		ret = regmap_write(st->map, INV_ICM20948_REG_FIFO_WM_TH, 512);  /* Watermark threshold */
		if (ret) {
			mutex_unlock(&st->lock);
			return ret;
		}
		inv_icm20948_fifo_flush(st);
	}

	st->use_fifo = enable;
	mutex_unlock(&st->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm20948_configure_fifo);

int inv_icm20948_setup_trigger(struct inv_icm20948_state *st)
{
	int ret;

	st->indio_gyro->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_triggered_buffer_setup(st->indio_gyro, NULL, inv_icm20948_gyro_trigger_handler, NULL);
	if (ret)
		return ret;

	st->indio_accel->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_triggered_buffer_setup(st->indio_accel, NULL, inv_icm20948_accel_trigger_handler, NULL);
	if (ret)
		goto cleanup_gyro;

	if (st->indio_magn) {
		st->indio_magn->modes |= INDIO_BUFFER_TRIGGERED;
		ret = iio_triggered_buffer_setup(st->indio_magn, NULL, inv_icm20948_magn_trigger_handler, NULL);
		if (ret)
			goto cleanup_accel;
	}

	return 0;

cleanup_accel:
	iio_triggered_buffer_cleanup(st->indio_accel);
cleanup_gyro:
	iio_triggered_buffer_cleanup(st->indio_gyro);
	return ret;
}
EXPORT_SYMBOL_GPL(inv_icm20948_setup_trigger);

void inv_icm20948_cleanup_buffer(struct inv_icm20948_state *st)
{
	if (st->indio_magn)
		iio_triggered_buffer_cleanup(st->indio_magn);
	iio_triggered_buffer_cleanup(st->indio_accel);
	iio_triggered_buffer_cleanup(st->indio_gyro);
}
EXPORT_SYMBOL_GPL(inv_icm20948_cleanup_buffer);
