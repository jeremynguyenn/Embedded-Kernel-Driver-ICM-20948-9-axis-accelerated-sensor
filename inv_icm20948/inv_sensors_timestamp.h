/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2025 Invensense, Inc.
 * 
 * This header is shared across modules; ensure it's included in Makefile dependencies.
 */

#ifndef INV_SENSORS_TIMESTAMP_H_
#define INV_SENSORS_TIMESTAMP_H_

#include <linux/types.h>
#include <linux/iio/iio.h>

#ifdef CONFIG_DEBUG_SPINLOCK
#include <linux/spinlock_types.h>  /* For spinlock debugging */
#endif

/**
 * struct inv_sensors_timestamp - Common timestamp management for Invensense sensors
 * @lock: Spinlock to protect timestamp updates
 * @ts: Last reported timestamp
 * @period: Sampling period in nanoseconds
 */
struct inv_sensors_timestamp {
	spinlock_t lock;
	s64 ts;
	u64 period;
};

/**
 * inv_sensors_timestamp_init - Initialize timestamp structure
 * @ts: Timestamp structure to initialize
 */
static inline void inv_sensors_timestamp_init(struct inv_sensors_timestamp *ts)
{
	spin_lock_init(&ts->lock);
	ts->ts = 0;
	ts->period = 0;
}
EXPORT_SYMBOL_GPL(inv_sensors_timestamp_init);

/**
 * inv_sensors_timestamp_update - Update timestamp based on sampling rate
 * @ts: Timestamp structure
 * @new_ts: New timestamp value or 0 to use previous timestamp + period
 *
 * When new_ts is non-zero, it is used as the new timestamp.
 * Otherwise, period is added to the previous timestamp.
 *
 * Returns: The updated timestamp.
 */
static inline s64 inv_sensors_timestamp_update(struct inv_sensors_timestamp *ts,
					      s64 new_ts)
{
	s64 result;
	unsigned long flags;

	spin_lock_irqsave(&ts->lock, flags);
	if (new_ts) {
		ts->ts = new_ts;
	} else if (ts->period) {
		ts->ts += ts->period;
	}
	result = ts->ts;
	spin_unlock_irqrestore(&ts->lock, flags);
	
	return result;
}
EXPORT_SYMBOL_GPL(inv_sensors_timestamp_update);

/**
 * inv_sensors_timestamp_set_period - Set the sampling period
 * @ts: Timestamp structure
 * @period_ns: Period in nanoseconds
 */
static inline void inv_sensors_timestamp_set_period(struct inv_sensors_timestamp *ts,
						  u64 period_ns)
{
	unsigned long flags;

	spin_lock_irqsave(&ts->lock, flags);
	ts->period = period_ns;
	spin_unlock_irqrestore(&ts->lock, flags);
}
EXPORT_SYMBOL_GPL(inv_sensors_timestamp_set_period);

#endif /* INV_SENSORS_TIMESTAMP_H_ */
