# SPDX-License-Identifier: GPL-2.0-or-later
#
# Makefile for Invensense ICM-20948 IMU driver
#

# Main module: inv_icm20948.ko
obj-$(CONFIG_INV_ICM20948) += inv_icm20948.o

# Core module components
inv_icm20948-y := inv_icm20948_core.o \
                  inv_icm20948_buffer.o \
                  inv_icm20948_gyro.o \
                  inv_icm20948_accel.o \
                  inv_icm20948_magn.o \
                  inv_icm20948_temp.o

# Optional components based on Kconfig
inv_icm20948-$(CONFIG_INV_ICM20948_I2C) += inv_icm20948_i2c.o
inv_icm20948-$(CONFIG_INV_ICM20948_SPI) += inv_icm20948_spi.o
inv_icm20948-$(CONFIG_INV_ICM20948_DEBUG) += inv_icm20948_debug.o

# Ensure proper dependency handling
ccflags-y += -I$(src)