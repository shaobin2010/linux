#
# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2011-2018 ARM or its affiliates
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; version 2.
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#

DIR = drivers/amlogic/isp_module/subdev/sensor

obj-m += iv009_isp_sensor.o

FW_SRC_OBJ := app/soc_sensor.o \
			  src/driver/sensor/ACameraDefault.o \
			  src/driver/sensor/Dummy_drv.o \
			  src/driver/sensor/IMX224_drv.o \
			  src/driver/sensor/IMX227_drv.o \
			  src/driver/sensor/IMX290_drv.o \
			  src/driver/sensor/IMX307_drv.o \
			  src/driver/sensor/IMX334_drv.o \
			  src/driver/sensor/IMX335_drv.o \
			  src/driver/sensor/IMX415_drv.o \
			  src/driver/sensor/OV08a10_drv.o \
			  src/driver/sensor/OV13858_drv.o \
			  src/driver/sensor/OV5640_drv.o \
			  src/driver/sensor/OV5675_drv.o \
			  src/driver/sensor/SC2232h_drv.o \
			  src/driver/sensor/SC2335_drv.o \
			  src/driver/sensor/SC4238_drv.o \
			  src/driver/sensor/LT6911C_drv.o \
			  src/driver/sensor/sensor_bsp_common.o \
			  src/driver/sensor/sensor_bus_config.o \
			  src/driver/sensor/virtcam_drv.o \
			  src/fw_lib/acamera_logger.o \
			  src/fw_lib/acamera_math.o \
			  src/fw_lib/acamera_sbus.o \
			  src/fw_lib/acamera_sbus_i2c.o \
			  src/fw_lib/acamera_sbus_isp.o \
			  src/fw_lib/acamera_sbus_isp_sw.o \
			  src/fw_lib/acamera_sbus_spi.o \
			  src/fw_lib/sensor_init.o \
			  src/platform/system_am_adap.o \
			  src/platform/system_am_adap_cmpr.o \
			  src/platform/system_am_mipi_aphy1_interface.o \
			  src/platform/system_am_mipi.o \
			  src/platform/system_hw_io.o \
			  src/platform/system_i2c.o \
			  src/platform/system_log.o \
			  src/platform/system_sensor.o \
			  src/platform/system_spi.o \
			  src/platform/system_spinlock.o \
			  src/platform/system_stdlib.o \
			  src/platform/system_sw_io.o \
			  src/platform/system_timer.o 


iv009_isp_sensor-objs += $(FW_SRC_OBJ)

INCLUDE_DIRS := $(addprefix -I,$(shell find ../ -type d 2> /dev/null))

ccflags-y:=-I$(DIR)/app -I$(DIR)/inc -I$(DIR)/app/control -I$(DIR)/inc/api -I$(DIR)/inc/isp -I$(DIR)/inc/sys -I$(DIR)/src/platform -I$(DIR)/src/fw  -I$(DIR)/src/fw_lib -I$(DIR)/src/calibration -I$(DIR)/src/driver/sensor -I$(DIR)/src/driver/lens -I$(_KDIR)/include/linux/

ccflags-y += -Wno-declaration-after-statement -Wno-error -Wno-strict-prototypes

ccflags-y += -DCONFIG_LINUX_OS

ifeq ($(SEAMLESS_SUPPORT),)
    ccflags-y += -DCONFIG_SEAMLESS_N
else
    ccflags-y += -DCONFIG_SEAMLESS_$(SEAMLESS_SUPPORT)
endif


ifeq ($(PLATFORM_VERSION),)
    ccflags-y += -DCONFIG_T7
else
    ccflags-y += -DCONFIG_$(PLATFORM_VERSION)
endif


