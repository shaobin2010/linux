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

DIR = drivers/amlogic/isp_module/subdev/iq

obj-m += iv009_isp_iq.o

FW_SRC_OBJ := app/soc_iq.o \
			  src/drv_otp/acamera_calibration_imx227_otp.o \
			  src/drv_otp/acamera_calibration_os08a10_otp.o \
			  src/drv_otp/acamera_calibration_ov5675_otp.o \
			  src/drv_otp/acamera_get_calibration_otp.o \
			  src/drv_tuning/acamera_calibrations.o \
			  src/fw_lib/acamera_logger.o \
			  src/fw_lib/acamera_math.o \
			  src/fw_lib/acamera_sbus.o \
			  src/fw_lib/acamera_sbus_i2c.o \
			  src/fw_lib/acamera_sbus_isp.o \
			  src/fw_lib/acamera_sbus_isp_sw.o \
			  src/fw_lib/acamera_sbus_spi.o \
			  src/platform/system_hw_io.o \
			  src/platform/system_i2c.o \
			  src/platform/system_log.o \
			  src/platform/system_spi.o \
			  src/platform/system_spinlock.o \
			  src/platform/system_stdlib.o \
			  src/platform/system_sw_io.o \
			  src/platform/system_timer.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx290.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx290_lens_4mm.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx290_lens_8mm.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx307.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx307_demo.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx334.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx335.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx415.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_os04a10_tv.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_os08a10_ipc.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_os08a10_slt.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_os08a10_tv.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_ov13858.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_ov2718.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_sc4238.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_sc4238_night.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_sc8238cs_tv.o \
			  src/calibration/acamera_calibrations_dynamic_linear_dummy.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx224.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx227.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx290.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx290_lens_4mm.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx290_lens_8mm.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx290_slt.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx307.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx307_demo.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx334.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx335.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx415.o \
			  src/calibration/acamera_calibrations_dynamic_linear_imx481.o \
			  src/calibration/acamera_calibrations_dynamic_linear_os04a10_tv.o \
			  src/calibration/acamera_calibrations_dynamic_linear_os08a10_ipc.o \
			  src/calibration/acamera_calibrations_dynamic_linear_os08a10_panel.o \
			  src/calibration/acamera_calibrations_dynamic_linear_os08a10_slt.o \
			  src/calibration/acamera_calibrations_dynamic_linear_os08a10_tv.o \
			  src/calibration/acamera_calibrations_dynamic_linear_ov13858.o \
			  src/calibration/acamera_calibrations_dynamic_linear_ov2718.o \
			  src/calibration/acamera_calibrations_dynamic_linear_ov5675.o \
			  src/calibration/acamera_calibrations_dynamic_linear_sc2232h.o \
			  src/calibration/acamera_calibrations_dynamic_linear_sc2335.o \
			  src/calibration/acamera_calibrations_dynamic_linear_sc4238.o \
			  src/calibration/acamera_calibrations_dynamic_linear_sc4238_night.o \
			  src/calibration/acamera_calibrations_dynamic_linear_sc8238cs_tv.o \
			  src/calibration/acamera_calibrations_static_fs_lin_dummy.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx224.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx290.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx290_lens_4mm.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx290_lens_8mm.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx307.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx307_demo.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx334.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx335.o \
			  src/calibration/acamera_calibrations_static_fs_lin_imx415.o \
			  src/calibration/acamera_calibrations_static_fs_lin_os04a10_tv.o \
			  src/calibration/acamera_calibrations_static_fs_lin_os08a10_ipc.o \
			  src/calibration/acamera_calibrations_static_fs_lin_os08a10_slt.o \
			  src/calibration/acamera_calibrations_static_fs_lin_os08a10_tv.o \
			  src/calibration/acamera_calibrations_static_fs_lin_ov13858.o \
			  src/calibration/acamera_calibrations_static_fs_lin_ov2718.o \
			  src/calibration/acamera_calibrations_static_fs_lin_sc4238.o \
			  src/calibration/acamera_calibrations_static_fs_lin_sc8238cs_tv.o \
			  src/calibration/acamera_calibrations_static_linear_dummy.o \
			  src/calibration/acamera_calibrations_static_linear_imx224.o \
			  src/calibration/acamera_calibrations_static_linear_imx227.o \
			  src/calibration/acamera_calibrations_static_linear_imx290.o \
			  src/calibration/acamera_calibrations_static_linear_imx290_lens_4mm.o \
			  src/calibration/acamera_calibrations_static_linear_imx290_lens_8mm.o \
			  src/calibration/acamera_calibrations_static_linear_imx290_slt.o \
			  src/calibration/acamera_calibrations_static_linear_imx307.o \
			  src/calibration/acamera_calibrations_static_linear_imx307_demo.o \
			  src/calibration/acamera_calibrations_static_linear_imx334.o \
			  src/calibration/acamera_calibrations_static_linear_imx335.o \
			  src/calibration/acamera_calibrations_static_linear_imx415.o \
			  src/calibration/acamera_calibrations_static_linear_imx481.o \
			  src/calibration/acamera_calibrations_static_linear_os04a10_tv.o \
			  src/calibration/acamera_calibrations_static_linear_os08a10_ipc.o \
			  src/calibration/acamera_calibrations_static_linear_os08a10_panel.o \
			  src/calibration/acamera_calibrations_static_linear_os08a10_slt.o \
			  src/calibration/acamera_calibrations_static_linear_os08a10_tv.o \
			  src/calibration/acamera_calibrations_static_linear_ov13858.o \
			  src/calibration/acamera_calibrations_static_linear_ov2718.o \
			  src/calibration/acamera_calibrations_static_linear_ov5675.o \
			  src/calibration/acamera_calibrations_static_linear_sc2232h.o \
			  src/calibration/acamera_calibrations_static_linear_sc2335.o \
			  src/calibration/acamera_calibrations_static_linear_sc4238.o \
			  src/calibration/acamera_calibrations_static_linear_sc8238cs_tv.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_imx224.o \
			  src/calibration/acamera_calibrations_dynamic_fs_lin_dummy.o \
			  src/calibration/acamera_get_calibrations_dummy.o



iv009_isp_iq-objs += $(FW_SRC_OBJ)

INCLUDE_DIRS := $(addprefix -I,$(shell find ../ -type d 2> /dev/null))

ccflags-y:=-I$(DIR)/app -I$(DIR)/inc -I$(DIR)/app/control -I$(DIR)/inc/api -I$(DIR)/inc/isp -I$(DIR)/inc/sys -I$(DIR)/src/drv_tuning -I$(DIR)/src/drv_otp -I$(DIR)/src/platform -I$(DIR)/src/fw  -I$(DIR)/src/fw_lib -I$(DIR)/src/calibration -I$(DIR)/src/driver/sensor -I$(DIR)/src/driver/lens -I$(_KDIR)/include/linux/

ccflags-y += -Wno-declaration-after-statement -Wno-error

