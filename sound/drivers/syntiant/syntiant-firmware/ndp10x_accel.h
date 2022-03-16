/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2021 Syntiant Corporation
 * All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of
 * Syntiant Corporation and its suppliers, if any.  The intellectual and
 * technical concepts contained herein are proprietary to Syntiant Corporation
 * and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 * process, and are protected by trade secret or copyright law.  Dissemination
 * of this information or reproduction of this material is strictly forbidden
 * unless prior written permission is obtained from Syntiant Corporation.
 *
 */

#ifndef NDP10X_ACCEL_H
#define NDP10X_ACCEL_H

#include <syntiant-firmware/ndp10x_firmware.h>
    
#ifdef __cplusplus
extern "C" {
#endif

enum ndp10x_accel_constants_e {
    KX120_INT_SRC_REG = 0x13,
    KX120_INT_REL_REG = 0x17,
    KX120_INT_STATUS_BIT = 0x10,
    KX120_DRDY_INT = 0x10,
    KX120_XOUT_L_REG = 0x06,
    MC3419_INT_STATUS_REG = 0x14,
    MC3419_ACQ_INT_BIT = 0x80,
    MC3419_XOUT_L_REG = 0x0d,
};

uint32_t ndp10x_accel_get_and_process_data(struct ndp10x_fw_state_s *fw_state,
                      struct ndp10x_fw_serial_s *serial, unsigned int address,
                      int sensor_number);
int ndp10x_accel_process_interrupt(struct ndp10x_fw_state_s *fw_state,
                               struct ndp10x_fw_serial_s *serial,
                               unsigned int address, int sensor_number);
#ifdef __cplusplus
}
#endif
#endif
