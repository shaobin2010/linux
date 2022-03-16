/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2020 Syntiant Corporation
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

#ifndef NDP10X_DA217_H
#define NDP10X_DA217_H

#include <syntiant-firmware/ndp10x_firmware.h>
    
#ifdef __cplusplus
extern "C" {
#endif

void update_fw_state_raw_sample_buf(struct ndp10x_fw_state_s *fw_state);
int ndp10x_da217_driver(struct ndp10x_fw_state_s *fw_state,
                             int sensor_number);
void ndp10x_da217_state_init(struct ndp10x_fw_state_s *fw_state);
    
#ifdef __cplusplus
}
#endif
#endif
