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

#ifndef NDP10X_VM3011_H
#define NDP10X_VM3011_H

#include <syntiant-firmware/ndp10x_firmware.h>
    
#ifdef __cplusplus
extern "C" {
#endif

int ndp10x_vm3011_driver(struct ndp10x_fw_state_s *fw_state,
                             int sensor_number, int cause);

void
ndp10x_vm3011_timed_spl_check(struct ndp10x_fw_state_s *fw_state,
 int sensor_number, uint8_t p_auto_adj, uint8_t p_auto_adj_int);

void ndp10x_vm3011_set_threshold(struct ndp10x_fw_state_s *fw_state,
 uint32_t address, uint8_t threshold);

uint8_t ndp10x_vm3011_get_spl(struct ndp10x_fw_state_s *fw_state,
     uint32_t address, int sensor_number);

void ndp10x_vm3011_dnn(struct ndp10x_fw_state_s *fw_state, 
    volatile struct ndp10x_fw_sensor_configuration_s *sensor,
    uint32_t *state, uint32_t *timeout,
    int sensor_number, uint32_t address, uint32_t *clear_sum);

void
ndp10x_vm3011_set_pga_max_gain(struct ndp10x_fw_state_s * fw_state,
 uint32_t address, uint8_t max_pga_gain);

#ifdef __cplusplus
}
#endif
#endif
