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

#ifndef NDP10X_ACCEL_ALGO_H
#define NDP10X_ACCEL_ALGO_H

#include <syntiant-firmware/ndp10x_firmware.h>
    
#ifdef __cplusplus
extern "C" {
#endif

enum ndp10x_accel_algo_constants_e {
    ACCEL_PICKUP_DETECTED = 0x01,
    ACCEL_SM_DETECTED = 0x02,
    ACCEL_TILT_DETECTED = 0x04,
    ACCEL_DT_DETECTED = 0x08,
    ACCEL_FREEFALL_DETECTED = 0x10,
    ACCEL_STEP_DETECTED = 0x20,
    ACCEL_SHAKE_DETECTED = 0x40,
    ACCEL_ANDROID_TILT_DETECTED = 0x400,
    ACCEL_FLIP_DETECTED = 0x800,
    ACCEL_ELDERFALL_DETECTED = 0x1000
};

void sensor_transformation(uint16_t *current_sample, uint8_t xy_swap,
               uint8_t x_invert, uint8_t y_invert, uint8_t z_invert);
void reset_algo_detected_flags(struct ndp10x_fw_state_s *fw_state);
void init_algo_state_structure(struct ndp10x_fw_state_s *fw_state);

int16_t convert_2scomplement_decimal(uint16_t twos_complement);
uint16_t convert_decimal_to_2scomplement(int16_t decimal);
uint8_t posterior_handler(uint8_t activation, uint8_t phwin,
                          uint8_t phbackoff, uint16_t* phwin_count,
                          uint16_t* phbackoff_count);
uint32_t ndp10x_algo_main_loop(struct ndp10x_fw_state_s *fw_state,
                          uint16_t acc_x, uint16_t acc_y, uint16_t acc_z);

int ndp10x_check_stepcnt(struct ndp10x_fw_state_s *fw_state);
int ndp10x_check_elderly_fall(struct ndp10x_fw_state_s *fw_state,
    int16_t acc_x, int16_t acc_y, int16_t acc_z);
int ndp10x_check_significant_motion(struct ndp10x_fw_state_s *fw_state);
#ifdef __cplusplus
}
#endif
#endif
