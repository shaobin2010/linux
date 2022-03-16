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

#ifndef NDP10X_SENSOR_H
#define NDP10X_SENSOR_H

#include <syntiant-firmware/ndp10x_firmware.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ndp10x_sensor_driver_cause_e {
    NDP10X_SENSOR_DRIVER_CAUSE_DNN = 0,
    NDP10X_SENSOR_DRIVER_CAUSE_INT = 1,
    NDP10X_SENSOR_DRIVER_CAUSE_TIMER = 2
};

/* process GPIO interrupts (for sensor drivers) */
int ndp10x_sensor_int(uint32_t gpios, uint32_t *ints,
                      struct ndp10x_fw_state_s *fw_state);

/* process DNN completions (for sensor drivers) */
int ndp10x_sensor_dnn(struct ndp10x_fw_state_s *fw_state);

/* process MCU timer interrupt (for sensor drivers) */
void ndp10x_sensor_timer(struct ndp10x_fw_state_s *fw_state, int timer);

/* convenience function to record a match in the results ring */
void ndp10x_sensor_record_match(struct ndp10x_fw_state_s *fw_state,
                                uint32_t match_id);

/* FIFO emulator for DNN static input vector */
void ndp10x_sensor_invec_push(struct ndp10x_fw_state_s *fw_state, uint8_t *new,
                              int newlen);

/* reset the static input vector */
void ndp10x_sensor_invec_reset(void);

/* true if NDP is configured for bit-bang I2S interface to run the DNN */
int ndp10x_sensor_dnn_manual(struct ndp10x_fw_state_s *fw_state);

/* bit-bang I2S interface for 4 bytes to run the DNN */
void ndp10x_sensor_dnn_manual_run(struct ndp10x_fw_state_s *fw_state);

#ifdef __cplusplus
}
#endif
#endif
