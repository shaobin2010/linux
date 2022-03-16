/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2017-2020 Syntiant Corporation
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

#ifndef NDP10X_AGC_H
#define NDP10X_AGC_H

#include <syntiant-firmware/ndp10x_firmware.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief initializes the agc state.
 *
 * @param astate agc state object
 */
void ndp10x_agc_state_init(struct ndp10x_fw_state_s *fw_state,
                           struct ndp10x_agc_state_s *astate);

/**
 * @brief process a new frames & adjusts the mic gain based on stats from new
 * frame.
 *
 * @param astate agc state object
 *
 * @return 8-bit SNR estimation
 */
uint32_t ndp10x_agc_process_new_frame(struct ndp10x_fw_state_s *fw_state,
                                      struct ndp10x_agc_state_s *astate);

/**
 * @brief Track noise level breaching threshold value for "noise_threshold_win"
 * number of consecutive windows.
 *
 * @param astate agc state object
 *
 * @return 8-bit noise level for reporting it to the host, else send 0.
 */
uint8_t ndp10x_process_noise_level(struct ndp10x_fw_state_s *fw_state,
                                   struct ndp10x_agc_state_s *agc);

#ifdef __cplusplus
}
#endif

#endif
