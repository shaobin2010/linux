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

#ifndef NDP10X_PH_H
#define NDP10X_PH_H

#include <syntiant-firmware/ndp10x_firmware.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief initializes the Softmax smoother from posterior handler params.
 *
 * @param ph_params Posterior Handler params
 * @param lengths smoothing queue lengths
 */
void ndp10x_ph_get_smoother_queue_lengths(struct ndp10x_ph_params_s *ph_params,
                                          uint8_t *lengths);

/**
 * @brief process a new frame result and picks a winner
 *
 * @param ph_state posterior handler state
 * @param result smoother and softmax result state
 * @param tankptr current holding tank pointer
 */
void ndp10x_ph_process_frame(struct ndp10x_ph_state_s *ph_state,
                             struct ndp10x_result_s *result, uint32_t tankptr);

#ifdef __cplusplus
}
#endif
#endif
