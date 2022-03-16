/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2017-2021 Syntiant Corporation
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

#ifndef NDP10X_INT_HANDLER_H
#define NDP10X_INT_HANDLER_H

#include <syntiant-firmware/ndp10x_firmware.h>
#include <syntiant-firmware/ndp10x_mb.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief record a match in the notification ring
 *
 * @param fw_state
 * @param summary summary value for the match
 * @param tankptr tank pointer for the match
 */
void ndp10x_record_match(struct ndp10x_fw_state_s *fw_state, uint32_t summary,
                         uint32_t tankptr);

/**
 * @brief handles the dnn interrupt. This interrupt happens whenever dnn block
 * completes one inference.
 *
 * @param fw_state
 * @return true if a match was recorded (and the MATCH mbox should be sent)
 */
int ndp10x_dnn_int_handler(struct ndp10x_fw_state_s *fw_state);

/**
 * @brief handles the mailbox interrupt. This interrupt happens whenever there
 * is a new Request from host or Response from host.
 *
 * @param fw_state
 * @param opens protected mode open address ranges
 * @param cs CCM decryption state
 */
void ndp10x_mb_int_handler(struct ndp10x_fw_state_s *fw_state,
                           struct ndp10x_mb_smap_opens_s *opens,
                           struct ndp10x_ccm_state_s *cs);

/**
 * @brief perform the holding tank MSB work-around logic
 * @param fw_state
 */
void ndp10x_tank_msb_fix(struct ndp10x_fw_state_s *fw_state);

/**
 * @brief handles the freq block interrupt. This interrupt happens whenever
 * freq block is done computing filterbanks for a frame.
 * @param fw_state
 */
void ndp10x_freq_int_handler(struct ndp10x_fw_state_s *fw_state);

/**
 * @brief handles the dual timer block interrupt
 * @param fw_state
 */
void ndp10x_dualtimer_int_handler(struct ndp10x_fw_state_s *fw_state);

#ifdef __cplusplus
}
#endif
#endif
