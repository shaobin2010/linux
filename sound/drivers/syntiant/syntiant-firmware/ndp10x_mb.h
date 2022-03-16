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

#ifndef NDP10X_MB_H
#define NDP10X_MB_H

#include <syntiant-firmware/ndp10x_firmware.h>
#include <syntiant-firmware/ndp10x_ccm.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ndp10x_mb_smap_interval_s {
    uint32_t lo;
    uint32_t hi;
};
    
struct ndp10x_mb_smap_opens_s {
    int nwrite_opens;
    struct ndp10x_mb_smap_interval_s *write_opens;
    int nread_opens;
    struct ndp10x_mb_smap_interval_s *read_opens;
};

/**
 * @brief Handles M2H responses & H2M requests.
 *
 * @param mb_state NDP10X firmware mailbox state object
 * @param opens protected mode open address ranges
 */
void ndp10x_mb_respond(struct ndp10x_mb_state_s *mb_state,
                       struct ndp10x_mb_smap_opens_s *opens,
                       struct ndp10x_ccm_state_s *cs);

/**
 * @brief Sends a M2H request to host
 *
 * @param mb_state NDP10X firmware mailbox state object
 * @param req Request to be sent to Host
 */
void ndp10x_mb_send_m2h_request(struct ndp10x_fw_state_s *fw_state, uint8_t req);

/**
 * @brief Send a MATCH request to host
 *
 * @param mb_state  NDP10X firmware mailbox state object
 */
void ndp10x_mb_send_match(struct ndp10x_fw_state_s *fw_state);

#ifdef __cplusplus
}
#endif

#endif
