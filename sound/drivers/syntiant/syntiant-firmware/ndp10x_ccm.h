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

/*
 * NOTE:
 * 1. this header file is used in the syntiant-ilib repo as an interface file
 * 2. Some compilers do not pack the structs which means they are always 32 bit
 * aligned so they are explicitly made 32 bit aligned so please follow that.
 */

#ifndef NDP10X_CCM_H
#define NDP10X_CCM_H

#ifdef __cplusplus
extern "C" {
#endif

enum ndp10x_ccm_constants_e {
    B_FLAG = 0x3b,
    C_FLAG = 0x03,
    NONCE_SIZE = 11,
    AES_BLOCK_LEN = 16
};
    
struct ndp10x_ccm_state_s {
    uint32_t nn_offset;
    uint32_t nn_left;
    uint32_t nn_state;
    uint32_t length;
    /*
     * at initialization iv_cbc[1:11] contains the nonce
     */
    uint32_t iv_cbc[AES_BLOCK_LEN / sizeof(uint32_t)];
    /*
     * at initialization data contains message integrity check (MIC) tag
     */
    uint32_t data[AES_BLOCK_LEN / sizeof(uint32_t)];
    uint32_t iv_ctr[AES_BLOCK_LEN / sizeof(uint32_t)]; /* big endian counter */
    uint32_t ctr_tag[AES_BLOCK_LEN / sizeof(uint32_t)];
};

int ndp10x_ccm_hw_init(void);
int ndp10x_ccm_secured(void);
int ndp10x_ccm_start(struct ndp10x_ccm_state_s *cs);
int ndp10x_ccm_decrypt(struct ndp10x_ccm_state_s *cs);
int ndp10x_ccm_finish(struct ndp10x_ccm_state_s *cs);

#ifdef __cplusplus
}
#endif
#endif
