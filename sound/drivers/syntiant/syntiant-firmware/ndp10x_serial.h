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

#ifndef NDP10X_SERIAL_H
#define NDP10X_SERIAL_H

#include <syntiant-firmware/ndp10x_firmware.h>
    
#ifdef __cplusplus
extern "C" {
#endif

void ndp10x_serial_run(struct ndp10x_fw_state_s *fw_state,
                       volatile struct ndp10x_fw_serial_s *serial);
    
#ifdef __cplusplus
}
#endif
#endif
