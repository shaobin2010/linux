/*
SYNTIANT CONFIDENTIAL
_____________________

Copyright (c) 2017-2021 Syntiant Corporation
All Rights Reserved.

NOTICE:  All information contained herein is, and remains the property of
Syntiant Corporation and its suppliers, if any.  The intellectual and
technical concepts contained herein are proprietary to Syntiant Corporation
and its suppliers and may be covered by U.S. and Foreign Patents, patents in
process, and are protected by trade secret or copyright law.  Dissemination of
this information or reproduction of this material is strictly forbidden unless
prior written permission is obtained from Syntiant Corporation.
*/

/* -----------------------------------------------
   * * * * * * * * * * * * * * * * * * * * * * * *

   W A R N I N G

   This file is auto-generated by the
   make_c_headers.py script in syntiant-packager.

   Issue `make headers` from that repository to
   regenerate this file.  It will be found in
   <repo-dir>/include.

   * * * * * * * * * * * * * * * * * * * * * * * *
   ----------------------------------------------- */

#ifndef SYNTIANT_PACKAGE_TAGS_H
#define SYNTIANT_PACKAGE_TAGS_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#define MAGIC_VALUE 0x53BDE5A1U

/**
 *@brief tag values common in packager and ilib
 */
enum syntiant_package_tag_values_e {
    TAG_HEADER = 0X1,
    TAG_FIRMWARE_VERSION_STRING_V1 = 0X2,
    TAG_PAD = 0X3,
    TAG_CHECKSUM = 0X4,
    TAG_NN_VERSION_STRING_V1 = 0X5,
    TAG_NDP10X_NN_CONFIG_V1 = 0X6,
    TAG_NDP10X_NN_PARAMETERS_V1 = 0X7,
    TAG_FIRMWARE = 0X8,
    TAG_NDP10X_HW_CONFIG_V1 = 0X9,
    TAG_NN_LABELS_V1 = 0XA,
    TAG_NDP10X_HW_CONFIG_V2 = 0XB,
    TAG_NDP10X_NN_PARAMETERS_V2 = 0XC,
    TAG_NN_PH_PARAMETERS_V1 = 0XD,
    TAG_NDP10X_B0_ENCRYPTED_FIRMWARE = 0XE,
    TAG_NN_PH_PARAMETERS_V2 = 0XF,
    TAG_NN_PH_PARAMETERS_V3 = 0X10,
    TAG_FIRMWARE_VERSION_STRING_V2 = 0X11,
    TAG_NN_VERSION_STRING_V2 = 0X12,
    TAG_PACKAGE_VERSION_STRING = 0X13,
    TAG_NDP10X_B0_NN_CONFIG_V1 = 0X14,
    TAG_BOARD_CALIBRATION_PARAMS_V1 = 0X15,
    TAG_NDP10X_NN_PARAMETERS_V3 = 0X16,
    TAG_NN_PH_PARAMETERS_V4 = 0X17,
    TAG_NN_LABELS_V2 = 0X18,
    TAG_NDP10X_B0_NN_CONFIG_V2 = 0X19,
    TAG_NDP10X_B0_NN_CONFIG_V3 = 0X1A,
    TAG_BOARD_CALIBRATION_PARAMS_V2 = 0X1B,
    TAG_UILIB_EXT_CLK = 0X1C,
    TAG_UILIB_INT_CLK = 0X1D,
    TAG_UILIB_SPI_WRITE = 0X1E,
    TAG_UILIB_MCU_WRITE = 0X1F,
    TAG_NDP12X_A0_NN_PARAMETERS_V1 = 0X20,
    TAG_NDP12X_A0_NN_PARAMETERS_ENCRYPTED_V1 = 0X21,
    TAG_NDP12X_A0_FIRMWARE_V1 = 0X22,
    TAG_NDP12X_A0_ENCRYPTED_FIRMWARE_V1 = 0X23,
    TAG_NDP12X_A0_NN_CONFIG_V1 = 0X24,
    TAG_NDP12X_A0_NN_CONFIG_ENCRYPTED_V1 = 0X25,
    TAG_NDP12X_A0_HW_CONFIG_V1 = 0X26,
    TAG_NDP12X_A0_HW_CONFIG_ENCRYPTED_V1 = 0X27,
    TAG_NDP12X_A0_DSP_FIRMWARE_V1 = 0X28,
    TAG_NDP12X_A0_DSP_ENCRYPTED_FIRMWARE_V1 = 0X29,
    TAG_DSP_FIRMWARE_VERSION_STRING_V1 = 0X2A,
    TAG_BOARD_CALIBRATION_PARAMS_V3 = 0X2B,
    TAG_SYNPKG_FILE = 0X2C,
    TAG_TFLITE_MODEL_FILE = 0X2D,
    TAG_POSTERIOR_FILE = 0X2E,
    TAG_TFLITE_FB_FILE = 0X2F,
    TAG_COMPOSITE_SOUND_CONTENT = 0X30,
    TAG_NN_PH_PARAMETERS_V5 = 0X31,
    TAG_NDP12X_A0_NN_METADATA = 0X32,
    TAG_NDP12X_A0_DSP_FLOW_RULES_V1 = 0X33,
    TAG_NN_PHP_COLLECTION_V1 = 0X34,
    TAG_NN_LABELS_V3 = 0X35,
    TAG_NDP12X_A0_DSP_FLOW_COLLECTION_V1 = 0X36,
    TAG_NDP12X_A0_MCU_ORCHERSTARTOR_V1 = 0X37,
    TAG_BOARD_CALIBRATION_PARAMS_V4 = 0X38,
    TAG_HOST_CONFIG = 0X39,
    TAG_PACKAGERLIB_VERSION_STRING = 0X3A,
    TAG_NN_PARAMS = 0X3B,
    TAG_NDP120_B0_FIRMWARE_V1 = 0X3C,
    TAG_NDP120_B0_ENCRYPTED_FIRMWARE_V1 = 0X3D,
    TAG_NDP120_B0_DSP_FE_CONFIG_V1 = 0X3E,
    TAG_NDP120_B0_NN_PARAMETERS_V1 = 0X3F,
    TAG_NDP120_B0_NN_METADATA = 0X40,
    TAG_NDP120_B0_DSP_FLOW_COLLECTION_V1 = 0X41,
    TAG_NDP120_B0_MCU_ORCHESTRATOR_V1 = 0X42,
    TAG_NDP120_B0_DSP_FIRMWARE_V1 = 0X43,
    TAG_NDP120_B0_HW_CONFIG_V1 = 0X44,
    TAG_NN_PH_PARAMETERS_V6 = 0X45,
    TAG_NDP10X_B0_NN_PARAMETERS_ENCRYPTED_V1 = 0x47
};

#define TAG_MULTI_SEGMENT_BIT 0x80000000
#endif
