/*
 *
 * SYNTIANT CONFIDENTIAL
 * _____________________
 *
 *   Copyright (c) 2018 Syntiant Corporation
 *   All Rights Reserved.
 *
 *  NOTICE:  All information contained herein is, and remains the property of
 *  Syntiant Corporation and its suppliers, if any.  The intellectual and
 *  technical concepts contained herein are proprietary to Syntiant Corporation
 *  and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 *  process, and are protected by trade secret or copyright law.  Dissemination
 *  of this information or reproduction of this material is strictly forbidden
 *  unless prior written permission is obtained from Syntiant Corporation.
 */
/*
 * ******* Automatically generated for ndp10x_spi v1.00  DO NOT MODIFY!!!!!
 *          Generated Fri May  3 23:13:19 2019 UTC
 */
#ifndef NDP10X_SPI_REGS_H
#define NDP10X_SPI_REGS_H

/*
 * block ndp10x_spi, base 0x00000000
 */
#define NDP10X_SPI 0x00
#define NDP10X_SPI_SIZE 0x80
/* register ndp10x_spi.id0 */
#define NDP10X_SPI_ID0 0x00
/* register ndp10x_spi.id1 */
#define NDP10X_SPI_ID1 0x01
/* register ndp10x_spi.intsts */
#define NDP10X_SPI_INTSTS 0x02
#define NDP10X_SPI_INTSTS_MATCH_INT_SHIFT 0
#define NDP10X_SPI_INTSTS_MATCH_INT_MASK 0x01
#define NDP10X_SPI_INTSTS_MATCH_INT(v)                                         \
    ((v) << NDP10X_SPI_INTSTS_MATCH_INT_SHIFT)
#define NDP10X_SPI_INTSTS_MATCH_INT_INSERT(x, v)                               \
    ((x) | ((v) << NDP10X_SPI_INTSTS_MATCH_INT_SHIFT))
#define NDP10X_SPI_INTSTS_MATCH_INT_MASK_INSERT(x, v)                          \
    (((x) & ~NDP10X_SPI_INTSTS_MATCH_INT_MASK)                                 \
        | ((v) << NDP10X_SPI_INTSTS_MATCH_INT_SHIFT))
#define NDP10X_SPI_INTSTS_MATCH_INT_EXTRACT(x)                                 \
    (((x)&NDP10X_SPI_INTSTS_MATCH_INT_MASK)                                    \
        >> NDP10X_SPI_INTSTS_MATCH_INT_SHIFT)
#define NDP10X_SPI_INTSTS_MATCH_INT_DEFAULT 0x00000000
#define NDP10X_SPI_INTSTS_MBIN_INT_SHIFT 1
#define NDP10X_SPI_INTSTS_MBIN_INT_MASK 0x02
#define NDP10X_SPI_INTSTS_MBIN_INT(v) ((v) << NDP10X_SPI_INTSTS_MBIN_INT_SHIFT)
#define NDP10X_SPI_INTSTS_MBIN_INT_INSERT(x, v)                                \
    ((x) | ((v) << NDP10X_SPI_INTSTS_MBIN_INT_SHIFT))
#define NDP10X_SPI_INTSTS_MBIN_INT_MASK_INSERT(x, v)                           \
    (((x) & ~NDP10X_SPI_INTSTS_MBIN_INT_MASK)                                  \
        | ((v) << NDP10X_SPI_INTSTS_MBIN_INT_SHIFT))
#define NDP10X_SPI_INTSTS_MBIN_INT_EXTRACT(x)                                  \
    (((x)&NDP10X_SPI_INTSTS_MBIN_INT_MASK) >> NDP10X_SPI_INTSTS_MBIN_INT_SHIFT)
#define NDP10X_SPI_INTSTS_MBIN_INT_DEFAULT 0x00000000
#define NDP10X_SPI_INTSTS_MBOUT_INT_SHIFT 2
#define NDP10X_SPI_INTSTS_MBOUT_INT_MASK 0x04
#define NDP10X_SPI_INTSTS_MBOUT_INT(v)                                         \
    ((v) << NDP10X_SPI_INTSTS_MBOUT_INT_SHIFT)
#define NDP10X_SPI_INTSTS_MBOUT_INT_INSERT(x, v)                               \
    ((x) | ((v) << NDP10X_SPI_INTSTS_MBOUT_INT_SHIFT))
#define NDP10X_SPI_INTSTS_MBOUT_INT_MASK_INSERT(x, v)                          \
    (((x) & ~NDP10X_SPI_INTSTS_MBOUT_INT_MASK)                                 \
        | ((v) << NDP10X_SPI_INTSTS_MBOUT_INT_SHIFT))
#define NDP10X_SPI_INTSTS_MBOUT_INT_EXTRACT(x)                                 \
    (((x)&NDP10X_SPI_INTSTS_MBOUT_INT_MASK)                                    \
        >> NDP10X_SPI_INTSTS_MBOUT_INT_SHIFT)
#define NDP10X_SPI_INTSTS_MBOUT_INT_DEFAULT 0x00000000
#define NDP10X_SPI_INTSTS_DNN_INT_SHIFT 3
#define NDP10X_SPI_INTSTS_DNN_INT_MASK 0x08
#define NDP10X_SPI_INTSTS_DNN_INT(v) ((v) << NDP10X_SPI_INTSTS_DNN_INT_SHIFT)
#define NDP10X_SPI_INTSTS_DNN_INT_INSERT(x, v)                                 \
    ((x) | ((v) << NDP10X_SPI_INTSTS_DNN_INT_SHIFT))
#define NDP10X_SPI_INTSTS_DNN_INT_MASK_INSERT(x, v)                            \
    (((x) & ~NDP10X_SPI_INTSTS_DNN_INT_MASK)                                   \
        | ((v) << NDP10X_SPI_INTSTS_DNN_INT_SHIFT))
#define NDP10X_SPI_INTSTS_DNN_INT_EXTRACT(x)                                   \
    (((x)&NDP10X_SPI_INTSTS_DNN_INT_MASK) >> NDP10X_SPI_INTSTS_DNN_INT_SHIFT)
#define NDP10X_SPI_INTSTS_DNN_INT_DEFAULT 0x00000000
#define NDP10X_SPI_INTSTS_FREQ_INT_SHIFT 4
#define NDP10X_SPI_INTSTS_FREQ_INT_MASK 0x10
#define NDP10X_SPI_INTSTS_FREQ_INT(v) ((v) << NDP10X_SPI_INTSTS_FREQ_INT_SHIFT)
#define NDP10X_SPI_INTSTS_FREQ_INT_INSERT(x, v)                                \
    ((x) | ((v) << NDP10X_SPI_INTSTS_FREQ_INT_SHIFT))
#define NDP10X_SPI_INTSTS_FREQ_INT_MASK_INSERT(x, v)                           \
    (((x) & ~NDP10X_SPI_INTSTS_FREQ_INT_MASK)                                  \
        | ((v) << NDP10X_SPI_INTSTS_FREQ_INT_SHIFT))
#define NDP10X_SPI_INTSTS_FREQ_INT_EXTRACT(x)                                  \
    (((x)&NDP10X_SPI_INTSTS_FREQ_INT_MASK) >> NDP10X_SPI_INTSTS_FREQ_INT_SHIFT)
#define NDP10X_SPI_INTSTS_FREQ_INT_DEFAULT 0x00000000
#define NDP10X_SPI_INTSTS_AE_INT_SHIFT 5
#define NDP10X_SPI_INTSTS_AE_INT_MASK 0x20
#define NDP10X_SPI_INTSTS_AE_INT(v) ((v) << NDP10X_SPI_INTSTS_AE_INT_SHIFT)
#define NDP10X_SPI_INTSTS_AE_INT_INSERT(x, v)                                  \
    ((x) | ((v) << NDP10X_SPI_INTSTS_AE_INT_SHIFT))
#define NDP10X_SPI_INTSTS_AE_INT_MASK_INSERT(x, v)                             \
    (((x) & ~NDP10X_SPI_INTSTS_AE_INT_MASK)                                    \
        | ((v) << NDP10X_SPI_INTSTS_AE_INT_SHIFT))
#define NDP10X_SPI_INTSTS_AE_INT_EXTRACT(x)                                    \
    (((x)&NDP10X_SPI_INTSTS_AE_INT_MASK) >> NDP10X_SPI_INTSTS_AE_INT_SHIFT)
#define NDP10X_SPI_INTSTS_AE_INT_DEFAULT 0x00000000
#define NDP10X_SPI_INTSTS_WM_INT_SHIFT 6
#define NDP10X_SPI_INTSTS_WM_INT_MASK 0x40
#define NDP10X_SPI_INTSTS_WM_INT(v) ((v) << NDP10X_SPI_INTSTS_WM_INT_SHIFT)
#define NDP10X_SPI_INTSTS_WM_INT_INSERT(x, v)                                  \
    ((x) | ((v) << NDP10X_SPI_INTSTS_WM_INT_SHIFT))
#define NDP10X_SPI_INTSTS_WM_INT_MASK_INSERT(x, v)                             \
    (((x) & ~NDP10X_SPI_INTSTS_WM_INT_MASK)                                    \
        | ((v) << NDP10X_SPI_INTSTS_WM_INT_SHIFT))
#define NDP10X_SPI_INTSTS_WM_INT_EXTRACT(x)                                    \
    (((x)&NDP10X_SPI_INTSTS_WM_INT_MASK) >> NDP10X_SPI_INTSTS_WM_INT_SHIFT)
#define NDP10X_SPI_INTSTS_WM_INT_DEFAULT 0x00000000
#define NDP10X_SPI_INTSTS_DEFAULT 0x00000000
/* register ndp10x_spi.intctl */
#define NDP10X_SPI_INTCTL 0x03
#define NDP10X_SPI_INTCTL_MATCH_INTEN_SHIFT 0
#define NDP10X_SPI_INTCTL_MATCH_INTEN_MASK 0x01
#define NDP10X_SPI_INTCTL_MATCH_INTEN(v)                                       \
    ((v) << NDP10X_SPI_INTCTL_MATCH_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_MATCH_INTEN_INSERT(x, v)                             \
    ((x) | ((v) << NDP10X_SPI_INTCTL_MATCH_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_MATCH_INTEN_MASK_INSERT(x, v)                        \
    (((x) & ~NDP10X_SPI_INTCTL_MATCH_INTEN_MASK)                               \
        | ((v) << NDP10X_SPI_INTCTL_MATCH_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_MATCH_INTEN_EXTRACT(x)                               \
    (((x)&NDP10X_SPI_INTCTL_MATCH_INTEN_MASK)                                  \
        >> NDP10X_SPI_INTCTL_MATCH_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_MATCH_INTEN_DEFAULT 0x00000000
#define NDP10X_SPI_INTCTL_MBIN_INTEN_SHIFT 1
#define NDP10X_SPI_INTCTL_MBIN_INTEN_MASK 0x02
#define NDP10X_SPI_INTCTL_MBIN_INTEN(v)                                        \
    ((v) << NDP10X_SPI_INTCTL_MBIN_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_MBIN_INTEN_INSERT(x, v)                              \
    ((x) | ((v) << NDP10X_SPI_INTCTL_MBIN_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_MBIN_INTEN_MASK_INSERT(x, v)                         \
    (((x) & ~NDP10X_SPI_INTCTL_MBIN_INTEN_MASK)                                \
        | ((v) << NDP10X_SPI_INTCTL_MBIN_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_MBIN_INTEN_EXTRACT(x)                                \
    (((x)&NDP10X_SPI_INTCTL_MBIN_INTEN_MASK)                                   \
        >> NDP10X_SPI_INTCTL_MBIN_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_MBIN_INTEN_DEFAULT 0x00000000
#define NDP10X_SPI_INTCTL_MBOUT_INTEN_SHIFT 2
#define NDP10X_SPI_INTCTL_MBOUT_INTEN_MASK 0x04
#define NDP10X_SPI_INTCTL_MBOUT_INTEN(v)                                       \
    ((v) << NDP10X_SPI_INTCTL_MBOUT_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_MBOUT_INTEN_INSERT(x, v)                             \
    ((x) | ((v) << NDP10X_SPI_INTCTL_MBOUT_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_MBOUT_INTEN_MASK_INSERT(x, v)                        \
    (((x) & ~NDP10X_SPI_INTCTL_MBOUT_INTEN_MASK)                               \
        | ((v) << NDP10X_SPI_INTCTL_MBOUT_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_MBOUT_INTEN_EXTRACT(x)                               \
    (((x)&NDP10X_SPI_INTCTL_MBOUT_INTEN_MASK)                                  \
        >> NDP10X_SPI_INTCTL_MBOUT_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_MBOUT_INTEN_DEFAULT 0x00000000
#define NDP10X_SPI_INTCTL_DNN_INTEN_SHIFT 3
#define NDP10X_SPI_INTCTL_DNN_INTEN_MASK 0x08
#define NDP10X_SPI_INTCTL_DNN_INTEN(v)                                         \
    ((v) << NDP10X_SPI_INTCTL_DNN_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_DNN_INTEN_INSERT(x, v)                               \
    ((x) | ((v) << NDP10X_SPI_INTCTL_DNN_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_DNN_INTEN_MASK_INSERT(x, v)                          \
    (((x) & ~NDP10X_SPI_INTCTL_DNN_INTEN_MASK)                                 \
        | ((v) << NDP10X_SPI_INTCTL_DNN_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_DNN_INTEN_EXTRACT(x)                                 \
    (((x)&NDP10X_SPI_INTCTL_DNN_INTEN_MASK)                                    \
        >> NDP10X_SPI_INTCTL_DNN_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_DNN_INTEN_DEFAULT 0x00000000
#define NDP10X_SPI_INTCTL_FREQ_INTEN_SHIFT 4
#define NDP10X_SPI_INTCTL_FREQ_INTEN_MASK 0x10
#define NDP10X_SPI_INTCTL_FREQ_INTEN(v)                                        \
    ((v) << NDP10X_SPI_INTCTL_FREQ_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_FREQ_INTEN_INSERT(x, v)                              \
    ((x) | ((v) << NDP10X_SPI_INTCTL_FREQ_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_FREQ_INTEN_MASK_INSERT(x, v)                         \
    (((x) & ~NDP10X_SPI_INTCTL_FREQ_INTEN_MASK)                                \
        | ((v) << NDP10X_SPI_INTCTL_FREQ_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_FREQ_INTEN_EXTRACT(x)                                \
    (((x)&NDP10X_SPI_INTCTL_FREQ_INTEN_MASK)                                   \
        >> NDP10X_SPI_INTCTL_FREQ_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_FREQ_INTEN_DEFAULT 0x00000000
#define NDP10X_SPI_INTCTL_AE_INTEN_SHIFT 5
#define NDP10X_SPI_INTCTL_AE_INTEN_MASK 0x20
#define NDP10X_SPI_INTCTL_AE_INTEN(v) ((v) << NDP10X_SPI_INTCTL_AE_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_AE_INTEN_INSERT(x, v)                                \
    ((x) | ((v) << NDP10X_SPI_INTCTL_AE_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_AE_INTEN_MASK_INSERT(x, v)                           \
    (((x) & ~NDP10X_SPI_INTCTL_AE_INTEN_MASK)                                  \
        | ((v) << NDP10X_SPI_INTCTL_AE_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_AE_INTEN_EXTRACT(x)                                  \
    (((x)&NDP10X_SPI_INTCTL_AE_INTEN_MASK) >> NDP10X_SPI_INTCTL_AE_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_AE_INTEN_DEFAULT 0x00000000
#define NDP10X_SPI_INTCTL_WM_INTEN_SHIFT 6
#define NDP10X_SPI_INTCTL_WM_INTEN_MASK 0x40
#define NDP10X_SPI_INTCTL_WM_INTEN(v) ((v) << NDP10X_SPI_INTCTL_WM_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_WM_INTEN_INSERT(x, v)                                \
    ((x) | ((v) << NDP10X_SPI_INTCTL_WM_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_WM_INTEN_MASK_INSERT(x, v)                           \
    (((x) & ~NDP10X_SPI_INTCTL_WM_INTEN_MASK)                                  \
        | ((v) << NDP10X_SPI_INTCTL_WM_INTEN_SHIFT))
#define NDP10X_SPI_INTCTL_WM_INTEN_EXTRACT(x)                                  \
    (((x)&NDP10X_SPI_INTCTL_WM_INTEN_MASK) >> NDP10X_SPI_INTCTL_WM_INTEN_SHIFT)
#define NDP10X_SPI_INTCTL_WM_INTEN_DEFAULT 0x00000000
#define NDP10X_SPI_INTCTL_DEFAULT 0x00000000
/* register ndp10x_spi.ctl */
#define NDP10X_SPI_CTL 0x04
#define NDP10X_SPI_CTL_RESETN_SHIFT 0
#define NDP10X_SPI_CTL_RESETN_MASK 0x01
#define NDP10X_SPI_CTL_RESETN(v) ((v) << NDP10X_SPI_CTL_RESETN_SHIFT)
#define NDP10X_SPI_CTL_RESETN_INSERT(x, v)                                     \
    ((x) | ((v) << NDP10X_SPI_CTL_RESETN_SHIFT))
#define NDP10X_SPI_CTL_RESETN_MASK_INSERT(x, v)                                \
    (((x) & ~NDP10X_SPI_CTL_RESETN_MASK) | ((v) << NDP10X_SPI_CTL_RESETN_SHIFT))
#define NDP10X_SPI_CTL_RESETN_EXTRACT(x)                                       \
    (((x)&NDP10X_SPI_CTL_RESETN_MASK) >> NDP10X_SPI_CTL_RESETN_SHIFT)
#define NDP10X_SPI_CTL_RESETN_DEFAULT 0x00000001
#define NDP10X_SPI_CTL_CLKEN_SHIFT 1
#define NDP10X_SPI_CTL_CLKEN_MASK 0x02
#define NDP10X_SPI_CTL_CLKEN(v) ((v) << NDP10X_SPI_CTL_CLKEN_SHIFT)
#define NDP10X_SPI_CTL_CLKEN_INSERT(x, v)                                      \
    ((x) | ((v) << NDP10X_SPI_CTL_CLKEN_SHIFT))
#define NDP10X_SPI_CTL_CLKEN_MASK_INSERT(x, v)                                 \
    (((x) & ~NDP10X_SPI_CTL_CLKEN_MASK) | ((v) << NDP10X_SPI_CTL_CLKEN_SHIFT))
#define NDP10X_SPI_CTL_CLKEN_EXTRACT(x)                                        \
    (((x)&NDP10X_SPI_CTL_CLKEN_MASK) >> NDP10X_SPI_CTL_CLKEN_SHIFT)
#define NDP10X_SPI_CTL_CLKEN_DEFAULT 0x00000001
#define NDP10X_SPI_CTL_EXTCLK_SHIFT 2
#define NDP10X_SPI_CTL_EXTCLK_MASK 0x04
#define NDP10X_SPI_CTL_EXTCLK(v) ((v) << NDP10X_SPI_CTL_EXTCLK_SHIFT)
#define NDP10X_SPI_CTL_EXTCLK_INSERT(x, v)                                     \
    ((x) | ((v) << NDP10X_SPI_CTL_EXTCLK_SHIFT))
#define NDP10X_SPI_CTL_EXTCLK_MASK_INSERT(x, v)                                \
    (((x) & ~NDP10X_SPI_CTL_EXTCLK_MASK) | ((v) << NDP10X_SPI_CTL_EXTCLK_SHIFT))
#define NDP10X_SPI_CTL_EXTCLK_EXTRACT(x)                                       \
    (((x)&NDP10X_SPI_CTL_EXTCLK_MASK) >> NDP10X_SPI_CTL_EXTCLK_SHIFT)
#define NDP10X_SPI_CTL_EXTCLK_DEFAULT 0x00000000
#define NDP10X_SPI_CTL_BOOTDISABLE_SHIFT 3
#define NDP10X_SPI_CTL_BOOTDISABLE_MASK 0x08
#define NDP10X_SPI_CTL_BOOTDISABLE(v) ((v) << NDP10X_SPI_CTL_BOOTDISABLE_SHIFT)
#define NDP10X_SPI_CTL_BOOTDISABLE_INSERT(x, v)                                \
    ((x) | ((v) << NDP10X_SPI_CTL_BOOTDISABLE_SHIFT))
#define NDP10X_SPI_CTL_BOOTDISABLE_MASK_INSERT(x, v)                           \
    (((x) & ~NDP10X_SPI_CTL_BOOTDISABLE_MASK)                                  \
        | ((v) << NDP10X_SPI_CTL_BOOTDISABLE_SHIFT))
#define NDP10X_SPI_CTL_BOOTDISABLE_EXTRACT(x)                                  \
    (((x)&NDP10X_SPI_CTL_BOOTDISABLE_MASK) >> NDP10X_SPI_CTL_BOOTDISABLE_SHIFT)
#define NDP10X_SPI_CTL_BOOTDISABLE_DEFAULT 0x00000000
#define NDP10X_SPI_CTL_DEFAULT 0x00000003
/* register ndp10x_spi.cfg */
#define NDP10X_SPI_CFG 0x05
#define NDP10X_SPI_CFG_THREE_WIRE_SHIFT 0
#define NDP10X_SPI_CFG_THREE_WIRE_MASK 0x01
#define NDP10X_SPI_CFG_THREE_WIRE(v) ((v) << NDP10X_SPI_CFG_THREE_WIRE_SHIFT)
#define NDP10X_SPI_CFG_THREE_WIRE_INSERT(x, v)                                 \
    ((x) | ((v) << NDP10X_SPI_CFG_THREE_WIRE_SHIFT))
#define NDP10X_SPI_CFG_THREE_WIRE_MASK_INSERT(x, v)                            \
    (((x) & ~NDP10X_SPI_CFG_THREE_WIRE_MASK)                                   \
        | ((v) << NDP10X_SPI_CFG_THREE_WIRE_SHIFT))
#define NDP10X_SPI_CFG_THREE_WIRE_EXTRACT(x)                                   \
    (((x)&NDP10X_SPI_CFG_THREE_WIRE_MASK) >> NDP10X_SPI_CFG_THREE_WIRE_SHIFT)
#define NDP10X_SPI_CFG_THREE_WIRE_DEFAULT 0x00000000
#define NDP10X_SPI_CFG_OPEN_DRAIN_SHIFT 1
#define NDP10X_SPI_CFG_OPEN_DRAIN_MASK 0x02
#define NDP10X_SPI_CFG_OPEN_DRAIN(v) ((v) << NDP10X_SPI_CFG_OPEN_DRAIN_SHIFT)
#define NDP10X_SPI_CFG_OPEN_DRAIN_INSERT(x, v)                                 \
    ((x) | ((v) << NDP10X_SPI_CFG_OPEN_DRAIN_SHIFT))
#define NDP10X_SPI_CFG_OPEN_DRAIN_MASK_INSERT(x, v)                            \
    (((x) & ~NDP10X_SPI_CFG_OPEN_DRAIN_MASK)                                   \
        | ((v) << NDP10X_SPI_CFG_OPEN_DRAIN_SHIFT))
#define NDP10X_SPI_CFG_OPEN_DRAIN_EXTRACT(x)                                   \
    (((x)&NDP10X_SPI_CFG_OPEN_DRAIN_MASK) >> NDP10X_SPI_CFG_OPEN_DRAIN_SHIFT)
#define NDP10X_SPI_CFG_OPEN_DRAIN_DEFAULT 0x00000000
#define NDP10X_SPI_CFG_DRIVE_SHIFT 2
#define NDP10X_SPI_CFG_DRIVE_MASK 0x0c
#define NDP10X_SPI_CFG_DRIVE(v) ((v) << NDP10X_SPI_CFG_DRIVE_SHIFT)
#define NDP10X_SPI_CFG_DRIVE_INSERT(x, v)                                      \
    ((x) | ((v) << NDP10X_SPI_CFG_DRIVE_SHIFT))
#define NDP10X_SPI_CFG_DRIVE_MASK_INSERT(x, v)                                 \
    (((x) & ~NDP10X_SPI_CFG_DRIVE_MASK) | ((v) << NDP10X_SPI_CFG_DRIVE_SHIFT))
#define NDP10X_SPI_CFG_DRIVE_EXTRACT(x)                                        \
    (((x)&NDP10X_SPI_CFG_DRIVE_MASK) >> NDP10X_SPI_CFG_DRIVE_SHIFT)
#define NDP10X_SPI_CFG_DRIVE_DEFAULT 0x00000000
#define NDP10X_SPI_CFG_DRIVE_DRIVE2MA 0x0
#define NDP10X_SPI_CFG_DRIVE_DRIVE4MA 0x1
#define NDP10X_SPI_CFG_DRIVE_DRIVE6MA 0x2
#define NDP10X_SPI_CFG_DRIVE_DRIVE8MA 0x3
#define NDP10X_SPI_CFG_DRIVE_MAX 0x3
#define NDP10X_SPI_CFG_DRIVE_VALID(v) (v >= 0 && v <= 3)
#define NDP10X_SPI_CFG_INTEN_SHIFT 4
#define NDP10X_SPI_CFG_INTEN_MASK 0x10
#define NDP10X_SPI_CFG_INTEN(v) ((v) << NDP10X_SPI_CFG_INTEN_SHIFT)
#define NDP10X_SPI_CFG_INTEN_INSERT(x, v)                                      \
    ((x) | ((v) << NDP10X_SPI_CFG_INTEN_SHIFT))
#define NDP10X_SPI_CFG_INTEN_MASK_INSERT(x, v)                                 \
    (((x) & ~NDP10X_SPI_CFG_INTEN_MASK) | ((v) << NDP10X_SPI_CFG_INTEN_SHIFT))
#define NDP10X_SPI_CFG_INTEN_EXTRACT(x)                                        \
    (((x)&NDP10X_SPI_CFG_INTEN_MASK) >> NDP10X_SPI_CFG_INTEN_SHIFT)
#define NDP10X_SPI_CFG_INTEN_DEFAULT 0x00000000
#define NDP10X_SPI_CFG_INTNEG_SHIFT 5
#define NDP10X_SPI_CFG_INTNEG_MASK 0x20
#define NDP10X_SPI_CFG_INTNEG(v) ((v) << NDP10X_SPI_CFG_INTNEG_SHIFT)
#define NDP10X_SPI_CFG_INTNEG_INSERT(x, v)                                     \
    ((x) | ((v) << NDP10X_SPI_CFG_INTNEG_SHIFT))
#define NDP10X_SPI_CFG_INTNEG_MASK_INSERT(x, v)                                \
    (((x) & ~NDP10X_SPI_CFG_INTNEG_MASK) | ((v) << NDP10X_SPI_CFG_INTNEG_SHIFT))
#define NDP10X_SPI_CFG_INTNEG_EXTRACT(x)                                       \
    (((x)&NDP10X_SPI_CFG_INTNEG_MASK) >> NDP10X_SPI_CFG_INTNEG_SHIFT)
#define NDP10X_SPI_CFG_INTNEG_DEFAULT 0x00000000
#define NDP10X_SPI_CFG_DEFAULT 0x00000000
/* register ndp10x_spi.pll */
#define NDP10X_SPI_PLL 0x06
#define NDP10X_SPI_PLL_PLLRSTB_SHIFT 7
#define NDP10X_SPI_PLL_PLLRSTB_MASK 0x80
#define NDP10X_SPI_PLL_PLLRSTB(v) ((v) << NDP10X_SPI_PLL_PLLRSTB_SHIFT)
#define NDP10X_SPI_PLL_PLLRSTB_INSERT(x, v)                                    \
    ((x) | ((v) << NDP10X_SPI_PLL_PLLRSTB_SHIFT))
#define NDP10X_SPI_PLL_PLLRSTB_MASK_INSERT(x, v)                               \
    (((x) & ~NDP10X_SPI_PLL_PLLRSTB_MASK)                                      \
        | ((v) << NDP10X_SPI_PLL_PLLRSTB_SHIFT))
#define NDP10X_SPI_PLL_PLLRSTB_EXTRACT(x)                                      \
    (((x)&NDP10X_SPI_PLL_PLLRSTB_MASK) >> NDP10X_SPI_PLL_PLLRSTB_SHIFT)
#define NDP10X_SPI_PLL_PLLRSTB_DEFAULT 0x00000001
#define NDP10X_SPI_PLL_DEFAULT 0x00000080
/* register ndp10x_spi.match */
#define NDP10X_SPI_MATCH 0x10
#define NDP10X_SPI_MATCH_WINNER_SHIFT 0
#define NDP10X_SPI_MATCH_WINNER_MASK 0x3f
#define NDP10X_SPI_MATCH_WINNER(v) ((v) << NDP10X_SPI_MATCH_WINNER_SHIFT)
#define NDP10X_SPI_MATCH_WINNER_INSERT(x, v)                                   \
    ((x) | ((v) << NDP10X_SPI_MATCH_WINNER_SHIFT))
#define NDP10X_SPI_MATCH_WINNER_MASK_INSERT(x, v)                              \
    (((x) & ~NDP10X_SPI_MATCH_WINNER_MASK)                                     \
        | ((v) << NDP10X_SPI_MATCH_WINNER_SHIFT))
#define NDP10X_SPI_MATCH_WINNER_EXTRACT(x)                                     \
    (((x)&NDP10X_SPI_MATCH_WINNER_MASK) >> NDP10X_SPI_MATCH_WINNER_SHIFT)
#define NDP10X_SPI_MATCH_MATCH_SHIFT 6
#define NDP10X_SPI_MATCH_MATCH_MASK 0x40
#define NDP10X_SPI_MATCH_MATCH(v) ((v) << NDP10X_SPI_MATCH_MATCH_SHIFT)
#define NDP10X_SPI_MATCH_MATCH_INSERT(x, v)                                    \
    ((x) | ((v) << NDP10X_SPI_MATCH_MATCH_SHIFT))
#define NDP10X_SPI_MATCH_MATCH_MASK_INSERT(x, v)                               \
    (((x) & ~NDP10X_SPI_MATCH_MATCH_MASK)                                      \
        | ((v) << NDP10X_SPI_MATCH_MATCH_SHIFT))
#define NDP10X_SPI_MATCH_MATCH_EXTRACT(x)                                      \
    (((x)&NDP10X_SPI_MATCH_MATCH_MASK) >> NDP10X_SPI_MATCH_MATCH_SHIFT)
#define NDP10X_SPI_MATCH_MULT_SHIFT 7
#define NDP10X_SPI_MATCH_MULT_MASK 0x80
#define NDP10X_SPI_MATCH_MULT(v) ((v) << NDP10X_SPI_MATCH_MULT_SHIFT)
#define NDP10X_SPI_MATCH_MULT_INSERT(x, v)                                     \
    ((x) | ((v) << NDP10X_SPI_MATCH_MULT_SHIFT))
#define NDP10X_SPI_MATCH_MULT_MASK_INSERT(x, v)                                \
    (((x) & ~NDP10X_SPI_MATCH_MULT_MASK) | ((v) << NDP10X_SPI_MATCH_MULT_SHIFT))
#define NDP10X_SPI_MATCH_MULT_EXTRACT(x)                                       \
    (((x)&NDP10X_SPI_MATCH_MULT_MASK) >> NDP10X_SPI_MATCH_MULT_SHIFT)
/* register ndp10x_spi.sample */
#define NDP10X_SPI_SAMPLE 0x20
/* register ndp10x_spi.mbin */
#define NDP10X_SPI_MBIN 0x30
/* register ndp10x_spi.mbin_resp */
#define NDP10X_SPI_MBIN_RESP 0x31
/* register ndp10x_spi.mbout */
#define NDP10X_SPI_MBOUT 0x32
/* register ndp10x_spi.mbout_resp */
#define NDP10X_SPI_MBOUT_RESP 0x33
/* register array ndp10x_spi.maddr[4] */
#define NDP10X_SPI_MADDR(i) (0x40 + ((i) << 0))
#define NDP10X_SPI_MADDR_COUNT 4
/* register array ndp10x_spi.mdata[4] */
#define NDP10X_SPI_MDATA(i) (0x44 + ((i) << 0))
#define NDP10X_SPI_MDATA_COUNT 4

#endif
