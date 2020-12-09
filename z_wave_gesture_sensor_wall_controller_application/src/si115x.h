/***************************************************************************//**
 * @file si115x.h
 * @brief public header for si115x driver
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *******************************************************************************
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/

#ifndef SI115X_H
#define SI115X_H

#include <stdint.h>
#include <stdbool.h>
#include "em_i2c.h"
#include "em_chip.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @addtogroup kitdrv
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Si115x
 * @{
 ******************************************************************************/
/*******************************************************************************
 *****************************   ERROR CODES   *********************************
 ******************************************************************************/
#define _SI115X_ECODE_OK                (0)
#define _SI115X_ECODE_ILLEGAL_HANDLE    (1 << 0)
#define _SI115X_ECODE_PARAM_ERROR       (1 << 1)
#define _SI115X_ECODE_I2C_ERROR         (1 << 2)
#define _SI115X_ECODE_SI115X_ERROR      (1 << 3)
#define _SI115X_ECODE_BUSY              (1 << 4)

typedef enum
{
  SI115X_ECODE_OK = _SI115X_ECODE_OK, /** A successful return value */
  SI115X_ECODE_ILLEGAL_HANDLE = _SI115X_ECODE_ILLEGAL_HANDLE, /** An illegal Si115x handle */
  SI115X_ECODE_PARAM_ERROR = _SI115X_ECODE_PARAM_ERROR, /** An illegal input parameter */
  SI115X_ECODE_I2C_ERROR = _SI115X_ECODE_I2C_ERROR, /** An I2C error */
  SI115X_ECODE_SI115X_ERROR = _SI115X_ECODE_SI115X_ERROR, /** A Si115x CMD_ERR error */
  SI115X_ECODE_BUSY = _SI115X_ECODE_BUSY, /** Si115x is busy */
} SI115X_Ecode_TypeDef;

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** I2C device default address for Si115x */
#define SI115X_I2C_ADDR_DEFAULT                             0x53
/** I2C device alternate address for Si115x. */
#define SI115X_I2C_ADDR_ALT                                 0x52

/** Revision ID for Si115x-AA. */
#define SI115X_REV_ID_AA                                    0x10
/** Revision ID for Si115x-AB. */
#define SI115X_REV_ID_AB                                    0x11

#define _SI115X_MAX_CHANNELS                                6
#define _SI115X_COMMAND_TIMEOUT                             10
#define _SI115X_MAX_BLOCK_WRITE_LENGTH                      8
#define _SI115X_MAX_REGISTER_ADDR                           0x2C

/* Commands opcodes */
#define SI115X_CMD_RESET_CMD_CTR                            0x00
#define SI115X_CMD_RESET_SW                                 0x01
#define SI115X_CMD_FORCE                                    0x11
#define SI115X_CMD_PAUSE                                    0x12
#define SI115X_CMD_START                                    0x13

#define SI115X_CMD_PARAM_ADDR_MASK                          0x3F
#define SI115X_CMD_PARAM_QUERY_MASK                         0x40
#define SI115X_CMD_PARAM_SET_MASK                           0x80

/* I2C register address for Si115x. */
#define SI115X_REG_PART_ID                                  0x00
#define SI115X_REG_HW_ID                                    0x01
#define SI115X_REG_REV_ID                                   0x02
#define SI115X_REG_HOSTIN0                                  0x0A
#define SI115X_REG_COMMAND                                  0x0B
#define SI115X_REG_IRQENABLE                                0x0F
#define SI115X_REG_RESPONSE1                                0x10
#define SI115X_REG_RESPONSE0                                0x11
#define SI115X_REG_IRQ_STATUS                               0x12
#define SI115X_REG_HOSTOUT0                                 0x13
#define SI115X_REG_HOSTOUT1                                 0x14
#define SI115X_REG_HOSTOUT2                                 0x15
#define SI115X_REG_HOSTOUT3                                 0x16
#define SI115X_REG_HOSTOUT4                                 0x17
#define SI115X_REG_HOSTOUT5                                 0x18
#define SI115X_REG_HOSTOUT6                                 0x19
#define SI115X_REG_HOSTOUT7                                 0x1A
#define SI115X_REG_HOSTOUT8                                 0x1B
#define SI115X_REG_HOSTOUT9                                 0x1C
#define SI115X_REG_HOSTOUT10                                0x1D
#define SI115X_REG_HOSTOUT11                                0x1E
#define SI115X_REG_HOSTOUT12                                0x1F
#define SI115X_REG_HOSTOUT13                                0x20
#define SI115X_REG_HOSTOUT14                                0x21
#define SI115X_REG_HOSTOUT15                                0x22
#define SI115X_REG_HOSTOUT16                                0x23
#define SI115X_REG_HOSTOUT17                                0x24
#define SI115X_REG_HOSTOUT18                                0x25
#define SI115X_REG_HOSTOUT19                                0x26
#define SI115X_REG_HOSTOUT20                                0x27
#define SI115X_REG_HOSTOUT21                                0x28
#define SI115X_REG_HOSTOUT22                                0x29
#define SI115X_REG_HOSTOUT23                                0x2A
#define SI115X_REG_HOSTOUT24                                0x2B
#define SI115X_REG_HOSTOUT25                                0x2C

/* Bit fields for RESPONSE0 */
#define _SI115X_RESPONSE0_CMD_CTR_SHIFT                     0
#define _SI115X_RESPONSE0_CMD_CTR_MASK                      0xF
#define _SI115X_RESPONSE0_CMD_CTR_ERR_INVALID_CMD           0x0
#define _SI115X_RESPONSE0_CMD_CTR_ERR_INVALID_PARAM         0x1
#define _SI115X_RESPONSE0_CMD_CTR_ERR_SATURATION            0x2
#define _SI115X_RESPONSE0_CMD_CTR_ERR_OVERFLOW              0x3
#define SI115X_RESPONSE0_CMD_CTR_ERR_INVALID_CMD            (_SI115X_RESPONSE0_CMD_CTR_ERR_INVALID_CMD << 0)
#define SI115X_RESPONSE0_CMD_CTR_ERR_INVALID_PARAM          (_SI115X_RESPONSE0_CMD_CTR_ERR_INVALID_PARAM << 0)
#define SI115X_RESPONSE0_CMD_CTR_ERR_SATURATION             (_SI115X_RESPONSE0_CMD_CTR_ERR_SATURATION << 0)
#define SI115X_RESPONSE0_CMD_CTR_ERR_OVERFLOW               (_SI115X_RESPONSE0_CMD_CTR_ERR_OVERFLOW << 0)

#define _SI115X_RESPONSE0_CMD_ERR_SHIFT                     4
#define _SI115X_RESPONSE0_CMD_ERR_MASK                      0x10

#define _SI115X_RESPONSE0_SLEEP_SHIFT                       5
#define _SI115X_RESPONSE0_SLEEP_MASK                        0x20
#define _SI115X_RESPONSE0_SLEEP                             0x20

#define _SI115X_RESPONSE0_SUSPEND_SHIFT                     6
#define _SI115X_RESPONSE0_SUSPEND_MASK                      0x40
#define _SI115X_RESPONSE0_SUSPEND                           0x40

#define _SI115X_RESPONSE0_RUNNING_SHIFT                     7
#define _SI115X_RESPONSE0_RUNNING_MASK                      0x80
#define _SI115X_RESPONSE0_RUNNING                           0x80

#define _SI115X_I2C_ADDR_RESETVALUE                         // todo
#define _SI115X_I2C_ADDR_MASK                               // todo
#define _SI115X_CHAN_LIST_RESETVALUE                        0x00
#define _SI115X_CHAN_LIST_MASK                              0x3F
#define _SI115X_CHAN_LIST_CHAN0_EN_SHIFT                    0
#define _SI115X_CHAN_LIST_CHAN0_EN_MASK                     0x01
#define _SI115X_CHAN_LIST_CHAN1_EN_SHIFT                    1
#define _SI115X_CHAN_LIST_CHAN1_EN_MASK                     0x02
#define _SI115X_CHAN_LIST_CHAN2_EN_SHIFT                    2
#define _SI115X_CHAN_LIST_CHAN2_EN_MASK                     0x04
#define _SI115X_CHAN_LIST_CHAN3_EN_SHIFT                    3
#define _SI115X_CHAN_LIST_CHAN3_EN_MASK                     0x08
#define _SI115X_CHAN_LIST_CHAN4_EN_SHIFT                    4
#define _SI115X_CHAN_LIST_CHAN4_EN_MASK                     0x10
#define _SI115X_CHAN_LIST_CHAN5_EN_SHIFT                    5
#define _SI115X_CHAN_LIST_CHAN5_EN_MASK                     0x20

/** Parameter table addresses. */
#define SI115X_CHAN_LIST                                    0x01
#define SI115X_MEASRATE_H                                   0x1A
#define SI115X_MEASRATE_L                                   0x1B
#define SI115X_MEASCOUNT0                                   0x1C
#define SI115X_MEASCOUNT1                                   0x1D
#define SI115X_MEASCOUNT2                                   0x1E
#define SI115X_LED1_A                                       0x1F
#define SI115X_LED1_B                                       0x20
#define SI115X_LED3_A                                       0x21
#define SI115X_LED3_B                                       0x22
#define SI115X_LED2_A                                       0x23
#define SI115X_LED2_B                                       0x24
#define SI115X_THRESHOLD0_H                                 0x25
#define SI115X_THRESHOLD0_L                                 0x26
#define SI115X_THRESHOLD1_H                                 0x27
#define SI115X_THRESHOLD1_L                                 0x28
#define SI115X_UPPER_THRESHOLD_H                            0x29
#define SI115X_UPPER_THRESHOLD_L                            0x2A
#define SI115X_BURST                                        0x2B
#define SI115X_LOWER_THRESHOLD_H                            0x2C
#define SI115X_LOWER_THRESHOLD_L                            0x2D

#define _SI115X_MEASRATE_MASK                               0x0FFF

#define _SI115X_BURST_RESETVALUE                            0x00
#define _SI115X_BURST_MASK                                  0xFF
#define _SI115X_BURST_BURST_EN_SHIFT                        7
#define _SI115X_BURST_BURST_EN_MASK                         0x80
#define _SI115X_BURST_BURST_COUNT_SHIFT                     0
#define _SI115X_BURST_BURST_COUNT_MASK                      0x7F

//LED current
#define SI115X_LED_CURRENT_6MA                              0x00
#define SI115X_LED_CURRENT_11MA                             0x08
#define SI115X_LED_CURRENT_17MA                             0x10
#define SI115X_LED_CURRENT_22MA                             0x18
#define SI115X_LED_CURRENT_28MA                             0x20
#define SI115X_LED_CURRENT_33MA                             0x28
#define SI115X_LED_CURRENT_39MA                             0x30
#define SI115X_LED_CURRENT_44MA                             0x38
#define SI115X_LED_CURRENT_50MA                             0x12
#define SI115X_LED_CURRENT_55MA                             0x21
#define SI115X_LED_CURRENT_66MA                             0x29
#define SI115X_LED_CURRENT_77MA                             0x31
#define SI115X_LED_CURRENT_83MA                             0x22
#define SI115X_LED_CURRENT_88MA                             0x39
#define SI115X_LED_CURRENT_100MA                            0x2A
#define SI115X_LED_CURRENT_111MA                            0x23
#define SI115X_LED_CURRENT_116MA                            0x32
#define SI115X_LED_CURRENT_133MA                            0x3A
#define SI115X_LED_CURRENT_138MA                            0x24
#define SI115X_LED_CURRENT_155MA                            0x33
#define SI115X_LED_CURRENT_166MA                            0x2C
#define SI115X_LED_CURRENT_177MA                            0x3B
#define SI115X_LED_CURRENT_194MA                            0x34
#define SI115X_LED_CURRENT_199MA                            0x2D
#define SI115X_LED_CURRENT_221MA                            0x3C
#define SI115X_LED_CURRENT_232MA                            0x35
#define SI115X_LED_CURRENT_265MA                            0x3D
#define SI115X_LED_CURRENT_271MA                            0x36
#define SI115X_LED_CURRENT_310MA                            0x3E
#define SI115X_LED_CURRENT_354MA                            0x3F

#define _SI115X_CHANNEL_CONFIG_OFFSET                       2
#define _SI115X_CHANNEL_CONFIG_LENGTH                       4
#define _SI115X_ADCCONFIG_OFFSET                            0
#define _SI115X_ADCSENS_OFFSET                              1
#define _SI115X_ADCPOST_OFFSET                              2
#define _SI115X_MEASCONFIG_OFFSET                           3

#define _SI115X_ADCCONFIG0                                  0x02
#define _SI115X_ADCCONFIG1                                  0x06
#define _SI115X_ADCCONFIG2                                  0x0A
#define _SI115X_ADCCONFIG3                                  0x0E
#define _SI115X_ADCCONFIG4                                  0x12
#define _SI115X_ADCCONFIG5                                  0x16

#define _SI115X_ADCSENS0                                    0x03
#define _SI115X_ADCSENS1                                    0x07
#define _SI115X_ADCSENS2                                    0x0B
#define _SI115X_ADCSENS3                                    0x0F
#define _SI115X_ADCSENS4                                    0x13
#define _SI115X_ADCSENS5                                    0x17

#define _SI115X_ADCPOST0                                    0x04
#define _SI115X_ADCPOST1                                    0x08
#define _SI115X_ADCPOST2                                    0x0C
#define _SI115X_ADCPOST3                                    0x10
#define _SI115X_ADCPOST4                                    0x14
#define _SI115X_ADCPOST5                                    0x18

#define _SI115X_MEASCONFIG0                                 0x05
#define _SI115X_MEASCONFIG1                                 0x09
#define _SI115X_MEASCONFIG2                                 0x0D
#define _SI115X_MEASCONFIG3                                 0x11
#define _SI115X_MEASCONFIG4                                 0x15
#define _SI115X_MEASCONFIG5                                 0x19

/* Bit fields for ADCCONFIGx */
#define _SI115X_ADCCONFIG_RESETVALUE                        0x00
#define _SI115X_ADCCONFIG_MASK                              0x7F
#define _SI115X_ADCCONFIG_ADCMUX_SHIFT                      0
#define _SI115X_ADCCONFIG_ADCMUX_MASK                       0x1F
#define _SI115X_ADCCONFIG_ADCMUX_DEFAULT                    0
#define _SI115X_ADCCONFIG_ADCMUX_SMALL_IR                   0x0
#define _SI115X_ADCCONFIG_ADCMUX_MEDIUM_IR                  0x1
#define _SI115X_ADCCONFIG_ADCMUX_LARGE_IR                   0x2
#define _SI115X_ADCCONFIG_ADCMUX_VISIBLE                    0xB
#define _SI115X_ADCCONFIG_ADCMUX_LARGE_VISIBLE              0xD
#define SI115X_ADCCONFIG_ADCMUX_DEFAULT                     (_SI115X_ADCCONFIG_ADCMUX_DEFAULT << 0)
#define SI115X_ADCCONFIG_ADCMUX_SMALL_IR                    (_SI115X_ADCCONFIG_ADCMUX_SMALL_IR << 0)
#define SI115X_ADCCONFIG_ADCMUX_MEDIUM_IR                   (_SI115X_ADCCONFIG_ADCMUX_MEDIUM_IR << 0)
#define SI115X_ADCCONFIG_ADCMUX_LARGE_IR                    (_SI115X_ADCCONFIG_ADCMUX_LARGE_IR << 0)
#define SI115X_ADCCONFIG_ADCMUX_VISIBLE                     (_SI115X_ADCCONFIG_ADCMUX_VISIBLE << 0)
#define SI115X_ADCCONFIG_ADCMUX_LARGE_VISIBLE               (_SI115X_ADCCONFIG_ADCMUX_LARGE_VISIBLE << 0)

#define _SI115X_ADCCONFIG_DECIM_RATE_MASK                   0x60
#define _SI115X_ADCCONFIG_DECIM_RATE_SHIFT                  5
#define _SI115X_ADCCONFIG_DECIM_RATE_DEFAULT                0
#define _SI115X_ADCCONFIG_DECIM_RATE_1024                   0
#define _SI115X_ADCCONFIG_DECIM_RATE_2048                   1
#define _SI115X_ADCCONFIG_DECIM_RATE_4096                   2
#define _SI115X_ADCCONFIG_DECIM_RATE_512                    3
#define SI115X_ADCCONFIG_DECIM_RATE_DEFAULT                 (_SI115X_ADCCONFIG_DECIM_RATE_DEFAULT << 5)
#define SI115X_ADCCONFIG_DECIM_RATE_1024                    (_SI115X_ADCCONFIG_DECIM_RATE_1024 << 5)
#define SI115X_ADCCONFIG_DECIM_RATE_2048                    (_SI115X_ADCCONFIG_DECIM_RATE_2048 << 5)
#define SI115X_ADCCONFIG_DECIM_RATE_4096                    (_SI115X_ADCCONFIG_DECIM_RATE_4096 << 5)
#define SI115X_ADCCONFIG_DECIM_RATE_512                     (_SI115X_ADCCONFIG_DECIM_RATE_512 << 5)

/* Bit fields for ADCSENSx */
#define _SI115X_ADCSENS_RESETVALUE                          0x00
#define _SI115X_ADCSENS_MASK                                0xFF

#define _SI115X_ADCSENS_HW_GAIN_SHIFT                       0
#define _SI115X_ADCSENS_HW_GAIN_MASK                        0x0F
#define _SI115X_ADCSENS_HW_GAIN_DEFAULT                     0
#define _SI115X_ADCSENS_HW_GAIN_1X                          0
#define _SI115X_ADCSENS_HW_GAIN_2X                          1
#define _SI115X_ADCSENS_HW_GAIN_4X                          2
#define _SI115X_ADCSENS_HW_GAIN_8X                          3
#define _SI115X_ADCSENS_HW_GAIN_16X                         4
#define _SI115X_ADCSENS_HW_GAIN_32X                         5
#define _SI115X_ADCSENS_HW_GAIN_64X                         6
#define _SI115X_ADCSENS_HW_GAIN_128X                        7
#define _SI115X_ADCSENS_HW_GAIN_256X                        8
#define _SI115X_ADCSENS_HW_GAIN_512X                        9
#define _SI115X_ADCSENS_HW_GAIN_1024X                       10
#define _SI115X_ADCSENS_HW_GAIN_2048X                       11
#define SI115X_ADCSENS_HW_GAIN_DEFAULT                      (_SI115X_ADCSENS_HW_GAIN_DEFAULT << 0)
#define SI115X_ADCSENS_HW_GAIN_1X                           (_SI115X_ADCSENS_HW_GAIN_1X << 0)
#define SI115X_ADCSENS_HW_GAIN_2X                           (_SI115X_ADCSENS_HW_GAIN_2X << 0)
#define SI115X_ADCSENS_HW_GAIN_4X                           (_SI115X_ADCSENS_HW_GAIN_4X << 0)
#define SI115X_ADCSENS_HW_GAIN_8X                           (_SI115X_ADCSENS_HW_GAIN_8X << 0)
#define SI115X_ADCSENS_HW_GAIN_16X                          (_SI115X_ADCSENS_HW_GAIN_16X << 0)
#define SI115X_ADCSENS_HW_GAIN_32X                          (_SI115X_ADCSENS_HW_GAIN_32X << 0)
#define SI115X_ADCSENS_HW_GAIN_64X                          (_SI115X_ADCSENS_HW_GAIN_64X << 0)
#define SI115X_ADCSENS_HW_GAIN_128X                         (_SI115X_ADCSENS_HW_GAIN_128X << 0)
#define SI115X_ADCSENS_HW_GAIN_256X                         (_SI115X_ADCSENS_HW_GAIN_256X << 0)
#define SI115X_ADCSENS_HW_GAIN_512X                         (_SI115X_ADCSENS_HW_GAIN_512X << 0)
#define SI115X_ADCSENS_HW_GAIN_1024X                        (_SI115X_ADCSENS_HW_GAIN_1024X << 0)
#define SI115X_ADCSENS_HW_GAIN_2048X                        (_SI115X_ADCSENS_HW_GAIN_2048X << 0)

#define _SI115X_ADCSENS_SW_GAIN_SHIFT                       4
#define _SI115X_ADCSENS_SW_GAIN_MASK                        0x70
#define _SI115X_ADCSENS_SW_GAIN_DEFAULT                     0
#define _SI115X_ADCSENS_SW_GAIN_1X                          0
#define _SI115X_ADCSENS_SW_GAIN_2X                          1
#define _SI115X_ADCSENS_SW_GAIN_4X                          2
#define _SI115X_ADCSENS_SW_GAIN_8X                          3
#define _SI115X_ADCSENS_SW_GAIN_16X                         4
#define _SI115X_ADCSENS_SW_GAIN_32X                         5
#define _SI115X_ADCSENS_SW_GAIN_64X                         6
#define _SI115X_ADCSENS_SW_GAIN_128X                        7
#define SI115X_ADCSENS_SW_GAIN_DEFAULT                      (_SI115X_ADCSENS_SW_GAIN_DEFAULT << 4)
#define SI115X_ADCSENS_SW_GAIN_1X                           (_SI115X_ADCSENS_SW_GAIN_1X << 4)
#define SI115X_ADCSENS_SW_GAIN_2X                           (_SI115X_ADCSENS_SW_GAIN_2X << 4)
#define SI115X_ADCSENS_SW_GAIN_4X                           (_SI115X_ADCSENS_SW_GAIN_4X << 4)
#define SI115X_ADCSENS_SW_GAIN_8X                           (_SI115X_ADCSENS_SW_GAIN_8X << 4)
#define SI115X_ADCSENS_SW_GAIN_16X                          (_SI115X_ADCSENS_SW_GAIN_16X << 4)
#define SI115X_ADCSENS_SW_GAIN_32X                          (_SI115X_ADCSENS_SW_GAIN_32X << 4)
#define SI115X_ADCSENS_SW_GAIN_64X                          (_SI115X_ADCSENS_SW_GAIN_64X << 4)
#define SI115X_ADCSENS_SW_GAIN_128X                         (_SI115X_ADCSENS_SW_GAIN_128X << 4)

#define _SI115X_ADCSENS_HSIG_SHIFT                          7
#define _SI115X_ADCSENS_HSIG_MASK                           0x80
#define _SI115X_ADCSENS_HSIG_DEFAULT                        0
#define _SI115X_ADCSENS_HSIG_NORMAL_RANGE                   0
#define _SI115X_ADCSENS_HSIG_HIGH_RANGE                     1
#define SI115X_ADCSENS_HSIG_DEFAULT                         (_SI115X_ADCSENS_HSIG_DEFAULT << 7)
#define SI115X_ADCSENS_HSIG_NORMAL_RANGE                    (_SI115X_ADCSENS_HSIG_NORMAL_RANGE << 7)
#define SI115X_ADCSENS_HSIG_HIGH_RANGE                      (_SI115X_ADCSENS_HSIG_HIGH_RANGE << 7)

/* Bit fields for ADCPOST */
#define _SI115X_ADCPOST_RESETVALUE                          0x00
#define _SI115X_ADCPOST_MASK                                0x7F

#define _SI115X_ADCPOST_THRESH_EN_SHIFT                     0
#define _SI115X_ADCPOST_THRESH_EN_MASK                      0x3
#define _SI115X_ADCPOST_THRESH_EN_DEFAULT                   0
#define _SI115X_ADCPOST_THRESH_EN_DISABLED                  0
#define _SI115X_ADCPOST_THRESH_EN_THRESHOLD0                1
#define _SI115X_ADCPOST_THRESH_EN_THRESHOLD1                2
#define _SI115X_ADCPOST_THRESH_EN_THRESHOLD_WINDOW          3
#define SI115X_ADCPOST_THRESH_EN_DEFAULT                    (_SI115X_ADCPOST_THRESH_EN_DEFAULT << 0)
#define SI115X_ADCPOST_THRESH_EN_DISABLED                   (_SI115X_ADCPOST_THRESH_EN_DISABLED << 0)
#define SI115X_ADCPOST_THRESH_EN_THRESHOLD0                 (_SI115X_ADCPOST_THRESH_EN_THRESHOLD0 << 0)
#define SI115X_ADCPOST_THRESH_EN_THRESHOLD1                 (_SI115X_ADCPOST_THRESH_EN_THRESHOLD1 << 0)
#define SI115X_ADCPOST_THRESH_EN_THRESHOLD_WINDOW           (_SI115X_ADCPOST_THRESH_EN_THRESHOLD_WINDOW << 0)

#define _SI115X_ADCPOST_THRESH_POL_SHIFT                    2
#define _SI115X_ADCPOST_THRESH_POL_MASK                     0x4
#define _SI115X_ADCPOST_THRESH_POL_DEFAULT                  0
#define _SI115X_ADCPOST_THRESH_POL_LARGER                   0
#define _SI115X_ADCPOST_THRESH_POL_SMALLER                  1
#define SI115X_ADCPOST_THRESH_POL_DEFAULT                   (_SI115X_ADCPOST_THRESH_POL_DEFAULT << 2)
#define SI115X_ADCPOST_THRESH_POL_LARGER                    (_SI115X_ADCPOST_THRESH_POL_LARGER << 2)
#define SI115X_ADCPOST_THRESH_POL_SMALLER                   (_SI115X_ADCPOST_THRESH_POL_SMALLER << 2)

#define _SI115X_ADCPOST_POSTSHIFT_SHIFT                     3
#define _SI115X_ADCPOST_POSTSHIFT_MASK                      0x38
#define _SI115X_ADCPOST_POSTSHIFT_DEFAULT                   0
#define SI115X_ADCPOST_POSTSHIFT_DEFAULT                    (_SI115X_ADCPOST_POSTSHIFT_DEFAULT << 3)

#define _SI115X_ADCPOST_24BIT_OUT_SHIFT                     6
#define _SI115X_ADCPOST_24BIT_OUT_MASK                      0x40
#define _SI115X_ADCPOST_24BIT_OUT_DEFAULT                   0
#define _SI115X_ADCPOST_24BIT_OUT_16B_UINT                  0
#define _SI115X_ADCPOST_24BIT_OUT_24B_INT                   1
#define SI115X_ADCPOST_24BIT_OUT_DEFAULT                    (_SI115X_ADCPOST_24BIT_OUT_DEFAULT << 6)
#define SI115X_ADCPOST_24BIT_OUT_16B_UINT                   (_SI115X_ADCPOST_24BIT_OUT_16B_UINT << 6)
#define SI115X_ADCPOST_24BIT_OUT_24B_INT                    (_SI115X_ADCPOST_24BIT_OUT_24B_INT << 6)

/* Bit fields for MEASCONFIG */
#define _SI115X_MEASCONFIG_RESETVALUE                       0x00
#define _SI115X_MEASCONFIG_MASK                             0xFF
#define _SI115X_MEASCONFIG_LED1_EN_SHIFT                    0
#define _SI115X_MEASCONFIG_LED1_EN_MASK                     0x1
#define _SI115X_MEASCONFIG_LED3_EN_SHIFT                    1
#define _SI115X_MEASCONFIG_LED3_EN_MASK                     0x2
#define _SI115X_MEASCONFIG_LED2_EN_SHIFT                    2
#define _SI115X_MEASCONFIG_LED2_EN_MASK                     0x4

#define _SI115X_MEASCONFIG_BANK_SEL_SHIFT                   3
#define _SI115X_MEASCONFIG_BANK_SEL_MASK                    0x4
#define _SI115X_MEASCONFIG_BANK_SEL_DEFAULT                 0
#define _SI115X_MEASCONFIG_BANK_SEL_BANK_A                  0
#define _SI115X_MEASCONFIG_BANK_SEL_BANK_B                  1
#define SI115X_MEASCONFIG_BANK_SEL_DEFAULT                  (_SI115X_MEASCONFIG_BANK_SEL_DEFAULT << 3)
#define SI115X_MEASCONFIG_BANK_SEL_BANK_A                   (_SI115X_MEASCONFIG_BANK_SEL_BANK_A << 3)
#define SI115X_MEASCONFIG_BANK_SEL_BANK_B                   (_SI115X_MEASCONFIG_BANK_SEL_BANK_B << 3)

#define _SI115X_MEASCONFIG_COUNTER_INDEX_SHIFT              6
#define _SI115X_MEASCONFIG_COUNTER_INDEX_MASK               0xC0
#define _SI115X_MEASCONFIG_COUNTER_INDEX_DEFAULT            0
#define _SI115X_MEASCONFIG_COUNTER_INDEX_DISABLED           0
#define _SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT0         1
#define _SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT1         2
#define _SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT2         3

#define SI115X_MEASCONFIG_COUNTER_INDEX_DEFAULT             (_SI115X_MEASCONFIG_COUNTER_INDEX_DEFAULT << 6)
#define SI115X_MEASCONFIG_COUNTER_INDEX_DISABLED            (_SI115X_MEASCONFIG_COUNTER_INDEX_DISABLED << 6)
#define SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT0          (_SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT0 << 6)
#define SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT1          (_SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT1 << 6)
#define SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT2          (_SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT2 << 6)

/*******************************************************************************
 ********************************   ENUMS   ************************************
 ******************************************************************************/

/** ADC decimation rate (in ADC clock cycles). */
typedef enum
{
  /** 1024 ADC decimation rate. */
  si115xDecim1024 = SI115X_ADCCONFIG_DECIM_RATE_1024,

  /** 2048 ADC decimation rate. */
  si115xDecim2048 = SI115X_ADCCONFIG_DECIM_RATE_2048,

  /** 2048 ADC decimation rate. */
  si115xDecim4096 = SI115X_ADCCONFIG_DECIM_RATE_4096,

  /** 512 ADC decimation rate. */
  si115xDecim512 = SI115X_ADCCONFIG_DECIM_RATE_512,
} SI115X_Decim_TypeDef;

/** ADC photodiode selection. */
typedef enum
{
  /** Small IR photo diode. */
  si115xAdcMuxSmallIr = SI115X_ADCCONFIG_ADCMUX_SMALL_IR,

  /** Medium IR photo diode. */
  si115xAdcMuxMediumIr = SI115X_ADCCONFIG_ADCMUX_MEDIUM_IR,

  /** Large IR photo diode. */
  si115xAdcMuxLargeIr = SI115X_ADCCONFIG_ADCMUX_LARGE_IR,

  /** Visible photo Diode. */
  si115xAdcMuxSmallVisible = SI115X_ADCCONFIG_ADCMUX_VISIBLE,

  /** Large visible photo diode. */
  si115xAdcMuxLargeVisible = SI115X_ADCCONFIG_ADCMUX_LARGE_VISIBLE,
} SI115X_AdcMux_TypeDef;

/** ADC signal range selection. */
typedef enum
{
  /** Normal signal range. */
  si115xHsigNormal = SI115X_ADCSENS_HSIG_NORMAL_RANGE,

  /** High signal range. */
  si115xHsigHigh = SI115X_ADCSENS_HSIG_HIGH_RANGE,
} SI115X_Hsig_TypeDef;

/** ADC software accumulation. */
typedef enum
{
  /** Software gain of 1. */
  si115xSwGain1x = SI115X_ADCSENS_SW_GAIN_1X,

  /** Software gain of 2. */
  si115xSwGain2x = SI115X_ADCSENS_SW_GAIN_2X,

  /** Software gain of 4. */
  si115xSwGain4x = SI115X_ADCSENS_SW_GAIN_4X,

  /** Software gain of 8. */
  si115xSwGain8x = SI115X_ADCSENS_SW_GAIN_8X,

  /** Software gain of 16. */
  si115xSwGain16x = SI115X_ADCSENS_SW_GAIN_16X,

  /** Software gain of 32. */
  si115xSwGain32x = SI115X_ADCSENS_SW_GAIN_32X,

  /** Software gain of 64. */
  si115xSwGain64x = SI115X_ADCSENS_SW_GAIN_64X,

  /** Software gain of 128. */
  si115xSwGain128x = SI115X_ADCSENS_SW_GAIN_128X,
} SI115X_SwGain_TypeDef;

/** ADC hardware gain. */
typedef enum
{
  /** Hardware gain of 1. */
  si115xHwGain1x = SI115X_ADCSENS_HW_GAIN_1X,

  /** Hardware gain of 2. */
  si115xHwGain2x = SI115X_ADCSENS_HW_GAIN_2X,

  /** Hardware gain of 4. */
  si115xHwGain4x = SI115X_ADCSENS_HW_GAIN_4X,

  /** Hardware gain of 8. */
  si115xHwGain8x = SI115X_ADCSENS_HW_GAIN_8X,

  /** Hardware gain of 16. */
  si115xHwGain16x = SI115X_ADCSENS_HW_GAIN_16X,

  /** Hardware gain of 32. */
  si115xHwGain32x = SI115X_ADCSENS_HW_GAIN_32X,

  /** Hardware gain of 64. */
  si115xHwGain64x = SI115X_ADCSENS_HW_GAIN_64X,

  /** Hardware gain of 128. */
  si115xHwGain128x = SI115X_ADCSENS_HW_GAIN_128X,

  /** Hardware gain of 256. */
  si115xHwGain256x = SI115X_ADCSENS_HW_GAIN_256X,

  /** Hardware gain of 512. */
  si115xHwGain512x = SI115X_ADCSENS_HW_GAIN_512X,

  /** Hardware gain of 1024. */
  si115xHwGain1024x = SI115X_ADCSENS_HW_GAIN_1024X,

  /** Hardware gain of 2048. */
  si115xHwGain2048x = SI115X_ADCSENS_HW_GAIN_2048X,

} SI115X_HwGain_TypeDef;

/** ADC output format. */
typedef enum
{
  /** 16-bit unsigned output. */
  si115xRes16bUint = SI115X_ADCPOST_24BIT_OUT_16B_UINT,

  /** 24-bit signed output. */
  si115xRes24bInt = SI115X_ADCPOST_24BIT_OUT_24B_INT,
} SI115X_Res_TypeDef;

/** ADC threshold polarity. */
typedef enum
{
  /** Threshold triggers when result is larger than threshold. */
  si115xThreshPolLarger = SI115X_ADCPOST_THRESH_POL_LARGER,

  /** Threshold triggers when result is smaller than threshold. */
  si115xThreshPolSmaller = SI115X_ADCPOST_THRESH_POL_SMALLER,
} SI115X_ThreshPol_TypeDef;

/** ADC threshold type. */
typedef enum
{
  /** Threshold is disabled. */
  si115xThreshEnNone = SI115X_ADCPOST_THRESH_EN_DISABLED,

  /** Threshold uses value in THRESHOLD0. */
  si115xThreshEnThreshold0 = SI115X_ADCPOST_THRESH_EN_THRESHOLD0,

  /** Threshold uses value in THRESHOLD1. */
  si115xThreshEnThreshold1 = SI115X_ADCPOST_THRESH_EN_THRESHOLD1,

  /** Threshold uses window threshold in UPPER_THRESHOLD and LOWER_THRESHOLD. */
  si115xThreshEnWindow = SI115X_ADCPOST_THRESH_EN_THRESHOLD_WINDOW,
} SI115X_ThreshEn_TypeDef;

/** ADC measurement counter selection. */
typedef enum
{
  /** Channel is disabled in Autonomous Mode. */
  si115xCounterIndexDisabled = SI115X_MEASCONFIG_COUNTER_INDEX_DISABLED,

  /** Channel uses MEASCOUNT0 for Autonomous Mode. */
  si115xCounterIndexMeascount0 = SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT0,

  /** Channel uses MEASCOUNT1 for Autonomous Mode. */
  si115xCounterIndexMeascount1 = SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT1,

  /** Channel uses MEASCOUNT2 for Autonomous Mode. */
  si115xCounterIndexMeascount2 = SI115X_MEASCONFIG_COUNTER_INDEX_MEASCOUNT2,
} SI115X_CounterIndex_TypeDef;

/** Channel LED bank selection. */
typedef enum
{
  /** Channel uses LED currents from LED_A bank. */
  si115xBankSelLedA = SI115X_MEASCONFIG_BANK_SEL_BANK_A,

  /** Channel uses LED currents from LED_B bank. */
  si115xBankSelLedB = SI115X_MEASCONFIG_BANK_SEL_BANK_B,
} SI115X_BankSel_TypeDef;

/** Global LED current selection. */
typedef enum
{
  si115xLedCurrent6mA = SI115X_LED_CURRENT_6MA, /**< LED current 5.5mA */
  si115xLedCurrent11mA = SI115X_LED_CURRENT_11MA, /**< LED current 11mA */
  si115xLedCurrent17mA = SI115X_LED_CURRENT_17MA, /**< LED current 17mA */
  si115xLedCurrent22mA = SI115X_LED_CURRENT_22MA, /**< LED current 22mA */
  si115xLedCurrent28mA = SI115X_LED_CURRENT_28MA, /**< LED current 28mA */
  si115xLedCurrent33mA = SI115X_LED_CURRENT_33MA, /**< LED current 33mA */
  si115xLedCurrent39mA = SI115X_LED_CURRENT_39MA, /**< LED current 39mA */
  si115xLedCurrent44mA = SI115X_LED_CURRENT_44MA, /**< LED current 44mA */
  si115xLedCurrent50mA = SI115X_LED_CURRENT_50MA, /**< LED current 50mA */
  si115xLedCurrent55mA = SI115X_LED_CURRENT_55MA, /**< LED current 55mA */
  si115xLedCurrent66mA = SI115X_LED_CURRENT_66MA, /**< LED current 66mA */
  si115xLedCurrent77mA = SI115X_LED_CURRENT_77MA, /**< LED current 77mA */
  si115xLedCurrent83mA = SI115X_LED_CURRENT_83MA, /**< LED current 83mA */
  si115xLedCurrent88mA = SI115X_LED_CURRENT_88MA, /**< LED current 88mA */
  si115xLedCurrent100mA = SI115X_LED_CURRENT_100MA, /**< LED current 100mA */
  si115xLedCurrent111mA = SI115X_LED_CURRENT_111MA, /**< LED current 111mA */
  si115xLedCurrent116mA = SI115X_LED_CURRENT_116MA, /**< LED current 116mA */
  si115xLedCurrent133mA = SI115X_LED_CURRENT_133MA, /**< LED current 133mA */
  si115xLedCurrent138mA = SI115X_LED_CURRENT_138MA, /**< LED current 138mA */
  si115xLedCurrent155mA = SI115X_LED_CURRENT_155MA, /**< LED current 155mA */
  si115xLedCurrent166mA = SI115X_LED_CURRENT_166MA, /**< LED current 166mA */
  si115xLedCurrent177mA = SI115X_LED_CURRENT_177MA, /**< LED current 177mA */
  si115xLedCurrent194mA = SI115X_LED_CURRENT_194MA, /**< LED current 194mA */
  si115xLedCurrent199mA = SI115X_LED_CURRENT_199MA, /**< LED current 199mA */
  si115xLedCurrent221mA = SI115X_LED_CURRENT_221MA, /**< LED current 221mA */
  si115xLedCurrent232mA = SI115X_LED_CURRENT_232MA, /**< LED current 232mA */
  si115xLedCurrent265mA = SI115X_LED_CURRENT_265MA, /**< LED current 265mA */
  si115xLedCurrent271mA = SI115X_LED_CURRENT_271MA, /**< LED current 271mA */
  si115xLedCurrent310mA = SI115X_LED_CURRENT_310MA, /**< LED current 310mA */
  si115xLedCurrent354mA = SI115X_LED_CURRENT_354MA /**< LED current 354mA */
} SI115X_LedCurrent_TypeDef;

/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/

typedef struct
{
  uint8_t ADCPOST;
} SI115X_Channel_Param_TypeDef;

typedef struct
{
  SI115X_Channel_Param_TypeDef channels[_SI115X_MAX_CHANNELS];
} SI115X_Param_TypeDef;

/** SI115X device structure. */
typedef struct
{
  /** I2C port used for Si115x communication. */
  I2C_TypeDef *i2cPort;

  /** 7-bit I2C address of Si115x. */
  uint8_t i2cAddress;

  /** Bit-field of chan_en used for reading out data. */
  uint8_t chan_list;

  /** Bit-field of res24b used for reading out data. */
  uint8_t res;
} SI115X_TypeDef;

/** SI115X channel initialization structure. */
typedef struct
{
  /** Channel enable. */
  bool enable;

  /** ADC decimation rate. */
  SI115X_Decim_TypeDef decim;

  /** ADC photo diode selection. */
  SI115X_AdcMux_TypeDef adcMux;

  /** ADC signal range selection. */
  SI115X_Hsig_TypeDef hsig;

  /** ADC software accumulation. */
  SI115X_SwGain_TypeDef swGain;

  /** ADC hardware gain. */
  SI115X_HwGain_TypeDef hwGain;

  /** ADC output format. */
  SI115X_Res_TypeDef res;

  /** ADC post processing right shift. */
  uint8_t postshift;

  /** ADC threshold polarity. */
  SI115X_ThreshPol_TypeDef threshPol;

  /** ADC threshold mode. */
  SI115X_ThreshEn_TypeDef threshEn;

  /** ADC measurement counter selection. */
  SI115X_CounterIndex_TypeDef counterIndex;

  /** Channel LED bank selection. */
  SI115X_BankSel_TypeDef bankSel;

  /** Channel LED1 enable. */
  bool led1En;

  /** Channel LED2 enable. */
  bool led2En;

  /** Channel LED3 enable. */
  bool led3En;

} SI115X_InitChannel_TypeDef;

// todo: fix port
#define SI115X_DEFAULT                                                        \
{                                                                             \
  0,                            /* Default I2C port. */                       \
  SI115X_I2C_ADDR_DEFAULT,      /* Default I2C address. */                    \
  0,                            /* CHAN_LIST defaults of 0. */                \
  0,                            /* res24b default of 0. */                    \
}

/** Default configuration for forced mode conversions. */
#define SI115X_INIT_CHANNEL_FORCED_DEFAULT                                    \
{                                                                             \
  true,                         /* Channel is enabled. */                     \
  si115xDecim512,               /* ADC 512 decimation rate. */                \
  si115xAdcMuxLargeIr,          /* Large IR photodiode. */                    \
  si115xHsigNormal,             /* Normal ADC signal range. */                \
  si115xSwGain1x,               /* 1x software accumulation. */               \
  si115xHwGain1x,               /* 1x hardware gain. */                       \
  si115xRes16bUint,             /* Unsigned 16-bit output format. */          \
  0,                            /* No right shift post measurement. */        \
  si115xThreshPolSmaller,       /* Threshold compares using less than. */     \
  si115xThreshEnNone,           /* No threshold comparison. */                \
  si115xCounterIndexDisabled,   /* No measurement counter enabled. */         \
  si115xBankSelLedA,            /* Select LED bank A. */                      \
  false,                        /* LED1 disabled. */                          \
  false,                        /* LED2 disabled. */                          \
  false,                        /* LED3 disabled. */                          \
}

/** Default configuration for autonomous mode conversions. */
#define SI115X_INIT_CHANNEL_AUTO_DEFAULT                                      \
{                                                                             \
  true,                         /* Channel is enabled. */                     \
  si115xDecim512,               /* ADC 512 decimation rate. */                \
  si115xAdcMuxLargeIr,          /* Large IR photodiode. */                    \
  si115xHsigNormal,             /* Normal ADC signal range. */                \
  si115xSwGain1x,               /* 1x software accumulation. */               \
  si115xHwGain1x,               /* 1x hardware gain. */                       \
  si115xRes16bUint,             /* Unsigned 16-bit output format. */          \
  0,                            /* No right shift post measurement. */        \
  si115xThreshPolSmaller,       /* Threshold compares using less than. */     \
  si115xThreshEnNone,           /* No threshold comparison. */                \
  si115xCounterIndexMeascount0, /* Measurement counter 0 selected */          \
  si115xBankSelLedA,            /* Select LED bank A. */                      \
  false,                        /* LED1 disabled. */                          \
  false,                        /* LED2 disabled. */                          \
  false,                        /* LED3 disabled. */                          \
    }
/** Si115X global initialization struct. */
typedef struct
{
  /** Measurement rate for autonomous mode. */
  uint16_t measrate;

  /** Measurement counter 0. */
  uint8_t meascount0;

  /** Measurement counter 1. */
  uint8_t meascount1;

  /** Measurement counter 2. */
  uint8_t meascount2;

  /** LED1 bank A current. */
  SI115X_LedCurrent_TypeDef led1a_current;

  /** LED1 bank B current. */
  SI115X_LedCurrent_TypeDef led1b_current;

  /** LED2 bank A current. */
  SI115X_LedCurrent_TypeDef led2a_current;

  /** LED2 bank B current. */
  SI115X_LedCurrent_TypeDef led2b_current;

  /** LED3 bank A current. */
  SI115X_LedCurrent_TypeDef led3a_current;

  /** LED3 bank B current. */
  SI115X_LedCurrent_TypeDef led3b_current;

  /** Threshold 0 value. */
  uint16_t threshold0;

  /** Threshold 1 value. */
  uint16_t threshold1;

  /** Window threshold upper value. */
  uint16_t upper_threshold;

  /** Window threshold lower value. */
  uint16_t lower_threshold;

  /** Burst mode enable. */
  bool burstEn;

  /** Burst mode number of conversions. */
  uint8_t burstCount;

} SI115X_Init_TypeDef;

/** Default configuration for global initialization structure. */
#define SI115X_INIT_DEFAULT                                                   \
    {                                                                         \
      1250,                         /**< 1 second measurement rate. */        \
      1,                            /**< Measurement counter 0 1x divider. */ \
      1,                            /**< Measurement counter 1 1x divider. */ \
      1,                            /**< Measurement counter 2 1x divider. */ \
      SI115X_LED_CURRENT_310MA,    /**< LED1 bank A 310 mA LED current. */   \
      SI115X_LED_CURRENT_6MA,      /**< LED1 bank A 6 mA LED current. */     \
      SI115X_LED_CURRENT_6MA,      /**< LED1 bank A 6 mA LED current. */     \
      SI115X_LED_CURRENT_6MA,      /**< LED1 bank A 6 mA LED current. */     \
      SI115X_LED_CURRENT_6MA,      /**< LED1 bank A 6 mA LED current. */     \
      SI115X_LED_CURRENT_6MA,      /**< LED1 bank A 6 mA LED current. */     \
      0,                            /**< Threshold 0 value of 0. */           \
      0,                            /**< Threshold 1 value of 0. */           \
      0,                            /**< Window threshold upper value of 0. */\
      0,                            /**< Window threshold lower value of 0. */\
      false,                        /**< Burst mode disabled. */              \
      0,                            /**< No burst conversions. */             \
}

/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

// todo: organize the ordering of these
SI115X_Ecode_TypeDef SI115X_Init(SI115X_TypeDef *si115x,
                                 const SI115X_Init_TypeDef *init);
SI115X_Ecode_TypeDef SI115X_InitChannel(SI115X_TypeDef *si115x, uint8_t channel,
                                        const SI115X_InitChannel_TypeDef *init);
SI115X_Ecode_TypeDef SI115X_QueryParam(SI115X_TypeDef *si115x, uint8_t address, uint8_t *param);
SI115X_Ecode_TypeDef SI115X_SetParam(SI115X_TypeDef *si115x, uint8_t address,
                                     uint8_t value);
SI115X_Ecode_TypeDef SI115X_SendCmd(SI115X_TypeDef *si115x, uint8_t command);
SI115X_Ecode_TypeDef SI115X_Reset(SI115X_TypeDef *si115x);
SI115X_Ecode_TypeDef SI115X_ResetCmdCounter(SI115X_TypeDef *si115x);
SI115X_Ecode_TypeDef SI115X_Force(SI115X_TypeDef *si115x);
SI115X_Ecode_TypeDef SI115X_Start(SI115X_TypeDef *si115x);
SI115X_Ecode_TypeDef SI115X_Pause(SI115X_TypeDef *si115x);
SI115X_Ecode_TypeDef SI115X_IntEnable(SI115X_TypeDef *si115x, uint8_t flags);
SI115X_Ecode_TypeDef SI115X_IntGet(SI115X_TypeDef *si115x, uint8_t *irq_status);
SI115X_Ecode_TypeDef SI115X_GetChannelData(SI115X_TypeDef *si115x,
                                           uint8_t channel, int32_t *data);

// I2C Functions
SI115X_Ecode_TypeDef SI115X_WriteToRegister(SI115X_TypeDef *si115x,
                                            uint8_t address, uint8_t value);
SI115X_Ecode_TypeDef SI115X_ReadFromRegister(SI115X_TypeDef *si115x,
                                             uint8_t address,
                                             uint8_t *data);
SI115X_Ecode_TypeDef SI115X_BlockRead(SI115X_TypeDef *si115x, uint8_t address,
                                      uint8_t length, uint8_t *values);
SI115X_Ecode_TypeDef SI115X_BlockWrite(SI115X_TypeDef *si115x, uint8_t address,
                                       uint8_t length, uint8_t *values);

/** @} (end group Si115x) */
/** @} (end group kitdrv) */

#ifdef __cplusplus
}
#endif

#endif /* SI115X_H */
