/***************************************************************************//**
 * @file pir_config.h
 * @brief PIR Config
 * @version 1.0.3
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
 * # Experimental Quality
 * This code has not been formally tested and is provided as-is. It is not
 * suitable for production environments. In addition, this code will not be
 * maintained and there may be no bug maintenance planned for these resources.
 * Silicon Labs may update projects from time to time.
 ******************************************************************************/

#ifndef PIR_CONFIG_H_
#define PIR_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef EFM32PG12B500F1024GL125 /* BRD2501: SLSTK3402A: PG12 STK */

// PIR Occupancy Sensor Analog Pins
#define ADC_P_PORT        gpioPortC
#define ADC_P_PIN         9
#define ADC_P_APORT       adcPosSelAPORT2XCH9

#define ADC_N_PORT        gpioPortA
#define ADC_N_PIN         6
#define ADC_N_APORT       adcNegSelAPORT4YCH14

#define MOTION_B_PORT     gpioPortD
#define MOTION_B_PIN      10

#define LDO_SHDN_B_PORT   gpioPortD
#define LDO_SHDN_B_PIN    8

#elif defined(EFR32MG12P332F1024GL125) /* BRD4001A (WSTK) + BRD4162A (MG12) */

// PIR Occupancy Sensor Analog Pins
#define ADC_P_PORT        gpioPortD
#define ADC_P_PIN         8
#define ADC_P_APORT       adcPosSelAPORT3XCH0

#define ADC_N_PORT        gpioPortA
#define ADC_N_PIN         6
#define ADC_N_APORT       adcNegSelAPORT4YCH14

#define MOTION_B_PORT     gpioPortB
#define MOTION_B_PIN      6

#define LDO_SHDN_B_PORT   gpioPortC
#define LDO_SHDN_B_PIN    9

#elif defined(ZGM130S037HGN) /* BRD4001A (WSTK) + BRD4202A (ZGM130S) */

// PIR Occupancy Sensor Analog Pins
#define ADC_P_PORT        gpioPortA
#define ADC_P_PIN         2
#define ADC_P_APORT       adcPosSelAPORT3XCH10

#define ADC_N_PORT        gpioPortC
#define ADC_N_PIN         6
#define ADC_N_APORT       adcNegSelAPORT2YCH6

// Use LED0 on WSTK
#define MOTION_B_PORT     gpioPortF
#define MOTION_B_PIN      4

#define LDO_SHDN_B_PORT   gpioPortF
#define LDO_SHDN_B_PIN    3

#endif

#ifdef __cplusplus
}
#endif

#endif // PIR_CONFIG_H
