/***************************************************************************//**
 * @file
 * @brief Header file for demo of OCCUPANCY-EXP-EB
 * @version 1.0.2
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef SRC_OCCUPANCY_EXP_H_
#define SRC_OCCUPANCY_EXP_H_

#include "em_adc.h"
//#include "em_opamp.h"
#include "em_gpio.h"

#ifdef EFM32PG12B500F1024GL125
/* BRD2501: SLSTK3402A: PG12 STK*/

/* PIR Occupancy Sensor Analog Pins */
/* If PIR_SettleCapacitor is used on a part that has errata
 * VDAC_E201, this should be placed on a APORT X bus. */
  #define ADC_P_PORT        gpioPortC
  #define ADC_P_PIN         9
  #define ADC_P_APORT       adcPosSelAPORT2XCH9
  #define ADC_P_OPA_APORT   opaPosSelAPORT2XCH9

/* If PIR_SettleCapacitor is used on a part that has errata
 * VDAC_E201, this should be placed on a APORT Y bus. */
  #define ADC_N_PORT        gpioPortA
  #define ADC_N_PIN         6
  #define ADC_N_APORT       adcNegSelAPORT4YCH14
  #define ADC_N_OPA_OUTMODE opaOutModeAPORT4YCH14

  #define LED_PORT          BSP_GPIO_LED0_PORT
  #define LED_PIN           BSP_GPIO_LED0_PIN

  #define MOTION_B_PORT     gpioPortD
  #define MOTION_B_PIN      10

  #define LDO_SHDN_B_PORT   gpioPortD
  #define LDO_SHDN_B_PIN    8

/* Si1153 Ambient Light Sensor Pins */
  #define SENSOR_SCL_PORT   gpioPortC
  #define SENSOR_SCL_PIN    11

  #define SENSOR_SDA_PORT   gpioPortC
  #define SENSOR_SDA_PIN    10

  #define SENSOR_INT_PORT   gpioPortD
  #define SENSOR_INT_PIN    11

#elif defined(EFR32MG12P332F1024GL125)
/* BRD4001A (WSTK) + BRD4162A (MG12) */
/* PIR Occupancy Sensor Analog Pins */
/* If PIR_SettleCapacitor is used on a part that has errata
 * VDAC_E201, this should be placed on a APORT X bus. */
  #define ADC_P_PORT        gpioPortD
  #define ADC_P_PIN         8
  #define ADC_P_APORT       adcPosSelAPORT3XCH0
  #define ADC_P_OPA_APORT   opaPosSelAPORT3XCH0

/* If PIR_SettleCapacitor is used on a part that has errata
 * VDAC_E201, this should be placed on a APORT Y bus. */
  #define ADC_N_PORT        gpioPortA
  #define ADC_N_PIN         6
  #define ADC_N_APORT       adcNegSelAPORT4YCH14
  #define ADC_N_OPA_OUTMODE opaOutModeAPORT4YCH14

  #define LED_PORT          BSP_GPIO_LED0_PORT
  #define LED_PIN           BSP_GPIO_LED0_PIN

  #define MOTION_B_PORT     gpioPortB
  #define MOTION_B_PIN      6

  #define LDO_SHDN_B_PORT   gpioPortC
  #define LDO_SHDN_B_PIN    9

/* Si1153 Ambient Light Sensor Pins */
  #define SENSOR_SCL_PORT   gpioPortC
  #define SENSOR_SCL_PIN    10

  #define SENSOR_SDA_PORT   gpioPortC
  #define SENSOR_SDA_PIN    11

  #define SENSOR_INT_PORT   gpioPortB
  #define SENSOR_INT_PIN    7
#elif defined(ZGM130S037HGN)
/* BRD4001A (WSTK) + BRD4202A (ZGM130S) */
/* PIR Occupancy Sensor Analog Pins */
/* If PIR_SettleCapacitor is used on a part that has errata
 * VDAC_E201, this should be placed on a APORT X bus. */
  #define ADC_P_PORT        gpioPortA
  #define ADC_P_PIN         2
  #define ADC_P_APORT       adcPosSelAPORT3XCH10

/* If PIR_SettleCapacitor is used on a part that has errata
 * VDAC_E201, this should be placed on a APORT Y bus. */
  #define ADC_N_PORT        gpioPortC
  #define ADC_N_PIN         6
  #define ADC_N_APORT       adcNegSelAPORT2YCH6

  //#define LED_PORT          BSP_GPIO_LED0_PORT
  //#define LED_PIN           BSP_GPIO_LED0_PIN

  // Conflict with USART0
  //#define MOTION_B_PORT     gpioPortA
  //#define MOTION_B_PIN      0

  // Use LED0 on WSTK
  #define MOTION_B_PORT     gpioPortF
  #define MOTION_B_PIN      4

  #define LDO_SHDN_B_PORT   gpioPortF
  #define LDO_SHDN_B_PIN    3

/* Si1153 Ambient Light Sensor Pins */
/*
  #define SENSOR_SCL_PORT   gpioPortC
  #define SENSOR_SCL_PIN    10

  #define SENSOR_SDA_PORT   gpioPortC
  #define SENSOR_SDA_PIN    11

  #define SENSOR_INT_PORT   gpioPortB
  #define SENSOR_INT_PIN    7
*/
#endif

/**
 * @brief Enum for the possible demo modes.
 */
enum {
  demoModeOccupancy=0,
  demoModeAmbientLight=1,
  demoModeOccupancyLowPower=2,
  demoModeAmbientLightLowPower=3,
} typedef Demo_Mode_TypeDef;

/*******************************************************************************
 ****************************   TEXT DISPLAY   ********************************
 ******************************************************************************/
#define NUM_DEMOS 4                     /* Number of demos in the menu. Used for button handling. */
#define MENU_INDENT "   "               /* Indent to menu items. */
#define PIR_STRING "PIR"
#define ALS_STRING "ALS"
#define PIR_LOW_PWR_STRING "PIR LCD Off"
#define ALS_LOW_PWR_STRING "ALS LCD Off"
#define HEADER_STRING "\r\n   DEMO MODE\n\n"
#define DOC_STRING "Push BTN1 to\n   cycle modes.\n\n" MENU_INDENT "Push BTN0 to\n   start test.\n\n"
#define CURSOR_STRING " ->"
#define DEMO_LINE_OFFSET 3

#define CLEAR_SCREEN "\f"

#define PIR_DEMO_STRING "\r\n"                    \
                        "    PIR DEMO    \n\n"    \
                        " Use the PIR\n"          \
                        " GUI on the PC."


#define RTC_PULSE_FREQUENCY    (LS013B7DH03_POLARITY_INVERSION_FREQUENCY)
#endif /* SRC_OCCUPANCY_EXP_H_ */
