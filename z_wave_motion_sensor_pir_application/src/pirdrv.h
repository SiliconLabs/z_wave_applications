/***************************************************************************//**
 * @file
 * @brief Header file for PIR demonstration code.
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

#ifndef SRC_PIRDRV_H_
#define SRC_PIRDRV_H_

#include <stdint.h>
#include <stdbool.h>
//#include "bspconfig.h"

/**
 * @brief Configures whether to use the internal op-amp or an external op-amp.
 */
typedef enum {
  pirOpampModeInternal = 0,
  pirOpampModeExternal = 1,
} PIR_OpampMode_TypeDef;

/**
 * @brief Selects whether to use the dedicated main op-amp ports or use the APORT selection.
 */
typedef enum {
  pirOpampMainPorts = 0,
  pirOpampAport = 1,
} PIR_Opamp_Port_t;

/**
 * @brief Selects whether to operate the ADC in single ended positive, negative or differential mode.
 */
typedef enum {
  adcSourceAdcP = 0,
  adcSourceAdcN = 1,
  adcSourceAdcDiff = 2,
} AdcSource_Typedef_t;

/**
 * @brief Initialization struct for PIR operation.
 */
typedef struct {
  PIR_OpampMode_TypeDef opampMode;    /** opampMode: Use internal or external opamp. */
  uint32_t motionOnTime;              /** motionOnTime: The duration of time which motion is asserted after a motion event. */
  uint32_t winSize;                   /** winSize: The peak to peak window size for motion detection in lsb. */
} PIR_Init_TypeDef;

/**
 * @brief Default initialization struct for PIR.
 */
#define PIR_INIT_DEFAULT                                                                                  \
  {                                                                                                       \
    .opampMode = pirOpampModeExternal,  /**< opampMode: Select whether to initialize the internal opamp. */ \
    .motionOnTime = 128,                /**< motionOnTime: 4 seconds @ 32 Hz = 128 */                       \
    .winSize = 1024,                    /**< winSize: peak to peak size of window (lsb). */                 \
  }

/**
 * @brief A PIR sample.
 */
typedef struct {
  int32_t timeMs;                     /**< timeMs: A 16-bit timestamp from the 1024 Hz counter. */
  int32_t adc;                        /**< adc: The ADC output measuring the PIR voltage. */
  int32_t winBase;                    /**< winBase: The center of the detector window. */
  int32_t adcThreshHigh;              /**< adcThreshHigh: The upper threshold of the detector window. */
  int32_t adcThreshLow;               /**< adcThreshLow: The lower threshold of the detector window. */
  bool motionB;                       /**< motionB: Whether motion was detected (active low). */
} PirSample_t;

void PIR_Main(bool lowPowerMode);
void PIR_Init(PIR_Init_TypeDef *init, bool enterEM2);
bool PIR_DetectMotion(bool lowPowerMode);
void PIR_Start(void);
void PIR_Stop(void);
//void PIR_SettleCapacitor(uint32_t pirSettleTimeMs);
void PIR_MotionOff(bool lowPowerMode);
void PIR_MotionOn(bool lowPowerMode);

/* PIR sample queue interface to the UART stream to the PC. */
void PIR_WriteQueue(PirSample_t *pirSample);
PirSample_t* PIR_ReadQueue(void);
uint32_t PIR_GetQueueSize(void);


#define LETIMER_PERIOD 65535            /** The maximum timestamp counter value. Used to handle rollover. */
#define QUEUE_LENGTH 4                  /** Length of the queue to hold samples. Must be larger than the ADC queue. */

#define BUFFER_LENGTH  64               /** Max size of UART buffer */
uint8_t msgBuffer[BUFFER_LENGTH];       /** The UART buffer to hold messages to the PC. */

#endif /* SRC_PIRDRV_H_ */
