/***************************************************************************//**
 * @file
 * @brief PIR demonstration code for OCCUPANCY-EXP-EB
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

#include <stdbool.h>
#include <stdio.h>

#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_cryotimer.h"
//#include "em_opamp.h"
#include "em_adc.h"
//#include "em_letimer.h"
//#include "em_rtcc.h"

//#include "rtcdriver.h"
//#include "uart_debug.h"
#include "occupancy_exp.h"
#include "pirdrv.h"
#include "DebugPrint.h"
#include "ZAF_uart_utils.h"
#include "board.h"
#include <zaf_event_helper.h>
#include "events.h"

//static void PIR_InitTimestampClock(void);
static void PIR_InitAdc(bool enterEM2);
//static void PIR_InitNonInverting(void);
static void PIR_UpdateThresholds(int32_t winBase, uint32_t winSize);
static void PIR_TransmitSample(PirSample_t *pirSample);

static PIR_Init_TypeDef config;    /* Configuration of the detection algorithm. */
static bool _lowPowerMode = true;   /* Holds lowPowerMode state for use in ADC IRQ handler. */
static bool motionDetected = false; /* Output of PIR_DetectMotion. */
volatile bool pirInt = false;       /* Interrupt flag indicating the completion of PIR_DetectMotion. */
static uint32_t lockOutCounter = 0; /* The time, in samples, remaining on the lockout window after detection. */
static int32_t winBase = 0;         /* The low frequency mean of the PIR input signal. */

/* The adcSource variable should be set to adcSourceAdcDiff for PIR operation.
 * The single ended modes are additional modes for signal chain analysis with a voltage source. */
static AdcSource_Typedef_t adcSource = adcSourceAdcDiff;

static PirSample_t pirQueue[QUEUE_LENGTH];
static int32_t iRead = -1;
static int32_t iWrite = -1;

static bool pirStartNoise = false;
static uint32_t pirStartCounter = 0;

/* As a rule, this module operates with 16-bit values for sampled data. The only functions that should
 * shift data are functions that directly interact with the ADC registers and are required to transform
 * the values to a shifted variant due to oversampling or lack there of.
 */



/**
 * @brief Main function for PIR occupancy sensing.
 *
 * @param[in] lowPowerMode
 *  Disables the LCD and UART data streaming when true and enters EM2.
 */
void PIR_Main(bool lowPowerMode)
{
  /* Enable LDO to startup VPIR. */
  GPIO_PinOutSet(LDO_SHDN_B_PORT, LDO_SHDN_B_PIN);

  PIR_Init_TypeDef pirInit = PIR_INIT_DEFAULT;
  pirInit.opampMode = pirOpampModeExternal;

  _lowPowerMode = lowPowerMode;
  if (!lowPowerMode) {
    //UART_Debug_Init();
    //PIR_InitTimestampClock();
    printf(CLEAR_SCREEN PIR_DEMO_STRING);
  }

  PIR_Init(&pirInit, true);
  PIR_Start();

  while (1) {
    if (pirInt == true) {
      pirInt = false;

      if (!lowPowerMode) {
        PirSample_t *pirSample;
        while (PIR_GetQueueSize() > 0) {
          pirSample = PIR_ReadQueue();
          PIR_TransmitSample(pirSample);
        }
      } // if (!lowPowerMode)
    } // if (pirInt == true)

    EMU_EnterEM2(true);
  } // while(1)
}

/**
 * @brief Initializes peripherals for PIR.
 */
void PIR_Init(PIR_Init_TypeDef *init, bool enterEM2)
{
  config.motionOnTime = init->motionOnTime;
  config.opampMode = init->opampMode;
  config.winSize = init->winSize;

  /* Analog initialization */
  if (init->opampMode == pirOpampModeInternal) {
    //PIR_InitNonInverting();
  }
  PIR_InitAdc(enterEM2);

  /* Initialize LED. */
  PIR_MotionOff(enterEM2);
}

/**
 * @brief
 *   Begins running the motion detection sampling and algorithm.
 */
void PIR_Start(void)
{
  lockOutCounter = 0;
  pirStartNoise = true;
  pirStartCounter = 0;
  //pirInt = false;
  ADC_Start(ADC0, adcStartSingle);
  ADC_IntEnable(ADC0, ADC_IF_SINGLE);         /* Wake up on FIFO reaching threshold. */
  ADC_IntEnable(ADC0, ADC_IF_SINGLECMP);      /* Wake up on ADC exceeding window threshold. */
  NVIC_EnableIRQ(ADC0_IRQn);
}

/**
 * @brief
 *   Begins running the motion detection sampling and algorithm.
 */
void PIR_Stop(void)
{
  pirStartNoise = false;
  pirStartCounter = 0;
  ADC_IntDisable(ADC0, ADC_IF_SINGLE);
  ADC_IntDisable(ADC0, ADC_IF_SINGLECMP);
  NVIC_DisableIRQ(ADC0_IRQn);
  //PIR_MotionOff(_lowPowerMode);
  Board_SetLed(BOARD_RGB1_R, LED_OFF);
  Board_SetLed(BOARD_RGB1_G, LED_OFF);
  Board_SetLed(BOARD_RGB1_B, LED_OFF);
}

/**
 * @brief
 *  Initializes the non inverting amplifier for PIR signal chain.
 */

/*static void PIR_InitNonInverting(void)
{
  PIR_Opamp_Port_t port = pirOpampMainPorts;
  CMU_ClockEnable(cmuClock_VDAC0, true);

  * Errata VDAC_E201 for PG/JG/MG/BG12 causes contention if APORT is used for output.
   *  Use of dedicated opamp output pin or careful mapping of APORT buses is recommended.

  OPAMP_Init_TypeDef opaInit = OPA_INIT_NON_INVERTING;
  opaInit.resInMux = opaResInMuxDisable;
  opaInit.resSel = opaResSelDefault;

  if (port == pirOpampMainPorts) {
    opaInit.posSel = opaPosSelPosPad;
    opaInit.negSel = opaNegSelNegPad;
    opaInit.outMode = opaOutModeDisable;
    opaInit.outPen = VDAC_OPA_OUT_MAINOUTEN;
  } else {
     These APORT mappings match the main predefined analog paths.
    opaInit.posSel = opaPosSelAPORT4XCH5;
    opaInit.negSel = opaNegSelAPORT3YCH7;
    opaInit.outMode = opaOutModeDisable;
    opaInit.outPen = VDAC_OPA_OUT_MAINOUTEN;   Workaround for VDAC_E201, use dedicate opamp output pin.
  }

  const uint32_t opampCh = 1;
  OPAMP_Enable(VDAC0, OPA1, &opaInit);

   Set INCBW for 2.5x increase in GBW, only stable for G > 3.
  VDAC0->OPA[opampCh].CTRL |= VDAC_OPA_CTRL_INCBW;

   Set DRIVESTRENGTH=0 to minimize current.
  VDAC0->OPA[opampCh].CTRL &= ~_VDAC_OPA_CTRL_DRIVESTRENGTH_MASK;
  VDAC0->OPA[opampCh].CTRL |= (0 << _VDAC_OPA_CTRL_DRIVESTRENGTH_SHIFT);
}*/

/**
 * @brief
 *  Charges up the slow charging capacitor by bypassing the opamp feedback network.
 *
 *  Not used in the OCCUPANCY-EXP-EVB as the R_f * C_g settle time is short enough.
 *  May be used if R_f * C_g is increased further.
 *
 * @param[in] pirSettleTimeMs
 *  The time in milliseconds to short the feedback network with a unity gain buffer.
 */
/*void PIR_SettleCapacitor(uint32_t pirSettleTimeMs)
{
  CMU_ClockEnable(cmuClock_VDAC0, true);
   WARNING: See errata VDAC_E201 for PG/MG/BG12.
   * VDAC will drive output on all AY, BY, CY, DY APORTs. The ADC_P
   * should placed on an X port, and the output placed on a Y port.
  OPAMP_Init_TypeDef opaInit = OPA_INIT_UNITY_GAIN;
  opaInit.posSel = ADC_P_OPA_APORT;
  opaInit.outMode = ADC_N_OPA_OUTMODE;
  opaInit.outPen = VDAC_OPA_OUT_APORTOUTEN;

  OPAMP_Enable(VDAC0, OPA0, &opaInit);
  RTCDRV_Init();
  RTCDRV_Delay(pirSettleTimeMs);
  RTCDRV_DeInit();

   Clear all APORT requests to avoid APORT contention with ADC.
  VDAC0->OPA[0].OUT = 0;
  VDAC0->OPA[0].MUX = (VDAC_OPA_MUX_POSSEL_DISABLE << _VDAC_OPA_MUX_POSSEL_SHIFT)
                      | (VDAC_OPA_MUX_NEGSEL_DISABLE << _VDAC_OPA_MUX_NEGSEL_SHIFT);
  OPAMP_Disable(VDAC0, OPA0);
  CMU_ClockEnable(cmuClock_VDAC0, false);
}*/

/**
 * @brief
 *  Initializes the ADC, CRYOTIMER and PRS for PIR operation.
 *
 * @details
 *   PRS triggering of the ADC in EM2 is only possible on > Series 1 MCUs (PG/JG/BG/MG).
 *
 * @param[in] lowPowerMode
 *  Sets up ADC to run in EM2 when set to true.
 */
static void PIR_InitAdc(bool enterEM2)
{
  /* Initialize the ADC. */
  CMU_ClockEnable(cmuClock_ADC0, true);

  ADC_Init_TypeDef adcInit = ADC_INIT_DEFAULT;
  adcInit.ovsRateSel = adcOvsRateSel2;

  if (enterEM2) {
    adcInit.em2ClockConfig = adcEm2ClockOnDemand;
    adcInit.prescale = ADC_PrescaleCalc(cmuAUXHFRCOFreq_38M0Hz, CMU_AUXHFRCOBandGet());
    adcInit.timebase = ADC_TimebaseCalc(CMU_AUXHFRCOBandGet());
  } else {
    adcInit.timebase = ADC_TimebaseCalc(0);
  }

  ADC_InitSingle_TypeDef adcInitSingle = ADC_INITSINGLE_DEFAULT;
  adcInitSingle.resolution = adcResOVS;
  adcInitSingle.reference = adcRef1V25;
  adcInitSingle.prsEnable = true;
  adcInitSingle.prsSel = adcPRSSELCh0;
  adcInitSingle.leftAdjust = true;
  adcInitSingle.acqTime = adcAcqTime64;


  switch (adcSource) {
    case adcSourceAdcP:
      adcInitSingle.posSel = ADC_P_APORT;
      adcInitSingle.negSel = adcNegSelVSS;
      adcInitSingle.diff = false;
      break;
    case adcSourceAdcN:
      /* The posSel of the ADC_N is not defined for the kit, only the negSel. */
      adcInitSingle.posSel = adcPosSelAPORT3XCH14;
      adcInitSingle.negSel = adcNegSelVSS;
      adcInitSingle.diff = false;
      break;
    case adcSourceAdcDiff:
      adcInitSingle.posSel = ADC_P_APORT;
      adcInitSingle.negSel = ADC_N_APORT;
      adcInitSingle.diff = true;
      break;
  }

  if (enterEM2) {
    CMU_ClockEnable(cmuClock_AUX, true);
    CMU_AUXHFRCOBandSet(cmuAUXHFRCOFreq_38M0Hz);
    CMU->ADCCTRL = CMU_ADCCTRL_ADC0CLKSEL_AUXHFRCO;
  }

  ADC_Init(ADC0, &adcInit);
  ADC_InitSingle(ADC0, &adcInitSingle);

  /* Set ADC FIFO level to max to minimize EM0 wakeups when the ADC sample remains within window. */
  static const uint32_t dataValidLevel = 3; /* ADC SINGLE IRQ is when DVL+1 single channels are available in the FIFO. */
  ADC0->SINGLECTRLX &= ~_ADC_SINGLECTRLX_DVL_MASK;
  ADC0->SINGLECTRLX |= (dataValidLevel << _ADC_SINGLECTRLX_DVL_SHIFT);

  PIR_UpdateThresholds(winBase, config.winSize);

  /****************************************************************************
  ************************ CRYOTIMER Initialization **************************
  ****************************************************************************/
  CMU_OscillatorEnable(cmuOsc_ULFRCO, true, true);
  CMU_ClockEnable(cmuClock_CRYOTIMER, true);

  /* Run CRYOTIMER on ULFRCO (1024 Hz) and trigger ADC to set the sampling period. */
  CRYOTIMER_Init_TypeDef cryoInit = CRYOTIMER_INIT_DEFAULT;
  cryoInit.osc = cryotimerOscULFRCO;
  cryoInit.presc = cryotimerPresc_1;
  cryoInit.period = cryotimerPeriod_32; // Sampling frequency is 1024 / 32 = 32 Hz.
  CRYOTIMER_Init(&cryoInit);

  /****************************************************************************
  *************************** PRS Initialization *****************************
  ****************************************************************************/
  /* PRS is the triggering connection between CRYOTIMER and ADC. */
  CMU_ClockEnable(cmuClock_PRS, true);
  PRS_SourceAsyncSignalSet(0, PRS_CH_CTRL_SOURCESEL_CRYOTIMER, PRS_CH_CTRL_SIGSEL_CRYOTIMERPERIOD);
}

/**
 * @brief
 *  Updates the ADC hardware thresholds for a motion event trigger.
 *
 *  Inputs must mast the conversion data representation. For oversampled
 *  conversions, the resolution is 16-bit, regardless if the OVS setting
 *  does not achieve 16-bit resolution.
 *
 * @param[in] winBase
 *  The midpoint of the window. Thresholds are the base +/- 0.5*winSize
 *
 * @param[in] winSize
 *  The total peak to peak width of the window.
 */
static void PIR_UpdateThresholds(int32_t winBase, uint32_t windowSize)
{
  int32_t posThresh = winBase + windowSize / 2;
  int32_t negThresh = winBase - windowSize / 2;

  /* Ensure thresholds are within int16_t for ADC window comparator. */
  int32_t posThreshMax, negThreshMin;
  if (adcSource == adcSourceAdcDiff) {
    posThreshMax = 32767;
    negThreshMin = -32768;
  } else {
    posThreshMax = 65535;
    negThreshMin = 0;
  }

  if (posThresh > posThreshMax) {
    posThresh = posThreshMax;
  }
  if (negThresh < negThreshMin) {
    negThresh = negThreshMin;
  }

  ADC0->CMPTHR = ((posThresh << _ADC_CMPTHR_ADGT_SHIFT) & _ADC_CMPTHR_ADGT_MASK)
                 | ((negThresh << _ADC_CMPTHR_ADLT_SHIFT) & _ADC_CMPTHR_ADLT_MASK);
}

/**
 * @brief
 *   Runs the motion detection algorithm on the latest sample.
 *
 * @param[in] lowPowerMode
 *    Transmits the motion detection results over UART if lowPowerMode is false.
 *
 * @return
 *   Motion detected status. 0 = no motion, 1 = motion
 */
bool PIR_DetectMotion(bool lowPowerMode)
{
  bool motion = false;
  uint32_t numSamples = 0;
  int32_t adcSample;
  PirSample_t pirSample;
  bool dataValid = ADC0->SINGLEFIFOCOUNT > 0;
  int32_t adcThreshHigh, adcThreshLow;
  static int32_t lastTimestamp = -1;
  if (lastTimestamp < 0) {
    //lastTimestamp = ~LETIMER_CounterGet(LETIMER0);
  }

  /* Read out each ADC sample, update the LPF mean, and check if it was within the window thresholds. */
  while (dataValid) {

    numSamples++;
    adcSample = ADC_DataSingleGet(ADC0) << 3; // adjust this with oversampling for OSR < 64. Need to left shift to get to 16b.
    /* Capture the ADC and winBase of the current sample. */
    pirSample.adc = adcSample;
    pirSample.winBase = winBase;
    if (adcSource == adcSourceAdcDiff) {
      adcThreshHigh = ((int16_t) ((ADC0->CMPTHR & _ADC_CMPTHR_ADGT_MASK) >> _ADC_CMPTHR_ADGT_SHIFT));
      adcThreshLow = ((int16_t) ((ADC0->CMPTHR & _ADC_CMPTHR_ADLT_MASK) >> _ADC_CMPTHR_ADLT_SHIFT));
    } else {
      adcThreshHigh = ((uint16_t) ((ADC0->CMPTHR & _ADC_CMPTHR_ADGT_MASK) >> _ADC_CMPTHR_ADGT_SHIFT));
      adcThreshLow = ((uint16_t) ((ADC0->CMPTHR & _ADC_CMPTHR_ADLT_MASK) >> _ADC_CMPTHR_ADLT_SHIFT));
    }

    /* If window was broken, move window thresholds to include the latest ADC reading.
     * Thresholds are recalculated in software because samples are batch processed. */
    if (adcSample > adcThreshHigh) {
      winBase = adcSample - config.winSize / 2;
      motion = true;
    } else if (adcSample < adcThreshLow) {
      winBase = adcSample + config.winSize / 2;
      motion = true;
    } else {
      /* Window was not broken, update winBase to follow the low frequency drift using a DT 1st order LPF.
       * Let a = 2^-a_shift
       * Equivalent continuous RC is given by Ts * (1-a) / a */
      uint32_t a_shift = 5;
      winBase = (adcSample >> a_shift) + (winBase - (winBase >> a_shift));
    }

    PIR_UpdateThresholds(winBase, config.winSize);
    dataValid = ADC0->SINGLEFIFOCOUNT > 0;

    /* Transmit the PIR sample data over UART if not in low power mode. */
    if (!lowPowerMode) {
      if (!dataValid) {
        /* Last sample caused this wake up, so time stamp counter is valid. */
        //lastTimestamp = ((uint16_t) ~LETIMER_CounterGet(LETIMER0));
      } else {
        /* Estimate time stamp based on last measured time stamp and sample rate. */
        //const uint32_t samplePeriod = 32;
        //lastTimestamp = (lastTimestamp + samplePeriod) % LETIMER_PERIOD;
      }
      pirSample.timeMs = lastTimestamp;
      pirSample.motionB = !(lockOutCounter > 0 || motion);
      pirSample.adcThreshHigh = adcThreshHigh;
      pirSample.adcThreshLow = adcThreshLow;
      PIR_WriteQueue(&pirSample);
    } // if (!lowPowerMode)
  } // while (dataValid)

  if (lockOutCounter > 0) {
    lockOutCounter -= (numSamples < lockOutCounter) ? numSamples : lockOutCounter;
    if (lockOutCounter == 0) {
      PIR_MotionOff(lowPowerMode);
    }
  }

  /* Assert motion. */
  if (motion) {
    lockOutCounter = config.motionOnTime;
    if(pirStartNoise)
    {
      pirStartCounter += 1;
      if(pirStartCounter == 5)
      {
        pirStartNoise = false;
      }
    }
    else
    {
      PIR_MotionOn(lowPowerMode);
      //DebugPrintf("\r\n** motion: %u!!!\r\n", motion);
    }
  }

  //return lockOutCounter > 0;
  return motion;
}

/**
 * @brief
 *  ADC interrupt handler to call PIR detection algorithm
 */
void ADC0_IRQHandler(void)
{
  uint32_t flags;
  flags = ADC_IntGetEnabled(ADC0);
  ADC_IntClear(ADC0, flags);
  NVIC_ClearPendingIRQ(ADC0_IRQn);
  motionDetected = PIR_DetectMotion(_lowPowerMode);
  //DebugPrintf("\r\n** motion: %u!!!\r\n", motionDetected);
  //pirInt = true;
  // Issue a event
}

/**
 * @brief Turns on the LED to indicate motion.
 *
 * @param[in] lowPowerMode
 *  Enables LCD event update when false
 */
void PIR_MotionOn(bool lowPowerMode)
{
  //GPIO_PinOutClear(MOTION_B_PORT, MOTION_B_PIN);
  //GPIO_PinOutSet(MOTION_B_PORT, MOTION_B_PIN);
  Board_SetLed(BOARD_RGB1_R, LED_OFF);
  Board_SetLed(BOARD_RGB1_G, LED_ON);
  ZAF_EventHelperEventEnqueueFromISR(EVENT_APP_MOTION_DETECTED);
}

/**
 * @brief Turns off the LED to indicate end of motion.
 *
 * @param[in] lowPowerMode
 *  Enables LCD event update when false
 */
void PIR_MotionOff(bool lowPowerMode)
{
  //GPIO_PinOutSet(MOTION_B_PORT, MOTION_B_PIN);
  //GPIO_PinOutClear(MOTION_B_PORT, MOTION_B_PIN);
  Board_SetLed(BOARD_RGB1_R, LED_ON);
  Board_SetLed(BOARD_RGB1_G, LED_OFF);
}

/**
 * @brief
 *  Initialize timers for time stamping samples to the GUI.
 *
 * @details
 *  LETIMER0 will operate at 1024 Hz time stamp clock.
 */
/*void PIR_InitTimestampClock(void)
{
   LETIMER Initialization - For time stamping
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_HFLE, true);
  CMU_ClockEnable(cmuClock_LETIMER0, true);
  CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_32); // 32,768 Hz / 32 = 1024 Hz = 976.5625 us period

  LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;
  LETIMER_Init(LETIMER0, &letimerInit);
}*/

/**
 * @brief
 *  Transmits the latest PIR sample over UART. Blocks until transfer completion.
 *
 * @param motionDetected
 *  Motion status is always an active low signal to the PC. They are active high into the MCU.
 */
void PIR_TransmitSample(PirSample_t *pirSample)
{
  uint8_t buffer[BUFFER_LENGTH];
  uint8_t msgLength = 0;                                  /* Length of the message being sent. Better be less than BUFFER_LENGTH... */

  msgLength = sprintf((char*) buffer,
                      "D%ld, %ld, %d, %ld, %ld, %ld, %d\r\n",
                      pirSample->timeMs,
                      pirSample->adc,
                      pirSample->motionB,
                      pirSample->winBase,
                      pirSample->adcThreshHigh,
                      pirSample->adcThreshLow,
                      0);

  if (msgLength >= BUFFER_LENGTH) {
    EFM_ASSERT(false);
  }

  //UART_Send(buffer, msgLength);
}

/**
 * @brief Writes a sample into the queue.
 *
 * @param[in] pirSample
 *  Pointer to a PirSample_t to be written to the queue. The sample is copied by value into the queue.
 */
void PIR_WriteQueue(PirSample_t *pirSample)
{
  if ((iRead == 0 && iWrite == QUEUE_LENGTH - 1)
      || (iWrite == (iRead - 1) % (QUEUE_LENGTH - 1))) {
    /* Queue is full. */
    return;
  } else {
    if (iWrite == -1) {
      /* First element. */
      iWrite = 0;
      iRead = 0;
    } else if (iRead == QUEUE_LENGTH - 1 && iWrite != 0) {
      /* Roll over */
      iWrite = 0;
    } else {
      iWrite++;
    }
    pirQueue[iWrite].timeMs = pirSample->timeMs;
    pirQueue[iWrite].adc = pirSample->adc;
    pirQueue[iWrite].winBase = pirSample->winBase;
    pirQueue[iWrite].adcThreshHigh = pirSample->adcThreshHigh;
    pirQueue[iWrite].adcThreshLow = pirSample->adcThreshLow;
    pirQueue[iWrite].motionB = pirSample->motionB;
  }
}

/**
 * @brief Read out a sample from the queue.
 *
 * @details Returns null if there are no samples available.
 */
PirSample_t* PIR_ReadQueue(void)
{
  if (iRead == -1) {
    /* Empty queue. */
    return NULL;
  }

  PirSample_t *pirSample = &pirQueue[iRead];
  if (iRead == iWrite) {
    /* Queue is empty after this. */
    iRead = -1;
    iWrite = -1;
  } else if (iRead == QUEUE_LENGTH - 1) {
    /* Roll over. */
    iRead = 0;
  } else {
    iRead++;
  }

  return pirSample;
}

/**
 * @brief Get number of samples in the queue.
 */
uint32_t PIR_GetQueueSize(void)
{
  if (iRead == -1 || iWrite == -1) {
    return 0;
  }
  if ((iRead == 0 && iWrite == QUEUE_LENGTH - 1)
      || (iWrite == (iRead - 1) % (QUEUE_LENGTH - 1))) {
    return QUEUE_LENGTH;
  }

  return (iWrite - iRead + 1) % QUEUE_LENGTH;
}
