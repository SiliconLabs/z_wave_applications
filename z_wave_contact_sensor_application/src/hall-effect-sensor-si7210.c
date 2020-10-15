// * Copyright 2018 Silicon Laboratories, Inc.                              *80*

#include "board.h"
#include "i2c-driver.h"
#include "hall-effect-sensor-si7210.h"
#include "stddef.h"
#include "DebugPrintConfig.h"
#define DEBUGPRINT
#include "DebugPrint.h"
#include "gpiointerrupt.h"
#include "events.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include <ZAF_PM_Wrapper.h>


#define SI7210_INIT_DELAY_MS                0
#define SI7210_WRITE_BUFFER_SIZE            2
#define SI7210_I2C_REG_LOCATION             0
#define SI7210_REG_DATA_LOCATION            1

// Action: Define threshold & tamper values by trial & error
#define DEFAULT_THRESHOLD_UT 	1920
#define DEFAULT_TAMPER_UT 		3968

// Action disable pre-defined ember values
// #define DEFAULT_THRESHOLD_UT \
//  EMBER_AF_PLUGIN_HALL_EFFECT_SENSOR_SI7210_THRESHOLD
//// Action: Needs to be set manually here
// #define DEFAULT_TAMPER_UT \
//  EMBER_AF_PLUGIN_HALL_EFFECT_SENSOR_SI7210_TAMPER
//// Action: Needs to be set manually here

#define NUMBER_OF_BITS_HIGH_NIBBLE_THRESHOLD  3
#define NUMBER_OF_BITS_LOW_NIBBLE_THRESHOLD   4
#define NUMBER_OF_BITS_HIGH_NIBBLE_TAMPER     2
#define NUMBER_OF_BITS_LOW_NIBBLE_TAMPER      4
#define SETTING_UT_PER_BIT                    5

//------------------------------------------------------------------------------
// Plugin events

static HalSi7210Configuration_t defaultSi7210Configuration = {
  1,    // active low for output pin;
  0x34, // this calculates the threshold value of 160 by the following formular:
        // threshold = (16 + op[3:0]) x (2 ^ op[6:4]);
        // with 0.005mT/bit on 20mT scale, it represents a 0.8mT threshold
  0,    // omnipolar;
  0x12, // this calculates the hysteresis value of 20 by the following formular:
        // hysteresis = (8 + hyst[2:0]) x (2 ^ hyst[5:3]);
        // with 0.005mT/bit on 20mT scale, it represents a 0.01mT hysteresis
  0x20, // this calculates the tamper value of 2048 by the following formular:
        // tamper = (16 + tamper[3:0]) x (2 ^ (tamper[5:4] + 5));
        // with 0.005mT/bit on 20mT scale, it represent a 10.24 mT tamper
  1 //auto wakeup enabled.
};

//static SPowerLock_t m_RadioPowerLock;
//forward declaration of local static functions
static void si7210Init(HalSi7210Configuration_t * configuration);
static uint8_t computeTamperSettingFromUtValue(uint16_t utValue);
static uint8_t computeThresholdSettingFromUtValue(uint16_t utValue);
static bool readByteFromI2cReg(uint8_t i2cAddress,
                               uint8_t i2cReg,
                               uint8_t * byte);
static bool writeByteToI2cReg(uint8_t i2cAddress,
                              uint8_t i2cReg,
                              uint8_t byte);
static bool wakeUpSensor(void);

//implementation of statis functions
static bool readByteFromI2cReg(uint8_t i2cAddress,
                               uint8_t i2cReg,
                               uint8_t * byte)
{
  uint8_t errorCode;

  errorCode = halI2cWriteBytes(i2cAddress,
                               &i2cReg,
                               sizeof(i2cReg));
  if (errorCode != I2C_DRIVER_ERR_NONE) {
    DPRINTF("Failed to write Si7210 register addr : %x", errorCode);
    return false;
  }

  errorCode = halI2cReadBytes(i2cAddress,
                              byte,
                              sizeof(*byte));
  if (errorCode != I2C_DRIVER_ERR_NONE) {
    DPRINTF("Failed to read Si7210 register : %x", errorCode);
    return false;
  }

  return true;
}

static bool writeByteToI2cReg(uint8_t i2cAddress, uint8_t i2cReg, uint8_t byte)
{
  uint8_t errorCode;
  uint8_t writeBuffer[SI7210_WRITE_BUFFER_SIZE];

  writeBuffer[SI7210_I2C_REG_LOCATION] = i2cReg;
  writeBuffer[SI7210_REG_DATA_LOCATION] = byte;

  errorCode = halI2cWriteBytes(HAL_SI7210_ADDR,
                               writeBuffer,
                               SI7210_WRITE_BUFFER_SIZE);
  if (errorCode != I2C_DRIVER_ERR_NONE) {
    DPRINTF("Failed to write Si7210!");
    return false;
  }
  return true;
}

static void si7210Init(HalSi7210Configuration_t * configuration)
{
  uint8_t idByte, chipId, revId;

  if (!readByteFromI2cReg(HAL_SI7210_ADDR, HAL_SI7210_REGISTER_IDS, &idByte)) {
    DPRINTF("Si7210 init failed, cannot read Id");
// Action: Commented out ember functions
//    emberEventControlSetDelayMS(
//      emberAfPluginHallEffectSensorSi7210InitEventControl,
//      SI7210_INIT_DELAY_MS);
    return;
  }

  chipId = ((idByte >> HAL_SI7210_CHIPID_BIT_SHIFT)
            & HAL_SI7210_CHIPID_BIT_MASK);
  revId = ((idByte >> HAL_SI7210_REVID_BIT_SHIFT)
           & HAL_SI7210_CHIPID_BIT_MASK);
  DPRINTF("ChipId: 0x%x, RevId: 0x%x\n", chipId, revId);

  halHallEffectSensorWriteConfiguration(configuration);
}

static uint8_t computeTamperSettingFromUtValue(uint16_t utValue)
{
  uint8_t lowerNibble, higherNibble;
  uint16_t TamperInSettingValue;

  // compute the tamper setting from plugin option
  // tamper setting = (16 + tamper[3:0]) x (2 ^ (tamper[5:4] + 5));
  // with 0.005mT/bit on 20mT
  for (higherNibble = 0;
       higherNibble < (1 << NUMBER_OF_BITS_HIGH_NIBBLE_TAMPER);
       higherNibble++) {
    for (lowerNibble = 0;
         lowerNibble < (1 << NUMBER_OF_BITS_LOW_NIBBLE_TAMPER);
         lowerNibble++) {
      TamperInSettingValue = (16 + lowerNibble) * (1 << (higherNibble + 5));
      if (TamperInSettingValue > (DEFAULT_TAMPER_UT / SETTING_UT_PER_BIT)) {
        return ((higherNibble << NUMBER_OF_BITS_LOW_NIBBLE_TAMPER)
                + lowerNibble);
      }
    }
  }
  return 0;
}

static uint8_t computeThresholdSettingFromUtValue(uint16_t utValue)
{
  uint8_t lowerNibble, higherNibble;
  uint16_t thresholdInSettingValue;

  // compute the threshold setting from plugin option
  // threshold setting = (16 + op[3:0]) x (2 ^ op[6:4]);
  // with 0.005mT/bit on 20mT scale.
  //
  for (higherNibble = 0;
       higherNibble < (1 << NUMBER_OF_BITS_HIGH_NIBBLE_THRESHOLD);
       higherNibble++) {
    for (lowerNibble = 0;
         lowerNibble < (1 << NUMBER_OF_BITS_LOW_NIBBLE_THRESHOLD);
         lowerNibble++) {
      thresholdInSettingValue = (16 + lowerNibble) * (1 << higherNibble);
      if (thresholdInSettingValue
          > (DEFAULT_THRESHOLD_UT / SETTING_UT_PER_BIT)) {
        return ((higherNibble << NUMBER_OF_BITS_LOW_NIBBLE_THRESHOLD)
                + lowerNibble);
      }
    }
  }
  return 0;
}

static bool wakeUpSensor(void)
{
  uint8_t errorCode;
  uint8_t writeBuffer[1];

  errorCode = halI2cWriteBytes(HAL_SI7210_ADDR,
                               writeBuffer,
                               0);
  if (errorCode != I2C_DRIVER_ERR_NONE) {
    DPRINTF("Failed to write Si7210!");
    return false;
  }
  return true;
}


//------------------------------------------------------------------------------
// Plugin private event handlers

// At this point, I2C driver is guaranteed to have initialized, so it is safe
// to call the i2c based init function
void PluginHallEffectSensorSi7210InitEventHandler(void)
{
  uint8_t thresholdValue;

//  Action: deactivate modification of sw_op & sw_tamper value
  thresholdValue = computeThresholdSettingFromUtValue(DEFAULT_THRESHOLD_UT);
  if (thresholdValue != 0 ) {
    defaultSi7210Configuration.op = thresholdValue;
  }
  thresholdValue = computeTamperSettingFromUtValue(DEFAULT_TAMPER_UT);
  if (thresholdValue != 0 ) {
    defaultSi7210Configuration.tamper = thresholdValue;
  }

  si7210Init(&defaultSi7210Configuration);
}

void PluginHallEffectSensorSi7210ReadEventHandler(void)
{
  int16_t bField;
// Action: disabled ember functions
//  emberEventControlSetInactive(
//    emberAfPluginHallEffectSensorSi7210ReadEventControl);
  halHallEffectSensorSi7210Read(&bField);
}

void EnableSwitch() {

	uint32_t resultPin;
	CMU_ClockEnable(cmuClock_GPIO, true);

	 /* Unlatch EM4 GPIO pin states after wakeup (OK to call even if not EM4 wakeup) */
	 EMU_UnlatchPinRetention();

	 GPIOINT_Init();
     NVIC_SetPriority(GPIO_ODD_IRQn, 5);
     NVIC_SetPriority(GPIO_EVEN_IRQn, 5);

     GPIO_Port_TypeDef port     = gpioPortA;
     uint32_t          pin      = 3;
     uint32_t          on_value = 0;

     // Action: Change from gpioModeInputPullFilter to gpioModeInputPull to see if voltage spike upon moving the magnet to the sensor is removed
     GPIO_PinModeSet(gpioPortA, 3, gpioModeInputPullFilter, 1);	// Input enabled with pull-up
#if 1
     //GPIO_DriveModeSet(gpioPortA,gpioDriveModeLowest);
     //GPIO_DriveStrengthSet(gpioPortA, gpioDriveStrengthWeakAlternateWeak);
#endif
     // returns 1 when Magnet is close and 0 when no magnet is present
     resultPin = GPIO_PinInGet(gpioPortA, 3);
     DPRINTF("\r\nResult Pin from EnableSwitch() %d\r\n", resultPin);

     //implement logic value here
     (1 == resultPin) ? (on_value = 0) : (on_value = 1);
     ButtonEnableEM4PinWakeup(port, pin, on_value);
     GPIO_ExtIntConfig(port, pin, pin, true, true, true);
}

//------------------------------------------------------------------------------
// Plugin defined callbacks

// The init callback, which will be called by the framework on init.
void emberAfPluginHallEffectSensorSi7210InitCallback(void)
{
// Action: disabled ember functions
//  emberEventControlSetDelayMS(emberAfPluginHallEffectSensorSi7210InitEventControl,
//                              SI7210_INIT_DELAY_MS);
}

//------------------------------------------------------------------------------
// Plugin public functions
bool halHallEffectSensorWriteConfiguration(
  HalSi7210Configuration_t * configuration)
{
  uint8_t dataByte;

  if (configuration == NULL) {
    DPRINTF("NUll input pointer!");
    return false;
  }

  //config the HAL_SI7210_REGISTER_OUTPUT_POL_OP register
  dataByte = (((configuration->low4Field & HAL_SI7210_LOW4FIELD_BIT_MASK)
               << HAL_SI7210_LOW4FIELD_BIT_SHIFT)
              | ((configuration->op & HAL_SI7210_OP_BIT_MASK)
                 << HAL_SI7210_OP_BIT_SHIFT));
  writeByteToI2cReg(HAL_SI7210_ADDR,
                    HAL_SI7210_REGISTER_OUTPUT_POL_OP,
                    dataByte);

  //config the HAL_SI7210_REGISTER_FIELD_POL_HYST register
  dataByte = (((configuration->fieldPolSel & HAL_SI7210_FIELD_POL_BIT_MASK)
               << HAL_SI7210_FIELD_POL_BIT_SHIFT)
              | ((configuration->hyst & HAL_SI7210_HYST_BIT_MASK)
                 << HAL_SI7210_HYST_BIT_SHIFT));
  writeByteToI2cReg(HAL_SI7210_ADDR,
                    HAL_SI7210_REGISTER_FIELD_POL_HYST,
                    dataByte);

  //config the HAL_SI7210_REGISTER_TAMPER_SL register
  readByteFromI2cReg(HAL_SI7210_ADDR,
                     HAL_SI7210_REGISTER_TAMPER_SL,
                     &dataByte);
  dataByte &= ~(HAL_SI7210_TAMPER_BIT_MASK << HAL_SI7210_TAMPER_BIT_SHIFT);
  dataByte |= ((configuration->tamper & HAL_SI7210_TAMPER_BIT_MASK)
               << HAL_SI7210_TAMPER_BIT_SHIFT);
  writeByteToI2cReg(HAL_SI7210_ADDR,
                    HAL_SI7210_REGISTER_TAMPER_SL,
                    dataByte);

  //config the HAL_SI7210_REGISTER_MEASURE register
  readByteFromI2cReg(HAL_SI7210_ADDR,
                     HAL_SI7210_REGISTER_MEASURE,
                     &dataByte);

  dataByte &= ~((HAL_SI7210_ONE_BURST_BIT_MASK
                 << HAL_SI7210_ONE_BURST_BIT_SHIFT)
                | (HAL_SI7210_STOP_BIT_MASK
                   << HAL_SI7210_STOP_BIT_SHIFT)
                | (HAL_SI7210_USESTORE_BIT_MASK
                   << HAL_SI7210_USESTORE_BIT_SHIFT));
  dataByte |= (((0 & HAL_SI7210_ONE_BURST_BIT_MASK)
                << HAL_SI7210_ONE_BURST_BIT_SHIFT)
               | ((0 & HAL_SI7210_STOP_BIT_MASK)
                  << HAL_SI7210_STOP_BIT_SHIFT)
               | ((1 & HAL_SI7210_USESTORE_BIT_MASK)
                  << HAL_SI7210_USESTORE_BIT_SHIFT));
  writeByteToI2cReg(HAL_SI7210_ADDR, HAL_SI7210_REGISTER_MEASURE, dataByte);

  return true;
}

bool halHallEffectSensorGetConfiguration(
  HalSi7210Configuration_t * configuration)
{
  return true;
}

//******************************************************************************
// Perform all I2C transactions to retrieve measurements from SI7210
//******************************************************************************
bool halHallEffectSensorSi7210Read(int16_t * bField)
{
  uint8_t dspSigM, dspSigL, dataByte;

  wakeUpSensor();

  readByteFromI2cReg(HAL_SI7210_ADDR,
                     HAL_SI7210_REGISTER_MEASURE,
                     &dataByte);

  dataByte &= ~((HAL_SI7210_ONE_BURST_BIT_MASK
                 << HAL_SI7210_ONE_BURST_BIT_SHIFT)
                | (HAL_SI7210_STOP_BIT_MASK
                   << HAL_SI7210_STOP_BIT_SHIFT)
                | (HAL_SI7210_USESTORE_BIT_MASK
                   << HAL_SI7210_USESTORE_BIT_SHIFT));
  //perform one conversion
  dataByte |= (((1 & HAL_SI7210_ONE_BURST_BIT_MASK)
                << HAL_SI7210_ONE_BURST_BIT_SHIFT)
               | ((0 & HAL_SI7210_STOP_BIT_MASK)
                  << HAL_SI7210_STOP_BIT_SHIFT)
               | ((1 & HAL_SI7210_USESTORE_BIT_MASK)
                  << HAL_SI7210_USESTORE_BIT_SHIFT));
  writeByteToI2cReg(HAL_SI7210_ADDR, HAL_SI7210_REGISTER_MEASURE, dataByte);

  if (!readByteFromI2cReg(HAL_SI7210_ADDR,
                          HAL_SI7210_REGISTER_DSPSIGM,
                          &dspSigM)) {
    DPRINTF("Si7210 read data failed");
    return false;
  }

  if (!readByteFromI2cReg(HAL_SI7210_ADDR,
                          HAL_SI7210_REGISTER_DSPSIGL,
                          &dspSigL)) {
    DPRINTF("Si7210 read data failed");
    return false;
  }

  // accroding the the Si7210 spec, the bField is calculated from the
  // following formula
  // Bfield = ((Dspsigm[6:0] * 256) + Dspsigl[7:0] -16384) * 1.25uT
  *bField = ((((((uint16_t)dspSigM & HAL_SI7210_DSPSIGM_BIT_MASK) * 256)
               + dspSigL) - 16384) * 5) / 4;

  DPRINTF("bfield : %d uT\n", *bField);

  if ((*bField >  DEFAULT_TAMPER_UT) || (*bField < -DEFAULT_TAMPER_UT)) {
// Action: comment below function
//    emberPluginHallEffectSensorSi7210TamperAlarmCallback();
	  DPRINTF("Tamper detected\n");
  }

  halHallEffectSensorWriteConfiguration(&defaultSi7210Configuration);

  return true;
}

//******************************************************************************
// Perform all I2C transactions to retrieve measurements from SI7210
//******************************************************************************

