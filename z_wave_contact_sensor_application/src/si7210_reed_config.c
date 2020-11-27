/**************************************************************************//**
 * @file si7210_config.c
 * @brief Si7210 Hall Sensor configuration
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

#include <si7210_reed_config.h>
#include <stdint.h>
#include <stddef.h>
#include "board.h"
#include "thunderboard/hall_si7210.h"
#include "thunderboard/hall_device.h"
#include "thunderboard/hall.h"
#include "i2cspm.h"
#include "em_i2c.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#define DEBUGPRINT
#include "DebugPrint.h"

#define CONFIG_OK           0
#define INIT_FAIL           255
#define CONFIG_FAIL         255
#define THRESHOLD_MT        0.8
#define HYSTERESIS_MT       0.2
#define OMNIPOLAR           0
#define UNIPOLAR_NEG        1
#define UNIPOLAR_POS        2
#define NON_INVERSE_POL     0
#define INVERSE_POL         1



/***************************************************************************//**
 * @brief
 *    I2C configuration for Si7021 & Si7021
 ******************************************************************************/
#define I2CSPM_INIT_SENSOR                                                    \
  { I2C0,                      /* Use I2C instance 0 */                       \
    gpioPortC,                 /* SCL port */                                 \
    11,                        /* SCL pin */                                  \
    gpioPortC,                 /* SDA port */                                 \
    10,                        /* SDA pin */                                  \
    15,                        /* Location of SCL */                          \
    15,                        /* Location of SDA */                          \
    0,                         /* Use currently configured reference clock */ \
    I2C_FREQ_STANDARD_MAX,     /* Set to standard rate  */                    \
    i2cClockHLRStandard,       /* Set to use 4:4 low/high duty cycle */       \
  }


/***************************************************************************//**
 * @brief
 *    Assignment of I2C struct variable
 ******************************************************************************/
static I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_SENSOR;


/***************************************************************************//**
 * @brief
 *    Assignment of Default Si7210 settings.
 *    Threshold (in mT)
 *    Hysteresis (in mT)
 *    Setting magnetic field polarity (sw_fieldpolsel field in register 0xC7)
 *    Setting sw_low4field field in register 0xC6 (Output = high when magnetic field is > threshold)
 ******************************************************************************/

HALL_Config SI7210_CONFIG = {
    THRESHOLD_MT,
    HYSTERESIS_MT,
    OMNIPOLAR,
    NON_INVERSE_POL
};

/***************************************************************************//**
 * @brief
 *    Initialize I2C peripheral
 ******************************************************************************/
void initI2C (void) {
  I2CSPM_Init(&i2cInit);
}

/***************************************************************************//**
 * @brief
 *    Sets up the route register of the I2C device to use the correct
 *    set of pins
 *
 * * @param[in] select
 *    The I2C bus route to use (None, Environmental sensors, Gas sensor, Hall
 *    sensor)
 *
 * @return
 *    Returns zero on OK, non-zero otherwise
 ******************************************************************************/
uint32_t BOARD_i2cBusSelect(uint8_t select)
{
  uint32_t status;

  (void) select;

  status = BOARD_OK;

  return status;
}

/***************************************************************************//**
 * @brief
 *   Initializing and configuring si7210 hall sensor
 *
 ******************************************************************************/
uint8_t configHallSensor(void) {
  uint32_t retInitDevice = HALL_initDevice();
  if (retInitDevice != HALL_OK) {
      return INIT_FAIL;
  }
  uint32_t retConfigDevice = HALL_configure(&SI7210_CONFIG);
  if (retConfigDevice != HALL_OK) {
        return CONFIG_FAIL;
    }
  return CONFIG_FAIL;
}


/***************************************************************************//**
 * @brief
 *   Configuration of the GPIO (Pin 3 on Port A) connected to the Hall Sensor output / Reed Switch input
 *
 ******************************************************************************/
void ConfigSensorPin(void) {

  uint32_t resultPin;
  EMU_UnlatchPinRetention();

  GPIOINT_Init();
  NVIC_SetPriority(GPIO_ODD_IRQn, 5);
  NVIC_SetPriority(GPIO_EVEN_IRQn, 5);

  GPIO_Port_TypeDef port     = gpioPortA;
  uint32_t          pin      = 3;
  uint32_t          on_value = 0;

  GPIO_PinModeSet(gpioPortA, 3, gpioModeInputPullFilter, 1);

  // returns 1 when Magnet is close and 0 when no magnet is present
  resultPin = GPIO_PinInGet(gpioPortA, 3);
  DPRINTF("\r\nHall Sensor output / Reed Switch input: %d\r\n", resultPin);
  (1 == resultPin) ? (on_value = 0) : (on_value = 1);
  ButtonEnableEM4PinWakeup(port, pin, on_value);
  GPIO_ExtIntConfig(port, pin, pin, true, true, true);
}


