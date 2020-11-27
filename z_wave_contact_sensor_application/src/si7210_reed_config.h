/**************************************************************************//**
 * @file si7210.h
 * @brief public header for Si7210 Hall Sensor configuration
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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "em_gpio.h"

#define   BOARD_OK   0
#define   BOARD_ERROR_I2C_TRANSFER_TIMEOUT   0x01
#define   BOARD_ERROR_I2C_TRANSFER_NACK   0x02
#define   BOARD_ERROR_I2C_TRANSFER_FAILED   0x03
#define   BOARD_ERROR_PIC_ID_MISMATCH   0x04
#define   BOARD_ERROR_PIC_FW_INVALID   0x05
#define   BOARD_ERROR_PIC_FW_UPDATE_FAILED   0x06
#define   BOARD_ERROR_NO_POWER_INT_CTRL   0x10
#define   BOARD_ERROR_I2C_BUS_SELECT_INVALID   0x11
#define   BOARD_ERROR_I2C_BUS_SELECT_FAILED   0x12

// Define Hall out / Reed Switch in
#define HALL_GPIO_PORT_OUT   gpioPortA
#define HALL_GPIO_PIN_OUT    3

#define SI7210_I2C_BUS_TIMEOUT  1000
#define SI7210_I2C_BUS_ADDRESS  0x30
#define SI7210_DEVICE_ID        0x13
#define SI7210_CHIP_ID          0x01
#define SI7210_REV_ID_MIN       0x03
#define SI7210_REV_ID_MAX       0x04


// change to zgm130s configuration
#define SI7210_I2C_DEVICE       (I2C0)
#define SI7210_SDA_LOCATION     (I2C_ROUTELOC0_SDALOC_LOC8)
#define SI7210_SCL_LOCATION     (I2C_ROUTELOC0_SCLLOC_LOC8)
#define SI7210_SDA_LOC          15
#define SI7210_SCL_LOC          15
#define SI7210_SDA_PORT         gpioPortC
#define SI7210_SDA_PIN          10
#define SI7210_SCL_PORT         gpioPortC
#define SI7210_SCL_PIN          11

#define I2CSPM_INIT_SI7210                                                   \
  { SI7210_I2C_DEVICE,        /* I2C instance                             */ \
    SI7210_SCL_PORT,          /* SCL port                                 */ \
    SI7210_SCL_PIN,           /* SCL pin                                  */ \
    SI7210_SDA_PORT,          /* SDA port                                 */ \
    SI7210_SDA_PIN,           /* SDA pin                                  */ \
    SI7210_SCL_LOC,           /* Port location of SCL signal              */ \
    SI7210_SDA_LOC,           /* Port location of SDA signal              */ \
    0,                        /* Use currently configured reference clock */ \
    I2C_FREQ_STANDARD_MAX,    /* Set to standard rate                     */ \
    i2cClockHLRStandard,      /* Set to use 4:4 low/high duty cycle       */ \
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
uint32_t BOARD_i2cBusSelect(uint8_t select);

/***************************************************************************//**
 * @brief
 *    Initialize I2C peripheral
 ******************************************************************************/
void initI2C (void);

/***************************************************************************//**
 * @brief
 *   Initializing and configuring si7210 hall sensor
 *
 ******************************************************************************/
uint8_t configHallSensor(void);

/***************************************************************************//**
 * @brief
 *   Configuration of the GPIO (Pin 3 on Port A) that is connected to the Hall Sensor output / Reed Switch input
 *
 ******************************************************************************/
void ConfigSensorPin();

/**
 * Configures a GPIO pin so it can wake up the system from deep sleep (EM4).
 *
 * Only available for a subset of pins.
 *
 * @param port      Port number
 * @param pin       Pin number
 * @param on_value  The pin value (0 or 1) that should trigger the wakeup.
 * @return          True if deep sleep wake up is possible for the
 *                  specified pin. Returns false otherwise.
 */
bool ButtonEnableEM4PinWakeup(GPIO_Port_TypeDef port, uint32_t pin, uint32_t on_value);
