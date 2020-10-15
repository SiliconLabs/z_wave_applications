/*
 * hall-effect-sensor-si7210.h
 *
 *  Created on: Jun 13, 2019
 *      Author: axbrugge
 */

#ifndef INC_HALL_EFFECT_SENSOR_SI7210_H_
#define INC_HALL_EFFECT_SENSOR_SI7210_H_

// * Copyright 2018 Silicon Laboratories, Inc.                              *80*

#include <stdbool.h>
#include <stdint.h>

#define HAL_SI7210_ADDR                      (0x30 << 1)

// Macros used to interface with I2C registers
#define HAL_SI7210_REGISTER_IDS               0xC0
#define HAL_SI7210_REGISTER_DSPSIGM           0xC1
#define HAL_SI7210_REGISTER_DSPSIGL           0xC2
#define HAL_SI7210_REGISTER_DSPSIGSEL         0xC3
#define HAL_SI7210_REGISTER_MEASURE           0xC4
#define HAL_SI7210_REGISTER_ARAUTOINC         0xC5
#define HAL_SI7210_REGISTER_OUTPUT_POL_OP     0xC6
#define HAL_SI7210_REGISTER_FIELD_POL_HYST    0xC7
#define HAL_SI7210_REGISTER_SLTIME            0xC8
#define HAL_SI7210_REGISTER_TAMPER_SL         0xC9
#define HAL_SI7210_REGISTER_DF                0xCD

#define HAL_SI7210_REVID_BIT_SHIFT      0
#define HAL_SI7210_REVID_BIT_MASK       0x0F
#define HAL_SI7210_CHIPID_BIT_SHIFT     4
#define HAL_SI7210_CHIPID_BIT_MASK      0x0F
#define HAL_SI7210_DSPSIGSEL_BIT_SHIFT  0
#define HAL_SI7210_DSPSIGSEL_BIT_MASK   0x07
#define HAL_SI7210_MEAS_BIT_SHIFT       7
#define HAL_SI7210_MEAS_BIT_MASK        0x01
#define HAL_SI7210_USESTORE_BIT_SHIFT   3
#define HAL_SI7210_USESTORE_BIT_MASK    0x01
#define HAL_SI7210_ONE_BURST_BIT_SHIFT  2
#define HAL_SI7210_ONE_BURST_BIT_MASK   0x01
#define HAL_SI7210_STOP_BIT_SHIFT       1
#define HAL_SI7210_STOP_BIT_MASK        0x01
#define HAL_SI7210_SLEEP_BIT_SHIFT      0
#define HAL_SI7210_SLEEP_BIT_MASK       0x01
#define HAL_SI7210_ARAUTOINC_BIT_SHIFT  0
#define HAL_SI7210_ARAUTOINC_BIT_MASK   0x01
#define HAL_SI7210_LOW4FIELD_BIT_SHIFT  7
#define HAL_SI7210_LOW4FIELD_BIT_MASK   0x01
#define HAL_SI7210_OP_BIT_SHIFT         0
#define HAL_SI7210_OP_BIT_MASK          0x7F
#define HAL_SI7210_FIELD_POL_BIT_SHIFT  6
#define HAL_SI7210_FIELD_POL_BIT_MASK   0x03
#define HAL_SI7210_HYST_BIT_SHIFT       0
#define HAL_SI7210_HYST_BIT_MASK        0x3F
#define HAL_SI7210_SLTIME_BIT_SHIFT
#define HAL_SI7210_SLTIME_BIT_MASK      0xFF
#define HAL_SI7210_TAMPER_BIT_SHIFT     2
#define HAL_SI7210_TAMPER_BIT_MASK      0x3F
#define HAL_SI7210_SLFAST_BIT_SHIFT     1
#define HAL_SI7210_SLFAST_BIT_MASK      0x01
#define HAL_SI7210_SLTIMEENA_BIT_SHIFT  0
#define HAL_SI7210_SLTIMEENA_BIT_MASK   0x01
#define HAL_SI7210_DSPSIGM_BIT_MASK     0x7F


// Struct used to store configuration messages, which is the data sent to the
// device on initialization
typedef struct HalSi7210Configuration{
  uint8_t low4Field;   // 0: active high output pin;
                       // 1: active low output pin
  uint8_t op;          // 7 bit threshold for high/low on output pin
  uint8_t fieldPolSel; // 2 bit field polarity selection:
                       // 00 : omnipolar
                       // 01 : unipolar negative
                       // 10 : unipolar positive
                       // 11 : unused
  uint8_t hyst;        // 6 bit hysteresis for output pin
  uint8_t tamper;      // 6 bit tamper value
  uint8_t slTimeena;   // auto wake up enable
} HalSi7210Configuration_t;

/** @brief Perform a read of the PYD1698
 *
 * This function can be used to generate a read of the ADC and configuration
 * state of the occupancy sensor
 *
 * @param bField  The B field read from the hall effect sensor,
 *                in unit of uT
 *
 * @return true: operation is successful, false: operation failed.
 */
bool halHallEffectSensorSi7210Read(int16_t * bField);

/** @brief Write a new set of configuration values to the sensor
 *
 * This function can be used to write new configuration parameters to the
 * sensor.  Note that it takes in a structure of all configuration parameters,
 * so if a single value is to be written, it is recommended to first make a call
 * to halHallEffectSensorGetConfiguration to first determine what the
 * state of the hall effect sensor is, modify the parameters that are to be
 * changed, and use that structure as the input to this write function.
 *
 * @param configuration: The entire configuration to be written to the sensor
 *
 * @return true: operation successful; false operation unsuccessful
 */
bool halHallEffectSensorWriteConfiguration(HalSi7210Configuration_t * configuration);

/** @brief Get the current configuration of the sensor
 *
 * This function can be used to determine how the hall effect sensor is
 * currently configured.
 *
 * @param configuration: The structure in which the current configuration
 * should be stored.
 *
 * @return true: operation successful; false operation unsuccessful
 */
bool halHallEffectSensorGetConfiguration(HalSi7210Configuration_t *configuration);


#endif /* INC_HALL_EFFECT_SENSOR_SI7210_H_ */
