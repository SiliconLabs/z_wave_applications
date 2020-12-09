/**************************************************************************//**
 * @file si115x_gesture.h
 * @brief public header for si115x gesture algorithm
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
#include "si115x.h"

/* The following section is for functions related to Gesture */
typedef enum
{
  NONE,
  UP,
  DOWN,
  LEFT,
  RIGHT,
  PROX
} gesture_t;

typedef struct
{
  uint32_t timestamp;        /* Timestamp to record */
  int32_t ps1;               /* PS1 */
  int32_t ps2;               /* PS2 */
  int32_t ps3;               /* PS3 */
} si115x_gesture_sample_t;

/**************************************************************************//**
 * @brief
 *  Implements the algorithm for detecting gestures on the sensor STK.
 *  Should be called with new sample data every time an interrupt is
 *  received.
 *
 * @param[in] samples
 *   New sample data received from the sensor.
 *
 * @return
 *   Returns the type of gesture detected (as defined by gesture_t).
 *****************************************************************************/
gesture_t si115x_gesture_algorithm(si115x_gesture_sample_t *samples);

/**************************************************************************//**
 * @brief
 *  Initializes si115x for gesture measurements.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  0 on success, negative if error.
 *****************************************************************************/
int16_t si115x_init_gesture(SI115X_TypeDef *si115x);

/**************************************************************************//**
 * @brief
 *  Reads the si115x sample.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[out] samples
 *  Returns a point to si115x_gesture_sample_t structure.
 *****************************************************************************/
void si115x_gesture_handler(SI115X_TypeDef *si115x, si115x_gesture_sample_t *samples);


