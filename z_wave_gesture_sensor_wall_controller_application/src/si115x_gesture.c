/**************************************************************************//**
 * @file si115x_gesture.c
 * @brief si115x gesture algorithm
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

#include "si115x_gesture.h"

#define PS_THRESHOLD 1500

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
int16_t si115x_init_gesture(SI115X_TypeDef *si115x)
{
  int16_t  retval;
  SI115X_Init_TypeDef init = SI115X_INIT_DEFAULT;
  //init.measrate = 25; /* 50 Hz sample rate */
  init.led1a_current = si115xLedCurrent354mA;
  init.led2a_current = si115xLedCurrent354mA;
  init.led3a_current = si115xLedCurrent354mA;

  SI115X_InitChannel_TypeDef initProximityCH0 = SI115X_INIT_CHANNEL_FORCED_DEFAULT;
  initProximityCH0.adcMux = si115xAdcMuxLargeIr;
  initProximityCH0.hwGain = si115xHwGain2x;
  initProximityCH0.led1En = true;  // Enable LED1
  initProximityCH0.res = si115xRes16bUint;

  SI115X_InitChannel_TypeDef initProximityCH1 = SI115X_INIT_CHANNEL_FORCED_DEFAULT;
  initProximityCH1.adcMux = si115xAdcMuxLargeIr;
  initProximityCH1.hwGain = si115xHwGain2x;
  initProximityCH1.led3En = true;  // Enable LED3
  initProximityCH1.res = si115xRes16bUint;

  SI115X_InitChannel_TypeDef initProximityCH2 = SI115X_INIT_CHANNEL_FORCED_DEFAULT;
  initProximityCH2.adcMux = si115xAdcMuxLargeIr;
  initProximityCH2.hwGain = si115xHwGain2x;
  initProximityCH2.led2En = true;  // Enable LED2
  initProximityCH2.res = si115xRes16bUint;

  retval = SI115X_Init(si115x, &init);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  retval = SI115X_InitChannel(si115x, 0, &initProximityCH0);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  retval = SI115X_InitChannel(si115x, 1, &initProximityCH1);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  retval = SI115X_InitChannel(si115x, 2, &initProximityCH2);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  // Enable interrupt
  //retval = SI115X_IntEnable(si115x, 1 << PROXIMITY_CHANNEL);  // Enable interrupt
  //if (retval != SI115X_ECODE_OK) {
  //  return retval;
  //}

  return SI115X_ECODE_OK;
}

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
void si115x_gesture_handler(SI115X_TypeDef *si115x, si115x_gesture_sample_t *samples)
{

  uint8_t buffer[7];
  SI115X_BlockRead(si115x,
                   SI115X_REG_IRQ_STATUS,
                   7,
                   buffer);
  //samples->irq_status = buffer[0];
  samples->ps1  = buffer[1] <<  8;
  samples->ps1 |= buffer[2];
  samples->ps2  = buffer[3] <<  8;
  samples->ps2 |= buffer[4];
  samples->ps3  = buffer[5] <<  8;
  samples->ps3 |= buffer[6];
}

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
gesture_t si115x_gesture_algorithm(si115x_gesture_sample_t *samples)
{
  uint16_t        ps[3];

  static uint32_t ps_entry_time[3] = { 0, 0, 0 };
  static uint32_t ps_exit_time[3]  = { 0, 0, 0 };

  static uint8_t  ps_state[3] = { 0, 0, 0 };

  uint8_t         array_counter;
  uint32_t diff_x ;
  uint32_t diff_y1 ;
  uint32_t diff_y2 ;
  uint32_t ps_time[3] ;
  uint32_t ps_avg;
  gesture_t  ret = NONE;  /*gesture result return value */
  /*save new samples into ps array */
  ps[0] = samples->ps1;
  ps[1] = samples->ps2;
  ps[2] = samples->ps3;

  /* Check state of all three measurements */
  for (array_counter = 0; array_counter < 3; array_counter++)
  {
    /* If measurement higher than the ps_threshold value, */
    /*   record the time of entry and change the state to look for the exit time */
    if (ps[array_counter] >= PS_THRESHOLD)
    {
      ret = PROX;
      if (ps_state[array_counter] == 0)
      {
        ps_state[array_counter]      = 1;
        ps_entry_time[array_counter] = samples->timestamp;
      }
    }
    else
    {
      if (ps_state[array_counter] == 1)
      {
        ps_state[array_counter]     = 0;
        ps_exit_time[array_counter] = samples->timestamp;
      }
    }
  }

  /* If there is no object in front of the board, look at history to see if a gesture occured */
  if ((ps[0] < PS_THRESHOLD) && (ps[1] < PS_THRESHOLD) && (ps[2] < PS_THRESHOLD))
  {
    /* If the ps_max values are high enough and there exit entry and exit times, */
    /*   then begin processing gestures */
    if ((ps_entry_time[0] != 0) && (ps_entry_time[1] != 0) && (ps_entry_time[2] != 0)
        && (ps_exit_time[0] != 0) && (ps_exit_time[1] != 0) && (ps_exit_time[2] != 0))
    {
      /* Make sure no timestamps overflowed, indicated possibility if any of them are close to overflowing */
      if ((ps_exit_time[0] > 0xFC000000L) || (ps_exit_time[1] > 0xFC000000L) || (ps_exit_time[2] > 0xFC000000L)
          || (ps_entry_time[0] > 0xFC000000L) || (ps_entry_time[1] > 0xFC000000L) || (ps_entry_time[2] > 0xFC000000L))
      {         /* If any of them are close to overflowing, overflow them all so they all have the same reference */
        ps_exit_time[0] += 0x1FFFFFFFL;
        ps_exit_time[1] += 0x1FFFFFFFL;
        ps_exit_time[2] += 0x1FFFFFFFL;

        ps_entry_time[0] += 0x1FFFFFFFL;
        ps_entry_time[1] += 0x1FFFFFFFL;
        ps_entry_time[2] += 0x1FFFFFFFL;
      }

      /* Calculate the midpoint (between entry and exit times) of each waveform */
      /*  the order of these midpoints helps determine the gesture */
      ps_time[0] = (ps_exit_time[0] - ps_entry_time[0]) / 2;
      ps_time[0] = ps_time[0] + ps_entry_time[0];

      ps_time[1] = (ps_exit_time[1] - ps_entry_time[1]) / 2;
      ps_time[1] = ps_time[1] + ps_entry_time[1];

      ps_time[2] = (ps_exit_time[2] - ps_entry_time[2]) / 2;
      ps_time[2] = ps_time[2] + ps_entry_time[2];

      /* The diff_x and diff_y values help determine a gesture by comparing the */
      /*  LED measurements that are on a single axis */
      if (ps_time[1] > ps_time[2])
      {
        diff_x = ps_time[1] - ps_time[2];
      }
      else
      {
        diff_x = ps_time[2] - ps_time[1];
      }
      if( ps_time[0] > ps_time[1] )
      {
        diff_y1 = ps_time[0] - ps_time[1];
      }
	  else
      {
        diff_y1 = ps_time[1] - ps_time[0];
      }

      if( ps_time[0] > ps_time[2] )
      {
        diff_y2 = ps_time[0] - ps_time[2];
      }
	  else
      {
        diff_y2 = ps_time[2] - ps_time[0];
      }


      /* Take the average of all three midpoints to make a comparison point for each midpoint */
      ps_avg = (uint32_t) ps_time[0] + (uint32_t) ps_time[1] + (uint32_t) ps_time[2];
      ps_avg = ps_avg / 3;

      if ((ps_exit_time[0] - ps_entry_time[0]) > 10 || (ps_exit_time[1] - ps_entry_time[1]) > 10 || (ps_exit_time[2] - ps_entry_time[2]) > 10)
      {
        if( ( (ps_time[0] < ps_time[1]) &&  (diff_y1 > diff_x) ) || ( (ps_time[0] <= ps_time[2]) && (diff_y2 > diff_x) ) )
        {           /* An up gesture occured if the bottom LED had its midpoint first */
          ret = RIGHT;//was  UP;//
        }
        else if  ( ( (ps_time[0] < ps_time[1]) &&  (diff_y1 > diff_x) ) || ( (ps_time[0] > ps_time[2]) && (diff_y2 > diff_x) ) )
        {           /* A down gesture occured if the bottom LED had its midpoint last */
          ret = LEFT; // was DOWN;
        }
        else if((ps_time[0] < ps_time[1]) && (ps_time[2] < ps_time[1]) && (diff_x > ((diff_y1+diff_y2)/2)))
        {           /* A left gesture occured if the left LED had its midpoint last */
          ret = UP;// was LEFT;
        }
        else if( (ps_time[0] < ps_time[2]) && (ps_time[1] < ps_time[2])  && (diff_x > ((diff_y1+diff_y2)/2)))
        {           /* A right gesture occured if the right LED had midpoint later than the right LED */
          ret = DOWN; // was RIGHT;
        }
      }
    }
    for (array_counter = 0; array_counter < 3; array_counter++)
    {
      ps_exit_time[array_counter]  = 0;
      ps_entry_time[array_counter] = 0;
    }
  }

  return ret;
}
