/***************************************************************************//**
 * @file si115x.c
 * @brief Driver for the Si115x Optical Sensor
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
 * # Evaluation Quality
 * This code has been minimally tested to ensure that it builds and is suitable
 * as a demonstration for evaluation purposes only. This code will be maintained
 * at the sole discretion of Silicon Labs.
 ******************************************************************************/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2cspm.h"
#include "si115x.h"

/***************************************************************************//**
 * @addtogroup kitdrv
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup Si115x
 * @brief Silicon Labs Si115x Optical Sensor Driver
 * @details
 * @{
 ******************************************************************************/

/*******************************************************************************
 *****************************   I2C FUNCTIONS *********************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief
 *  Writes to an I2C register.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] addr
 *  I2C register address.
 *
 * @param[in] data
 *  Data to write to the register.
 *
 * @return
 *  0 on success, negative if an I2C error occurred.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_WriteToRegister(SI115X_TypeDef *si115x,
                                            uint8_t addr, uint8_t data)
{
  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }
  if (addr > _SI115X_MAX_REGISTER_ADDR) {
    return SI115X_ECODE_PARAM_ERROR;
  }

  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[2];
  uint8_t i2c_read_data[1];

  seq.addr = si115x->i2cAddress << 1;
  seq.flags = I2C_FLAG_WRITE;

  /* Select register and data to write */
  i2c_write_data[0] = addr;
  i2c_write_data[1] = data;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len = 2;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len = 0;

  ret = I2CSPM_Transfer(si115x->i2cPort, &seq);

  if (ret != i2cTransferDone) {
    return SI115X_ECODE_I2C_ERROR;
  }
  return SI115X_ECODE_OK;
}

/**************************************************************************//**
 * @brief
 *  Reads from an I2C register
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] address
 *  I2C register address.
 *
 * @param[out] data
 *  Output of the I2C read.
 *
 * @return
 *  The register value if non-negative, I2C error if negative.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_ReadFromRegister(SI115X_TypeDef *si115x,
                                             uint8_t address, uint8_t *data)
{
  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }
  if (address > _SI115X_MAX_REGISTER_ADDR) {
    return SI115X_ECODE_PARAM_ERROR;
  }

  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  *data = 0xFF;
  uint8_t i2c_write_data[1];

  seq.addr = si115x->i2cAddress << 1;
  seq.flags = I2C_FLAG_WRITE_READ;
  /* Select register to start reading from */
  i2c_write_data[0] = address;
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len = 1;
  /* Select length of data to be read */
  seq.buf[1].data = data;
  seq.buf[1].len = 1;

  ret = I2CSPM_Transfer(si115x->i2cPort, &seq);

  if (ret != i2cTransferDone) {
    return SI115X_ECODE_I2C_ERROR;
  }
  return SI115X_ECODE_OK;
}

/**************************************************************************//**
 * @brief
 *  Writes to a block of I2C registers.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] address
 *  I2C register address.
 *
 * @param[in] length
 *  Number of registers to write. Must be less than SI115X_MAX_BLOCK_WRITE_LENGTH.
 *
 * @param[in] data
 *  Pointer to the data to write to the registers.
 *
 * @return
 *  0 on success, negative if an I2C error occurred.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_BlockWrite(SI115X_TypeDef *si115x, uint8_t address,
                                       uint8_t length, uint8_t *data)
{
  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }
  if (length > _SI115X_MAX_BLOCK_WRITE_LENGTH) {
    return SI115X_ECODE_PARAM_ERROR;
  }

  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[_SI115X_MAX_BLOCK_WRITE_LENGTH];
  uint8_t i2c_read_data[1];

  seq.addr = si115x->i2cAddress << 1;
  seq.flags = I2C_FLAG_WRITE;

  /* Select register to start writing to*/
  i2c_write_data[0] = address;
  for (uint8_t i = 0; i < length; i++) {
    i2c_write_data[i + 1] = data[i];
  }
  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len = 1 + length;
  seq.buf[1].data = i2c_read_data;
  seq.buf[1].len = 0;

  ret = I2CSPM_Transfer(si115x->i2cPort, &seq);

  if (ret != i2cTransferDone) {
    return SI115X_ECODE_I2C_ERROR;
  }

  return SI115X_ECODE_OK;
}

/**************************************************************************//**
 * @brief
 *  Reads from a block of I2C registers.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] address
 *  I2C register address.
 *
 * @param[in] length
 *  Number of registers to read.
 *
 * @param[out] data
 *  Pointer to an array to store the data. Must equal or larger than the number of registers read.
 *
 * @return
 *  0 on success, negative if an I2C error occurred.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_BlockRead(SI115X_TypeDef *si115x, uint8_t address,
                                      uint8_t length, uint8_t *data)
{
  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }

  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;
  uint8_t i2c_write_data[1];

  seq.addr = si115x->i2cAddress << 1;
  seq.flags = I2C_FLAG_WRITE_READ;

  /* Select register to start reading from */
  i2c_write_data[0] = address;

  seq.buf[0].data = i2c_write_data;
  seq.buf[0].len = 1;

  /* Select length of data to be read */
  seq.buf[1].data = data;
  seq.buf[1].len = length;

  ret = I2CSPM_Transfer(si115x->i2cPort, &seq);

  if (ret != i2cTransferDone) {
    return SI115X_ECODE_I2C_ERROR;
  }

  return SI115X_ECODE_OK;
}

/*******************************************************************************
 *****************************   COMMANDS **** *********************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief
 *  Waits for the Si115x to be in sleep mode.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  0 on success, if the function timed out, or there was an I2C error.
 *****************************************************************************/
static SI115X_Ecode_TypeDef _SI115X_WaitForSleep(SI115X_TypeDef *si115x)
{
  SI115X_Ecode_TypeDef retval;
  uint8_t count = 0;
  uint8_t resp0;

  while (count < _SI115X_COMMAND_TIMEOUT) {
    retval = SI115X_ReadFromRegister(si115x, SI115X_REG_RESPONSE0, &resp0);
    if ((resp0 & _SI115X_RESPONSE0_SLEEP_MASK) == _SI115X_RESPONSE0_SLEEP) {
      return SI115X_ECODE_OK;
    }

    if (retval != SI115X_ECODE_OK) {
      return retval;
    } else {
      // Si115x is not asleep, continue waiting
      count++;
    }
  }

  // Si115x was not asleep within SI115X_COMMAND_TIMEOUT
  return SI115X_ECODE_BUSY;
}
/**************************************************************************//**
 * @brief
 *  Reads the command counter.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  command counter if non-negative, I2C error if negative
 *****************************************************************************/
static SI115X_Ecode_TypeDef SI115X_CheckCmdCtr(SI115X_TypeDef *si115x,
                                               uint8_t *cmd_ctr)
{
  uint8_t resp0;
  SI115X_Ecode_TypeDef retval;
  retval = SI115X_ReadFromRegister(si115x, SI115X_REG_RESPONSE0, &resp0);

  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  if (resp0 & _SI115X_RESPONSE0_CMD_ERR_MASK) {
    return SI115X_ECODE_SI115X_ERROR;
  } else {
    *cmd_ctr = resp0 & _SI115X_RESPONSE0_CMD_CTR_MASK;
    return SI115X_ECODE_OK;
  }
}

/**************************************************************************//**
 * @brief
 *  Sends a command.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] command
 *  Command opcode to send.
 *
 * @return
 *  0 on success, I2C error or command counter timeout if negative.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_SendCmd(SI115X_TypeDef *si115x, uint8_t command)
{
  SI115X_Ecode_TypeDef retval;
  uint8_t count = 0;
  uint8_t cmd_ctr_before;
  uint8_t cmd_ctr_after;

  retval = _SI115X_WaitForSleep(si115x);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  // Get the current cmd_ctr to use in command execution verification
  retval = SI115X_CheckCmdCtr(si115x, &cmd_ctr_before);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  // Send the command
  retval = SI115X_WriteToRegister(si115x, SI115X_REG_COMMAND, command);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  // Verify the command has executed
  switch(command) {
  case SI115X_CMD_RESET_CMD_CTR:
    /* CMD_CTR should be 0 after a ResetCmdCtr */
    retval = SI115X_CheckCmdCtr(si115x, &cmd_ctr_after);
    if (retval != SI115X_ECODE_OK) {
      return retval;
    } else {
      if (cmd_ctr_after == 0) {
        return SI115X_ECODE_OK;
      }
      /* If command counter is not cleared, break and return BUSY */
    }
    break;
  case SI115X_CMD_RESET_SW:
    /* Device is in software reset.
     * Caller is required to handle delay.
     * Nothing more to do here. */
    return retval;
    break;
  default:
    count = 0;
    while (count < _SI115X_COMMAND_TIMEOUT) {
      retval = SI115X_CheckCmdCtr(si115x, &cmd_ctr_after);
      if (retval != SI115X_ECODE_OK) {
        return retval;
      }
      if (cmd_ctr_after != cmd_ctr_before) {
        // CMD_CTR updated, command execution is complete.
        return SI115X_ECODE_OK;
      } else {
        count++;
      }
    }
    break;
  }

  return SI115X_ECODE_BUSY;
}

/**************************************************************************//**
 * @brief
 *  Resets the sensor.
 *
 * @note
 * The caller must handle waiting for the require start up time.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  0 on success, negative if I2C error.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_Reset(SI115X_TypeDef *si115x)
{
  SI115X_Ecode_TypeDef retval;
  /* Use a direct I2C write to avoid pre and post cmd_err, cmd_ctr checks. */
  retval = SI115X_WriteToRegister(si115x, SI115X_REG_COMMAND,
      SI115X_CMD_RESET_SW);
  si115x->chan_list = 0;
  si115x->res = 0;
  return retval;
}

/**************************************************************************//**
 * @brief
 *  Resets the command counter and clears any errors.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  0 on success, negative if I2C error.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_ResetCmdCounter(SI115X_TypeDef *si115x)
{
  return SI115X_SendCmd(si115x, SI115X_CMD_RESET_CMD_CTR);
}

/**************************************************************************//**
 * @brief
 *  Starts a force measurement. Host must implement blocking.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  0 on success, negative if I2C error.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_Force(SI115X_TypeDef *si115x)
{
  return SI115X_SendCmd(si115x, SI115X_CMD_FORCE);
}

/**************************************************************************//**
 * @brief
 *  Starts autonomous mode measurements
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  0 on success, negative if I2C error.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_Start(SI115X_TypeDef *si115x)
{
  return SI115X_SendCmd(si115x, SI115X_CMD_START);
}

/**************************************************************************//**
 * @brief
 *  Pauses autonomous mode measurements
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  0 on success, negative if I2C error.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_Pause(SI115X_TypeDef *si115x)
{
  return SI115X_SendCmd(si115x, SI115X_CMD_PAUSE);
}

/**************************************************************************//**
 * @brief
 *  Sets a parameter table register.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] address
 *  The parameter table register address to write.
 *
 * @param[in] value
 *  The value to write to the parameter table register.
 *
 * @return
 *  0 on success, negative if I2C error.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_SetParam(SI115X_TypeDef *si115x, uint8_t address,
                                     uint8_t value)
{
  // This function does not use _SI115X_SendCommand in order to optimize the
  // command write as a length 3 burst write of HOSTIN0 and COMMAND all at once.

  SI115X_Ecode_TypeDef retval;
  uint8_t cmd_ctr_before;
  uint8_t cmd_ctr_after;

  uint8_t command = SI115X_CMD_PARAM_SET_MASK
      | (address & SI115X_CMD_PARAM_ADDR_MASK);
  uint8_t i2c_write_data[2];

  i2c_write_data[0] = value;
  i2c_write_data[1] = command;

  if ((retval = _SI115X_WaitForSleep(si115x)) != 0) {
    return retval;
  }

  // Get the current cmd_ctr to use in command execution verification
  retval = SI115X_CheckCmdCtr(si115x, &cmd_ctr_before);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  // Send the command
  retval = SI115X_BlockWrite(si115x, SI115X_REG_HOSTIN0, 2, i2c_write_data);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  // Verify the command has executed
  uint8_t count = 0;
  if (command != SI115X_CMD_RESET_CMD_CTR && command != SI115X_CMD_RESET_SW) {
    while (count < _SI115X_COMMAND_TIMEOUT) {
      retval = SI115X_CheckCmdCtr(si115x, &cmd_ctr_after);

      if (retval != SI115X_ECODE_OK) {
        return retval;
      }

      if (cmd_ctr_after != cmd_ctr_before) {
        // cmd_ctr updated, command has completed, proceed to return
        return SI115X_ECODE_OK;
      } else {
        // cmd_ctr has not yet incremented, continue waiting for command completion.
        count++;
      }
    }
  }
  return SI115X_ECODE_BUSY;
}

/**************************************************************************//**
 * @brief
 *  Reads a parameter table register.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] address
 *  The parameter table register address to read.
 *
 * @param[out] param
 *  The value read from the parameter table
 *
 * @return
 *  The parameter register value if non-negative, I2C error or command timeout if negative.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_QueryParam(SI115X_TypeDef *si115x, uint8_t address,
                                       uint8_t *param)
{

  SI115X_Ecode_TypeDef retval;
  uint8_t command = SI115X_CMD_PARAM_QUERY_MASK
      | (address & SI115X_CMD_PARAM_ADDR_MASK);

  retval = SI115X_SendCmd(si115x, command);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  // Command executed successfully, retrieve parameter value from RESPONSE1.
  retval = SI115X_ReadFromRegister(si115x, SI115X_REG_RESPONSE1, param);
  return retval;
}

/****************************** ROUTINES ****************************************/

/**************************************************************************//**
 * @brief
 *  Initializes a Si115x channel.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] channel
 *  The channel to initialize.
 *
 * @param[in] init
 *  A pointer to the Si115x channel initialization structure.
 *
 * @return
 *  0 on success, I2C error or command timeout if negative.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_InitChannel(SI115X_TypeDef *si115x, uint8_t channel,
                                        const SI115X_InitChannel_TypeDef *init)
{
  int16_t tmp;
  uint8_t channelParamOffset;
  SI115X_Ecode_TypeDef retval;

  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }
  if (channel >= _SI115X_MAX_CHANNELS) {
    return SI115X_ECODE_PARAM_ERROR;
  }
  if (init == NULL) {
    return SI115X_ECODE_PARAM_ERROR;
  }

  /* CHAN_LIST */
  uint8_t chan_list;
  retval = SI115X_QueryParam(si115x, SI115X_CHAN_LIST, &chan_list);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  chan_list &= ~(1 << channel);
  chan_list |= (1 << channel);
  retval = SI115X_SetParam(si115x, SI115X_CHAN_LIST, chan_list);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  /* Store the chan_list so that GetChannelData knows the output field formatting. */
  si115x->chan_list |= (1 << channel);

  channelParamOffset = channel * _SI115X_CHANNEL_CONFIG_LENGTH
      + _SI115X_CHANNEL_CONFIG_OFFSET;

  /* ADCCONFIGx */
  tmp = init->decim | init->adcMux;
  retval = SI115X_SetParam(si115x,
      channelParamOffset + _SI115X_ADCCONFIG_OFFSET, tmp);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  /* ADCSENSx */
  tmp = init->hsig | init->swGain | init->hwGain;
  retval = SI115X_SetParam(si115x, channelParamOffset + _SI115X_ADCSENS_OFFSET,
      tmp);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  /* ADCPOSTx */
  if (init->postshift
      > (_SI115X_ADCPOST_POSTSHIFT_MASK >> _SI115X_ADCPOST_POSTSHIFT_SHIFT)) {
    return SI115X_ECODE_PARAM_ERROR;
  }
  tmp = ((init->postshift << _SI115X_ADCPOST_POSTSHIFT_SHIFT)
      & _SI115X_ADCPOST_POSTSHIFT_MASK);
  tmp |= init->res | init->threshPol | init->threshEn;
  retval = SI115X_SetParam(si115x, channelParamOffset + _SI115X_ADCPOST_OFFSET,
      tmp);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  /* Store the res24b so that GetChannelData knows the output field formatting. */
  if (init->res) {
    si115x->res |= (1 << channel);
  }

  /* MEASCONFIGx */
  tmp = init->counterIndex | init->bankSel;
  tmp |= (init->led1En << _SI115X_MEASCONFIG_LED1_EN_SHIFT);
  tmp |= (init->led2En << _SI115X_MEASCONFIG_LED2_EN_SHIFT);
  tmp |= (init->led3En << _SI115X_MEASCONFIG_LED3_EN_SHIFT);
  retval = SI115X_SetParam(si115x,
      channelParamOffset + _SI115X_MEASCONFIG_OFFSET, tmp);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  return SI115X_ECODE_OK;
}

/**************************************************************************//**
 * @brief
 *  Initializes the Si115x global parameters.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] init
 *  A pointer to the Si115x initialization structure.
 *
 * @return
 *  0 on success, I2C error or command timeout if negative.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_Init(SI115X_TypeDef *si115x,
                                 const SI115X_Init_TypeDef *init)
{
  uint8_t high, low, tmp;
  SI115X_Ecode_TypeDef retval;

  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }

  if (init == NULL) {
    return SI115X_ECODE_PARAM_ERROR;
  }

  // CHAN_LIST is covered in the SI115X_InitChannel

  high = (init->measrate & _SI115X_MEASRATE_MASK) >> 8;
  low = (init->measrate & _SI115X_MEASRATE_MASK) & 0xFF;
  retval = SI115X_SetParam(si115x, SI115X_MEASRATE_H, high);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_MEASRATE_L, low);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  retval = SI115X_SetParam(si115x, SI115X_MEASCOUNT0, init->meascount0);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_MEASCOUNT1, init->meascount1);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_MEASCOUNT2, init->meascount2);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  retval = SI115X_SetParam(si115x, SI115X_LED1_A, init->led1a_current);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_LED1_B, init->led1b_current);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_LED2_A, init->led2a_current);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_LED2_B, init->led2b_current);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_LED3_A, init->led3a_current);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_LED3_B, init->led3b_current);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  high = (init->threshold0) >> 8;
  low = (init->threshold0) & 0xFF;
  retval = SI115X_SetParam(si115x, SI115X_THRESHOLD0_H, high);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  retval = SI115X_SetParam(si115x, SI115X_THRESHOLD0_L, low);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  high = (init->threshold1) >> 8;
  low = (init->threshold1) & 0xFF;
  retval = SI115X_SetParam(si115x, SI115X_THRESHOLD1_H, high);
  retval = SI115X_SetParam(si115x, SI115X_THRESHOLD1_L, low);

  // Revision -AB parts have UPPER_THRESHOLD and LOWER_THRESHOLD
  uint8_t rev_id;
  retval = SI115X_ReadFromRegister(si115x, SI115X_REG_REV_ID, &rev_id);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  if (rev_id >= SI115X_REV_ID_AB) {
    high = (init->upper_threshold) >> 8;
    low = (init->upper_threshold) & 0xFF;
    retval = SI115X_SetParam(si115x, SI115X_UPPER_THRESHOLD_H, high);
    if (retval != SI115X_ECODE_OK) {
      return retval;
    }
    retval = SI115X_SetParam(si115x, SI115X_UPPER_THRESHOLD_L, low);
    if (retval != SI115X_ECODE_OK) {
      return retval;
    }

    high = (init->lower_threshold) >> 8;
    low = (init->lower_threshold) & 0xFF;
    retval = SI115X_SetParam(si115x, SI115X_LOWER_THRESHOLD_H, high);
    if (retval != SI115X_ECODE_OK) {
      return retval;
    }
    retval = SI115X_SetParam(si115x, SI115X_LOWER_THRESHOLD_L, low);
    if (retval != SI115X_ECODE_OK) {
      return retval;
    }
  }

  tmp = (init->burstEn << _SI115X_BURST_BURST_EN_SHIFT)
              | ((init->burstCount & _SI115X_BURST_BURST_COUNT_MASK)
                  << _SI115X_BURST_BURST_COUNT_SHIFT);
  retval = SI115X_SetParam(si115x, SI115X_BURST, tmp);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }
  return SI115X_ECODE_OK;
}

/**************************************************************************//**
 * @brief
 *  Enables interrupts.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] flags
 *  The channel to initialize.
 *
 * @return
 *  0 on success, I2C error if negative
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_IntEnable(SI115X_TypeDef *si115x, uint8_t flags)
{
  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }
  return SI115X_WriteToRegister(si115x, SI115X_REG_IRQENABLE, flags);
}

/**************************************************************************//**
 * @brief
 *  Gets the interrupt status
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @return
 *  Interrupt status if non-negative, I2C error if negative
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_IntGet(SI115X_TypeDef *si115x, uint8_t *irq_status)
{
  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }
  return SI115X_ReadFromRegister(si115x, SI115X_REG_IRQ_STATUS, irq_status);
}

/**************************************************************************//**
 * @brief
 *  Gets the ADC output for the specified channel.
 *
 * @notes
 *  This function is not compatible with configurations that use Burst mode
 *  or configurations have varying numbers of tasks per wakeup due to
 *  multiple channels configured with different counters and MEASCOUNT values.
 *
 * @param[in] si115x
 *  Si115x device to use.
 *
 * @param[in] channel
 *  The channel to retrieve data.
 *
 * @param[out] data
 *  A pointer to store the output of the channel. The stored data will match the output
 *  format as specified in the channel res24 configuration.
 *
 * @return
 *  0 on success, negative if I2C error.
 *****************************************************************************/
SI115X_Ecode_TypeDef SI115X_GetChannelData(SI115X_TypeDef *si115x,
                                           uint8_t channel, int32_t *data)
{
  if (si115x == NULL) {
    return SI115X_ECODE_ILLEGAL_HANDLE;
  }

  if (channel >= _SI115X_MAX_CHANNELS) {
    return SI115X_ECODE_PARAM_ERROR;
  }

  uint8_t hostoutOffset = 0;
  int8_t retval;
  bool res24b;
  bool channel_enabled;

  /* Find the location of the channel data by adding the size of the other channels. */
  for (uint8_t channel_num = 0; channel_num < channel; channel_num++) {
    res24b = (si115x->res >> channel_num) & 0x1;
    channel_enabled = (si115x->chan_list >> channel_num) & 0x1;
    if (channel_enabled) {
      hostoutOffset += (res24b ? 3 : 2);
    }
  }
  res24b = (si115x->res >> channel) & 0x1;

  /* Read out the raw data. */
  uint8_t rawData[3];
  uint8_t readLength = (res24b ? 3 : 2);
  retval = SI115X_BlockRead(si115x, SI115X_REG_HOSTOUT0 + hostoutOffset,
      readLength, rawData);
  if (retval != SI115X_ECODE_OK) {
    return retval;
  }

  /* Join bytes and convert signedness if necessary. */
  if (res24b) {
    *data = (rawData[0] << 16) | (rawData[1] << 8) | rawData[2];
    if (*data > 8388607) {
      *data = *data - 16777216;
    }
  } else {
    *data = (rawData[0] << 8) | rawData[1];
  }
  return SI115X_ECODE_OK;
}
/** @} (end group Si115x) */
/** @} (end group kitdrv) */
