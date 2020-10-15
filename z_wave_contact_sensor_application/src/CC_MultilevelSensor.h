/*
 * CC_MultilevelSensor.h
 *
 *  Created on: Jul 18, 2019
 *      Author: axbrugge
 */

#ifndef INC_CC_MULTILEVELSENSOR_H_
#define INC_CC_MULTILEVELSENSOR_H_


/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

#include <ZAF_types.h>

extern uint8_t CC_MultilevelSensor_GetSensorType_handler(uint8_t endpoint);

/**
 * All Multilevel sensor types are defined in document SDS13812
 * Values not defined in this document MUST NOT be used.
 * For the purpose of this design, only two types are implemented
 * All other values are reserved and SHALL be ignored by the receiving device.
 */
typedef enum
{
	CMD_CLASS_SENSORTYPE_MULTILEVEL_AIR_T	= SENSOR_MULTILEVEL_REPORT_TEMPERATURE_VERSION_1_V11,	/** Sensor Type Air Temperature */
	CMD_CLASS_SENSORTYPE_MULTILEVEL_HUMIDITY = SENSOR_MULTILEVEL_REPORT_RELATIVE_HUMIDITY_VERSION_2_V11,	/** Sensor Type Humidity */
}
cc_sensortype_multilevel_t;



/**
 * @brief Handler for CC Multilevel Sensor.
 * @param[in] rxOpt pointer receive options of type RECEIVE_OPTIONS_TYPE_EX
 * @param[in] pCmd pointer Payload from the received frame
 * @param[in] cmdLength Number of command uint8_ts including the command
 * @return receive frame status.
 */
received_frame_status_t handleCommandClassMultilevelSensor(
  RECEIVE_OPTIONS_TYPE_EX *rxOpt,
  ZW_APPLICATION_TX_BUFFER *pCmd,
  uint8_t cmdLength);

// Action: Added Get Sensor Type handler for MultilevelSensor CC
/**
 * Returns the current value for a given endpoint.
 * @param[in] endpoint Given endpoint.
 * @return Current value.
 */
extern uint8_t CC_MultilevelSensor_GetSensorType_handler(uint8_t endpoint);

/****************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                       */
/****************************************************************************/
/**
 * Multilevel Sensor Get Structure
 * -----------------------------------------
 *
 */
typedef struct
{
  cc_sensortype_multilevel_t type;
  uint8_t precision : 3; /**< Define decimal points (3 bits)*/
  uint8_t scale : 2; /**< What scale to be used (2 bits)*/
  uint8_t size : 3; /**< Length in bytes of Sensor Value field (3 bits)*/
  uint8_t lockTimeoutSec; /**< Lock Timeout Seconds, valid 1-59 decimal*/
  uint8_t sensorvalue1; /**< Auto-relock time in seconds MSB */
  uint8_t sensorvalue2; /**< Auto-relock time in seconds LSB */
  uint8_t sensorvalue3; /**< Hold and release time in seconds MSB */
  uint8_t sensorvalue4; /**< Hold and release time in seconds LSB */
} cc_multilevel_sensor_t;



/**
 * Multilevel Sensor Get Supported Structure
 * -----------------------------------------
 *
 */
typedef struct
{
  uint8_t sensorBitMask1; 	/**< Supported sensor type Bitmask 1 */
  uint8_t sensorBitMask2; 	/**< Supported sensor type Bitmask 1 */
  uint8_t sensorBitMask3; 	/**< Supported sensor type Bitmask 1 */
  uint8_t sensorBitMask4; 	/**< Supported sensor type Bitmask 1 */
}cc_multilevel_sensor_supported_t;




/**
 * Multilevel Sensor Get Supported Scale Structure
 * -----------------------------------------
 */
typedef struct
{
  uint8_t sensorType; 	/**< Supported sensor type  */
  uint8_t scaleBitmask : 4;  /** advertise the supported scales (4-bit)  */
}cc_multilevel_sensor_supported_scale_t;

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

// Nothing here.

/****************************************************************************/
/*                           EXPORTED FUNCTIONS                             */
/****************************************************************************/

/**
 * ACTION: added function prototype
 */

extern void CC_MultilevelSensor_SupportedGet_handler(cc_multilevel_sensor_supported_t* pData);
extern void CC_MultilevelSensor_SupportedScale_Get_handler(cc_multilevel_sensor_supported_scale_t* pData);
extern void CC_MultilevelSensor_Get_handler(cc_multilevel_sensor_t* pData);



#endif /* INC_CC_MULTILEVELSENSOR_H_ */
