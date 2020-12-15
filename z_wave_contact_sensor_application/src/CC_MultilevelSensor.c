/*
 * CC_MultilevelSensor.c
 *
 *  Created on: Jul 18, 2019
 *      Author: axbrugge
 */

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <ZW_basis_api.h>
#include <ZW_TransportLayer.h>
#include <ZW_TransportEndpoint.h>
#include "config_app.h"
#include <CC_MultilevelSensor.h>
#include <CC_Common.h>
#include "misc.h"
#include <agi.h>
#include <ZW_application_transport_interface.h>  // ACTION: added from CC_Powerlevel.c
#include "DebugPrintConfig.h"
#define DEBUGPRINT
#include "DebugPrint.h"
/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

received_frame_status_t handleCommandClassMultilevelSensor(			// Action: Change name of CC handler
  RECEIVE_OPTIONS_TYPE_EX *rxOpt,
  ZW_APPLICATION_TX_BUFFER *pCmd,
  uint8_t cmdLength)
{

  DPRINTF("sensor multilevel %d\r\n", rxOpt->destNode.endpoint);

  ZAF_TRANSPORT_TX_BUFFER  TxBuf;
  ZW_APPLICATION_TX_BUFFER *pTxBuf = &(TxBuf.appTxBuf);

  switch (pCmd->ZW_Common.cmd)
  {
    case SENSOR_MULTILEVEL_GET_V11:		// 0x04

      if (true == Check_not_legal_response_job(rxOpt))
      {
        return RECEIVED_FRAME_STATUS_FAIL;
      }
      memset((uint8_t*)pTxBuf, 0, sizeof(ZW_APPLICATION_TX_BUFFER));
      TRANSMIT_OPTIONS_TYPE_SINGLE_EX* pTxOptionsEx;
      RxToTxOptions(rxOpt, &pTxOptionsEx);
      pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.cmdClass     = COMMAND_CLASS_SENSOR_MULTILEVEL_V11;	// 0x31
      pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.cmd          = SENSOR_MULTILEVEL_REPORT_V11;			// 0x05
      cc_multilevel_sensor_t multilevelSensor;
      multilevelSensor.type = pCmd->ZW_SensorMultilevelGetV11Frame.sensorType;
      CC_MultilevelSensor_Get_handler(&multilevelSensor);
      pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.sensorType = multilevelSensor.type; // old: CC_MultilevelSensor_GetSensorType_handler(rxOpt->destNode.endpoint);
      pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.level  = 	// old: GetTargetLevel(rxOpt->destNode.endpoint);
    		  (multilevelSensor.precision << 5) | (multilevelSensor.scale << 3) | (multilevelSensor.size);
      switch (multilevelSensor.size)
      {
      	  case 0x01:

      		  DPRINTF("multilevelSensor.size == 1\n");
      		  pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.sensorValue1     = multilevelSensor.sensorvalue1;
      		  break;

      	  case 0x02:

      		  DPRINTF("multilevelSensor.size == 2\n");
      		  pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.sensorValue1     = multilevelSensor.sensorvalue1;
      		  pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.sensorValue2     = multilevelSensor.sensorvalue2;
      		  break;

      	  case 0x04:

      		  DPRINTF("multilevelSensor.size == 4\n");
			  pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.sensorValue1     = multilevelSensor.sensorvalue1;
			  pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.sensorValue2     = multilevelSensor.sensorvalue2;
			  pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.sensorValue3     = multilevelSensor.sensorvalue3;
			  pTxBuf->ZW_SensorMultilevelReport4byteV11Frame.sensorValue4     = multilevelSensor.sensorvalue4;
			  break;

      	  default:

      		  DPRINTF("no valid size\n");
      		  return RECEIVED_FRAME_STATUS_FAIL;
      }


      if (EQUEUENOTIFYING_STATUS_SUCCESS != Transport_SendResponseEP(
          (uint8_t *)pTxBuf,
          sizeof(pTxBuf->ZW_SensorMultilevelReport4byteV11Frame),
          pTxOptionsEx,
          NULL))
      {
        /*Job failed */
        return RECEIVED_FRAME_STATUS_FAIL;
      }
      break;

    case SENSOR_MULTILEVEL_SUPPORTED_GET_SENSOR_V11:  // 0x01

    	if (true == Check_not_legal_response_job(rxOpt))
    	      {
    	        return RECEIVED_FRAME_STATUS_FAIL;
    	      }
    	memset((uint8_t*)pTxBuf, 0, sizeof(ZW_APPLICATION_TX_BUFFER));
        RxToTxOptions(rxOpt, &pTxOptionsEx);
    	pTxBuf->ZW_SensorMultilevelSupportedSensorReport4byteV11Frame.cmdClass	= COMMAND_CLASS_SENSOR_MULTILEVEL_V11;	// 0x31
    	pTxBuf->ZW_SensorMultilevelSupportedSensorReport4byteV11Frame.cmd		= SENSOR_MULTILEVEL_SUPPORTED_SENSOR_REPORT_V11; // 0x2
    	cc_multilevel_sensor_supported_t multiLevelSensor_supported;
    	memset((uint8_t *)&multiLevelSensor_supported, 0x00, sizeof(multiLevelSensor_supported));
    	CC_MultilevelSensor_SupportedGet_handler(&multiLevelSensor_supported);
    	pTxBuf->ZW_SensorMultilevelSupportedSensorReport4byteV11Frame.bitMask1 = (uint8_t)(multiLevelSensor_supported.sensorBitMask1);
    	pTxBuf->ZW_SensorMultilevelSupportedSensorReport4byteV11Frame.bitMask2 = (uint8_t)(multiLevelSensor_supported.sensorBitMask2);
    	pTxBuf->ZW_SensorMultilevelSupportedSensorReport4byteV11Frame.bitMask3 = (uint8_t)(multiLevelSensor_supported.sensorBitMask3);
    	pTxBuf->ZW_SensorMultilevelSupportedSensorReport4byteV11Frame.bitMask4 = (uint8_t)(multiLevelSensor_supported.sensorBitMask4);


    	if(EQUEUENOTIFYING_STATUS_SUCCESS != Transport_SendResponseEP(
    	          (uint8_t *)pTxBuf,
    	          sizeof(pTxBuf->ZW_SensorMultilevelSupportedSensorReport4byteV11Frame),
    	          pTxOptionsEx,
    	          NULL))
    	      {
    	        return RECEIVED_FRAME_STATUS_FAIL;
    	      }
    	      break;


    case SENSOR_MULTILEVEL_SUPPORTED_GET_SCALE_V11:  // Get command has an additional parameter
    	if (true == Check_not_legal_response_job(rxOpt))
    	       {
    	    	 return RECEIVED_FRAME_STATUS_FAIL;
    	       }

    	uint8_t mySensorType;
    	memset((uint8_t*)pTxBuf, 0, sizeof(ZW_APPLICATION_TX_BUFFER) );
    	mySensorType = (pCmd->ZW_SensorMultilevelSupportedScaleReportV11Frame.sensorType);
    	RxToTxOptions(rxOpt, &pTxOptionsEx);
    	pTxBuf->ZW_SensorMultilevelSupportedScaleReportV11Frame.cmdClass	= COMMAND_CLASS_SENSOR_MULTILEVEL_V11;	// 0x31
    	pTxBuf->ZW_SensorMultilevelSupportedScaleReportV11Frame.cmd		= SENSOR_MULTILEVEL_SUPPORTED_GET_SCALE_V11; // 0x3
    	cc_multilevel_sensor_supported_scale_t multiLevelSensor_supportedScale;
    	memset((uint8_t *)&multiLevelSensor_supportedScale, 0x00, sizeof(multiLevelSensor_supportedScale));
    	multiLevelSensor_supportedScale.sensorType = mySensorType;
    	CC_MultilevelSensor_SupportedScale_Get_handler(&multiLevelSensor_supportedScale);	//  filling out the multiLevelSensor_supported struct
    	pTxBuf->ZW_SensorMultilevelSupportedScaleReportV11Frame.sensorType = mySensorType;
    	pTxBuf->ZW_SensorMultilevelSupportedScaleReportV11Frame.properties1  |= multiLevelSensor_supportedScale.scaleBitmask;

    	if (EQUEUENOTIFYING_STATUS_SUCCESS != Transport_SendResponseEP(
    	          (uint8_t *)pTxBuf,
    	          sizeof(pTxBuf->ZW_SensorMultilevelSupportedScaleReportV11Frame),
    	          pTxOptionsEx,
    	          NULL))
    	      {
    	        /*Job failed */
    	        return RECEIVED_FRAME_STATUS_FAIL;
    	      }
    	break;

    default:
    	return RECEIVED_FRAME_STATUS_NO_SUPPORT;
  	  }
   	  return RECEIVED_FRAME_STATUS_SUCCESS;
  }



