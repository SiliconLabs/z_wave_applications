#ifndef _CC_COLOR_SWITCH_H_
#define _CC_COLOR_SWITCH_H_

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <CC_Common.h>
#include <ZAF_types.h>
#include <ZW_TransportEndpoint.h>


/****************************************************************************/

JOB_STATUS
CmdClassColorSwitchSetTransmit(
  AGI_PROFILE* pProfile,
  uint8_t sourceEndpoint,
  VOID_CALLBACKFUNC(pCbFunc)(TRANSMISSION_RESULT * pTransmissionResult),
  uint8_t count,
  uint8_t redValue,
  uint8_t greenValue,
  uint8_t blueValue,
  uint8_t duration);

#endif
