/**
* @file
* BRD4200/BRD4201/BRD4202 board support functions
* @copyright 2018 Silicon Laboratories Inc.
*/

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <ZW_typedefs.h>
#include "ZAF/ApplicationUtilities/board.h"
#define DEBUGPRINT
#include "DebugPrint.h"
#include <ZW_radio_api.h>
/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

#define GPIO_PORT_SAW1  gpioPortB
#define GPIO_PORT_SAW2  gpioPortB
#define GPIO_PIN_SAW1   14
#define GPIO_PIN_SAW2   15

/****************************************************************************/
/*                      PRIVATE FUNCTIONS                                   */
/****************************************************************************/

/*
 * SAW filter setup function.
 *
 * Applicable for boards with SAW filter (BRD4200 and BRD4201)
 */
#ifndef BRD4202
static uint32_t BoardSawFilterSetup(uint8_t rfRegion)
{
  uint8_t levelSaw1;
  uint8_t levelSaw2;

  /*
   * Select the appopriate SAW filter by setting the GPIO
   * pins "SAW1" and "SAW2" according to the table below:
   *
   *        |  GPIO SAW1  |  GPIO SAW2
   * ----------------------------------
   * HK SAW |      0      |      0
   * US SAW |      0      |      1
   * EU SAW |      1      |      0
   */

  switch (rfRegion)
  {
  // EU SAW filter countries/regions
  case REGION_EU:
  case REGION_IN:
  case REGION_RU:
  case REGION_CN:
  case TF_REGION_866:
  case TF_REGION_870:
  case TF_REGION_878:
  case TF_REGION_882:
  case TF_REGION_886:
  case TF_REGION_835_3CH:
  case TF_REGION_840_3CH:
  case TF_REGION_850_3CH:
    /* Select EU SAW filter */
    levelSaw1 = 1;
    levelSaw2 = 0;
    break;

  // US SAW filter countries/regions
  case REGION_US:
  case REGION_IL:
  case TF_REGION_906:
  case TF_REGION_910:
    /* Select US SAW filter */
    levelSaw1 = 0;
    levelSaw2 = 1;
    break;

  // HK SAW filter countries/regions
  case REGION_HK:
  case REGION_ANZ:
  case REGION_JP:
  case REGION_KR:
  case TF_REGION_932_3CH:
  case TF_REGION_940_3CH:
    /* Select HK SAW filter */
    levelSaw1 = 0;
    levelSaw2 = 0;
    break;

  default:
    DPRINTF("\r\nBRD420x SAW filter setup failure. Unknown RF region %d\r\n", rfRegion);
    return false;
  }

  GPIO_PinModeSet(GPIO_PORT_SAW1, GPIO_PIN_SAW1, gpioModePushPull, levelSaw1);
  GPIO_PinModeSet(GPIO_PORT_SAW2, GPIO_PIN_SAW2, gpioModePushPull, levelSaw2);
  return true;
}
#endif


/****************************************************************************/
/*                      PUBLIC FUNCTIONS                                    */
/****************************************************************************/

uint32_t BRD420xBoardInit(uint8_t rfRegion)
{
  uint32_t status = true;
#ifndef BRD4202
  status = BoardSawFilterSetup(rfRegion);
#endif

  return status;
}
