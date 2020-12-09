#include <CC_ColorSwitch_Control.h>
#include <CC_ColorSwitch.h>
#include <ZW_TransportLayer.h>
#include <agi.h>

typedef struct _color_component_t_
{
  uint8_t colorId;
  uint8_t value;
}color_component_t;

typedef struct _color_switch_set_3byte_v2_frame_t_
{
  uint8_t colorComponentCount;
  color_component_t colors[3];
  uint8_t dimmingDuration;
}color_switch_set_3byte_v2_frame_t;

CMD_CLASS_GRP cmdClassGrp;

JOB_STATUS
CmdClassColorSwitchSetTransmit(
  AGI_PROFILE* pProfile,
  uint8_t sourceEndpoint,
  VOID_CALLBACKFUNC(pCbFunc)(TRANSMISSION_RESULT * pTransmissionResult),
  uint8_t count,
  uint8_t redValue,
  uint8_t greenValue,
  uint8_t blueValue,
  uint8_t duration)
{
  color_switch_set_3byte_v2_frame_t color_switch_set;
  color_switch_set.colorComponentCount = count;
  color_switch_set.dimmingDuration = duration;

  color_switch_set.colors[0].colorId = ECOLORCOMPONENT_RED;
  color_switch_set.colors[0].value = redValue;
  color_switch_set.colors[1].colorId = ECOLORCOMPONENT_GREEN;
  color_switch_set.colors[1].value = greenValue;
  color_switch_set.colors[2].colorId = ECOLORCOMPONENT_BLUE;
  color_switch_set.colors[2].value = blueValue;

  cmdClassGrp.cmdClass = COMMAND_CLASS_SWITCH_COLOR_V3;
  cmdClassGrp.cmd = SWITCH_COLOR_SET;


  return cc_engine_multicast_request(
      pProfile,
      sourceEndpoint,
      &cmdClassGrp,
      (uint8_t*)&color_switch_set,
      sizeof(color_switch_set),
      true,
      pCbFunc);

}
