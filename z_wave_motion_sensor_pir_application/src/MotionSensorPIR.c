/**
 * Z-Wave Motion Sensor PIR application example project
 *
 * Hardware: BRD4001A(WSTK) + BRD4202A(ZGM130S Radio board) + BRD8030A (
 * Occupancy sensor EXP board)
 *
 * Note: The project is based on the SensorPIR demo in the Z-Wave SDK, but
 * works with occupancy sensor expansion board, instead of BRD8029A (Buttons
 * and LEDs EXP Board). Buttons and LEDs are limited and moved to WSTK main
 * board. It's required to tie pin P4 and P12 together at the Breakout Pads
 * to allow EM4 wake-up using PB2.
 *
 * How To Use:
 * 1. Build the project(s) and download to ZGM130S037HGN
 * 2. Push PB1 on WSTK to enter learn mode to add/remove the device
 * 3. Push PB2 on WSTK to start/stop PIR motion sensor measurements
 * 4. RGB LED on the radio board will turn green if motion is detected
 *
 * @copyright 2018 Silicon Laboratories Inc.
 */

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include "config_rf.h"
#include <stdbool.h>
#include <stdint.h>

#include "SizeOf.h"
#include "Assert.h"
#include "DebugPrintConfig.h"
#include "occupancy_exp.h"
#include "pirdrv.h"

//To enable DEBUGPRINT you must first undefine DISABLE_USART0
//In Simplicity Studio open Project->Properties->C/C++ General->Paths and Symbols
//Remove DISABLE_USART0 from list found under # Symbols->GNU C
#if !defined(DISABLE_USART1) || !defined(DISABLE_USART0)
//#define DEBUGPRINT
#endif

#include "DebugPrint.h"
#include "config_app.h"
#include "ZAF_app_version.h"
#include <ZAF_file_ids.h>
#include "nvm3.h"
#include "ZAF_nvm3_app.h"
#include <em_system.h>
#include <ZW_slave_api.h>
#include <ZW_classcmd.h>
#include <ZW_TransportLayer.h>

#include <ZAF_uart_utils.h>

#include <misc.h>

/*IO control*/
#include <board.h>

#include <ev_man.h>
#include <AppTimer.h>
#include <string.h>
#include <SwTimer.h>
#include <EventDistributor.h>
#include <ZW_system_startup_api.h>
#include <ZW_application_transport_interface.h>

#include <association_plus.h>
#include <agi.h>
#include <CC_Association.h>
#include <CC_AssociationGroupInfo.h>
#include <CC_Version.h>
#include <CC_ZWavePlusInfo.h>
#include <CC_PowerLevel.h>
#include <CC_DeviceResetLocally.h>
#include <CC_Indicator.h>
#include <CC_Basic.h>
#include <CC_FirmwareUpdate.h>
#include <CC_ManufacturerSpecific.h>

#include <CC_Battery.h>
#include <CC_Notification.h>

#include <CC_MultiChanAssociation.h>
#include <CC_Supervision.h>
#include <notification.h>
#include <CC_WakeUp.h>
#include <zaf_event_helper.h>
#include <zaf_job_helper.h>

#include <ZAF_Common_helper.h>
#include <ZAF_PM_Wrapper.h>
#include <ZAF_network_learn.h>
#include <ZAF_TSE.h>
#include <ota_util.h>
#include "ZAF_adc.h"
#include <ZAF_CmdPublisher.h>
#include <em_wdog.h>
#include "events.h"

/****************************************************************************/
/* Application specific button and LED definitions                          */
/****************************************************************************/

#ifdef RADIO_BOARD_EFR32ZG13P32
  // The EFR32ZG13P32 device has reduced number of GPIO pins and hence less
  // button inputs available for the application to use. Therefore alternative
  // button mapping is required

  #define PIR_EVENT_BTN        APP_BUTTON_LEARN_RESET  // Overload the LEARN_RESET button to also function as the PIR event button
                                                       // (It's the only button that can wakeup from EM4 on EFR32ZG13P32)

  // Define the button events used to signify PIR sensor state transitions:
  //
  // PIR_EVENT_TRANSITION_TO_ACTIVE
  //   Triggered by the BTN_EVENT_HOLD event which means the button is pressed and held
  #define PIR_EVENT_TRANSITION_TO_ACTIVE(event)   (BTN_EVENT_HOLD(PIR_EVENT_BTN) == (BUTTON_EVENT)event)
  //
  // PIR_EVENT_TRANSITION_TO_DEACTIVE
  //   Triggered by the BTN_EVENT_UP event which means the button was released after a BTN_EVENT_HOLD period
  #define PIR_EVENT_TRANSITION_TO_DEACTIVE(event) (BTN_EVENT_UP(PIR_EVENT_BTN) == (BUTTON_EVENT)event)

#else

  #define PIR_EVENT_BTN        APP_WAKEUP_SLDR_BTN  // We prefer a wakeup enabled slider, but a button will do

  // Define the button events used to signify PIR sensor state transitions:
  //
  // PIR_EVENT_TRANSITION_TO_ACTIVE
  //   The PIR_EVENT_BTN could be allocated to either a slider or a button.
  //   - The slider will always send a DOWN event when moved to the ON position.
  //   - A button will send SHORT_PRESS or HOLD event when pressed. Only the
  //     HOLD event will be followed by an UP event when the button is
  //     released. Since we need the UP event later to cancel the power
  //     lock, we ignore the SHORT_PRESS event here.
  #define PIR_EVENT_TRANSITION_TO_ACTIVE(event)   ((BTN_EVENT_DOWN(PIR_EVENT_BTN) == (BUTTON_EVENT)event) || \
                                                   (BTN_EVENT_HOLD(PIR_EVENT_BTN) == (BUTTON_EVENT)event))
  // PIR_EVENT_TRANSITION_TO_DEACTIVE
  //   The PIR_EVENT_BTN could be allocated to either a slider or a button.
  //   The slider will always send an UP event when moved to the OFF position.
  //   A button will send either an UP, SHORT_PRESS or LONG_PRESS on release
  //   depending on how long it has been pressed.
  #define PIR_EVENT_TRANSITION_TO_DEACTIVE(event) ((BTN_EVENT_UP(PIR_EVENT_BTN) == (BUTTON_EVENT)event) || \
                                                   (BTN_EVENT_SHORT_PRESS(PIR_EVENT_BTN) == (BUTTON_EVENT)event) || \
                                                   (BTN_EVENT_LONG_PRESS(PIR_EVENT_BTN) == (BUTTON_EVENT)event))

#endif
#define BATTERY_REPORT_BTN   APP_BUTTON_A         // This button cannot wake up the device from EM4
                                                  // (i.e. it will generally not work with SensorPIR)

/* Ensure we did not allocate the same physical button to more than one function */
#ifndef RADIO_BOARD_EFR32ZG13P32 // Skipped for EFR32ZG13P32 where the shortage of GPIOs means we need to assign dual function to buttons
STATIC_ASSERT((APP_BUTTON_LEARN_RESET != PIR_EVENT_BTN) &&
              (APP_BUTTON_LEARN_RESET != BATTERY_REPORT_BTN) &&
              (PIR_EVENT_BTN != BATTERY_REPORT_BTN),
              STATIC_ASSERT_FAILED_button_overlap);
#endif

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

/**
 * Application states. Function AppStateManager(..) includes the state
 * event machine.
 */
typedef enum _STATE_APP_
{
  STATE_APP_STARTUP,
  STATE_APP_IDLE,
  STATE_APP_LEARN_MODE,
  STATE_APP_RESET,
  STATE_APP_TRANSMIT_DATA,
  STATE_APP_POWERDOWN
}
STATE_APP;

/**
 * The following values determine how long the application sleeps between
 * Wake Up Notifications. The more sleep, the less battery consumption.
 * The values in Sensor PIR are set to relatively low values because of
 * testing. An actual product would benefit from having high values.
 * All values in seconds.
 */
#define DEFAULT_SLEEP_TIME 300   // 5 minutes
#define MIN_SLEEP_TIME     20
#define MAX_SLEEP_TIME     86400 // 24 hours
#define STEP_SLEEP_TIME    20

/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

static SPowerLock_t m_RadioPowerLock;
static SPowerLock_t m_PeripheralPowerLock;
static bool pirStart = false;


/**
 * Please see the description of app_node_information_t.
 */
static uint8_t cmdClassListNonSecureNotIncluded[] =
{
  COMMAND_CLASS_ZWAVEPLUS_INFO,
  COMMAND_CLASS_ASSOCIATION,
  COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2,
  COMMAND_CLASS_ASSOCIATION_GRP_INFO,
  COMMAND_CLASS_TRANSPORT_SERVICE_V2,
  COMMAND_CLASS_VERSION,
  COMMAND_CLASS_MANUFACTURER_SPECIFIC,
  COMMAND_CLASS_DEVICE_RESET_LOCALLY,
  COMMAND_CLASS_INDICATOR,
  COMMAND_CLASS_POWERLEVEL,
  COMMAND_CLASS_BATTERY,
  COMMAND_CLASS_SECURITY_2,
  COMMAND_CLASS_NOTIFICATION_V3,
  COMMAND_CLASS_WAKE_UP,
  COMMAND_CLASS_SUPERVISION,
  COMMAND_CLASS_FIRMWARE_UPDATE_MD_V5
};

/**
 * Please see the description of app_node_information_t.
 */
static uint8_t cmdClassListNonSecureIncludedSecure[] =
{
  COMMAND_CLASS_ZWAVEPLUS_INFO,
  COMMAND_CLASS_TRANSPORT_SERVICE_V2,
  COMMAND_CLASS_SECURITY_2,
  COMMAND_CLASS_SUPERVISION
};

/**
 * Please see the description of app_node_information_t.
 */
static uint8_t cmdClassListSecure[] =
{
  COMMAND_CLASS_VERSION,
  COMMAND_CLASS_ASSOCIATION,
  COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2,
  COMMAND_CLASS_ASSOCIATION_GRP_INFO,
  COMMAND_CLASS_MANUFACTURER_SPECIFIC,
  COMMAND_CLASS_DEVICE_RESET_LOCALLY,
  COMMAND_CLASS_INDICATOR,
  COMMAND_CLASS_POWERLEVEL,
  COMMAND_CLASS_BATTERY,
  COMMAND_CLASS_NOTIFICATION_V3,
  COMMAND_CLASS_WAKE_UP,
  COMMAND_CLASS_FIRMWARE_UPDATE_MD_V5
};

/**
 * Structure includes application node information list's and device type.
 */
app_node_information_t m_AppNIF =
{
  cmdClassListNonSecureNotIncluded, sizeof(cmdClassListNonSecureNotIncluded),
  cmdClassListNonSecureIncludedSecure, sizeof(cmdClassListNonSecureIncludedSecure),
  cmdClassListSecure, sizeof(cmdClassListSecure),
  DEVICE_OPTIONS_MASK, {GENERIC_TYPE, SPECIFIC_TYPE}
};


/**
* Set up security keys to request when joining a network.
* These are taken from the config_app.h header file.
*/
static const uint8_t SecureKeysRequested = REQUESTED_SECURITY_KEYS;

static const SAppNodeInfo_t AppNodeInfo =
{
  .DeviceOptionsMask = DEVICE_OPTIONS_MASK,
  .NodeType.generic = GENERIC_TYPE,
  .NodeType.specific = SPECIFIC_TYPE,
  .CommandClasses.UnSecureIncludedCC.iListLength = sizeof_array(cmdClassListNonSecureNotIncluded),
  .CommandClasses.UnSecureIncludedCC.pCommandClasses = cmdClassListNonSecureNotIncluded,
  .CommandClasses.SecureIncludedUnSecureCC.iListLength = sizeof_array(cmdClassListNonSecureIncludedSecure),
  .CommandClasses.SecureIncludedUnSecureCC.pCommandClasses = cmdClassListNonSecureIncludedSecure,
  .CommandClasses.SecureIncludedSecureCC.iListLength = sizeof_array(cmdClassListSecure),
  .CommandClasses.SecureIncludedSecureCC.pCommandClasses = cmdClassListSecure
};

static const SRadioConfig_t RadioConfig =
{
  .iListenBeforeTalkThreshold = ELISTENBEFORETALKTRESHOLD_DEFAULT,
  .iTxPowerLevelMax = APP_MAX_TX_POWER,
  .iTxPowerLevelAdjust = APP_MEASURED_0DBM_TX_POWER,
  .eRegion = APP_FREQ
};

static const SProtocolConfig_t ProtocolConfig = {
  .pVirtualSlaveNodeInfoTable = NULL,
  .pSecureKeysRequested = &SecureKeysRequested,
  .pNodeInfo = &AppNodeInfo,
  .pRadioConfig = &RadioConfig
};


/**
 * Setup AGI lifeline table from config_app.h
 */
CMD_CLASS_GRP  agiTableLifeLine[] = {AGITABLE_LIFELINE_GROUP};

/**
 * Setup AGI root device groups table from config_app.h
 */
AGI_GROUP agiTableRootDeviceGroups[] = {AGITABLE_ROOTDEVICE_GROUPS};

static const AGI_PROFILE lifelineProfile = {
    ASSOCIATION_GROUP_INFO_REPORT_PROFILE_GENERAL,
    ASSOCIATION_GROUP_INFO_REPORT_PROFILE_GENERAL_LIFELINE
};

/**************************************************************************************************
 * Configuration for Z-Wave Plus Info CC
 **************************************************************************************************
 */
static const SCCZWavePlusInfo CCZWavePlusInfo = {
                               .pEndpointIconList = NULL,
                               .roleType = APP_ROLE_TYPE,
                               .nodeType = APP_NODE_TYPE,
                               .installerIconType = APP_ICON_TYPE,
                               .userIconType = APP_USER_ICON_TYPE
};

/**
 * Application state-machine state.
 */
static STATE_APP currentState = STATE_APP_IDLE;

/**
 * Parameter is used to save wakeup reason after ApplicationInit(..)
 */
static EResetReason_t g_eResetReason;

/**
 * A timer handle use in relation with handling sending basic set command
 * when an event occur. When the timer time out a basic set command
 * with the negated value is sent
 */
static uint8_t eventJobsTimerHandle = 0;

static uint8_t supportedEvents = NOTIFICATION_EVENT_HOME_SECURITY_MOTION_DETECTION_UNKNOWN_LOCATION;

static uint8_t basicValue = 0x00;

// Timer
static SSwTimer EventJobsTimer;
static bool eventJobsTimerStarted = false;

#ifdef DEBUGPRINT
static uint8_t m_aDebugPrintBuffer[96];
#endif

static TaskHandle_t g_AppTaskHandle;

// Pointer to AppHandles that is passed as input to ApplicationTask(..)
SApplicationHandles* g_pAppHandles;

// Prioritized events that can wakeup protocol thread.
typedef enum EApplicationEvent
{
  EAPPLICATIONEVENT_TIMER = 0,
  EAPPLICATIONEVENT_ZWRX,
  EAPPLICATIONEVENT_ZWCOMMANDSTATUS,
  EAPPLICATIONEVENT_APP
} EApplicationEvent;

static void EventHandlerZwRx(void);
static void EventHandlerZwCommandStatus(void);
static void EventHandlerApp(void);

// Event distributor object
static SEventDistributor g_EventDistributor;

// Event distributor event handler table
static const EventDistributorEventHandler g_aEventHandlerTable[4] =
{
  AppTimerNotificationHandler,  // Event 0
  EventHandlerZwRx,
  EventHandlerZwCommandStatus,
  EventHandlerApp
};

/* True Status Engine (TSE) variables */
static s_CC_indicator_data_t ZAF_TSE_localActuationIdentifyData = {
  .rxOptions = {
    .rxStatus = 0,          /* rxStatus, verified by the TSE for Multicast */
    .securityKey = 0,       /* securityKey, ignored by the TSE */
    .sourceNode = {0,0},    /* sourceNode (nodeId, endpoint), verified against lifeline destinations by the TSE */
    .destNode = {0,0}       /* destNode (nodeId, endpoint), verified by the TSE for local endpoint */
  },
  .indicatorId = 0x50      /* Identify Indicator*/
};

void AppResetNvm(void);

SBatteryData BatteryData;

#define BATTERY_DATA_UNASSIGNED_VALUE (CMD_CLASS_BATTERY_LEVEL_FULL + 1)  // Just some value not defined in cc_battery_level_t

#define APP_EVENT_QUEUE_SIZE 5

/**
 * The following four variables are used for the application event queue.
 */
static SQueueNotifying m_AppEventNotifyingQueue;
static StaticQueue_t m_AppEventQueueObject;
static EVENT_APP eventQueueStorage[APP_EVENT_QUEUE_SIZE];
static QueueHandle_t m_AppEventQueue;

static nvm3_Handle_t* pFileSystemApplication;

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/
void ZCB_BattReportSentDone(TRANSMISSION_RESULT * pTransmissionResult);
void DeviceResetLocallyDone(TRANSMISSION_RESULT * pTransmissionResult);
void DeviceResetLocally(void);
STATE_APP GetAppState();
void AppStateManager( EVENT_APP event);
static void ChangeState( STATE_APP newState);
void ZCB_JobStatus(TRANSMISSION_RESULT * pTransmissionResult);
void SetDefaultConfiguration(void);
bool LoadConfiguration(void);

static void LearnCompleted(void);
static void ApplicationTask(SApplicationHandles* pAppHandles);

void ZCB_EventJobsTimer(SSwTimer *pTimer);

SBatteryData readBatteryData(void);
void writeBatteryData(const SBatteryData* pBatteryData);

bool CheckBatteryLevel(void);
bool ReportBatteryLevel(void);

/**
* @brief Called when protocol puts a frame on the ZwRxQueue.
*/
static void EventHandlerZwRx(void)
{
  QueueHandle_t Queue = g_pAppHandles->ZwRxQueue;
  SZwaveReceivePackage RxPackage;

  // Handle incoming replies
  while (xQueueReceive(Queue, (uint8_t*)(&RxPackage), 0) == pdTRUE)
  {
    DPRINTF("\r\nIncoming Rx msg  type:%u class:%u ", RxPackage.eReceiveType, RxPackage.uReceiveParams.Rx.Payload.rxBuffer.ZW_Common.cmdClass);

    switch (RxPackage.eReceiveType)
    {
      case EZWAVERECEIVETYPE_SINGLE:
      {
        ZAF_CP_CommandPublish(ZAF_getCPHandle(), &RxPackage);
        break;
      }

      case EZWAVERECEIVETYPE_NODE_UPDATE:
      {
        /*ApplicationSlaveUpdate() was called from this place in version prior to SDK7*/
        CC_WakeUp_stayAwakeIfActive();
        break;
      }

      case EZWAVERECEIVETYPE_SECURITY_EVENT:
      {
        /*ApplicationSecurityEvent() was used to support CSA, not needed in SDK7*/
        break;
      }

    default:
      {
        ASSERT(false);
        break;
      }
    }
  }

}

/**
* @brief Triggered when protocol puts a message on the ZwCommandStatusQueue.
*/
static void EventHandlerZwCommandStatus(void)
{
  QueueHandle_t Queue = g_pAppHandles->ZwCommandStatusQueue;
  SZwaveCommandStatusPackage Status;

  // Handle incoming replies
  while (xQueueReceive(Queue, (uint8_t*)(&Status), 0) == pdTRUE)
  {
    DPRINT("\r\nIncoming Status msg\r\n");

    switch (Status.eStatusType)
    {
      case EZWAVECOMMANDSTATUS_TX:
      {
        SZWaveTransmitStatus* pTxStatus = &Status.Content.TxStatus;
        if (!pTxStatus->bIsTxFrameLegal)
        {
          DPRINT("\r\nAuch - not sure what to do");
        }
        else
        {
          DPRINT("\r\nTx Status received");
          if (ZAF_TSE_TXCallback == pTxStatus->Handle)
          {
            // TSE do not use the TX result to anything. Hence, we pass NULL.
            ZAF_TSE_TXCallback(NULL);
          }
          else if (pTxStatus->Handle)
          {
            void(*pCallback)(uint8_t txStatus, TX_STATUS_TYPE* extendedTxStatus) = pTxStatus->Handle;
            pCallback(pTxStatus->TxStatus, &pTxStatus->ExtendedTxStatus);
          }
        }

        break;
      }

      case EZWAVECOMMANDSTATUS_GENERATE_RANDOM:
      {
        DPRINT("\r\nGenerate Random status");
        break;
      }

      case EZWAVECOMMANDSTATUS_LEARN_MODE_STATUS:
      {
        DPRINTF("\r\nLearn status %d\n", Status.Content.LearnModeStatus.Status);
        if (ELEARNSTATUS_ASSIGN_COMPLETE == Status.Content.LearnModeStatus.Status)
        {
          LearnCompleted();
        }
        else if(ELEARNSTATUS_LEARN_MODE_COMPLETED_TIMEOUT == Status.Content.LearnModeStatus.Status)
        {
          ZAF_EventHelperEventEnqueue((EVENT_APP) EVENT_SYSTEM_LEARNMODE_FINISHED);
        }
        else if(ELEARNSTATUS_SMART_START_IN_PROGRESS == Status.Content.LearnModeStatus.Status)
        {
          ZAF_EventHelperEventEnqueue(EVENT_APP_SMARTSTART_IN_PROGRESS);
        }
        else if(ELEARNSTATUS_LEARN_IN_PROGRESS == Status.Content.LearnModeStatus.Status)
        {
          ZAF_EventHelperEventEnqueue(EVENT_APP_LEARN_IN_PROGRESS);
        }
        else if(ELEARNSTATUS_LEARN_MODE_COMPLETED_FAILED == Status.Content.LearnModeStatus.Status)
        {
          //Reformats protocol and application NVM. Then soft reset.
          ZAF_EventHelperEventEnqueue((EVENT_APP) EVENT_SYSTEM_RESET);
        }
        break;
      }

      case EZWAVECOMMANDSTATUS_NETWORK_LEARN_MODE_START:
      {
        break;
      }

      case EZWAVECOMMANDSTATUS_SET_DEFAULT:
      { // Received when protocol is started (not implemented yet), and when SetDefault command is completed
        DPRINTF("\r\nProtocol Ready");
        ZAF_EventHelperEventEnqueue(EVENT_APP_FLUSHMEM_READY);
        break;
      }

      case EZWAVECOMMANDSTATUS_INVALID_TX_REQUEST:
      {
        DPRINTF("\r\nERROR: Invalid TX Request to protocol - %d", Status.Content.InvalidTxRequestStatus.InvalidTxRequest);
        break;
      }

      case EZWAVECOMMANDSTATUS_INVALID_COMMAND:
      {
        DPRINTF("\r\nERROR: Invalid command to protocol - %d", Status.Content.InvalidCommandStatus.InvalidCommand);
        break;
      }

      case EZWAVECOMMANDSTATUS_ZW_SET_MAX_INCL_REQ_INTERVALS:
      {
        // Status response from calling the ZAF_SetMaxInclusionRequestIntervals function
        DPRINTF("SetMaxInclusionRequestIntervals status: %s\r\n",
                 Status.Content.NetworkManagementStatus.statusInfo[0] == true ? "SUCCESS" : "FAIL");

        // Add your application code here...
        break;
      }

      case EZWAVECOMMANDSTATUS_PM_SET_POWERDOWN_CALLBACK:
      {
        // Status response from calling the ZAF_PM_SetPowerDownCallback function
        if(false == Status.Content.SetPowerDownCallbackStatus.result)
        {
          DPRINT("Failed to register PowerDown Callback function\r\n");
          ASSERT(false);
        }
        break;
      }

      default:
        ASSERT(false);
        break;
    }
  }
}

/**
 *
 */
static void EventHandlerApp(void)
{
  uint8_t event;

  while (xQueueReceive(m_AppEventQueue, (uint8_t*)(&event), 0) == pdTRUE)
  {
    AppStateManager((EVENT_APP)event);
  }
}

/**
 * @brief See description for function prototype in ZW_basis_api.h.
 */
ZW_APPLICATION_STATUS
ApplicationInit(EResetReason_t eResetReason)
{
  // NULL - We dont have the Application Task handle yet
  AppTimerInit(EAPPLICATIONEVENT_TIMER, NULL);

  /* hardware initialization */
  Board_Init();

  /* Resolve pin conflicts */
  for (uint32_t led = 0; led < BOARD_LED_COUNT; led++)
  {
    Board_ConfigLed(led, false);
  }
  Board_ConfigLed(BOARD_LED1, true); // Configure LED0 (LED0 on WSTK)
  Board_ConfigLed(BOARD_LED3, true); // Configure LED2 (LED1 on WSTK)
  Board_ConfigLed(BOARD_RGB1_R, true);
  Board_ConfigLed(BOARD_RGB1_G, true);
  Board_ConfigLed(BOARD_RGB1_B, true);
  /* PIR gpio */
  GPIO_PinModeSet(LDO_SHDN_B_PORT, LDO_SHDN_B_PIN, gpioModePushPull, 0);  /* Disable the LDO. */
  GPIO_PinModeSet(ADC_P_PORT, ADC_P_PIN, gpioModeDisabled, 0); // ADC_P
  GPIO_PinModeSet(ADC_N_PORT, ADC_N_PIN, gpioModeDisabled, 0); // ADC_N
#ifdef DISABLE_USART0
  GPIO_PinModeSet(MOTION_B_PORT, MOTION_B_PIN, gpioModePushPull, 1);      /* Disable the EXP side LED. */
#endif
  //GPIO_PinModeSet(SENSOR_SCL_PORT, SENSOR_SCL_PIN, gpioModeWiredAndPullUp, 1);
  //GPIO_PinModeSet(SENSOR_SDA_PORT, SENSOR_SDA_PIN, gpioModeWiredAndPullUp, 1);
  //GPIO_PinModeSet(SENSOR_INT_PORT, SENSOR_INT_PIN, gpioModeWiredAndPullUp, 1);

  BRD420xBoardInit(RadioConfig.eRegion);

#ifdef DEBUGPRINT
  ZAF_UART0_enable(115200, true, false);
  DebugPrintConfig(m_aDebugPrintBuffer, sizeof(m_aDebugPrintBuffer), ZAF_UART0_tx_send);
#endif

  g_eResetReason = eResetReason;

  /* Init state machine*/
  currentState = STATE_APP_STARTUP;

  DPRINT("\n\n-----------------------------\n");
  DPRINT("Z-Wave Sample App: Sensor PIR\n");
  DPRINTF("SDK: %d.%d.%d ZAF: %d.%d.%d.%d [Freq: %d]\n",
          SDK_VERSION_MAJOR,
          SDK_VERSION_MINOR,
          SDK_VERSION_PATCH,
          ZAF_GetAppVersionMajor(),
          ZAF_GetAppVersionMinor(),
          ZAF_GetAppVersionPatchLevel(),
          ZAF_BUILD_NO,
          RadioConfig.eRegion);
  DPRINT("-----------------------------\n");
  DPRINTF("%s: Send battery report\n", Board_GetButtonLabel(BATTERY_REPORT_BTN));
  DPRINTF("%s: Toggle learn mode\n", Board_GetButtonLabel(APP_BUTTON_LEARN_RESET));
  DPRINT("      Hold 5 sec: Reset\n");
  DPRINTF("%s: Activate PIR event\n", Board_GetButtonLabel(PIR_EVENT_BTN));
  DPRINT("      (leave deactivated to allow going to sleep)\n");
  //DPRINTF("%s: Learn mode + identify\n", Board_GetLedLabel(APP_LED_INDICATOR));
  DPRINTF("%s: Learn mode + identify\n", Board_GetLedLabel(APP_LED_C));
  DPRINT("-----------------------------\n\n");

  DPRINTF("ApplicationInit eResetReason = %d\n", g_eResetReason);
  DPRINTF("Board_GetGpioEm4Flags()      = %08x\n", Board_GetGpioEm4Flags());
  if (g_eResetReason == ERESETREASON_EM4_WUT || g_eResetReason == ERESETREASON_EM4_EXT_INT)
  {
    #ifdef DEBUGPRINT
      Board_DebugPrintEm4WakeupFlags(Board_GetGpioEm4Flags());
    #endif
  }

  CC_ZWavePlusInfo_Init(&CCZWavePlusInfo);

  CC_Version_SetApplicationVersionInfo(ZAF_GetAppVersionMajor(),
                                       ZAF_GetAppVersionMinor(),
                                       ZAF_GetAppVersionPatchLevel(),
                                       ZAF_BUILD_NO);

  /* Register task function */
  bool bWasTaskCreated = ZW_ApplicationRegisterTask(
                                                    ApplicationTask,
                                                    EAPPLICATIONEVENT_ZWRX,
                                                    EAPPLICATIONEVENT_ZWCOMMANDSTATUS,
                                                    &ProtocolConfig
                                                    );
  ASSERT(bWasTaskCreated);

  return(APPLICATION_RUNNING);
}

/**
 * A pointer to this function is passed to ZW_ApplicationRegisterTask() making it the FreeRTOS
 * application task.
 */
static void
ApplicationTask(SApplicationHandles* pAppHandles)
{
  // Init
  DPRINT("Enabling watchdog\n");
  WDOGn_Enable(DEFAULT_WDOG, true);

  g_AppTaskHandle = xTaskGetCurrentTaskHandle();
  g_pAppHandles = pAppHandles;

  ZAF_Init(g_AppTaskHandle, pAppHandles, &ProtocolConfig, CC_WakeUp_stayAwakeIfActive);

  AppTimerSetReceiverTask(g_AppTaskHandle);
  /* Make sure to call AppTimerEm4PersistentRegister() _after_ ZAF_Init().
   * It will access the app handles */
  AppTimerEm4PersistentRegister(&EventJobsTimer, false, ZCB_EventJobsTimer);

  // Initialize CC Wake Up
  CC_WakeUp_setConfiguration(WAKEUP_PAR_DEFAULT_SLEEP_TIME, DEFAULT_SLEEP_TIME);
  CC_WakeUp_setConfiguration(WAKEUP_PAR_MAX_SLEEP_TIME,     MAX_SLEEP_TIME);
  CC_WakeUp_setConfiguration(WAKEUP_PAR_MIN_SLEEP_TIME,     MIN_SLEEP_TIME);
  CC_WakeUp_setConfiguration(WAKEUP_PAR_SLEEP_STEP,         STEP_SLEEP_TIME);

  ZAF_PM_Register(&m_RadioPowerLock, PM_TYPE_RADIO);
  ZAF_PM_Register(&m_PeripheralPowerLock, PM_TYPE_PERIPHERAL);

  // Initialize Queue Notifier for events in the application.
  m_AppEventQueue = xQueueCreateStatic(
    sizeof_array(eventQueueStorage),
    sizeof(eventQueueStorage[0]),
    (uint8_t*)eventQueueStorage,
    &m_AppEventQueueObject
  );

  QueueNotifyingInit(
      &m_AppEventNotifyingQueue,
      m_AppEventQueue,
      g_AppTaskHandle,
      EAPPLICATIONEVENT_APP);

  ZAF_EventHelperInit(&m_AppEventNotifyingQueue);

  ZAF_JobHelperInit();
  ZAF_EventHelperEventEnqueue(EVENT_APP_INIT);

  //Enables events on test board
  Board_EnableButton(APP_BUTTON_LEARN_RESET);
  //Board_EnableButton(BATTERY_REPORT_BTN);
  Board_EnableButton(PIR_EVENT_BTN);

  //Board_IndicatorInit(APP_LED_INDICATOR);
  Board_IndicatorInit(APP_LED_C);
  Board_IndicateStatus(BOARD_STATUS_IDLE);

  CommandClassSupervisionInit(
      CC_SUPERVISION_STATUS_UPDATES_NOT_SUPPORTED,
      NULL,
      NULL);

  EventDistributorConfig(
    &g_EventDistributor,
    sizeof_array(g_aEventHandlerTable),
    g_aEventHandlerTable,
    NULL
    );

  DPRINTF("\nIsWakeupCausedByRtccTimeout=%s\n", (IsWakeupCausedByRtccTimeout()) ? "true" : "false");
  DPRINTF("CompletedSleepDurationMs   =%u\n", GetCompletedSleepDurationMs());

  // Wait for and process events
  DPRINT("\r\nSensorPIR Event processor Started\n");
  uint32_t iMaxTaskSleep = 0xFFFFFFFF;
  for (;;)
  {
    EventDistributorDistribute(&g_EventDistributor, iMaxTaskSleep, 0);
  }
}

/**
 * @brief See description for function prototype in ZW_TransportEndpoint.h.
 */
received_frame_status_t
Transport_ApplicationCommandHandlerEx(
  RECEIVE_OPTIONS_TYPE_EX *rxOpt,
  ZW_APPLICATION_TX_BUFFER *pCmd,
  uint8_t cmdLength)
{
  received_frame_status_t frame_status = RECEIVED_FRAME_STATUS_NO_SUPPORT;
  DPRINTF("\r\nTransport_ApplicationCommandHandlerEx cmdClass:0x%02x\n", pCmd->ZW_Common.cmdClass);


  /* Call command class handlers */
  switch (pCmd->ZW_Common.cmdClass)
  {
    case COMMAND_CLASS_VERSION:
      frame_status = CC_Version_handler(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_ASSOCIATION_GRP_INFO:
      frame_status = CC_AGI_handler( rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_ASSOCIATION:
      frame_status = handleCommandClassAssociation(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_INDICATOR:
      frame_status = handleCommandClassIndicator(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_POWERLEVEL:
      frame_status = CC_Powerlevel_handler(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_MANUFACTURER_SPECIFIC:
      frame_status = handleCommandClassManufacturerSpecific(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_ZWAVEPLUS_INFO:
      frame_status = handleCommandClassZWavePlusInfo(rxOpt, pCmd, cmdLength);
      break;

   case COMMAND_CLASS_BATTERY:
      frame_status = CC_Battery_handler(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_NOTIFICATION_V3:
      frame_status = handleCommandClassNotification(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_WAKE_UP:
      frame_status = CC_WakeUp_handler(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_SUPERVISION:
      frame_status = handleCommandClassSupervision(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2:
      frame_status = handleCommandClassMultiChannelAssociation(rxOpt, pCmd, cmdLength);
      break;

	case COMMAND_CLASS_FIRMWARE_UPDATE_MD_V5:
      frame_status = handleCommandClassFWUpdate(rxOpt, pCmd, cmdLength);
    break;
  }
  return frame_status;
}


static void
LearnCompleted(void)
{
  uint8_t bNodeID = g_pAppHandles->pNetworkInfo->NodeId;
  /*If bNodeID= 0xff.. learn mode failed*/
DPRINTF("\r\n LEARN COMPLETED  bNodeID = %u\r\n", bNodeID);
  if (bNodeID != NODE_BROADCAST)
  {
    /*Success*/
    if (bNodeID == 0)
    {
      SetDefaultConfiguration();
    }
  }
  ZAF_EventHelperEventEnqueue((EVENT_APP) EVENT_SYSTEM_LEARNMODE_FINISHED);
  Transport_OnLearnCompleted(bNodeID);
}

/**
 * @brief See description for function prototype in misc.h.
 */
uint8_t
GetMyNodeID(void)
{
  return g_pAppHandles->pNetworkInfo->NodeId;
}

/**
 * @brief Returns the current state of the application state machine.
 * @return Current state
 */
STATE_APP
GetAppState(void)
{
  return currentState;
}

/**
 * @brief The core state machine of this sample application.
 * @param event The event that triggered the call of AppStateManager.
 */
void
AppStateManager(EVENT_APP event)
{
  DPRINTF("AppStateManager St: %d, Ev: %d\r\n", currentState, event);

  if ((BTN_EVENT_LONG_PRESS(APP_BUTTON_LEARN_RESET) == (BUTTON_EVENT)event) || (EVENT_SYSTEM_RESET == (EVENT_SYSTEM)event))
  {
    /*Force state change to activate system-reset without taking care of current
      state.*/
    ChangeState(STATE_APP_RESET);
    /* Send reset notification*/
    DeviceResetLocally();
  }

  if (PIR_EVENT_TRANSITION_TO_DEACTIVE(event))
  {
    ZAF_PM_Cancel(&m_RadioPowerLock);
  }

  switch(currentState)
  {

    case STATE_APP_STARTUP:
      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
      }

      if (EVENT_APP_INIT == event)
      {

        /* Load the application settings from NVM3 file system */
        bool filesExist = LoadConfiguration();

        /* Setup AGI group lists*/
        AGI_Init();
        CC_AGI_LifeLineGroupSetup(agiTableLifeLine, (sizeof(agiTableLifeLine)/sizeof(CMD_CLASS_GRP)), ENDPOINT_ROOT );
        AGI_ResourceGroupSetup(agiTableRootDeviceGroups, (sizeof(agiTableRootDeviceGroups)/sizeof(AGI_GROUP)), ENDPOINT_ROOT);

        /* Check the battery level.
         * If required, go to TRANSMIT state to send the report to the lifeline.
         * The Battery Report must be sent out before the WakeUp Notification. Therefore this function
         * must called prior to anything CC Wakeup related.
         */
        /*
        if (true == CheckBatteryLevel())
        {
          // Battery level has changed, so enqueue the event.
          ChangeState(STATE_APP_TRANSMIT_DATA);
          if (false == ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB))
          {
            DPRINT("\r\n** EVENT_APP_NEXT_EVENT_JOB fail\r\n");
          }
          ZAF_JobHelperJobEnqueue(EVENT_APP_SEND_BATTERY_LEVEL_REPORT);
        }*/

        CC_WakeUp_init(g_eResetReason, pFileSystemApplication);

        InitNotification(pFileSystemApplication);

        AddNotification(
            &lifelineProfile,
            NOTIFICATION_TYPE_HOME_SECURITY,
            &supportedEvents,
            1,
            false,
            0,
            NOTIFICATION_STATUS_UNSOLICIT_ACTIVATED,
            filesExist
          );
        /*
         * Initialize Event Scheduler.
         */
        Transport_OnApplicationInitSW( &m_AppNIF, CC_WakeUp_stayAwakeIfActive);

        /* Re-load and process EM4 persistent application timers.
         * NB: Before calling AppTimerEm4PersistentLoadAll here all
         *     application timers must have been been registered with
         *     AppTimerRegister() or AppTimerEm4PersistentRegister().
         *     Essentially it means that all CC handlers must be
         *     initialized first.
         */
        AppTimerEm4PersistentLoadAll(g_eResetReason);

        CC_FirmwareUpdate_Init(NULL, NULL, NULL, true);

        if (ERESETREASON_EM4_EXT_INT == g_eResetReason)
        {
          uint32_t em4_wakeup_flags = Board_GetGpioEm4Flags();

          if (0 != em4_wakeup_flags)
          {
            Board_ProcessEm4PinWakeupFlags(em4_wakeup_flags);
          }
          ChangeState(STATE_APP_IDLE);
        }

        if (ERESETREASON_EM4_WUT == g_eResetReason)
        {
          /* If we entered TRANSMIT state to send battery report, don't change it */
          if (STATE_APP_TRANSMIT_DATA != currentState)
          {
            ChangeState(STATE_APP_IDLE);
          }
        }
        if(ERESETREASON_PIN == g_eResetReason ||
           ERESETREASON_POWER_ON == g_eResetReason ||
           ERESETREASON_SOFTWARE == g_eResetReason ||
           ERESETREASON_WATCHDOG == g_eResetReason)
        {
          /* Init state machine*/
          ZAF_EventHelperEventEnqueue(EVENT_EMPTY);
        }

        if(ERESETREASON_EM4_EXT_INT != g_eResetReason)
        {
          /* Enter SmartStart*/
          /* Protocol will commence SmartStart only if the node is NOT already included in the network */
          ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_INCLUSION_SMARTSTART, g_eResetReason);
        }

        break;
      }
      if (EVENT_APP_IS_POWERING_DOWN == event)
      {
        Board_IndicateStatus(BOARD_STATUS_POWER_DOWN);
        ChangeState(STATE_APP_POWERDOWN);
        break;
      }
      ChangeState(STATE_APP_IDLE);
      break;

    case STATE_APP_IDLE:
      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
        LoadConfiguration();
      }

      if (EVENT_APP_SMARTSTART_IN_PROGRESS == event)
      {
        ChangeState(STATE_APP_LEARN_MODE);
      }

      if ((BTN_EVENT_SHORT_PRESS(APP_BUTTON_LEARN_RESET) == (BUTTON_EVENT)event) ||
          (EVENT_SYSTEM_LEARNMODE_START == (EVENT_SYSTEM)event))
      {
        if (EINCLUSIONSTATE_EXCLUDED != g_pAppHandles->pNetworkInfo->eInclusionState){
          DPRINT("\r\nLEARN_MODE_EXCLUSION");
          ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_EXCLUSION_NWE, g_eResetReason);
        }
        else{
          DPRINT("\r\nLEARN_MODE_INCLUSION");
          ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_INCLUSION, g_eResetReason);
        }
        Board_IndicateStatus(BOARD_STATUS_LEARNMODE_ACTIVE);
        ChangeState(STATE_APP_LEARN_MODE);
      }
/*
      if (PIR_EVENT_TRANSITION_TO_ACTIVE(event))
      {
        ZAF_PM_StayAwake(&m_RadioPowerLock, 0);
        DPRINT("\r\n*!*!* PIR_EVENT_BTN");
        ChangeState(STATE_APP_TRANSMIT_DATA);

        if (false == ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB))
        {
          DPRINT("\r\n** EVENT_APP_NEXT_EVENT_JOB fail\r\n");
        }
        //Add event's on job-queue
        ZAF_JobHelperJobEnqueue(EVENT_APP_BASIC_START_JOB);
        ZAF_JobHelperJobEnqueue(EVENT_APP_NOTIFICATION_START_JOB);
        ZAF_JobHelperJobEnqueue(EVENT_APP_START_TIMER_EVENTJOB_STOP);
      }
*/
      if (EVENT_APP_MOTION_DETECTED == event)
      {
    	DPRINT("\r\n** Motion detected!!!\r\n");
    	ZAF_PM_StayAwake(&m_RadioPowerLock, 0);
        if(eventJobsTimerStarted)
        {
    	  AppTimerEm4PersistentRestart(&EventJobsTimer);
        }
    	ChangeState(STATE_APP_TRANSMIT_DATA);

        if (false == ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB))
        {
          DPRINT("\r\n** EVENT_APP_NEXT_EVENT_JOB fail\r\n");
        }
        //Add event's on job-queue
        ZAF_JobHelperJobEnqueue(EVENT_APP_BASIC_START_JOB);
        ZAF_JobHelperJobEnqueue(EVENT_APP_NOTIFICATION_START_JOB);
        ZAF_JobHelperJobEnqueue(EVENT_APP_START_TIMER_EVENTJOB_STOP);
      }

      if (BTN_EVENT_SHORT_PRESS(PIR_EVENT_BTN) == (BUTTON_EVENT)event)
      {
    	if(pirStart)
        {
          pirStart = false;
          PIR_Stop();
          DPRINT("\r\n** Sleep!!!\r\n");
          ZAF_PM_Cancel(&m_PeripheralPowerLock);
          ZAF_PM_Cancel(&m_RadioPowerLock);
        }
        else
        {
          ZAF_PM_StayAwake(&m_PeripheralPowerLock, 0);
          DPRINT("\r\n** Wake up!!!\r\n");
          //Init PIR
          pirStart = true;
          DPRINT("\r\n** Start PIR!!!\r\n");
          /* Enable LDO to startup VPIR. */
          GPIO_PinOutSet(LDO_SHDN_B_PORT, LDO_SHDN_B_PIN);
          PIR_Init_TypeDef pirInit = PIR_INIT_DEFAULT;
          pirInit.opampMode = pirOpampModeExternal;
          PIR_Init(&pirInit, true);
          PIR_Start();
        }

        /* BATTERY_REPORT_BTN pressed. Send a battery level report */
/*        DPRINT("\r\nBattery Level report transmit (keypress trig)\r\n");
        ChangeState(STATE_APP_TRANSMIT_DATA);

        if (false == ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB))
        {
          DPRINT("\r\n** EVENT_APP_NEXT_EVENT_JOB fail\r\n");
        }

        //Add event's on job-queue
        ZAF_JobHelperJobEnqueue(EVENT_APP_SEND_BATTERY_LEVEL_REPORT);*/

      }
      break;
    case STATE_APP_LEARN_MODE:
      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
        LoadConfiguration();
      }

      if ((BTN_EVENT_SHORT_PRESS(APP_BUTTON_LEARN_RESET) == (BUTTON_EVENT)event) ||
          (EVENT_SYSTEM_LEARNMODE_STOP == (EVENT_SYSTEM)event))
      {
        ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_DISABLE, g_eResetReason);

        //Go back to smart start if the node was never included.
        //Protocol will not commence SmartStart if the node is already included in the network.
        ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_INCLUSION_SMARTSTART, g_eResetReason);

        Board_IndicateStatus(BOARD_STATUS_IDLE);
        ChangeState(STATE_APP_IDLE);

        /* If we are in a network and the Identify LED state was changed to idle due to learn mode, report it to lifeline */
        CC_Indicator_RefreshIndicatorProperties();
        ZAF_TSE_Trigger((void *)CC_Indicator_report_stx, &ZAF_TSE_localActuationIdentifyData, true);
      }

      if (EVENT_SYSTEM_LEARNMODE_FINISHED == (EVENT_SYSTEM)event)
      {
        //Make sure that the application stays awake for 10 s after learn mode finished
        //to wait for more messages from the controller
        CC_WakeUp_stayAwake10s();

        /* Also tell application to automatically extend the stay awake period by 10
         * seconds on message activities - even though we did not get here by a proper
         * wakeup from EM4
         */
        CC_WakeUp_AutoStayAwakeAfterInclusion();

        //Go back to smart start if the node was excluded.
        //Protocol will not commence SmartStart if the node is already included in the network.
        ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_INCLUSION_SMARTSTART, g_eResetReason);

        Board_IndicateStatus(BOARD_STATUS_IDLE);
        ChangeState(STATE_APP_IDLE);

        /* If we are in a network and the Identify LED state was changed to idle due to learn mode, report it to lifeline */
        CC_Indicator_RefreshIndicatorProperties();
        ZAF_TSE_Trigger((void *)CC_Indicator_report_stx, &ZAF_TSE_localActuationIdentifyData, true);

        // Start the wakeup timer if the learn mode operation finished in Included state
        if (EINCLUSIONSTATE_EXCLUDED != g_pAppHandles->pNetworkInfo->eInclusionState)
        {
          CC_WakeUp_startWakeUpNotificationTimer();
        }
      }
      break;

    case STATE_APP_RESET:
      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
        /* Soft reset */
        Board_ResetHandler();
      }
      break;

    case STATE_APP_POWERDOWN:
      /* Device is powering down.. wait!*/
      break;

    case STATE_APP_TRANSMIT_DATA:
      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
        LoadConfiguration();
      }
/*
      if (EVENT_KEY_B6_PRESS == (EVENT_KEY)event)
      {
        DPRINT("\r\n TIMER RESTART\r\n");
        TimerRestart(&EventJobsTimer);
      }
*/
      if (EVENT_APP_NEXT_EVENT_JOB == event)
      {
        uint8_t event;
        /*check job-queue*/
        if (true == ZAF_JobHelperJobDequeue(&event))
        {
          /*Let the event scheduler fire the event on state event machine*/
          ZAF_EventHelperEventEnqueue(event);
        }
        else
        {
          ZAF_EventHelperEventEnqueue(EVENT_APP_FINISH_EVENT_JOB);
        }
      }

      if (EVENT_APP_BASIC_START_JOB == event)
      {
        if (JOB_STATUS_SUCCESS != CC_Basic_Set_tx( &agiTableRootDeviceGroups[0].profile, ENDPOINT_ROOT, BASIC_SET_TRIGGER_VALUE, ZCB_JobStatus))
          {
           DPRINT("\r\EVENT_APP_BASIC_START_JOB failed");
          basicValue = BASIC_SET_TRIGGER_VALUE;
          /*Kick next job*/
          ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        }
      }

      if (EVENT_APP_BASIC_STOP_JOB == event)
      {
        if (JOB_STATUS_SUCCESS != CC_Basic_Set_tx( &agiTableRootDeviceGroups[0].profile, ENDPOINT_ROOT, 0, ZCB_JobStatus))
        {
          DPRINT("\r\EVENT_APP_BASIC_STOP_JOB failed");
          basicValue = 0;
          /*Kick next job*/
          ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        }
      }


      if (EVENT_APP_NOTIFICATION_START_JOB == event)
      {
        DPRINT("\r\nEVENT_APP_NOTIFICATION_START_JOB");
        NotificationEventTrigger(&lifelineProfile,
        		                 NOTIFICATION_TYPE_HOME_SECURITY,
                                 supportedEvents,
                                 NULL, 0,
                                 ENDPOINT_ROOT);
        if (JOB_STATUS_SUCCESS !=  UnsolicitedNotificationAction(&lifelineProfile, ENDPOINT_ROOT, ZCB_JobStatus))
        {
          /*Kick next job*/
          ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        }
      }

      if (EVENT_APP_NOTIFICATION_STOP_JOB == event)
      {
        DPRINT("\r\nEVENT_APP_NOTIFICATION_STOP_JOB");
        NotificationEventTrigger(&lifelineProfile,
        		                 NOTIFICATION_TYPE_HOME_SECURITY,
                                 0,
                                 &supportedEvents,
								 1,
                                 ENDPOINT_ROOT);
        if (JOB_STATUS_SUCCESS !=  UnsolicitedNotificationAction(&lifelineProfile, ENDPOINT_ROOT, ZCB_JobStatus))
        {
          /*Kick next job*/
          ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        }
      }

      if (EVENT_APP_START_TIMER_EVENTJOB_STOP== event)
      {
        DPRINT("\r\n#EVENT_APP_START_TIMER_EVENTJOB_STOP\r\n");
        if(!eventJobsTimerStarted)
        {
          AppTimerEm4PersistentStart(&EventJobsTimer, BASIC_SET_TIMEOUT);
          eventJobsTimerStarted = true;
        }
        ChangeState(STATE_APP_IDLE);
        ZAF_PM_Cancel(&m_RadioPowerLock);
      }

      if (EVENT_APP_SEND_BATTERY_LEVEL_REPORT == event)
      {
        ReportBatteryLevel();
      }

      if (EVENT_APP_FINISH_EVENT_JOB == event)
      {
        DPRINT("\r\n#EVENT_APP_FINISH_EVENT_JOB\r\n");
        if (0 == eventJobsTimerHandle)
        {
           //Board_IndicateStatus(BOARD_STATUS_IDLE);
           eventJobsTimerStarted = false;
           ChangeState(STATE_APP_IDLE);
        }
      }
      break;

    default:
      // Do nothing.
      break;
  }
}

/**
 * @brief Sets the current state to a new, given state.
 * @param newState New state.
 */
static void
ChangeState(STATE_APP newState)
{
  DPRINTF("\r\nState changed: %d -> %d\n", currentState, newState);

  currentState = newState;
}

/**
 * @brief Transmission callback for Device Reset Locally call.
 * @param pTransmissionResult Result of each transmission.
 */
void
DeviceResetLocallyDone(TRANSMISSION_RESULT * pTransmissionResult)
{
  if (TRANSMISSION_RESULT_FINISHED == pTransmissionResult->isFinished)
  {
    /* Reset protocol */
    // Set default command to protocol
    SZwaveCommandPackage CommandPackage;
    CommandPackage.eCommandType = EZWAVECOMMANDTYPE_SET_DEFAULT;

    DPRINT("\nDisabling watchdog during reset\n");
    WDOGn_Enable(DEFAULT_WDOG, false);

    EQueueNotifyingStatus Status = QueueNotifyingSendToBack(g_pAppHandles->pZwCommandQueue, (uint8_t*)&CommandPackage, 500);
    ASSERT(EQUEUENOTIFYING_STATUS_SUCCESS == Status);
  }
}

/**
 * @brief Reset delay callback.
 */
void
DeviceResetLocally(void)
{
  DPRINT("\r\nCall locally reset");

  CC_DeviceResetLocally_notification_tx(&lifelineProfile, DeviceResetLocallyDone);
}

/**
 * @brief See description for function prototype in CC_Version.h.
 */
uint8_t
CC_Version_getNumberOfFirmwareTargets_handler(void)
{
  return 1; /*CHANGE THIS - firmware 0 version*/
}

/**
 * @brief See description for function prototype in CC_Version.h.
 */
void
handleGetFirmwareVersion(
  uint8_t bFirmwareNumber,
  VG_VERSION_REPORT_V2_VG *pVariantgroup)
{
  /*firmware 0 version and sub version*/
  if (bFirmwareNumber == 0)
  {
    pVariantgroup->firmwareVersion = ZAF_GetAppVersionMajor();
    pVariantgroup->firmwareSubVersion = ZAF_GetAppVersionMinor();
  }
  else
  {
    /*Just set it to 0 if firmware n is not present*/
    pVariantgroup->firmwareVersion = 0;
    pVariantgroup->firmwareSubVersion = 0;
  }
}

#define MY_BATTERY_SPEC_LEVEL_FULL         3000  // My battery's 100% level (millivolts)
#define MY_BATTERY_SPEC_LEVEL_EMPTY        2400  // My battery's 0% level (millivolts)
#define BATTERY_LEVEL_REPORTING_DECREMENTS   10  // Round off and report the level in 10% decrements (100%, 90%, 80%, etc)
/**
 * Report current battery level to battery command class handler
 *
 * @brief See description for function prototype in CC_Battery.h
 */
uint8_t
CC_Battery_BatteryGet_handler(uint8_t endpoint)
{
  uint32_t VBattery;
  uint8_t  accurateLevel;
  uint8_t  roundedLevel;

  UNUSED(endpoint);

  /*
   * Simple example how to use the ADC to measure the battery voltage
   * and convert to a percentage battery level on a linear scale.
   */
  ZAF_ADC_Enable();
  VBattery = ZAF_ADC_Measure_VSupply();
  DPRINTF("\r\nBattery voltage: %dmV", VBattery);
  ZAF_ADC_Disable();

  if (MY_BATTERY_SPEC_LEVEL_FULL <= VBattery)
  {
    // Level is full
    return (uint8_t)CMD_CLASS_BATTERY_LEVEL_FULL;
  }
  else if (MY_BATTERY_SPEC_LEVEL_EMPTY > VBattery)
  {
    // Level is empty (<0%)
    return (uint8_t)CMD_CLASS_BATTERY_LEVEL_WARNING;
  }
  else
  {
    // Calculate the percentage level from 0 to 100
    accurateLevel = (100 * (VBattery - MY_BATTERY_SPEC_LEVEL_EMPTY)) / (MY_BATTERY_SPEC_LEVEL_FULL - MY_BATTERY_SPEC_LEVEL_EMPTY);

    // And round off to the nearest "BATTERY_LEVEL_REPORTING_DECREMENTS" level
    roundedLevel =  (accurateLevel / BATTERY_LEVEL_REPORTING_DECREMENTS) * BATTERY_LEVEL_REPORTING_DECREMENTS; // Rounded down
    if ((accurateLevel % BATTERY_LEVEL_REPORTING_DECREMENTS) >= (BATTERY_LEVEL_REPORTING_DECREMENTS / 2))
    {
      roundedLevel += BATTERY_LEVEL_REPORTING_DECREMENTS; // Round up
    }
  }
  return roundedLevel;
}

/**
 * Checks if battery level was changed since last time it was reported.
 * If the node is not in the network, it will just return false.
 * @return true if change happened, false if battery level is unchanged.
 */
bool CheckBatteryLevel(void)
{
  uint8_t currentBatteryLevel;

  if (0 == GetMyNodeID())
  {
    // We are not network included. Nothing to do.
    DPRINTF("\r\n%s: Not included\r\n", __FUNCTION__);
    return false;
  }

  currentBatteryLevel = CC_Battery_BatteryGet_handler(ENDPOINT_ROOT);
  DPRINTF("\r\n%s: Current Level=%d, Last reported level=%d\r\n", __FUNCTION__, currentBatteryLevel, BatteryData.lastReportedBatteryLevel);

  if ((currentBatteryLevel == BatteryData.lastReportedBatteryLevel) ||
      (currentBatteryLevel == BatteryData.lastReportedBatteryLevel + BATTERY_LEVEL_REPORTING_DECREMENTS)) // Hysteresis
  {
    // Battery level hasn't changed (significantly) since last reported. Do nothing
    return false;
  }

  return true;
}

/**
 * Send battery level report
 */
bool ReportBatteryLevel(void)
{
  DPRINTF("\r\nSending battery level report\r\n");

  uint8_t currentBatteryLevel = CC_Battery_BatteryGet_handler(ENDPOINT_ROOT);
  // Battery level has changed. Send a new update to the lifeline
  if (JOB_STATUS_SUCCESS != CC_Battery_LevelReport_tx(&lifelineProfile,
                                                      ENDPOINT_ROOT,
                                                      currentBatteryLevel,
                                                      ZCB_BattReportSentDone
                                                      ))
  {
    DPRINTF("\r\n%s: TX FAILED ** \r\n", __FUNCTION__);
    ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
    return false;
  }

  // Report successfully sent. Update the last reported value and store in flash
  BatteryData.lastReportedBatteryLevel = currentBatteryLevel;
  writeBatteryData(&BatteryData);
  return true;
}


/**
 * @brief Callback function used for unsolicited commands.
 * @param pTransmissionResult Result of each transmission.
 */
void
ZCB_JobStatus(TRANSMISSION_RESULT * pTransmissionResult)
{
  if (TRANSMISSION_RESULT_FINISHED == pTransmissionResult->isFinished)
  {
    ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
  }
}

/**
 * @brief Function resets configuration to default values.
 */
void
SetDefaultConfiguration(void)
{
  Ecode_t errCode;

  DPRINT("\r\nSet Default Configuration");
  AssociationInit(true, pFileSystemApplication);

  DefaultNotificationStatus(NOTIFICATION_STATUS_UNSOLICIT_ACTIVATED);
  CC_WakeUp_notificationMemorySetDefault(pFileSystemApplication);
  DPRINT("\r\nEnded Set Default Configuration");

  BatteryData.lastReportedBatteryLevel = BATTERY_DATA_UNASSIGNED_VALUE;
  writeBatteryData(&BatteryData);

  uint32_t appVersion = ZAF_GetAppVersion();
  errCode = nvm3_writeData(pFileSystemApplication, ZAF_FILE_ID_APP_VERSION, &appVersion, ZAF_FILE_SIZE_APP_VERSION);
  ASSERT(ECODE_NVM3_OK == errCode); //Assert has been kept for debugging , can be removed from production code if this error can only be caused by some internal flash HW failure
}

/**
 * @brief This function loads the application settings from non-volatile memory.
 * If no settings are found, default values are used and saved.
 */
bool
LoadConfiguration(void)
{
  // Init file system
  ApplicationFileSystemInit(&pFileSystemApplication);

  uint32_t appVersion;
  Ecode_t versionFileStatus = nvm3_readData(pFileSystemApplication, ZAF_FILE_ID_APP_VERSION, &appVersion, ZAF_FILE_SIZE_APP_VERSION);

  if (ECODE_NVM3_OK == versionFileStatus)
  {
    if (ZAF_GetAppVersion() != appVersion)
    {
      // Add code for migration of file system to higher version here.
    }

    BatteryData = readBatteryData();

    /* Initialize association module */
    AssociationInit(false, pFileSystemApplication);

    loadStatusPowerLevel();
    return true;
  }
  else
  {
    DPRINT("\r\nApplication FileSystem Verify failed");
    loadInitStatusPowerLevel();

    // Reset the file system if ZAF_FILE_ID_APP_VERSION is missing since this indicates
    // corrupt or missing file system.
    AppResetNvm();
    return false;
  }
}

void AppResetNvm(void)
{
  DPRINT("\r\nResetting application FileSystem to default");

  ASSERT(0 != pFileSystemApplication); //Assert has been kept for debugging , can be removed from production code. This error can only be caused by some internal flash HW failure

  Ecode_t errCode = nvm3_eraseAll(pFileSystemApplication);
  ASSERT(ECODE_NVM3_OK == errCode); //Assert has been kept for debugging , can be removed from production code. This error can only be caused by some internal flash HW failure

  /* Initialize transport layer */
  Transport_SetDefault();

  /* Apparently there is no valid configuration in NVM3, so load */
  /* default values and save them. */
  SetDefaultConfiguration();

  AppTimerEm4PersistentResetStorage();
}

/**
 * @brief Handler for basic set. Handles received basic set commands.
 * @param[in] val Parameter dependent of the application device class.
 * @param[in] endpoint Endpoint
 * @return command handler return code
 */
e_cmd_handler_return_code_t
CC_Basic_Set_handler(uint8_t val, uint8_t endpoint)
{
  UNUSED(val);
  UNUSED(endpoint);
  /* CHANGE THIS - Fill in your application code here */

  // change return code if relevant
  return E_CMD_HANDLER_RETURN_CODE_NOT_SUPPORTED;
}

/**
 * @brief Handler for basic get. Handles received basic get commands.
 */
uint8_t
CC_Basic_GetTargetValue_handler(uint8_t endpoint)
{
  /* CHANGE THIS - Fill in your application code here */
  UNUSED(endpoint);
  return basicValue;
}

/**
 * @brief Report the target value
 * @return target value.
 */
uint8_t
CC_Basic_GetCurrentValue_handler(uint8_t endpoint)
{
  UNUSED(endpoint);
  /* CHANGE THIS - Fill in your application code here */
  return 0;
}

/**
 * @brief Report transition duration time.
 * @return duration time.
 */
uint8_t
CC_Basic_GetDuration_handler(uint8_t endpoint)
{
  UNUSED(endpoint);
  /* CHANGE THIS - Fill in your application code here */
  return 0;
}

/**
 * @brief Callback function used when sending battery report.
 */
void
ZCB_BattReportSentDone(TRANSMISSION_RESULT * pTransmissionResult)
{
  DPRINTF("\r\n%d  %d  %d ", pTransmissionResult->nodeId, pTransmissionResult->status, pTransmissionResult->isFinished);
  if (TRANSMISSION_RESULT_FINISHED == pTransmissionResult->isFinished)
  {
    DPRINTF("\r\nBattery level report transmit finished\r\n");
    ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
  }
}

/**
 * @brief event jobs timeout event
 *
 * @param pTimer Timer connected to this method
 */
void
ZCB_EventJobsTimer(SSwTimer *pTimer)
{
  DPRINTF("Timer callback: ZCB_EventJobsTimer() pTimer->Id=%d\n", pTimer->Id);
  eventJobsTimerHandle = 0;
  /* If the node has been woken up from EM4 because the event job timer timed out
   * the app will now be in the state STATE_APP_STARTUP. Need to switch to
   * STATE_APP_TRANSMIT_DATA to properly process the job events
   */
  ZAF_JobHelperJobEnqueue(EVENT_APP_NOTIFICATION_STOP_JOB);
  ZAF_JobHelperJobEnqueue(EVENT_APP_BASIC_STOP_JOB);

  if (STATE_APP_TRANSMIT_DATA != currentState)
  {
    ChangeState(STATE_APP_TRANSMIT_DATA);
  }
  // ToDo: kick off next job
  ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
  UNUSED(pTimer);
}

/**
 * @brief Reads battery data from file system.
 */
SBatteryData
readBatteryData(void)
{
  SBatteryData StoredBatteryData;

  Ecode_t errCode = nvm3_readData(pFileSystemApplication, ZAF_FILE_ID_BATTERYDATA, &StoredBatteryData, ZAF_FILE_SIZE_BATTERYDATA);
  if(ECODE_NVM3_OK != errCode)
  {
    StoredBatteryData.lastReportedBatteryLevel = BATTERY_DATA_UNASSIGNED_VALUE;
    writeBatteryData(&BatteryData);
  }

  return StoredBatteryData;
}

/**
 * @brief Writes battery data to file system.
 */
void writeBatteryData(const SBatteryData* pBatteryData)
{
  Ecode_t errCode = nvm3_writeData(pFileSystemApplication, ZAF_FILE_ID_BATTERYDATA, pBatteryData, ZAF_FILE_SIZE_BATTERYDATA);
  ASSERT(ECODE_NVM3_OK == errCode); //Assert has been kept for debugging , can be removed from production code. This error can only be caused by some internal flash HW failure
}

uint16_t handleFirmWareIdGet(uint8_t n)
{
  if (n == 0)
  {
    // Firmware 0
    return APP_FIRMWARE_ID;
  }
  // Invalid Firmware number
  return 0;
}

uint8_t CC_Version_GetHardwareVersion_handler(void)
{
  return 1;
}

void CC_ManufacturerSpecific_ManufacturerSpecificGet_handler(uint16_t * pManufacturerID,
                                                             uint16_t * pProductID)
{
  *pManufacturerID = APP_MANUFACTURER_ID;
  *pProductID = APP_PRODUCT_ID;
}

/*
 * This function will report a serial number in a binary format according to the specification.
 * The serial number is the chip serial number and can be verified using the Simplicity Commander.
 * The choice of reporting can be changed in accordance with the Manufacturer Specific
 * Command Class specification.
 */
void CC_ManufacturerSpecific_DeviceSpecificGet_handler(device_id_type_t * pDeviceIDType,
                                                       device_id_format_t * pDeviceIDDataFormat,
                                                       uint8_t * pDeviceIDDataLength,
                                                       uint8_t * pDeviceIDData)
{
  *pDeviceIDType = DEVICE_ID_TYPE_SERIAL_NUMBER;
  *pDeviceIDDataFormat = DEVICE_ID_FORMAT_BINARY;
  *pDeviceIDDataLength = 8;
  uint64_t uuID = SYSTEM_GetUnique();
  DPRINTF("\r\nuuID: %x", (uint32_t)uuID);
  *(pDeviceIDData + 0) = (uint8_t)(uuID >> 56);
  *(pDeviceIDData + 1) = (uint8_t)(uuID >> 48);
  *(pDeviceIDData + 2) = (uint8_t)(uuID >> 40);
  *(pDeviceIDData + 3) = (uint8_t)(uuID >> 32);
  *(pDeviceIDData + 4) = (uint8_t)(uuID >> 24);
  *(pDeviceIDData + 5) = (uint8_t)(uuID >> 16);
  *(pDeviceIDData + 6) = (uint8_t)(uuID >>  8);
  *(pDeviceIDData + 7) = (uint8_t)(uuID >>  0);
}
