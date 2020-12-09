/**************************************************************************//**
 * @file WallController_Gesture.c
 * @brief Z-Wave Wall Controller application with gesture sensor
 * @version 1.0.0
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

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
/* During development of your device, you may add features using command    */
/* classes which are not already included. Remember to include relevant     */
/* classes and utilities and add them in your make file.                    */
/****************************************************************************/
#include "config_rf.h"
#include <stdbool.h>
#include <stdint.h>
#include "SizeOf.h"
#include "Assert.h"
#include "DebugPrintConfig.h"
//#define DEBUGPRINT
#include "DebugPrint.h"
#include "config_app.h"
#include "ZAF_app_version.h"
#include <ZAF_file_ids.h>
#include "nvm3.h"
#include "ZAF_nvm3_app.h"
#include <em_system.h>
#include <ZW_radio_api.h>
#include <ZW_slave_api.h>
#include <ZW_classcmd.h>
#include <ZW_TransportLayer.h>
#include <ZAF_uart_utils.h>
#include <board.h>
#include <ev_man.h>
#include <AppTimer.h>
#include <SwTimer.h>
#include <EventDistributor.h>
#include <ZW_system_startup_api.h>
#include <ZW_application_transport_interface.h>
#include <association_plus.h>
#include <agi.h>
#include <CC_Association.h>
#include <CC_AssociationGroupInfo.h>
#include <CC_Basic.h>
#include <CC_CentralScene.h>
#include <CC_DeviceResetLocally.h>
#include <CC_Indicator.h>
#include <CC_ManufacturerSpecific.h>
#include <CC_MultiChanAssociation.h>
#include <CC_MultilevelSwitch_Control.h>
#include <CC_PowerLevel.h>
#include <CC_Security.h>
#include <CC_Supervision.h>
#include <CC_Version.h>
#include <CC_ZWavePlusInfo.h>
#include <CC_FirmwareUpdate.h>
#include <ZW_TransportMulticast.h>
#include "zaf_event_helper.h"
#include "zaf_job_helper.h"
#include <ZAF_Common_helper.h>
#include <ZAF_network_learn.h>
#include <ZAF_TSE.h>
#include <ota_util.h>
#include <ZAF_CmdPublisher.h>
#include <em_wdog.h>
#include "events.h"

#include "CC_ColorSwitch_Control.h"
#include "si115x.h"
#include "si115x_gesture.h"
#include "i2cspm.h"

/****************************************************************************/
/* Application specific button and LED definitions                          */
/****************************************************************************/

#define KEY01_BTN   APP_BUTTON_A
#define KEY02_BTN   APP_BUTTON_B
#define KEY03_BTN   APP_BUTTON_C

STATIC_ASSERT((APP_BUTTON_LEARN_RESET != KEY01_BTN) &&
              (APP_BUTTON_LEARN_RESET != KEY02_BTN) &&
              (APP_BUTTON_LEARN_RESET != KEY03_BTN) &&
              (KEY01_BTN != KEY02_BTN) &&
              (KEY01_BTN != KEY03_BTN) &&
              (KEY02_BTN != KEY03_BTN),
              STATIC_ASSERT_FAILED_button_overlap);

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

typedef enum {
  KEY01,
  KEY02,
  KEY03,
  KEY04,
  NUMBER_OF_KEYS
} key_id_t;

typedef enum {
  RED,
  GREEN,
  BLUE,
  WHITE,
  NUMBER_OF_COLORS
} colors_t;

typedef enum
{
  KEY_EVENT_SHORT_PRESS,
  KEY_EVENT_HOLD,
  KEY_EVENT_UP
} key_event_t;

typedef enum
{
  GESTURE_EVENT_NONE,
  GESTURE_EVENT_UP,
  GESTURE_EVENT_DOWN,
  GESTURE_EVENT_LEFT,
  GESTURE_EVENT_RIGHT
} gesture_event_t;

#define EVENT_APP_CC_NO_JOB 0xFF

/**
 * Application states. Function AppStateManager(..) includes the state
 * event machine.
 */
typedef enum
{
  STATE_APP_IDLE,
  STATE_APP_STARTUP,
  STATE_APP_AWAIT_KEYPRESS,
  STATE_APP_LEARN_MODE,
  STATE_APP_GET_NEXT_NODE,
  STATE_APP_INITIATE_TRANSMISSION,
  STATE_APP_AWAIT_TRANSMIT_CALLBACK,
  STATE_APP_RESET,
  STATE_APP_TRANSMIT_DATA
}
STATE_APP;

/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

/**
 * Please see the description of app_node_information_t.
 */
static uint8_t cmdClassListNonSecureNotIncluded[] =
{
  COMMAND_CLASS_ZWAVEPLUS_INFO,
  COMMAND_CLASS_ASSOCIATION_V2,
  COMMAND_CLASS_ASSOCIATION_GRP_INFO,
  COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2,
  COMMAND_CLASS_TRANSPORT_SERVICE_V2,
  COMMAND_CLASS_VERSION,
  COMMAND_CLASS_MANUFACTURER_SPECIFIC,
  COMMAND_CLASS_DEVICE_RESET_LOCALLY,
  COMMAND_CLASS_INDICATOR,
  COMMAND_CLASS_POWERLEVEL,
  COMMAND_CLASS_SECURITY,
  COMMAND_CLASS_SECURITY_2,
  COMMAND_CLASS_SUPERVISION,
  COMMAND_CLASS_CENTRAL_SCENE_V2,
  COMMAND_CLASS_FIRMWARE_UPDATE_MD_V5
};

/**
 * Please see the description of app_node_information_t.
 */
static uint8_t cmdClassListNonSecureIncludedSecure[] =
{
  COMMAND_CLASS_ZWAVEPLUS_INFO,
  COMMAND_CLASS_TRANSPORT_SERVICE_V2,
  COMMAND_CLASS_SECURITY,
  COMMAND_CLASS_SECURITY_2,
  COMMAND_CLASS_SUPERVISION
};

/**
 * Please see the description of app_node_information_t.
 */
static uint8_t cmdClassListSecure[] =
{
  COMMAND_CLASS_VERSION,
  COMMAND_CLASS_ASSOCIATION_V2,
  COMMAND_CLASS_ASSOCIATION_GRP_INFO,
  COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2,
  COMMAND_CLASS_MANUFACTURER_SPECIFIC,
  COMMAND_CLASS_DEVICE_RESET_LOCALLY,
  COMMAND_CLASS_INDICATOR,
  COMMAND_CLASS_POWERLEVEL,
  COMMAND_CLASS_CENTRAL_SCENE_V2,
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
//@ [wall_controller_agi_root_device_lifeline]
CMD_CLASS_GRP agiTableLifeLine[] = {AGITABLE_LIFELINE_GROUP};
//@ [wall_controller_agi_root_device_lifeline]

/**
 * @var agiTableRootDeviceGroups
 * AGI table defining the groups for root device.
 */
//@ [wall_controller_agi_other_groups]
AGI_GROUP agiTableRootDeviceGroups[] = {AGITABLE_ROOTDEVICE_GROUPS};
//@ [wall_controller_agi_other_groups]

/*
 * Validate that MAX_ASSOCIATION_GROUPS (defined in config_app.h) match the
 * number of elements in AGITABLE_ROOTDEVICE_GROUPS (also defined in
 * config_app.h) plus one (for Lifeline)
 */
STATIC_ASSERT((sizeof_array(agiTableRootDeviceGroups) + 1) == MAX_ASSOCIATION_GROUPS, MAX_ASSOCIATION_GROUPS_value_is_invalid);

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
static STATE_APP currentState;

/**
 * Parameter is used to save wakeup reason after ApplicationInit(..)
 */
EResetReason_t g_eResetReason;

typedef struct _NEXT_JOB_Q_
{
  agi_profile_t profile;
} NEXT_JOB_Q;

/**
 * Holds the latest button action.
 * The action is used to distinguish between which action to perform:
 * - Press    => Switch on/off
 * - Hold     => Start dimming
 * - Release  => Stop dimming
 */
static key_event_t keyEventGlobal;
static uint8_t centralSceneNumberHold;
static uint8_t centralSceneKeyAttributeHold;

static ESwTimerStatus centralSceneHoldTimerStatus = ESWTIMER_STATUS_FAILED;
// Timer
static SSwTimer CentralSceneHoldTimer;

static CCMLS_PRIMARY_SWITCH_T multiLevelDirection[NUMBER_OF_KEYS] = {CCMLS_PRIMARY_SWITCH_DOWN, CCMLS_PRIMARY_SWITCH_DOWN, CCMLS_PRIMARY_SWITCH_DOWN};
static uint8_t buttonStates[NUMBER_OF_KEYS] = {0, 0, 0, 0};
static key_id_t m_button;
static colors_t colorStates[NUMBER_OF_COLORS] = {RED, GREEN, BLUE, WHITE};
static colors_t m_color;
NEXT_JOB_Q nextJob;

static TaskHandle_t g_AppTaskHandle;

#ifdef DEBUGPRINT
static uint8_t m_aDebugPrintBuffer[96];
#endif

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

// Used by the application data file.
typedef struct SApplicationData
{
  uint8_t slowRefresh;
} SApplicationData;

#define FILE_SIZE_APPLICATIONDATA     (sizeof(SApplicationData))

static SApplicationData ApplicationData;

#define APP_EVENT_QUEUE_SIZE 5

/**
 * The following four variables are used for the application event queue.
 */
static SQueueNotifying m_AppEventNotifyingQueue;
static StaticQueue_t m_AppEventQueueObject;
static EVENT_APP eventQueueStorage[APP_EVENT_QUEUE_SIZE];
static QueueHandle_t m_AppEventQueue;

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

static nvm3_Handle_t* pFileSystemApplication;

static EVENT_APP pendingKeyPressEvent = EVENT_APP_CC_NO_JOB;

/* Initializes sensor for proximity sensing. */
SI115X_TypeDef si115x = SI115X_DEFAULT;
uint32_t timestamp= 0;
si115x_gesture_sample_t gestureSample;
// ACTION: SSwTimer instance
static SSwTimer proximityTimer;
static gesture_event_t gestureEventGlobal;

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

// No exported data.

/****************************************************************************/
/* PRIVATE FUNCTION PROTOTYPES                                              */
/****************************************************************************/

void DeviceResetLocallyDone(TRANSMISSION_RESULT * pTransmissionResult);
void DeviceResetLocally(void);
STATE_APP GetAppState(void);
void AppStateManager(EVENT_APP event);
static void ChangeState(STATE_APP newState);

static void ApplicationTask(SApplicationHandles* pAppHandles);
static void PrepareAGITransmission(uint8_t profile, key_id_t nextActiveButton);
static JOB_STATUS InitiateTransmission(void);
void ZCB_TransmitCallback(TRANSMISSION_RESULT * pTransmissionResult);
static void InitiateCentralSceneTX(uint8_t keyAttribute, uint8_t sceneNumber);
static void ZCB_CentralSceneHoldTimerCallback(SSwTimer *pTimer);
bool LoadConfiguration(void);
void SetDefaultConfiguration(void);

SApplicationData readAppData(void);
void writeAppData(const SApplicationData* pAppData);

void AppResetNvm(void);

I2C_TypeDef* Init_I2C(void);
SI115X_Ecode_TypeDef Init_Sensor(SI115X_TypeDef *si115x, uint8_t *data);
void gestureDetectionCallback(SSwTimer *pTimer);

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
    DPRINT("Incoming Rx msg\r\n");

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
    DPRINT("Incoming Status msg\r\n");

    switch (Status.eStatusType)
    {
      case EZWAVECOMMANDSTATUS_TX:
      {
        SZWaveTransmitStatus* pTxStatus = &Status.Content.TxStatus;
        if (!pTxStatus->bIsTxFrameLegal)
        {
          DPRINT("Auch - not sure what to do\r\n");
        }
        else
        {
          DPRINT("Tx Status received\r\n");
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
        DPRINT("Generate Random status\r\n");
        break;
      }

      case EZWAVECOMMANDSTATUS_LEARN_MODE_STATUS:
      {
        DPRINTF("Learn status %d\r\n", Status.Content.LearnModeStatus.Status);
        if (ELEARNSTATUS_ASSIGN_COMPLETE == Status.Content.LearnModeStatus.Status)
        {
          // When security S0 or higher is set, remove all settings which happen before secure inclusion
          // calling function SetDefaultConfiguration(). The same function is used when there is an
          // EINCLUSIONSTATE_EXCLUDED.
          if ( (EINCLUSIONSTATE_EXCLUDED == ZAF_GetInclusionState()) ||
                      (SECURITY_KEY_NONE != GetHighestSecureLevel(ZAF_GetSecurityKeys())) )
          {
            SetDefaultConfiguration();
          }
          ZAF_EventHelperEventEnqueue(EVENT_SYSTEM_LEARNMODE_FINISHED);
          ZAF_Transport_OnLearnCompleted();
        }
        else if(ELEARNSTATUS_SMART_START_IN_PROGRESS == Status.Content.LearnModeStatus.Status)
        {
          ZAF_EventHelperEventEnqueue(EVENT_APP_SMARTSTART_IN_PROGRESS);
        }
        else if(ELEARNSTATUS_LEARN_IN_PROGRESS == Status.Content.LearnModeStatus.Status)
        {
          ZAF_EventHelperEventEnqueue(EVENT_APP_LEARN_IN_PROGRESS);
        }
        else if(ELEARNSTATUS_LEARN_MODE_COMPLETED_TIMEOUT == Status.Content.LearnModeStatus.Status)
        {
          ZAF_EventHelperEventEnqueue((EVENT_APP) EVENT_SYSTEM_LEARNMODE_FINISHED);
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
        DPRINTF("Protocol Ready\r\n");
        ZAF_EventHelperEventEnqueue(EVENT_APP_FLUSHMEM_READY);

        break;
      }

      case EZWAVECOMMANDSTATUS_INVALID_TX_REQUEST:
      {
        DPRINTF("ERROR: Invalid TX Request to protocol - %d", Status.Content.InvalidTxRequestStatus.InvalidTxRequest);
        break;
      }

      case EZWAVECOMMANDSTATUS_INVALID_COMMAND:
      {
        DPRINTF("ERROR: Invalid command to protocol - %d", Status.Content.InvalidCommandStatus.InvalidCommand);
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
    //DPRINTF("Event: %d\r\n", event);
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

  g_eResetReason = eResetReason;

  /* hardware initialization */
  Board_Init();

  /* Resolve pin conflicts */
  for (uint32_t led = 0; led < BOARD_LED_COUNT; led++)
  {
    Board_ConfigLed(led, false);
  }
  Board_ConfigLed(BOARD_LED1, true); // Configure LED0 (LED0 on WSTK)
  Board_ConfigLed(BOARD_LED3, true); // Configure LED2 (LED1 on WSTK)

  BRD420xBoardInit(RadioConfig.eRegion);

#ifdef DEBUGPRINT
  ZAF_UART0_enable(115200, true, false);
  DebugPrintConfig(m_aDebugPrintBuffer, sizeof(m_aDebugPrintBuffer), ZAF_UART0_tx_send);
#endif

  /* Init state machine*/
  currentState = STATE_APP_STARTUP;

  DPRINT("\n\n----------------------------------\n");
  DPRINT("Z-Wave Sample App: Wall Controller\n");
  DPRINTF("SDK: %d.%d.%d ZAF: %d.%d.%d.%d [Freq: %d]\n",
          SDK_VERSION_MAJOR,
          SDK_VERSION_MINOR,
          SDK_VERSION_PATCH,
          ZAF_GetAppVersionMajor(),
          ZAF_GetAppVersionMinor(),
          ZAF_GetAppVersionPatchLevel(),
          ZAF_BUILD_NO,
          RadioConfig.eRegion);
  DPRINT("----------------------------------\n");

  DPRINTF("%s: KEY01 press/hold/release\n", Board_GetButtonLabel(KEY01_BTN));  // S1
  DPRINTF("%s: KEY02 press/hold/release\n", Board_GetButtonLabel(KEY02_BTN));  // S2
  DPRINTF("%s: KEY03 press/hold/release\n", Board_GetButtonLabel(KEY03_BTN)); // S3
  DPRINTF("%s: Toggle learn mode\n", Board_GetButtonLabel(APP_BUTTON_LEARN_RESET)); // S4
  DPRINT("      Hold 5 sec: Reset\n");
  //DPRINTF("%s: Learn mode + identify\n", Board_GetLedLabel(APP_LED_INDICATOR));  // D1
  DPRINTF("%s: Learn mode + identify\n", Board_GetLedLabel(APP_LED_C));
  DPRINT("----------------------------------\n\n");

  DPRINTF("ApplicationInit eResetReason = %d\n", g_eResetReason);

  CC_ZWavePlusInfo_Init(&CCZWavePlusInfo);

  CC_Version_SetApplicationVersionInfo(ZAF_GetAppVersionMajor(),
                                       ZAF_GetAppVersionMinor(),
                                       ZAF_GetAppVersionPatchLevel(),
                                       ZAF_BUILD_NO);

  // ACTION: Initialize I2C sensor
  uint8_t si115x_status;

  /* Initialize MCU peripherals */
  I2C_TypeDef* port = Init_I2C();
  si115x.i2cPort = port;

  SI115X_Ecode_TypeDef retval;
  retval = Init_Sensor(&si115x, &si115x_status);
  DPRINTF("\r\nProximity sensor found: 0x%x\r\n", si115x_status);

  // Initialize Si115x if I2C is working and the correct partID (0x53) is detected
  if((retval == SI115X_ECODE_OK) && (si115x_status == 0x53))
  {
    si115x_init_gesture(&si115x);
  }

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
  AppTimerSetReceiverTask(g_AppTaskHandle);
  AppTimerRegister(&CentralSceneHoldTimer, true, ZCB_CentralSceneHoldTimerCallback);

  // ACTION: Register si1153 sensor timer
  AppTimerRegister(&proximityTimer, true, gestureDetectionCallback);

  // ACTION: Start timer (100Hz)
  TimerStart(&proximityTimer, 10);

  ZAF_Init(g_AppTaskHandle, pAppHandles, &ProtocolConfig, NULL);

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

  Board_EnableButton(APP_BUTTON_LEARN_RESET);
  Board_EnableButton(KEY01_BTN);
  Board_EnableButton(KEY02_BTN);
  Board_EnableButton(KEY03_BTN);

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

  // Wait for and process events
  DPRINT("WallController Event processor Started\r\n");
  uint32_t iMaxTaskSleep = 10;
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
  /* Call command class handlers */
  switch (pCmd->ZW_Common.cmdClass)
  {
    case COMMAND_CLASS_VERSION:
      frame_status = handleCommandClassVersion(rxOpt, pCmd, cmdLength);
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
      frame_status = handleCommandClassPowerLevel(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_MANUFACTURER_SPECIFIC:
      frame_status = handleCommandClassManufacturerSpecific(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_ZWAVEPLUS_INFO:
      frame_status = handleCommandClassZWavePlusInfo(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2:
      frame_status = handleCommandClassMultiChannelAssociation(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_CENTRAL_SCENE_V2:
      frame_status = handleCommandClassCentralScene(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_SUPERVISION:
      frame_status = handleCommandClassSupervision(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_SECURITY:
    case COMMAND_CLASS_SECURITY_2:
      frame_status = handleCommandClassSecurity(rxOpt, pCmd, cmdLength);
      break;

	case COMMAND_CLASS_FIRMWARE_UPDATE_MD_V5:
      frame_status = handleCommandClassFWUpdate(rxOpt, pCmd, cmdLength);
    break;
  }
  return frame_status;
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
  //DPRINTF("AppStateManager St: %d, Ev: %d\r\n", currentState, event);

  if ((BTN_EVENT_LONG_PRESS(APP_BUTTON_LEARN_RESET) == (BUTTON_EVENT)event) ||
      (EVENT_SYSTEM_RESET == (EVENT_SYSTEM)event))
  {
    /*Force state change to activate system-reset without taking care of current state.*/
    ChangeState(STATE_APP_RESET);
    /* Send reset notification*/
    DeviceResetLocally();
  }

  switch(currentState)
  {
    case STATE_APP_STARTUP:

      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
      }

      if(EVENT_APP_INIT == event)
      {
        /* Load the application settings from NVM3 file system */
        LoadConfiguration();

        /*
         * Initialize AGI.
         */
        //@ [WALL_CONTROLLER_AGI_INIT]
        AGI_Init();

        CC_AGI_LifeLineGroupSetup(agiTableLifeLine, sizeof_array(agiTableLifeLine), ENDPOINT_ROOT);
        AGI_ResourceGroupSetup(agiTableRootDeviceGroups, sizeof_array(agiTableRootDeviceGroups), ENDPOINT_ROOT);
        //@ [WALL_CONTROLLER_AGI_INIT]

        /*
         * Initialize Event Scheduler.
         */
        Transport_OnApplicationInitSW( &m_AppNIF, NULL);

        /* Enter SmartStart*/
        /* Protocol will commence SmartStart only if the node is NOT already included in the network */
        ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_INCLUSION_SMARTSTART, g_eResetReason);

        /* Init state machine*/
        ZAF_EventHelperEventEnqueue(EVENT_EMPTY);

        break;
      }
      CC_FirmwareUpdate_Init(NULL, NULL, NULL, true);
      ChangeState(STATE_APP_IDLE);
      break;

    case STATE_APP_IDLE:
      if(EVENT_APP_REFRESH_MMI == event)
      {
        Board_IndicateStatus(BOARD_STATUS_IDLE);
        break;
      }

      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
        LoadConfiguration();
      }

      if(EVENT_APP_SMARTSTART_IN_PROGRESS == event)
      {
        ChangeState(STATE_APP_LEARN_MODE);
      }

      /**************************************************************************************
       * Learn/Reset button
       **************************************************************************************
       */
      if ((BTN_EVENT_SHORT_PRESS(APP_BUTTON_LEARN_RESET) == (BUTTON_EVENT)event) ||
          (EVENT_SYSTEM_LEARNMODE_START == (EVENT_SYSTEM)event))
      {
        if (EINCLUSIONSTATE_EXCLUDED != g_pAppHandles->pNetworkInfo->eInclusionState)
        {
          DPRINT("LEARN_MODE_EXCLUSION");
          ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_EXCLUSION_NWE, g_eResetReason);
        }
        else{
          DPRINT("LEARN_MODE_INCLUSION");
          ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_INCLUSION, g_eResetReason);
        }
        ChangeState(STATE_APP_LEARN_MODE);
        break;
      }

      /**************************************************************************************
       * KEY 1
       **************************************************************************************
       */

      if (EVENT_APP_GESTURE_DETECTED == event)
      {
        centralSceneNumberHold = 1;
        centralSceneKeyAttributeHold = CENTRAL_SCENE_NOTIFICATION_KEY_ATTRIBUTES_KEY_PRESSED_1_TIME_V2;

        ZAF_JobHelperJobEnqueue(EVENT_APP_CENTRAL_SCENE_JOB);

        PrepareAGITransmission(ASSOCIATION_GROUP_INFO_REPORT_PROFILE_CONTROL_KEY01, KEY01);
        ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        ChangeState(STATE_APP_TRANSMIT_DATA);
      }

      if (BTN_EVENT_HOLD(KEY01_BTN) == (BUTTON_EVENT)event)
      {
        DPRINT("\r\nK1HOLD\r\n");
        keyEventGlobal = KEY_EVENT_HOLD;
        centralSceneNumberHold = 1;
        centralSceneKeyAttributeHold = CENTRAL_SCENE_NOTIFICATION_KEY_ATTRIBUTES_KEY_HELD_DOWN_V2;

        ZAF_JobHelperJobEnqueue(EVENT_APP_CENTRAL_SCENE_JOB);

        PrepareAGITransmission(ASSOCIATION_GROUP_INFO_REPORT_PROFILE_CONTROL_KEY01, KEY01);
        ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        ChangeState(STATE_APP_TRANSMIT_DATA);
      }

      if ( ((BTN_EVENT_UP(KEY01_BTN) == (BUTTON_EVENT)event) || (BTN_EVENT_LONG_PRESS(KEY01_BTN) == (BUTTON_EVENT)event)) &&
           (keyEventGlobal == KEY_EVENT_HOLD) )
      {
        DPRINT("\r\nK1UP\r\n");
        keyEventGlobal = KEY_EVENT_UP;
        centralSceneNumberHold = 1;
        centralSceneKeyAttributeHold = CENTRAL_SCENE_NOTIFICATION_KEY_ATTRIBUTES_KEY_RELEASED_V2;

        ZAF_JobHelperJobEnqueue(EVENT_APP_CENTRAL_SCENE_JOB);

        PrepareAGITransmission(ASSOCIATION_GROUP_INFO_REPORT_PROFILE_CONTROL_KEY01, KEY01);
        ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        ChangeState(STATE_APP_TRANSMIT_DATA);
      }

      // ACTION: Perform gesture measurements on timer event
      if (EVENT_APP_GESTURE == event)
      {
         SI115X_Force(&si115x);
         // Sensor data ready
         si115x_gesture_handler(&si115x, &gestureSample);
         gestureSample.timestamp =  timestamp;
         //DPRINTF("\r\nPS1: %d, PS2: %d, PS3: %d\r\n", gestureSample.ps1, gestureSample.ps2, gestureSample.ps3);
         // Run gesture algorithm
         gesture_t gesture;
         char *directionStrings[] = {"NONE","UP","DOWN","LEFT","RIGHT","PROX"};
         gesture = si115x_gesture_algorithm(&gestureSample);
         if ((gesture == RIGHT) || (gesture == LEFT) ||(gesture == UP) ||(gesture == DOWN))
         {
           gestureEventGlobal = gesture;
           //DPRINTF("\r\nGesture: %s\r\n", directionStrings[gesture]);
           ZAF_EventHelperEventEnqueue(EVENT_APP_GESTURE_DETECTED);
         }
      }
      break;
    case STATE_APP_LEARN_MODE:
      if(EVENT_APP_REFRESH_MMI == event)
      {
        Board_IndicateStatus(BOARD_STATUS_LEARNMODE_ACTIVE);
      }

      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
        LoadConfiguration();
      }

      if ((BTN_EVENT_SHORT_PRESS(APP_BUTTON_LEARN_RESET) == (BUTTON_EVENT)event) ||
          (EVENT_SYSTEM_LEARNMODE_STOP == (EVENT_SYSTEM)event))
      {
        DPRINT("\r\nLEARN MODE DISABLE\r\n");
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

      if(EVENT_SYSTEM_LEARNMODE_FINISHED == (EVENT_SYSTEM)event)
      {
        //Go back to smart start if the node was excluded.
        //Protocol will not commence SmartStart if the node is already included in the network.
        ZAF_setNetworkLearnMode(E_NETWORK_LEARN_MODE_INCLUSION_SMARTSTART, g_eResetReason);

        DPRINT("\r\nLEARN MODE FINISH\r\n");
        ChangeState(STATE_APP_IDLE);

        /* If we are in a network and the Identify LED state was changed to idle due to learn mode, report it to lifeline */
        CC_Indicator_RefreshIndicatorProperties();
        ZAF_TSE_Trigger((void *)CC_Indicator_report_stx, &ZAF_TSE_localActuationIdentifyData, true);
      }
      break;

    case STATE_APP_RESET:
      if(EVENT_APP_REFRESH_MMI == event){}

      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
        /* Soft reset */
        Board_ResetHandler();
      }

      break;

    case STATE_APP_TRANSMIT_DATA:
      if(EVENT_APP_REFRESH_MMI == event)
      {
        // Nothing here.
      }

      if(EVENT_APP_FLUSHMEM_READY == event)
      {
        AppResetNvm();
        LoadConfiguration();
      }

      if (EVENT_APP_NEXT_EVENT_JOB == event)
      {
        uint8_t event;
        if (true == ZAF_JobHelperJobDequeue(&event))
        {
          /*
           * If we were able to dequeue an event from the job queue, let's process it right away
           * by adding it to the event queue.
           */
          ZAF_EventHelperEventEnqueue(event);
        }
        else
        {
          // If there are no more events, we'll finish the job handling.
          ZAF_EventHelperEventEnqueue(EVENT_APP_FINISH_EVENT_JOB);
        }
      }

      if (EVENT_APP_CENTRAL_SCENE_JOB == event)
      {
        if (KEY_EVENT_HOLD == keyEventGlobal)
        {
          if (ESWTIMER_STATUS_FAILED == centralSceneHoldTimerStatus)
          {
            // Start the timer only if it's not started already.
            centralSceneHoldTimerStatus = TimerStart(&CentralSceneHoldTimer, ( 0 == ApplicationData.slowRefresh ) ? 200 : 55000);
          }
        }
        else if (KEY_EVENT_UP == keyEventGlobal)
        {
          if (ESWTIMER_STATUS_FAILED != centralSceneHoldTimerStatus)
          {
            TimerStop(&CentralSceneHoldTimer);
            centralSceneHoldTimerStatus = ESWTIMER_STATUS_FAILED;
          }
        }

        InitiateCentralSceneTX(
          centralSceneKeyAttributeHold,
          centralSceneNumberHold);
      }

      if (EVENT_APP_CC_BASIC_JOB == event)
      {
        DPRINT("\r\nEVENT_APP_CC_BASIC_JOB");
        if(JOB_STATUS_SUCCESS != CC_Basic_Set_tx( &nextJob.profile, ENDPOINT_ROOT, buttonStates[m_button],ZCB_TransmitCallback))
        {
          DPRINT("\r\nEVENT_APP_CC_BASIC_JOB failed!\r\n");
          ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        }
      }

      if (EVENT_APP_CC_COLOR_SWITCH_JOB == event)
      {
        DPRINT("\r\nEVENT_APP_CC_COLOR_SWITCH_JOB");
        uint8_t colorValues[3] = {0, 0, 0};
        if(m_color == RED)
        {
          colorValues[0] = 80; // Set color value to a reasonable level
        } else if (m_color == GREEN) {
          colorValues[1] = 80;
        } else if (m_color == BLUE) {
          colorValues[2] = 80;
        } else if (m_color == WHITE) {
          colorValues[0] = 80;
          colorValues[1] = 80;
          colorValues[2] = 80;
        }
        if(JOB_STATUS_SUCCESS != CmdClassColorSwitchSetTransmit(&nextJob.profile, ENDPOINT_ROOT, ZCB_TransmitCallback, 3, colorValues[0], colorValues[1], colorValues[2], 0))
        {
          DPRINT("\r\nEVENT_APP_CC_COLOR_SWITCH_JOB failed!\r\n");
          ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
        }
      }

      if(EVENT_APP_CC_SWITCH_MULTILEVEL_JOB == event)
      {
        JOB_STATUS jobStatus = InitiateTransmission();
        DPRINT("\r\nEVENT_APP_CC_SWITCH_MULTILEVEL_JOB");
        if (JOB_STATUS_SUCCESS != jobStatus)
        {
          TRANSMISSION_RESULT transmissionResult = {0, 0, TRANSMISSION_RESULT_FINISHED};
          DPRINT("\r\nCC_SWITCH_MULTILEVEL ERROR");
          ZCB_TransmitCallback(&transmissionResult);
        }
      }

      /*
       * Catch important button press/release events and save it for when the transmit is done.
       * Important events are "Key Up" and "Key Long Press", which both are button release events
       * and signify the end to an action that was initiated earlier and therefore must be matched.
       */
      if ( ((BTN_EVENT_UP(KEY01_BTN) == (BUTTON_EVENT)event) || (BTN_EVENT_LONG_PRESS(KEY01_BTN) == (BUTTON_EVENT)event)) ||
           ((BTN_EVENT_UP(KEY02_BTN) == (BUTTON_EVENT)event) || (BTN_EVENT_LONG_PRESS(KEY02_BTN) == (BUTTON_EVENT)event)) ||
           ((BTN_EVENT_UP(KEY03_BTN) == (BUTTON_EVENT)event) || (BTN_EVENT_LONG_PRESS(KEY03_BTN) == (BUTTON_EVENT)event)) )
      {
        DPRINTF("\r\nReceived key press event (%d) in state STATE_APP_TRANSMIT_DATA", event);
        pendingKeyPressEvent = event;
      }

      if (EVENT_APP_FINISH_EVENT_JOB == event)
      {
        DPRINT("\r\nTransmitDone.");
        if (EVENT_APP_CC_NO_JOB != pendingKeyPressEvent)
        {
          // Button press/release event received earlier during transmit. Re-post it to the event queue now.
          DPRINTF("\r\nRe-posting pending key event (%d) in state STATE_APP_TRANSMIT_DATA)", pendingKeyPressEvent);
          ZAF_EventHelperEventEnqueue(pendingKeyPressEvent);
          pendingKeyPressEvent = EVENT_APP_CC_NO_JOB;
        }
        ChangeState(STATE_APP_IDLE);
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
  DPRINTF("\r\nState changed: %d -> %d\r\n", currentState, newState);

  currentState = newState;

  /**< Pre-action on new state is to refresh MMI */
  ZAF_EventHelperEventEnqueue(EVENT_APP_REFRESH_MMI);
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
 * @brief Send reset notification.
 */
void
DeviceResetLocally(void)
{
  agi_profile_t lifelineProfile = {
      ASSOCIATION_GROUP_INFO_REPORT_PROFILE_GENERAL,
      ASSOCIATION_GROUP_INFO_REPORT_PROFILE_GENERAL_LIFELINE
  };
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
  if(bFirmwareNumber == 0)
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


/**
 * @brief Function resets configuration to default values.
 *
 * Add application specific functions here to initialize configuration values stored in persistent memory.
 * Will be called at any of the following events:
 *  - Network Exclusion
 *  - Network Secure Inclusion (after S2 bootstrapping complete)
 *  - Device Reset Locally
 */
void
SetDefaultConfiguration(void)
{
  AssociationInit(true, pFileSystemApplication);

  ApplicationData.slowRefresh = 1;

  writeAppData(&ApplicationData);

  loadInitStatusPowerLevel();

  uint32_t appVersion = ZAF_GetAppVersion();
  nvm3_writeData(pFileSystemApplication, ZAF_FILE_ID_APP_VERSION, &appVersion, ZAF_FILE_SIZE_APP_VERSION);
}

/**
 * @brief This function loads the application settings from file system.
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

    ApplicationData = readAppData();

    /* Initialize association module */
    AssociationInit(false, pFileSystemApplication);

    return true;
  }
  else
  {
    DPRINT("Application FileSystem Verify failed\r\n");
    loadInitStatusPowerLevel();

    // Reset the file system if ZAF_FILE_ID_APP_VERSION is missing since this indicates
    // corrupt or missing file system.
    AppResetNvm();

    return false;
  }

}

void AppResetNvm(void)
{
  DPRINT("Resetting application FileSystem to default\r\n");

  ASSERT(0 != pFileSystemApplication); //Assert has been kept for debugging , can be removed from production code. This error can only be caused by some internal flash HW failure

  Ecode_t errCode = nvm3_eraseAll(pFileSystemApplication);
  ASSERT(ECODE_NVM3_OK == errCode);  //Assert has been kept for debugging , can be removed from production code. This error can only be caused by some internal flash HW failure

  /* Apparently there is no valid configuration in file system, so load */
  /* default values and save them to file system. */
  SetDefaultConfiguration();
}

/**
 * @brief Prepares the transmission of commands stored in the AGI table.
 *
 * @param profile The profile key.
 * @param srcEndpoint The source endpoint.
 */
static void
PrepareAGITransmission(
        uint8_t profile,
        key_id_t nextActiveButton)
{
  //DPRINTF("\r\nPrepareAGITransmission %d", keyEventGlobal);

  nextJob.profile.profile_MS = ASSOCIATION_GROUP_INFO_REPORT_PROFILE_CONTROL;
  nextJob.profile.profile_LS = profile;

  m_button = nextActiveButton;
  if ((GESTURE_EVENT_RIGHT == gestureEventGlobal) ||
      (GESTURE_EVENT_LEFT == gestureEventGlobal) ||
      (GESTURE_EVENT_UP == gestureEventGlobal) ||
      (GESTURE_EVENT_DOWN == gestureEventGlobal))
  {
    ZAF_JobHelperJobEnqueue(EVENT_APP_CC_COLOR_SWITCH_JOB);
    m_color = colorStates[gestureEventGlobal];
    gestureEventGlobal = GESTURE_EVENT_NONE;
  }
  else if (KEY_EVENT_SHORT_PRESS == keyEventGlobal)
  {
    // Button presses are no longer used

    // ZAF_JobHelperJobEnqueue(EVENT_APP_CC_COLOR_SWITCH_JOB);
    // m_color = colorStates[m_button];
    if (0xFF == buttonStates[m_button])
    {
      /*
       * If button is on, turn device off.
       */
      buttonStates[m_button] = 0x00;
      DPRINT("\r\nBasic OFF");
    }
    else
    {
      /*
       * If button is off, turn device on.
       */
      buttonStates[m_button] = 0xFF;
      DPRINT("\r\nBasic ON");
    }
  }
  else if (KEY_EVENT_HOLD == keyEventGlobal)
  {
    DPRINT("\r\npre EVENT_APP_CC_SWITCH_MULTILEVEL_JOB");
    ZAF_JobHelperJobEnqueue(EVENT_APP_CC_SWITCH_MULTILEVEL_JOB);
    if (CCMLS_PRIMARY_SWITCH_UP == multiLevelDirection[m_button])
    {
      multiLevelDirection[m_button] = CCMLS_PRIMARY_SWITCH_DOWN;
    }
    else
    {
      multiLevelDirection[m_button] = CCMLS_PRIMARY_SWITCH_UP;
    }
  }
  else if (KEY_EVENT_UP == keyEventGlobal)
  {
    DPRINT("\r\npre BUTTON_UP EVENT_APP_CC_SWITCH_MULTILEVEL_JOB");
    ZAF_JobHelperJobEnqueue(EVENT_APP_CC_SWITCH_MULTILEVEL_JOB);
  }
}

/**
 * Initiates a Central Scene Notification to the lifeline.
 * We don't care about the result, since we have to proceed no matter what.
 * Therefore a callback function is called in any case.
 * @param keyAttribute The key attribute in action.
 * @param sceneNumber The scene in action.
 */
static void InitiateCentralSceneTX(uint8_t keyAttribute, uint8_t sceneNumber)
{
  agi_profile_t lifelineProfile = {
      ASSOCIATION_GROUP_INFO_REPORT_PROFILE_GENERAL_NA_V2,
      ASSOCIATION_GROUP_INFO_REPORT_PROFILE_GENERAL_LIFELINE
  };

  JOB_STATUS jobStatus = CommandClassCentralSceneNotificationTransmit(
          &lifelineProfile,
          ENDPOINT_ROOT,
          keyAttribute,
          sceneNumber,
          ZCB_TransmitCallback);

  if (JOB_STATUS_SUCCESS != jobStatus)
  {
    TRANSMISSION_RESULT transmissionResult;
    transmissionResult.nodeId = 0;
    transmissionResult.status = TRANSMIT_COMPLETE_FAIL;
    transmissionResult.isFinished = TRANSMISSION_RESULT_FINISHED;
    DPRINT("\r\nLL failure");
    ZCB_TransmitCallback(&transmissionResult);
  }
  else
  {
	DPRINT("\r\nLL success");
  }
}


/**
 * @brief Processes the transmission to related nodes.
 * @return Status of the job.
 */
static JOB_STATUS
InitiateTransmission(void)
{
  DPRINTF("\r\n### ITrans %d", keyEventGlobal);
  if (KEY_EVENT_HOLD == keyEventGlobal)
  {
    DPRINT("\r\n### Multilevel TX.Change");

    return CmdClassMultilevelSwitchStartLevelChange(
                &nextJob.profile,
                ENDPOINT_ROOT,
                ZCB_TransmitCallback,
                multiLevelDirection[m_button],
                CCMLS_IGNORE_START_LEVEL_TRUE,
                CCMLS_SECONDARY_SWITCH_NO_INC_DEC,
                0,
                2,
                0);
  }
  else if (KEY_EVENT_UP == keyEventGlobal)
  {
    DPRINT("\r\n### Multilevel Stop level change");
    return CmdClassMultilevelSwitchStopLevelChange(
                &nextJob.profile,
                ENDPOINT_ROOT,
                ZCB_TransmitCallback);
  }
  return JOB_STATUS_BUSY;
}

/**
 * @brief Callback function setting the application state.
 * @details Sets the application state when done transmitting.
 * @param pTransmissionResult Result of each transmission.
 */
void
ZCB_TransmitCallback(TRANSMISSION_RESULT * pTransmissionResult)
{
  DPRINTF("\r\nTX CB for N %d: %d", pTransmissionResult->nodeId, pTransmissionResult->status);

  if (TRANSMISSION_RESULT_FINISHED == pTransmissionResult->isFinished)
  {
    ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
  }
}

/**
 * @brief See description for function prototype in CC_CentralScene.h.
 */
uint8_t getAppCentralSceneReportData(ZW_CENTRAL_SCENE_SUPPORTED_REPORT_1BYTE_V3_FRAME * pData)
{
  pData->supportedScenes = 3; // Number of buttons
  pData->properties1 = (1 << 1) | 1; // 1 bit mask byte & All keys are identical.
  pData->variantgroup1.supportedKeyAttributesForScene1 = 0x07; // 0b00000111.
  return 1;
}

/**
 * @brief See description for function prototype in CC_CentralScene.h.
 */
void getAppCentralSceneConfiguration(central_scene_configuration_t * pConfiguration)
{
  pConfiguration->slowRefresh = readAppData().slowRefresh;
}

/**
 * @brief See description for function prototype in CC_CentralScene.h.
 */
e_cmd_handler_return_code_t
setAppCentralSceneConfiguration(central_scene_configuration_t * pConfiguration)
{
  ApplicationData.slowRefresh = pConfiguration->slowRefresh;
  writeAppData(&ApplicationData);

  // just return success
  return E_CMD_HANDLER_RETURN_CODE_HANDLED;
}

void
ZCB_CentralSceneHoldTimerCallback(SSwTimer *pTimer)
{
  ZAF_JobHelperJobEnqueue(EVENT_APP_CENTRAL_SCENE_JOB);
  if (STATE_APP_TRANSMIT_DATA != currentState)
  {
    ZAF_EventHelperEventEnqueue(EVENT_APP_NEXT_EVENT_JOB);
    ChangeState(STATE_APP_TRANSMIT_DATA);
  }
  UNUSED(pTimer);
}

/**
 * @brief Reads application data from file system.
 */
SApplicationData
readAppData(void)
{
  SApplicationData AppData;

  Ecode_t errCode = nvm3_readData(pFileSystemApplication, FILE_ID_APPLICATIONDATA, &AppData, sizeof(SApplicationData));
  ASSERT(ECODE_NVM3_OK == errCode);//Assert has been kept for debugging , can be removed from production code. This error hard to occur when a corresponing write is successfull
                                    //Can only happended in case of some hardware failure

  return AppData;
}

/**
 * @brief Writes application data to file system.
 */
void writeAppData(const SApplicationData* pAppData)
{
  Ecode_t errCode = nvm3_writeData(pFileSystemApplication, FILE_ID_APPLICATIONDATA, pAppData, sizeof(SApplicationData));
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

I2C_TypeDef* Init_I2C(void)
{
  /* Initialize I2C peripheral */
  I2CSPM_Init_TypeDef init = I2CSPM_INIT_DEFAULT;
  I2CSPM_Init(&init);

  return init.port;
}

SI115X_Ecode_TypeDef Init_Sensor(SI115X_TypeDef *si115x, uint8_t *data)
{
  SI115X_Ecode_TypeDef retval;
  retval = SI115X_ReadFromRegister(si115x, 0x00, data);
  return retval;
}

void gestureDetectionCallback(SSwTimer *pTimer)
{
  UNUSED(pTimer);
  timestamp++;
  ZAF_EventHelperEventEnqueue(EVENT_APP_GESTURE);
}
