/**
 * @file
 * @brief Configuration file for Wall Controller sample application.
 * @details This file contains definitions for the Z-Wave+ Framework as well for the sample app.
 *
 * @copyright 2018 Silicon Laboratories Inc.
 */
#ifndef _CONFIG_APP_H_
#define _CONFIG_APP_H_

#include <ZW_product_id_enum.h>
#include <CC_ManufacturerSpecific.h>

/****************************************************************************
 *
 * Application version, which is generated during release of SDK.
 * The application developer can freely alter the version numbers
 * according to his needs.
 *
 ****************************************************************************/
#define APP_VERSION ZAF_VERSION_MAJOR
#define APP_REVISION ZAF_VERSION_MINOR
#define APP_PATCH ZAF_VERSION_PATCH

/**
 * Defines generic and specific device types for a Central Scene device type
 */
//@ [GENERIC_TYPE_ID]
#define GENERIC_TYPE          GENERIC_TYPE_WALL_CONTROLLER
#define SPECIFIC_TYPE         SPECIFIC_TYPE_NOT_USED
//@ [GENERIC_TYPE_ID]

/**
 * See ZW_basis_api.h for ApplicationNodeInformation field deviceOptionMask
 */
//@ [DEVICE_OPTIONS_MASK_ID]
#define DEVICE_OPTIONS_MASK   APPLICATION_NODEINFO_LISTENING
//@ [DEVICE_OPTIONS_MASK_ID]


/**
 * Defines used to initialize the Z-Wave Plus Info Command Class.
 */
//@ [APP_TYPE_ID]
//@ [APP_ROLE_TYPE]
#define APP_ROLE_TYPE       ZWAVEPLUS_INFO_REPORT_ROLE_TYPE_SLAVE_ALWAYS_ON
#define APP_NODE_TYPE       ZWAVEPLUS_INFO_REPORT_NODE_TYPE_ZWAVEPLUS_NODE
#define APP_ICON_TYPE       ICON_TYPE_GENERIC_WALL_CONTROLLER
#define APP_USER_ICON_TYPE  ICON_TYPE_GENERIC_WALL_CONTROLLER
//@ [APP_ROLE_TYPE]
//@ [APP_TYPE_ID]

/**
 * Defines used to initialize the Manufacturer Specific Command Class.
 */
#define APP_MANUFACTURER_ID     MFG_ID_ZWAVE
#define APP_PRODUCT_TYPE_ID     PRODUCT_TYPE_ID_ZWAVE_PLUS_V2
#define APP_PRODUCT_ID          PRODUCT_ID_WallController

#define APP_FIRMWARE_ID         APP_PRODUCT_ID | (APP_PRODUCT_TYPE_ID << 8)

/**
 * Defines used to initialize the Association Group Information (AGI)
 * Command Class.
 */
#define NUMBER_OF_ENDPOINTS         0
#define MAX_ASSOCIATION_GROUPS      3 // Number of elements in AGITABLE_ROOTDEVICE_GROUPS plus one (for Lifeline)
#define MAX_ASSOCIATION_IN_GROUP    5

/*
 * File identifiers for application file system
 * Range: 0x00000 - 0x0FFFF
 */
#define FILE_ID_APPLICATIONDATA  (0x00000)

//@ [AGI_TABLE_ID]
//@ [AGITABLE_LIFELINE_GROUP]
#define AGITABLE_LIFELINE_GROUP \
 {COMMAND_CLASS_CENTRAL_SCENE_V3, CENTRAL_SCENE_NOTIFICATION_V3}, \
 {COMMAND_CLASS_CENTRAL_SCENE_V3, CENTRAL_SCENE_CONFIGURATION_REPORT_V3}, \
 {COMMAND_CLASS_DEVICE_RESET_LOCALLY, DEVICE_RESET_LOCALLY_NOTIFICATION}, \
 {COMMAND_CLASS_INDICATOR, INDICATOR_REPORT_V3}
//@ [AGITABLE_LIFELINE_GROUP]

//@ [AGITABLE_ROOTDEVICE_GROUPS]
#define  AGITABLE_ROOTDEVICE_GROUPS \
 { /* Group 1 */ \
   {ASSOCIATION_GROUP_INFO_REPORT_PROFILE_CONTROL, ASSOCIATION_GROUP_INFO_REPORT_PROFILE_CONTROL_KEY01}, \
   1, \
   {{COMMAND_CLASS_SWITCH_COLOR_V3, SWITCH_COLOR_SET}}, \
   "GESTURE" \
 }, \
 { /* Group 2 */ \
   {ASSOCIATION_GROUP_INFO_REPORT_PROFILE_CONTROL, ASSOCIATION_GROUP_INFO_REPORT_PROFILE_CONTROL_KEY01}, \
   2, \
   {{COMMAND_CLASS_SWITCH_MULTILEVEL_V4, SWITCH_MULTILEVEL_START_LEVEL_CHANGE_V4},{COMMAND_CLASS_SWITCH_MULTILEVEL_V4, SWITCH_MULTILEVEL_STOP_LEVEL_CHANGE_V4}}, \
   "BTN0" \
 }

//@ [AGITABLE_ROOTDEVICE_GROUPS]
//@ [AGI_TABLE_ID]

/**
 * Security keys
 */
//@ [REQUESTED_SECURITY_KEYS_ID]
#define REQUESTED_SECURITY_KEYS (SECURITY_KEY_S0_BIT | SECURITY_KEY_S2_UNAUTHENTICATED_BIT | SECURITY_KEY_S2_AUTHENTICATED_BIT)
//@ [REQUESTED_SECURITY_KEYS_ID]

/**
 * Setup the Z-Wave frequency
 *
 * The definition is only set if it's not already set to make it possible to pass the frequency to
 * the compiler by command line or in the Studio project.
 */
#ifndef APP_FREQ
#define APP_FREQ REGION_US
#endif

#endif /* _CONFIG_APP_H_ */

