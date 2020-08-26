/***************************************************************************//**
 * @file config_app.h
 * @brief This header file contains defines for application version in a
 * generalized way.
 * @version 1.0.1
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

/****************************************************************************
 *
 * Defines device generic and specific types
 *
 ****************************************************************************/
//@ [GENERIC_TYPE_ID]
#define GENERIC_TYPE GENERIC_TYPE_SENSOR_NOTIFICATION
#define SPECIFIC_TYPE SPECIFIC_TYPE_NOTIFICATION_SENSOR
//@ [GENERIC_TYPE_ID]

/**
 * See ZW_basis_api.h for ApplicationNodeInformation field deviceOptionMask
 */
//@ [DEVICE_OPTIONS_MASK_ID]
#define DEVICE_OPTIONS_MASK   APPLICATION_NODEINFO_NOT_LISTENING
//@ [DEVICE_OPTIONS_MASK_ID]

/****************************************************************************
 *
 * Defines used to initialize the Z-Wave Plus Info Command Class.
 *
 ****************************************************************************/
//@ [APP_TYPE_ID]
#define APP_ROLE_TYPE ZWAVEPLUS_INFO_REPORT_ROLE_TYPE_SLAVE_SLEEPING_REPORTING
#define APP_NODE_TYPE ZWAVEPLUS_INFO_REPORT_NODE_TYPE_ZWAVEPLUS_NODE
#define APP_ICON_TYPE ICON_TYPE_SPECIFIC_SENSOR_NOTIFICATION_HOME_SECURITY
#define APP_USER_ICON_TYPE ICON_TYPE_SPECIFIC_SENSOR_NOTIFICATION_HOME_SECURITY
//@ [APP_TYPE_ID]

/****************************************************************************
 *
 * Defines used to initialize the Manufacturer Specific Command Class.
 *
 ****************************************************************************/
#define APP_MANUFACTURER_ID     MFG_ID_ZWAVE
#define APP_PRODUCT_TYPE_ID     PRODUCT_TYPE_ID_ZWAVE_PLUS_V2
#define APP_PRODUCT_ID          PRODUCT_ID_SensorPIR

#define APP_FIRMWARE_ID         APP_PRODUCT_ID | (APP_PRODUCT_TYPE_ID << 8)

/****************************************************************************
 *
 * Defines used to initialize the Association Group Information (AGI)
 * Command Class.
 *
 ****************************************************************************/
#define NUMBER_OF_ENDPOINTS         0
#define MAX_ASSOCIATION_GROUPS      2
#define MAX_ASSOCIATION_IN_GROUP    5

/*
 * File identifiers for application file system
 * Range: 0x00000 - 0x0FFFF
 */
#define FILE_ID_APPLICATIONDATA  (0x00000)

//@ [AGI_TABLE_ID]
#define AGITABLE_LIFELINE_GROUP \
  {COMMAND_CLASS_BATTERY, BATTERY_REPORT}, \
  {COMMAND_CLASS_NOTIFICATION_V8, NOTIFICATION_REPORT_V8}, \
  {COMMAND_CLASS_DEVICE_RESET_LOCALLY, DEVICE_RESET_LOCALLY_NOTIFICATION}, \
  {COMMAND_CLASS_INDICATOR, INDICATOR_REPORT_V3}

#define  AGITABLE_ROOTDEVICE_GROUPS \
  {{ASSOCIATION_GROUP_INFO_REPORT_PROFILE_NOTIFICATION, NOTIFICATION_REPORT_HOME_SECURITY_V4}, 1, {{COMMAND_CLASS_BASIC, BASIC_SET}}, "Basic set"}
//@ [AGI_TABLE_ID]

/**
 * Max notifications types
 */
#define MAX_NOTIFICATIONS 1

/**
 * The value basic set command should use  when an event occur
 */
#define BASIC_SET_TRIGGER_VALUE (0xFF)
/**
 * The timeout value in milli seconds that used when sending basic set command due to an event
 */
#define BASIC_SET_TIMEOUT    10000

//@ [REQUESTED_SECURITY_KEYS_ID]
#define REQUESTED_SECURITY_KEYS (SECURITY_KEY_S2_UNAUTHENTICATED_BIT | SECURITY_KEY_S2_AUTHENTICATED_BIT)
//#define REQUESTED_SECURITY_KEYS SECURITY_KEY_S2_UNAUTHENTICATED_BIT
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
