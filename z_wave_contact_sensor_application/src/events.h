/**
 * @file
 *
 * Definitions of events for Sensor PIR Certified App.
 *
 * @copyright 2019 Silicon Laboratories Inc.
 */
#ifndef APPS_SENSORPIR_EVENTS_H_
#define APPS_SENSORPIR_EVENTS_H_

/**
 * Defines events for the application.
 *
 * These events are not referred to anywhere else than in the application. Hence, they can be
 * altered to suit the application flow.
 *
 * The events are located in a separate file to make it possible to include them in other
 * application files. An example could be a peripheral driver that enqueues an event when something
 * specific happens, e.g. on motion detection.
 */
typedef enum EVENT_APP_REFERENCE_DESIGN
{
  EVENT_EMPTY = DEFINE_EVENT_APP_NBR,
  EVENT_APP_INIT,
  EVENT_APP_FLUSHMEM_READY,
  EVENT_APP_NEXT_EVENT_JOB,
  EVENT_APP_FINISH_EVENT_JOB,
  EVENT_APP_SEND_BATTERY_LEVEL_REPORT,
  EVENT_APP_IS_POWERING_DOWN,
  EVENT_APP_BASIC_STOP_JOB,
  EVENT_APP_BASIC_START_JOB,
  EVENT_APP_NOTIFICATION_START_JOB,
  EVENT_APP_NOTIFICATION_STOP_JOB,
  EVENT_APP_START_TIMER_EVENTJOB_STOP,
  EVENT_APP_SMARTSTART_IN_PROGRESS,
  EVENT_APP_LEARN_IN_PROGRESS,
  //Action: Added for door/window Sensor
  EVENT_APP_DOOR_OPEN,
  EVENT_APP_DOOR_CLOSED,
  EVENT_APP_DOORSTATE,
  EVENT_APP_DOOR_TAMPER
}
EVENT_APP;

#endif /* APPS_SENSORPIR_EVENTS_H_ */
