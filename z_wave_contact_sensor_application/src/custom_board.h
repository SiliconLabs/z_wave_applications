/*
 * custom_board.h
 *
 *  Created on: Jul 18, 2019
 *      Author: axbrugge
 */

#ifndef INC_CUSTOM_BOARD_H_
#define INC_CUSTOM_BOARD_H_

/*************************************************************************/
/* Configure LEDs                                                        */
/*************************************************************************/

/* NB: Mounted in parallel with "LED0" on the mainboard.
 *
 *     EFR32 peripheral: (none)
 */

//Action: Modify custom board LED
#define LED1_LABEL           "LED0"
#define LED1_GPIO_PORT       gpioPortD
#define LED1_GPIO_PIN        15
#define LED1_ON_VALUE        1
#define LED1_LETIM0_OUT0_LOC 23

/* NB: LED2 GPIO (PF3) conflicts with UART1_TX
 *     (see UART1_TX_PORT/UART1_TX_PIN in board.c).
 *     UART1 should not be used with this board.
 *
 *     EFR32 peripheral: DBG_TDI#0 (Use SWD for
 *     debug. JTAG can not be used)
 */
#define LED2_LABEL           "LED1"
#define LED2_GPIO_PORT       gpioPortF
#define LED2_GPIO_PIN        3
#define LED2_ON_VALUE        1
#define LED2_LETIM0_OUT0_LOC 27

/* NB: "LED2" (PA2) on BRD8029A cannot be used. It's shared with the mainboard
 *     VCOM_CTS pin that is currently driven low by the board controller even
 *     though VCOM hardware flow control is disabled.
 *     A bug (HWT_FW-636) has been filed with the hwtools firmware team.
 *     Until the bug has been resolved LED2 cannot be used.

 *     As a temporary workaround "LED1" on the WSK mainboard is used...
 */

// Un-comment when HWT_FW-636 has been resolved
//#define LED3_LABEL           "LED2"
//#define LED3_GPIO_PORT       gpioPortA
//#define LED3_GPIO_PIN        2
//#define LED3_ON_VALUE        1
//#define LED3_LETIM0_OUT0_LOC 2

/* NB: LED3 is located on the WSK mainboard.
 *     Using as stand-in for "LED2" (PA2) that does not currently work.
 */
#define LED3_LABEL           "LED1 (mb)"
#define LED3_GPIO_PORT       gpioPortF
#define LED3_GPIO_PIN        5
#define LED3_ON_VALUE        1
#define LED3_LETIM0_OUT0_LOC 29

/* NB: EFR32 peripheral: US0_CS#0
 */
#define LED4_LABEL           "LED3"
#define LED4_GPIO_PORT       gpioPortA
#define LED4_GPIO_PIN        3
#define LED4_ON_VALUE        1
#define LED4_LETIM0_OUT0_LOC 3


/*************************************************************************/
/* Configure RGB LEDs                                                    */
/*************************************************************************/

/* BRD8029A does not have any RGB led!
 * If paired with radio board ZGM130S then the RGB on that board can be used
 */

/*************************************************************************/
/* Configure push buttons                                                */
/*************************************************************************/

/* NB: BTN0 on BRD8029A is connected in parallel with PB0 on BRD4001A
 *     (Wireless Starter Kit mainboard). Not a problem, the two buttons
 *     simply provide the same functionality.
 */

//Action: Modify Button PB1 to Tamper Button (see schematic diagram)
#define PB1_LABEL           "TMP"
#define PB1_GPIO_PORT       gpioPortD
#define PB1_GPIO_PIN        14
#define PB1_ON_VALUE        0
#define PB1_CAN_WAKEUP_EM4  true

/* NB: BTN1 on BRD8029A is connected in parallel with PB1 on BRD4001A
 *     (Wireless Starter Kit mainboard). Not a problem, the two buttons
 *     simply provide the same functionality.
 */
//Action: Modify Button PB2 to Programming Button (see schematic diagram)
#define PB2_LABEL           "PRG"
#define PB2_GPIO_PORT       gpioPortF
#define PB2_GPIO_PIN        7
#define PB2_ON_VALUE        0
#define PB2_CAN_WAKEUP_EM4  true

#define PB3_LABEL           "BTN2"
#define PB3_GPIO_PORT       gpioPortC
#define PB3_GPIO_PIN        10
#define PB3_ON_VALUE        0
#define PB3_CAN_WAKEUP_EM4  true

#define PB4_LABEL           "BTN3"
#define PB4_GPIO_PORT       gpioPortC
#define PB4_GPIO_PIN        11
#define PB4_ON_VALUE        0
#define PB4_CAN_WAKEUP_EM4  false

/*************************************************************************/
/* Configure slider button                                               */
/*************************************************************************/

/* NB: SLIDER1 GPIO (PC9) conflicts with UART1_RX!!
 *     (see UART1_RX_PORT/UART1_RX_PIN in board.c)
 *     UART1 should not be used with this board.
 */
#define SLIDER1_LABEL          "SW1"
#define SLIDER1_GPIO_PORT      gpioPortA
#define SLIDER1_GPIO_PIN       3
#define SLIDER1_CAN_WAKEUP_EM4 true
#define SLIDER1_ON_VALUE       0

/*************************************************************************/
/* Map physical board IO devices to application LEDs and buttons         */
/*************************************************************************/

/* Map application LEDs to board LEDs */
// Action: modify mapping of LED1
#define APP_LED_INDICATOR         BOARD_LED1
// #define APP_LED_INDICATOR BOARD_LED2  // Positioned opposite APP_BUTTON_LEARN_RESET
#define APP_LED_B         BOARD_LED4
#define APP_LED_C         BOARD_LED3  // LED3 is currently "LED1" on the main board

#define APP_RGB_R         BOARD_RGB1_R
#define APP_RGB_G         BOARD_RGB1_G
#define APP_RGB_B         BOARD_RGB1_B

/* Mapping application buttons to board buttons */
#define APP_BUTTON_TMP         BOARD_BUTTON_PB1
#define APP_BUTTON_LEARN_RESET BOARD_BUTTON_PB2  // Supports EM4 wakeup
#define APP_BUTTON_B           BOARD_BUTTON_PB3  // Supports EM4 wakeup
#define APP_BUTTON_C           BOARD_BUTTON_PB4
#define APP_SLIDER_A           BOARD_BUTTON_SLIDER1

/* The next two are identical since on the BRD8029A only PB2 and PB3
 * can trigger a wakeup from EM4. PB2 is already used for learn/reset
 */
#define APP_WAKEUP_BTN_SLDR    BOARD_BUTTON_PB3 // Use this one when wakeup capability is required and button is preferred to slider
#define APP_WAKEUP_SLDR_BTN    BOARD_BUTTON_PB3 // Use this one when wakeup capability is required and slider is preferred to button

/*************************************************************************/
/* Configure UART1                                                       */
/*************************************************************************/

/* NB: The setting below for UART1 conflicts with LED2_GPIO_PORT/PIN and
 *     SLIDER1_GPIO_PORT/PIN. Currently UART1 is not used/enabled, but
 *     before we start using UART1, then we should find a pair of unused
 *     pins.
 */
//Action: Modify UART (see schematic diagram)
#define UART1_TX_PORT  gpioPortA
#define UART1_TX_PIN   0
#define UART1_RX_PORT  gpioPortC
#define UART1_RX_PIN   1


#endif /* INC_CUSTOM_BOARD_H_ */
