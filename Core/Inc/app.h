#ifndef APP_H
#define APP_H

#include "stm32f1xx_hal.h"

/*
 * DIAGNOSTIC SWITCH
 *  1 = normal: radio + capture + USB
 *  0 = USB-only test: the radio interrupts never start. Use this to find out
 *      whether a USB problem is caused by the PC/board or by our interrupts.
 */
#ifndef APP_RADIO_ENABLED
#define APP_RADIO_ENABLED 1
#endif

/* Main-loop side of the receiver: LCD, USB serial output, keyboard commands. */
void app_init(TIM_HandleTypeDef *htim_capture, TIM_HandleTypeDef *htim_rssi);
void app_loop(void);   /* call forever from while(1); never blocks for long */

#endif /* APP_H */
