/*
 * button_sleep.h
 *
 *  Created on: 13.12.2021
 *      Author: tobby
 */

#ifndef MAIN_BUTTONSLEEP_H_
#define MAIN_BUTTONSLEEP_H_

#include "../periphery.h"
#include "esp_sleep.h"

typedef enum{
	BTN_IDLE,
    BTN_WAKEUP,
	BTN_PRESSED_SHORT,
    BTN_PRESSED_MID,
    BTN_PRESSED_LONG,
}button_t;

typedef enum{
    WAKEUP_IDLE,
	WAKEUP_BTN_PRESSED_SHORT,
    WAKEUP_BTN_PRESSED_MID,
    WAKEUP_BTN_PRESSED_LONG,
    WAKEUP_TIMER,
    WAKEUP_RESET
}wakeup_t;

extern void button_sleep_init(void);
extern uint32_t ui32Button_read(void);
extern void deepSleep_activate(uint64_t sleepTimeus);
extern void deepSleep_activate();
extern button_t eButton_state(void);
extern wakeup_t eWakeup_state(void);

#endif /* MAIN_BUTTONSLEEP_H_ */
