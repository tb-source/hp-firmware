/*
 * wifi_time.h
 *
 *  Created on: 23.08.2023
 *      Author: T.Bretzke
 */

#ifndef MAIN_WIFITIME_H_
#define MAIN_WIFITIME_H_

#include "peripherie/button_sleep.h"
#include "periphery.h"
#include <stdint.h>
#include <stddef.h>
#include "esp_system.h"
#include "driver/i2c.h"
// #include "timegm.h"
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>



// #define PCF_ALARM_FLAG                  (1<<3)
// #define PCF_TIMER_FLAG                  (1<<2)
// #define PCF_ALARM_INTERRUPT_ENABLE      (1<<1)
// #define PCF_TIMER_INTERRUPT_ENABLE      (1<<0)

// #define PCF_CLKOUT_32768HZ              0b10000000
// #define PCF_CLKOUT_1024HZ               0b10000001
// #define PCF_CLKOUT_32HZ                 0b10000010
// #define PCF_CLKOUT_1HZ                  0b10000011
// #define PCF_CLKOUT_DISABLED             0b00000000

// #define PCF_TIMER_4096HZ                0b10000000
// #define PCF_TIMER_64HZ                  0b10000001
// #define PCF_TIMER_1HZ                   0b10000010
// #define PCF_TIMER_1_60HZ                0b10000011
// #define PCF_TIMER_DISABLED              0b00000011

// #define PCF_DISABLE_ALARM               80


// typedef struct {
//     uint8_t minute;
//     uint8_t hour;
//     uint8_t day;
//     uint8_t weekday;
// } PCF_Alarm;

// typedef struct {
//     uint8_t second;
//     uint8_t minute;
//     uint8_t hour;
//     uint8_t day;
//     uint8_t weekday;
//     uint8_t month;
//     uint16_t year;
// } PCF_DateTime;

// void RTC_getTime(void);
// void RTC_updateTime(void);
// void wifi_init_sta(void);
// extern time_t RTCExt_getUnixTime(void);
// extern void RTCExt_setUnixTime(void);

#endif /* MAIN_WIFITIME_H_ */
