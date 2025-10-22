/*
 * iic.h
 *
 *  Created on: 23.08.2023
 *      Author: T.Bretzke
 */

#ifndef MAIN_TLV493_H_
#define MAIN_TLV493_H_


#include <stdint.h>
#include <stddef.h>
#include "esp_system.h"
#include "driver/i2c_master.h"
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "periphery.h"
#include "esp_log.h"
#include <math.h>


#define TLV493_ADDR               0x5E

#define PCF8563_ADDR              0x51   
#define TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3"

typedef struct {
    int16_t mXdata;
    int16_t mYdata;
    int16_t mZdata;
    uint8_t mTempdata;
} tlvData_t;

typedef struct {
    uint8_t second;
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t weekday;
    uint8_t month;
    uint16_t year;
} pcfData_t;

extern void TLV_init(void);
extern void TLV_deinit(void);
extern int32_t i32TLV_getAngle(void);
extern void i2c_init(void);
extern void i2c_deinit(void);

extern time_t RTCExt_getUnixTime(void);
extern void RTCExt_setUnixTime(void);

#endif /* MAIN_IIC_H_ */
