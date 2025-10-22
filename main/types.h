/*
 * types.h
 *
 *  Created on: 23.08.2023
 *      Author: tobby
 */

#ifndef MAIN_TYPES_H_
#define MAIN_TYPES_H_

#include <stdio.h>
#include <stdbool.h>
#include "time.h"

#define CHANNELCOUNT 3
#define EVENTCOUNT 5

typedef struct {
    int32_t amount;
    int32_t time;
} eventData_t;

typedef struct {
    int32_t id;
    char name[100];
    char description[100];
    bool enable;
    int32_t frequency;
    eventData_t events[5];
} channelData_t;

typedef struct{
    int32_t id;
    char name[100];
    char status[20];
    int32_t battery;            //[mv]
    int32_t temperature;        //[°C/10]
    channelData_t channels[CHANNELCOUNT];
} deviceData_t;

typedef struct{
    bool wateringEnable;
    time_t wateringLastUnix;
    time_t wateringNextUnix;
    time_t wateringFreqUnix;
    uint32_t wateringAmount;        //watering amount in ml*20
    uint32_t wateringAmountLast;        //watering amount last in ml*20
} wateringTime_t;

typedef struct{
    wateringTime_t wateringEvent[EVENTCOUNT];
} wateringTimeEvent_t;

typedef struct{
    wateringTimeEvent_t wateringChannel[CHANNELCOUNT];
    time_t wateringNextUnix;
    time_t wateringLastUnix;
} wateringData_t;

#endif /* MAIN_TYPES_H_ */
