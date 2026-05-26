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
#include <stdint.h>
#include "time.h"


#define EVENTCOUNT 5

#if CONFIG_PERIPHERY_VARIANT_LG
#define CHANNELCOUNT 6
#define PUMPCOUNT 2
#define SELCOUNT 2
#endif
#if CONFIG_PERIPHERY_VARIANT_LH
#define CHANNELCOUNT 3
#define PUMPCOUNT 1
#define SELCOUNT 1
#endif

typedef enum
{
  WATYPE_UNKNOWN = 0,           //watering not defined
  WATYPE_TIMEBASED = 1,         //watering at fixed times
  WATYPE_MOISTUREBASED = 2,     //watering based on moisture level
  WATYPE_AIBASED = 3,       //watering based on AI prediction (not implemented yet)
} wateringType_t;

typedef struct {
    int32_t amount;     //amout in ml
    int32_t hour;       //time hour (0-23)
    int32_t minute;     //time minute (0-59)
} eventData_t;

typedef struct {
    bool senseEnabled;
    uint8_t macTable[6];
    int32_t maxMoisture;   //max moisture in 0-100%
    int32_t minMoisture;   //min moisture in 0-100%
    int32_t maxAmount;     //max watering amount in ml/day
} moistureData_t;

typedef struct {
    bool enabled;
    wateringType_t wateringType;
    int32_t frequency;
    moistureData_t moisture;
    eventData_t events[EVENTCOUNT];
} channelData_t;

typedef struct{
    int32_t batteryLevel;       //[mv]
    int32_t temperature;        //[°C/10]
    channelData_t channels[CHANNELCOUNT];
} deviceData_t;

typedef struct{
    bool wateringEnable;
    time_t wateringLastUnix;
    time_t wateringNextUnix;
    time_t wateringFreqUnix;
    uint32_t wateringAmount;        //watering amount in ml
    uint32_t wateringAmountLast;        //watering amount last in ml
} wateringTime_t;

typedef struct{
    wateringTime_t wateringEvent[EVENTCOUNT];
} wateringTimeEvent_t;

typedef struct{
    wateringTimeEvent_t wateringChannel[CHANNELCOUNT];
    time_t wateringNextUnix;
    time_t wateringLastUnix;
} wateringData_t;

typedef struct {
    float    temperature;
    uint32_t illuminance;
    uint8_t  moisture;
    uint16_t conductivity;
    uint8_t  battery;
    char     firmware[8];
    bool     valid;
} miflora_data_t;


#endif /* MAIN_TYPES_H_ */
