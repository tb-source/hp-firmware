/*
 * helper.c
 *
 *  Created on: 07.10.2023
 *      Author: tobby
 */

#include "helper.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "cJSON.h"

static const char *TAG = "json";

static void prv_format_mac_string(const uint8_t aui8Mac[6], char *pacBuf, size_t uiBufSize)
{
    if (aui8Mac == NULL || pacBuf == NULL || uiBufSize == 0u)
    {
        return;
    }

    snprintf(pacBuf,
             uiBufSize,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             aui8Mac[0],
             aui8Mac[1],
             aui8Mac[2],
             aui8Mac[3],
             aui8Mac[4],
             aui8Mac[5]);
}

static void prv_set_channel_defaults(deviceData_t *psDeviceData)
{
    for (int32_t i = 0; i < CHANNELCOUNT; i++)
    {
        channelData_t *psChannel = &psDeviceData->channels[i];
        memset(psChannel, 0, sizeof(*psChannel));
        psChannel->enabled = false;
    }
}

static wateringType_t prv_watering_type_from_text(const char *pacText)
{
    if (pacText == NULL)
    {
        return WATYPE_UNKNOWN;
    }
    if (strcmp(pacText, "timebased") == 0)
    {
        return WATYPE_TIMEBASED;
    }
    if (strcmp(pacText, "moisturebased") == 0)
    {
        return WATYPE_MOISTUREBASED;
    }
    if (strcmp(pacText, "aibased") == 0)
    {
        return WATYPE_AIBASED;
    }
    return WATYPE_UNKNOWN;
}

const char *prv_watering_type_to_text(wateringType_t eType)
{
    switch (eType)
    {
        case WATYPE_TIMEBASED: return "timebased";
        case WATYPE_MOISTUREBASED: return "moisturebased";
        case WATYPE_AIBASED: return "aibased";
        case WATYPE_UNKNOWN:
        default:
            return "unknown";
    }
}

static bool prv_parse_hh_mm(const char *pacTime, int32_t *pi32Hour, int32_t *pi32Minute)
{
    int iHour = 0;
    int iMinute = 0;

    if (pacTime == NULL || pi32Hour == NULL || pi32Minute == NULL)
    {
        return false;
    }

    if (sscanf(pacTime, "%d:%d", &iHour, &iMinute) != 2)
    {
        return false;
    }

    if (iHour < 0 || iHour > 23 || iMinute < 0 || iMinute > 59)
    {
        return false;
    }

    *pi32Hour = (int32_t)iHour;
    *pi32Minute = (int32_t)iMinute;
    return true;
}

esp_err_t deviceDataJson_parse(deviceData_t *psDeviceData, const char *pacJson, bool bSyncTime)
{
    cJSON *psRoot = NULL;
    cJSON *psDevice = NULL;

    if (psDeviceData == NULL || pacJson == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    psRoot = cJSON_Parse(pacJson);
    if (psRoot == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    {
        cJSON *psOk = cJSON_GetObjectItem(psRoot, "ok");
        if ((psOk != NULL) && !cJSON_IsTrue(psOk))
        {
            cJSON_Delete(psRoot);
            return ESP_FAIL;
        }
    }

    if (bSyncTime)
    {
        time_t tTimeToApply = 0;
        cJSON *psTime = cJSON_GetObjectItem(psRoot, "time");

        if (cJSON_IsNumber(psTime) && psTime->valuedouble > 1000000000.0)
        {
            tTimeToApply = (time_t)psTime->valuedouble;
        }
        else if (cJSON_IsObject(psTime))
        {
            cJSON *psAct = cJSON_GetObjectItem(psTime, "act");
            if (cJSON_IsNumber(psAct) && psAct->valuedouble > 1000000000.0)
            {
                tTimeToApply = (time_t)psAct->valuedouble;
            }
        }

        if (tTimeToApply > 0)
        {
            struct timeval sTime = {0};
            sTime.tv_sec = tTimeToApply;
            sTime.tv_usec = 0;
            settimeofday(&sTime, NULL);
        }
    }

    psDevice = cJSON_GetObjectItem(psRoot, "device");
    if (!cJSON_IsObject(psDevice))
    {
        cJSON_Delete(psRoot);
        return ESP_ERR_INVALID_ARG;
    }

    {
        cJSON *psBatteryLevel = cJSON_GetObjectItem(psDevice, "batteryLevel");
        cJSON *psTemperature = cJSON_GetObjectItem(psDevice, "temperature");
        cJSON *psChannels = cJSON_GetObjectItem(psDevice, "channels");

        if (cJSON_IsNumber(psBatteryLevel))
        {
            psDeviceData->batteryLevel = psBatteryLevel->valueint;
        }
        if (cJSON_IsNumber(psTemperature))
        {
            psDeviceData->temperature = psTemperature->valueint;
        }

        prv_set_channel_defaults(psDeviceData);

        if (cJSON_IsArray(psChannels))
        {
            int iChannelCount = cJSON_GetArraySize(psChannels);

            for (int i = 0; i < iChannelCount; i++)
            {
                cJSON *psChannel = cJSON_GetArrayItem(psChannels, i);
                cJSON *psNumber = NULL;
                channelData_t *psChannelData = NULL;
                int iChannelIndex = i;

                if (!cJSON_IsObject(psChannel))
                {
                    continue;
                }

                psNumber = cJSON_GetObjectItem(psChannel, "number");
                if (cJSON_IsNumber(psNumber))
                {
                    iChannelIndex = psNumber->valueint - 1;
                }

                if (iChannelIndex < 0 || iChannelIndex >= CHANNELCOUNT)
                {
                    continue;
                }

                psChannelData = &psDeviceData->channels[iChannelIndex];

                {
                    cJSON *psEnabled = cJSON_GetObjectItem(psChannel, "enabled");
                    cJSON *psFrequency = cJSON_GetObjectItem(psChannel, "frequency");
                    cJSON *psChannelWateringType = cJSON_GetObjectItem(psChannel, "wateringType");
                    cJSON *psMoisture = cJSON_GetObjectItem(psChannel, "moisture");
                    cJSON *psEvents = cJSON_GetObjectItem(psChannel, "events");

                    if (cJSON_IsBool(psEnabled))
                    {
                        psChannelData->enabled = cJSON_IsTrue(psEnabled);
                    }

                    if (cJSON_IsNumber(psFrequency))
                    {
                        psChannelData->frequency = psFrequency->valueint;
                    }

                    if (cJSON_IsString(psChannelWateringType) && psChannelWateringType->valuestring != NULL)
                    {
                        psChannelData->wateringType = prv_watering_type_from_text(psChannelWateringType->valuestring);
                    }
                    else if (cJSON_IsNumber(psChannelWateringType))
                    {
                        psChannelData->wateringType = (wateringType_t)psChannelWateringType->valueint;
                    }

                    if (cJSON_IsObject(psMoisture))
                    {
                        cJSON *psSenseEnabled = cJSON_GetObjectItem(psMoisture, "senseEnabled");
                        cJSON *psMin = cJSON_GetObjectItem(psMoisture, "minMoisture");
                        cJSON *psMax = cJSON_GetObjectItem(psMoisture, "maxMoisture");
                        cJSON *psMaxAmount = cJSON_GetObjectItem(psMoisture, "maxAmount");
                        cJSON *psMacTable = cJSON_GetObjectItem(psMoisture, "macTable");

                        if (cJSON_IsBool(psSenseEnabled)) { psChannelData->moisture.senseEnabled = cJSON_IsTrue(psSenseEnabled); }
                        if (cJSON_IsNumber(psMin)) { psChannelData->moisture.minMoisture = psMin->valueint; }
                        if (cJSON_IsNumber(psMax)) { psChannelData->moisture.maxMoisture = psMax->valueint; }
                        if (cJSON_IsNumber(psMaxAmount)) { psChannelData->moisture.maxAmount = psMaxAmount->valueint; }

                        memset(psChannelData->moisture.macTable, 0, sizeof(psChannelData->moisture.macTable));
                        if (cJSON_IsArray(psMacTable) && cJSON_GetArraySize(psMacTable) == 6)
                        {
                            for (int k = 0; k < 6; k++)
                            {
                                cJSON *psMacByte = cJSON_GetArrayItem(psMacTable, k);
                                if (cJSON_IsNumber(psMacByte) && psMacByte->valueint >= 0 && psMacByte->valueint <= 255)
                                {
                                    psChannelData->moisture.macTable[k] = (uint8_t)psMacByte->valueint;
                                }
                            }
                        }
                    }

                    memset(psChannelData->events, 0, sizeof(psChannelData->events));
                    if (cJSON_IsArray(psEvents))
                    {
                        int iEventCount = cJSON_GetArraySize(psEvents);
                        for (int j = 0; j < iEventCount && j < EVENTCOUNT; j++)
                        {
                            cJSON *psEvent = cJSON_GetArrayItem(psEvents, j);
                            cJSON *psAmount = NULL;
                            cJSON *psHour = NULL;
                            cJSON *psMinute = NULL;
                            cJSON *psTime = NULL;

                            if (!cJSON_IsObject(psEvent))
                            {
                                continue;
                            }

                            psAmount = cJSON_GetObjectItem(psEvent, "amount");
                            psHour = cJSON_GetObjectItem(psEvent, "hour");
                            psMinute = cJSON_GetObjectItem(psEvent, "minute");
                            psTime = cJSON_GetObjectItem(psEvent, "time");
                            if (!cJSON_IsString(psTime) && !cJSON_IsNumber(psTime))
                            {
                                psTime = cJSON_GetObjectItem(psEvent, "TIME");
                            }

                            if (cJSON_IsNumber(psAmount))
                            {
                                psChannelData->events[j].amount = psAmount->valueint;
                            }
                            if (cJSON_IsNumber(psHour))
                            {
                                psChannelData->events[j].hour = psHour->valueint;
                            }
                            if (cJSON_IsNumber(psMinute))
                            {
                                psChannelData->events[j].minute = psMinute->valueint;
                            }

                            if (cJSON_IsNumber(psTime) && !cJSON_IsNumber(psHour))
                            {
                                psChannelData->events[j].hour = psTime->valueint;
                            }
                            else if (cJSON_IsString(psTime) && psTime->valuestring != NULL)
                            {
                                int32_t iHour = 0;
                                int32_t iMinute = 0;
                                if (prv_parse_hh_mm(psTime->valuestring, &iHour, &iMinute))
                                {
                                    psChannelData->events[j].hour = iHour;
                                    psChannelData->events[j].minute = iMinute;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    cJSON_Delete(psRoot);
    return ESP_OK;
}

char *deviceDataJson_serialize(const deviceData_t *psDeviceData)
{
    cJSON *psTop = NULL;
    cJSON *psDevice = NULL;
    cJSON *psChannels = NULL;
    char *pacOut = NULL;

    if (psDeviceData == NULL)
    {
        return NULL;
    }

    psTop = cJSON_CreateObject();
    psDevice = cJSON_CreateObject();
    psChannels = cJSON_CreateArray();
    if (psTop == NULL || psDevice == NULL || psChannels == NULL)
    {
        cJSON_Delete(psTop);
        cJSON_Delete(psDevice);
        cJSON_Delete(psChannels);
        return NULL;
    }

    cJSON_AddItemToObject(psTop, "device", psDevice);
    cJSON_AddNumberToObject(psDevice, "batteryLevel", psDeviceData->batteryLevel);
    cJSON_AddNumberToObject(psDevice, "temperature", psDeviceData->temperature);
    cJSON_AddItemToObject(psDevice, "channels", psChannels);

    for (int32_t i = 0; i < CHANNELCOUNT; i++)
    {
        cJSON *psChannel = cJSON_CreateObject();
        cJSON *psMoisture = cJSON_CreateObject();
        cJSON *psEvents = cJSON_CreateArray();
        const channelData_t *psSource = &psDeviceData->channels[i];

        if (psChannel == NULL || psMoisture == NULL || psEvents == NULL)
        {
            cJSON_Delete(psTop);
            return NULL;
        }

        cJSON_AddItemToArray(psChannels, psChannel);
        cJSON_AddNumberToObject(psChannel, "number", i + 1);
        cJSON_AddBoolToObject(psChannel, "enabled", psSource->enabled);
        cJSON_AddNumberToObject(psChannel, "wateringType", psSource->wateringType);
        cJSON_AddNumberToObject(psChannel, "frequency", psSource->frequency);

        cJSON_AddItemToObject(psChannel, "moisture", psMoisture);
        cJSON_AddBoolToObject(psMoisture, "senseEnabled", psSource->moisture.senseEnabled);
        cJSON_AddNumberToObject(psMoisture, "minMoisture", psSource->moisture.minMoisture);
        cJSON_AddNumberToObject(psMoisture, "maxMoisture", psSource->moisture.maxMoisture);
        cJSON_AddNumberToObject(psMoisture, "maxAmount", psSource->moisture.maxAmount);
        {
            cJSON *psMacTable = cJSON_AddArrayToObject(psMoisture, "macTable");
            for (int32_t k = 0; k < 6; k++)
            {
                cJSON_AddItemToArray(psMacTable, cJSON_CreateNumber(psSource->moisture.macTable[k]));
            }
        }

        cJSON_AddItemToObject(psChannel, "events", psEvents);
        for (int32_t j = 0; j < EVENTCOUNT; j++)
        {
            const eventData_t *psEvent = &psSource->events[j];
            if ((j != 0) && (psEvent->amount <= 0))
            {
                continue;
            }

            cJSON *psEventJson = cJSON_CreateObject();
            if (psEventJson == NULL)
            {
                cJSON_Delete(psTop);
                return NULL;
            }

            cJSON_AddItemToArray(psEvents, psEventJson);
            cJSON_AddNumberToObject(psEventJson, "amount", psEvent->amount);

            {
                char acTime[8] = {0};
                snprintf(acTime, sizeof(acTime), "%02ld:%02ld", (long)psEvent->hour, (long)psEvent->minute);
                cJSON_AddStringToObject(psEventJson, "time", acTime);
            }
        }
    }

    pacOut = cJSON_PrintUnformatted(psTop);
    cJSON_Delete(psTop);
    return pacOut;
}

void data_logDeviceData(const char *pacTag, const deviceData_t *psData)
{
    const char *pacLogTag = (pacTag != NULL) ? pacTag : TAG;

    if (psData == NULL)
    {
        return;
    }

    ESP_LOGI(pacLogTag, "--- deviceData_t ---");
    ESP_LOGI(pacLogTag, "  battery=%ldmV  temperature=%.1fC",
             (long)psData->batteryLevel,
             (float)psData->temperature / 10.0f);

    for (int i = 0; i < CHANNELCOUNT; i++)
    {
        const channelData_t *psC = &psData->channels[i];
        char acMac[18] = {0};
        prv_format_mac_string(psC->moisture.macTable, acMac, sizeof(acMac));

        ESP_LOGI(pacLogTag, "  CH%d: en=%d  wtype=%s(%ld)  freq=%d  moisture(min=%ld max=%ld maxAmount=%ld sens=%d)",
                 i + 1,
                 (int)psC->enabled,
                prv_watering_type_to_text(psC->wateringType),
                (long)psC->wateringType,
                 (int)psC->frequency,
                 (long)psC->moisture.minMoisture,
                 (long)psC->moisture.maxMoisture,
                 (long)psC->moisture.maxAmount,
                 (int)psC->moisture.senseEnabled);
        ESP_LOGI(pacLogTag, "       hum-mac=%s", acMac);

        for (int j = 0; j < EVENTCOUNT; j++)
        {
            const eventData_t *psE = &psC->events[j];

            if (psE->amount == 0 && psE->hour == 0 && psE->minute == 0)
            {
                continue;
            }

            ESP_LOGI(pacLogTag,
                     "    EVT%d: %02ld:%02ld  %ldml",
                     j + 1,
                     (long)psE->hour,
                     (long)psE->minute,
                     (long)psE->amount);
        }
    }
}

void data_logWateringData(const char *pacTag, const wateringData_t *psData)
{
    const char *pacLogTag = (pacTag != NULL) ? pacTag : TAG;

    if (psData == NULL)
    {
        return;
    }

    ESP_LOGI(pacLogTag,
             "--- wateringData_t --- next=%lld last=%lld",
             (long long)psData->wateringNextUnix,
             (long long)psData->wateringLastUnix);

    for (int i = 0; i < CHANNELCOUNT; i++)
    {
        for (int j = 0; j < EVENTCOUNT; j++)
        {
            const wateringTime_t *psEvt = &psData->wateringChannel[i].wateringEvent[j];

            if (!psEvt->wateringEnable &&
                psEvt->wateringAmount == 0u &&
                psEvt->wateringNextUnix == 0 &&
                psEvt->wateringLastUnix == 0)
            {
                continue;
            }

            ESP_LOGI(pacLogTag,
                     "  CH%d EVT%d: en=%d amount=%lu amountLast=%lu freq=%lld last=%lld next=%lld",
                     i + 1,
                     j + 1,
                     (int)psEvt->wateringEnable,
                     (unsigned long)psEvt->wateringAmount,
                     (unsigned long)psEvt->wateringAmountLast,
                     (long long)psEvt->wateringFreqUnix,
                     (long long)psEvt->wateringLastUnix,
                     (long long)psEvt->wateringNextUnix);
        }
    }
}

const char* pacData_send_receive(char* pGetData, deviceData_t* peDevice_data)
{
    static volatile char eDataState = DATAFLOW_IDLE; 
    static char cDataCount = 0;
    static char cDataLen = 0;
    static char caDeviceData[5000];
    static char cData[512];

    // ESP_LOGI(TAG, "First byte: %c", *pGetData);

    switch(*pGetData)
    {
        case DATA_APPTOPOT:
        ESP_LOGI("DATA_DIRECTION", "APPTPPOT");
            switch(eDataState)
            {
                case DATAFLOW_IDLE:
                if (*(pGetData + 2)<'9')       //limit data length to 10 messages
                    {
                        // ESP_LOGI(TAG, "FIRST DATA: %s", pGetData);
                        //save data
                        if (strlen(pGetData)>4)
                        {
                            strcpy(caDeviceData, pGetData + 3);
                            if (*(pGetData + 1) == *(pGetData + 2))       //data only one meassage long
                            {
                                eDataState = DATAFLOW_END;
                                ESP_LOGI(TAG, "FINAL DATA: %s", caDeviceData);
                                deviceDataJson_parse(peDevice_data, caDeviceData, false);
                            }
                            else{
                               eDataState = DATAFLOW_RECEIVE;  
                            }
                        }       
                        else{
                            eDataState = DATAFLOW_ERROR; 
                        }            
                    }
                else
                    {
                        eDataState = DATAFLOW_ERROR; 
                    }
                // ESP_LOGI(TAG, "FINAL DATA: %s", caDeviceData);
                break;
                
                case DATAFLOW_RECEIVE:
                    // ESP_LOGI(TAG, "SECOND DATA: %s", pGetData);
                    //save data
                    if (strlen(pGetData)>4)
                    {
                        strcat(caDeviceData, pGetData + 3);
                        if (*(pGetData + 1)== *(pGetData + 2))       //last message
                            {
                                eDataState = DATAFLOW_END;
                                ESP_LOGI(TAG, "FINAL DATA1: %s", caDeviceData);
                                deviceDataJson_parse(peDevice_data, caDeviceData, false);
                            }
                    }       
                    else
                    {
                        eDataState = DATAFLOW_ERROR; 
                    }     
                break;

                default:
                eDataState = DATAFLOW_IDLE;
                    ESP_LOGI(TAG, "eDataState undefined" );
                break;
            }
            cData[0] = DATA_APPTOPOT;
            cData[1] = '1';  
            cData[2] = '1';
            cData[3] = 0;

            if(eDataState == DATAFLOW_ERROR)
            {
                strcat(cData,"Error");
            }
            else if (eDataState == DATAFLOW_END)
            {
                strcat(cData,"End");
                eDataState = DATAFLOW_IDLE;
            }
            else
            {
                strcat(cData,"OK");
             }
        break;

        case DATA_POTTOAPP: 
        ESP_LOGI("JSON", "POTTOAPP");
  
            switch(eDataState)
            {
                case DATAFLOW_IDLE:
                    cDataCount = '1';
                    if (*(pGetData + 1)=='1' && *(pGetData + 2)=='1')       //data only one message long
                    {
                        ESP_LOGI(TAG, "start data: %s", pGetData);
                        ESP_LOGI(TAG, "peDevice_data: %d", (int)(peDevice_data->batteryLevel));
                        //generate json data from struct
                        strcpy(caDeviceData, deviceDataJson_serialize(peDevice_data));
                        //device data
                        if (!strcmp(pGetData + 3,"device"))
                        {
                            ESP_LOGI(TAG, "dev detected:" );
                            cDataLen = (strlen(caDeviceData) / 500) + '1';                  
                            eDataState = DATAFLOW_SEND;
                        }
                        else
                        {
                            ESP_LOGI(TAG, "no dev detected:" );
                            cDataLen = '1';
                            eDataState = DATAFLOW_ERROR;
                        }                    
                    }
                    else
                    {
                        cDataLen = '1';
                        eDataState = DATAFLOW_ERROR; 
                    }
                break;

                case DATAFLOW_SEND:
                    if (*(pGetData + 1)=='1' && *(pGetData + 2)=='1')       //data only one message long
                    {
                        //check data confirm
                        if (!strcmp(pGetData + 3,"OK"))
                        {
                            ESP_LOGI("JSON", "JSON confirm: OK");
                            cDataCount++;             
                        }
                        else
                        {
                            cDataLen = '1';
                            cDataCount = '1';
                            eDataState = DATAFLOW_ERROR;
                        }                    
                    }
                    else
                    {
                        cDataLen = '1';
                        cDataCount = '1';
                        eDataState = DATAFLOW_ERROR; 
                    }                    
                break;

                case DATAFLOW_END:
                     if (*(pGetData + 1)=='1' && *(pGetData + 2)=='1')       //data only one message long
                    {
                        cDataLen = '1';
                        cDataCount = '1';    

                        //check data confirm
                        if (strcmp(pGetData + 3,"OK"))
                        {
                            eDataState = DATAFLOW_ERROR;      
                        }       
                        else
                        {
                            eDataState = DATAFLOW_IDLE;   
                        }          
                    }
                    else
                    {
                        eDataState = DATAFLOW_ERROR; 
                    }                      
                break;

                default:
                    ESP_LOGE(TAG, "eDataState undefined" );
                break;


            }
            cData[0] = DATA_POTTOAPP;
            cData[1] = cDataCount;  
            cData[2] = cDataLen;
            cData[3] = 0;

            if(eDataState == DATAFLOW_ERROR)
            {
                strcat(cData,"Error");
            }
            else if (eDataState == DATAFLOW_END)
            {
                strcat(cData,"End");
                eDataState = DATAFLOW_IDLE;
            }
            else
            {
                size_t eMessageSize = strlen(caDeviceData + (500 * (cDataCount - '1')));
                char pacHelper[501];
                char *pacStartPos = caDeviceData;
                strncpy(pacHelper, (pacStartPos + (500 * (cDataCount - '1'))),500);
                pacHelper[500]=0;
                ESP_LOGI("JSON", "Parse data: %s", pacHelper);
                strcat(cData, pacHelper);

                if (eMessageSize <= 500)
                {
                    eDataState = DATAFLOW_END;   
                }
             }
        break;

        default:
            ESP_LOGE(TAG, "DATA undefined");
        break;
    
    }
    ESP_LOGI(TAG, "Received data: %s", cData);
    return cData;
}
