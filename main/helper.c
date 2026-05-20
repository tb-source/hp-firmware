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

//parse JSON device data
esp_err_t data_convert_read(deviceData_t* device_data, char* json_data) {

	esp_err_t err = ESP_OK;
    cJSON *root = cJSON_Parse(json_data);

    if (root) {

        cJSON *device = cJSON_GetObjectItem(root, "device");
        cJSON *time = cJSON_GetObjectItem(root, "time");

        if (device){
            (*device_data).id = (int32_t)cJSON_GetObjectItem(device, "id")->valueint;
            strncpy((*device_data).name , cJSON_GetObjectItem(device, "name")->valuestring, sizeof((*device_data).name ) - 1);
            (*device_data).name[sizeof((*device_data).name) - 1] = '\0';
            strncpy((*device_data).status , cJSON_GetObjectItem(device, "status")->valuestring, sizeof((*device_data).status ) - 1);
            (*device_data).status[sizeof((*device_data).status) - 1] = '\0';
            (*device_data).battery = cJSON_GetObjectItem(device, "battery")->valueint;
            (*device_data).temperature = cJSON_GetObjectItem(device, "temperature")->valueint;

            cJSON *wateringtype = cJSON_GetObjectItem(device, "wateringtype");
            if (cJSON_IsNumber(wateringtype))
            {
                (*device_data).wateringtype = wateringtype->valueint;
            }

            ESP_LOGI(TAG, "Device id: %d", (int)(*device_data).id);
            ESP_LOGI(TAG, "Device name: %s", (*device_data).name);
            ESP_LOGI(TAG, "Device status: %s", (*device_data).status);
            ESP_LOGI(TAG, "Device batVolt: %d", (int)(*device_data).battery);
            ESP_LOGI(TAG, "Device temp: %d", (int)(*device_data).temperature);
            ESP_LOGI(TAG, "Device wateringtype: %d", (int)(*device_data).wateringtype);

            cJSON *channels = cJSON_GetObjectItem(device, "channels");
            memset((*device_data).channels, 0, sizeof((*device_data).channels));

            if (cJSON_IsArray(channels)) {
                int num_channels = cJSON_GetArraySize(channels);

                for (int i = 0; i < num_channels && i < CHANNELCOUNT; i++) {
                    //manage channel data
                    cJSON *channel = cJSON_GetArrayItem(channels, i);
                    strncpy((*device_data).channels[i].name, cJSON_GetObjectItem(channel, "name")->valuestring, sizeof((*device_data).channels[i].name) - 1);
                    (*device_data).channels[i].name[sizeof((*device_data).channels[i].name) - 1] = '\0';
                    (*device_data).channels[i].enable = cJSON_IsTrue(cJSON_GetObjectItem(channel, "enable"));
                    // (*device_data).channels[i].duration = cJSON_GetObjectItem(channel, "duration")->valueint;
                    (*device_data).channels[i].frequency = cJSON_GetObjectItem(channel, "frequency")->valueint;

                    cJSON *moisture = cJSON_GetObjectItem(channel, "moisture");
                    if (cJSON_IsObject(moisture))
                    {
                        cJSON *senseEnable = cJSON_GetObjectItem(moisture, "senseEnable");
                        cJSON *maxMoisture = cJSON_GetObjectItem(moisture, "maxMoisture");
                        cJSON *minMoisture = cJSON_GetObjectItem(moisture, "minMoisture");
                        cJSON *macTable = cJSON_GetObjectItem(moisture, "macTable");

                        if (!cJSON_IsBool(senseEnable)) { senseEnable = cJSON_GetObjectItem(moisture, "SENS"); }
                        if (!cJSON_IsNumber(maxMoisture)) { maxMoisture = cJSON_GetObjectItem(moisture, "MAX"); }
                        if (!cJSON_IsNumber(minMoisture)) { minMoisture = cJSON_GetObjectItem(moisture, "MIN"); }
                        if (!(cJSON_IsArray(macTable) || cJSON_IsString(macTable))) { macTable = cJSON_GetObjectItem(moisture, "MAC"); }

                        if (cJSON_IsBool(senseEnable))
                        {
                            (*device_data).channels[i].moisture.senseEnable = cJSON_IsTrue(senseEnable);
                        }
                        else if (cJSON_IsNumber(senseEnable))
                        {
                            (*device_data).channels[i].moisture.senseEnable = (senseEnable->valueint != 0);
                        }

                        if (cJSON_IsNumber(maxMoisture))
                        {
                            (*device_data).channels[i].moisture.maxMoisture = maxMoisture->valueint;
                        }
                        if (cJSON_IsNumber(minMoisture))
                        {
                            (*device_data).channels[i].moisture.minMoisture = minMoisture->valueint;
                        }

                        memset((*device_data).channels[i].moisture.macTable, 0, sizeof((*device_data).channels[i].moisture.macTable));
                        if (cJSON_IsArray(macTable) && cJSON_GetArraySize(macTable) == 6)
                        {
                            for (int k = 0; k < 6; k++)
                            {
                                cJSON *macByte = cJSON_GetArrayItem(macTable, k);
                                if (cJSON_IsNumber(macByte) && macByte->valueint >= 0 && macByte->valueint <= 255)
                                {
                                    (*device_data).channels[i].moisture.macTable[k] = (uint8_t)macByte->valueint;
                                }
                            }
                        }
                        else if (cJSON_IsString(macTable) && macTable->valuestring != NULL)
                        {
                            unsigned int auiMac[6] = {0};
                            if (sscanf(macTable->valuestring, "%2x:%2x:%2x:%2x:%2x:%2x",
                                       &auiMac[0], &auiMac[1], &auiMac[2], &auiMac[3], &auiMac[4], &auiMac[5]) == 6)
                            {
                                for (int k = 0; k < 6; k++)
                                {
                                    (*device_data).channels[i].moisture.macTable[k] = (uint8_t)auiMac[k];
                                }
                            }
                        }
                    }

                    //printing channel data
                    ESP_LOGI(TAG, "Name: %s", (*device_data).channels[i].name);
                    ESP_LOGI(TAG, "Enable: %s", (*device_data).channels[i].enable ? "true" : "false");
                    ESP_LOGI(TAG, "Frequency: %d", (int)(*device_data).channels[i].frequency);
                    ESP_LOGI(TAG, "Moisture senseEnable: %s", (*device_data).channels[i].moisture.senseEnable ? "true" : "false");
                    ESP_LOGI(TAG, "Moisture min/max: %d/%d",
                             (int)(*device_data).channels[i].moisture.minMoisture,
                             (int)(*device_data).channels[i].moisture.maxMoisture);

                    //manage watering event data
                    for (int j = 0; j < EVENTCOUNT; j++)
                    {
                        (*device_data).channels[i].events[j].amount = 0;
                        (*device_data).channels[i].events[j].hour = 0;
                        (*device_data).channels[i].events[j].minute = 0;
                    }
                    
                    cJSON *events = cJSON_GetObjectItem(channel, "events");
                    if (cJSON_IsArray(events)) {
                    int num_events = cJSON_GetArraySize(events);

                        for (int j = 0; j < num_events && j < EVENTCOUNT; j++) {
                            cJSON *event = cJSON_GetArrayItem(events, j);
                            (*device_data).channels[i].events[j].amount = cJSON_GetObjectItem(event, "amount")->valueint;

                            cJSON *hourItem = cJSON_GetObjectItem(event, "hour");
                            cJSON *minuteItem = cJSON_GetObjectItem(event, "minute");
                            cJSON *timeItem = cJSON_GetObjectItem(event, "time");

                            if (cJSON_IsNumber(hourItem))
                            {
                                (*device_data).channels[i].events[j].hour = hourItem->valueint;
                            }
                            if (cJSON_IsNumber(minuteItem))
                            {
                                (*device_data).channels[i].events[j].minute = minuteItem->valueint;
                            }

                            if (cJSON_IsNumber(timeItem) && !cJSON_IsNumber(hourItem))
                            {
                                (*device_data).channels[i].events[j].hour = timeItem->valueint;
                            }
                            else if (cJSON_IsString(timeItem) && timeItem->valuestring != NULL)
                            {
                                int iHour = 0;
                                int iMinute = 0;
                                if (sscanf(timeItem->valuestring, "%d:%d", &iHour, &iMinute) == 2)
                                {
                                    (*device_data).channels[i].events[j].hour = iHour;
                                    (*device_data).channels[i].events[j].minute = iMinute;
                                }
                            }

                            //Printing event data
                            ESP_LOGI(TAG, "Amount: %d", (int)(*device_data).channels[i].events[j].amount);
                            ESP_LOGI(TAG, "Time: %02d:%02d",
                                     (int)(*device_data).channels[i].events[j].hour,
                                     (int)(*device_data).channels[i].events[j].minute);
                        }
                    }
                }
            }
        }
        else{
            err = ESP_ERR_INVALID_ARG;
        }

        if (time)
        {

            struct timeval sTime;
            cJSON *timeObject = cJSON_GetObjectItem(time, "act");
            if(timeObject)
            {
                sTime.tv_sec = timeObject->valuedouble;
                sTime.tv_usec = 0;
                ESP_LOGI(TAG, "Time: %lld", sTime.tv_sec);
                settimeofday(&sTime, NULL);                
            }

        } 
        else 
        {
            err = ESP_ERR_INVALID_ARG;
        }
        cJSON_Delete(root);

    } else {
        err = ESP_ERR_INVALID_ARG;
    }

    return err;
}


//write JSON device data
char* data_convert_write(deviceData_t device_data)
{
    char* string = NULL;

    // Create cJSON object for the device
    cJSON *device = cJSON_CreateObject();
    cJSON_AddItemToObject(device,"id", cJSON_CreateNumber(device_data.id));
    cJSON_AddItemToObject(device,"name", cJSON_CreateString(device_data.name));
    cJSON_AddItemToObject(device,"status", cJSON_CreateString(device_data.status));
    cJSON_AddItemToObject(device,"battery", cJSON_CreateNumber(device_data.battery));
    cJSON_AddItemToObject(device,"temperature", cJSON_CreateNumber(device_data.temperature));
    cJSON_AddItemToObject(device,"wateringtype", cJSON_CreateNumber(device_data.wateringtype));

    // Create cJSON object for a new channel
    cJSON *channels = cJSON_AddArrayToObject(device,"channels");

    for (int32_t channelIndex = 0; channelIndex < CHANNELCOUNT; ++channelIndex)
    {
        cJSON *newChannel = cJSON_CreateObject();
        cJSON_AddItemToObject(newChannel, "name", cJSON_CreateString(device_data.channels[channelIndex].name));
        if (device_data.channels[channelIndex].enable){
           cJSON_AddItemToObject(newChannel, "enable", cJSON_CreateTrue());
        }
        else{
            cJSON_AddItemToObject(newChannel, "enable", cJSON_CreateFalse());
        }
        // cJSON_AddItemToObject(newChannel, "duration", cJSON_CreateNumber(device_data.channels[index].duration));
        cJSON_AddItemToObject(newChannel, "frequency", cJSON_CreateNumber(device_data.channels[channelIndex].frequency));

        cJSON *moisture = cJSON_AddObjectToObject(newChannel, "moisture");
        cJSON_AddItemToObject(moisture, "senseEnable", cJSON_CreateBool(device_data.channels[channelIndex].moisture.senseEnable));
        cJSON_AddItemToObject(moisture, "maxMoisture", cJSON_CreateNumber(device_data.channels[channelIndex].moisture.maxMoisture));
        cJSON_AddItemToObject(moisture, "minMoisture", cJSON_CreateNumber(device_data.channels[channelIndex].moisture.minMoisture));
        cJSON *macTable = cJSON_AddArrayToObject(moisture, "macTable");
        for (int32_t macIndex = 0; macIndex < 6; macIndex++)
        {
            cJSON_AddItemToArray(macTable, cJSON_CreateNumber(device_data.channels[channelIndex].moisture.macTable[macIndex]));
        }

        // Create cJSON object for a new channel
        cJSON *events = cJSON_AddArrayToObject(newChannel,"events");
        for (int32_t eventIndex = 0; eventIndex < EVENTCOUNT; ++eventIndex)
        {
            if ((eventIndex == 0) || (device_data.channels[channelIndex].events[eventIndex].amount > 0))
            {
                cJSON *newEvent = cJSON_CreateObject();
                cJSON_AddItemToObject(newEvent, "amount", cJSON_CreateNumber(device_data.channels[channelIndex].events[eventIndex].amount));
                cJSON_AddItemToObject(newEvent, "hour", cJSON_CreateNumber(device_data.channels[channelIndex].events[eventIndex].hour));
                cJSON_AddItemToObject(newEvent, "minute", cJSON_CreateNumber(device_data.channels[channelIndex].events[eventIndex].minute));

                char acTime[8] = {0};
                snprintf(acTime, sizeof(acTime), "%02d:%02d",
                         (int)device_data.channels[channelIndex].events[eventIndex].hour,
                         (int)device_data.channels[channelIndex].events[eventIndex].minute);
                cJSON_AddItemToObject(newEvent, "time", cJSON_CreateString(acTime));
                cJSON_AddItemToArray(events, newEvent);
            }
        }

        cJSON_AddItemToArray(channels, newChannel);
    }

    cJSON *deviceTop = cJSON_CreateObject();
    cJSON_AddItemToObject(deviceTop,"device", device);

    // Print the updated JSON object to a string
    // json_data = cJSON_Print(device);
    string = cJSON_PrintUnformatted(deviceTop);

    // Free cJSON objects and the JSON string
    cJSON_Delete(deviceTop);
    // free(updatedJsonString);

    return string;
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
                                data_convert_read(peDevice_data,caDeviceData);
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
                                data_convert_read(peDevice_data,caDeviceData);
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
                        ESP_LOGI(TAG, "peDevice_data: %d", (int)(peDevice_data->battery));
                        //generate json data from struct
                        strcpy(caDeviceData, data_convert_write(*peDevice_data));
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
                char* pacStartPos = &caDeviceData;
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
