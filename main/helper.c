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
            strncpy((*device_data).status , cJSON_GetObjectItem(device, "status")->valuestring, sizeof((*device_data).status ) - 1);
            (*device_data).battery = cJSON_GetObjectItem(device, "battery")->valueint;
            (*device_data).temperature = cJSON_GetObjectItem(device, "temperature")->valueint;

            ESP_LOGI(TAG, "Device id: %d", (int)(*device_data).id);
            ESP_LOGI(TAG, "Device name: %s", (*device_data).name);
            ESP_LOGI(TAG, "Device status: %s", (*device_data).status);
            ESP_LOGI(TAG, "Device batVolt: %d", (int)(*device_data).battery);
            ESP_LOGI(TAG, "Device temp: %d", (int)(*device_data).temperature);

            cJSON *channels = cJSON_GetObjectItem(device, "channels");

            if (cJSON_IsArray(channels)) {
                int num_channels = cJSON_GetArraySize(channels);

                for (int i = 0; i < num_channels; i++) {
                    //manage channel data
                    cJSON *channel = cJSON_GetArrayItem(channels, i);
                    (*device_data).channels[i].id = i + 1;//cJSON_GetObjectItem(channel, "id")->valueint;
                    strncpy((*device_data).channels[i].name, cJSON_GetObjectItem(channel, "name")->valuestring, sizeof((*device_data).channels[i].name) - 1);
                    strncpy((*device_data).channels[i].description, cJSON_GetObjectItem(channel, "description")->valuestring, sizeof((*device_data).channels[i].description) - 1);
                    (*device_data).channels[i].enable = cJSON_IsTrue(cJSON_GetObjectItem(channel, "enable"));
                    // (*device_data).channels[i].duration = cJSON_GetObjectItem(channel, "duration")->valueint;
                    (*device_data).channels[i].frequency = cJSON_GetObjectItem(channel, "frequency")->valueint;

                    //printing channel data
                    ESP_LOGI(TAG, "Channel ID: %d", (int)(*device_data).channels[i].id);
                    ESP_LOGI(TAG, "Name: %s", (*device_data).channels[i].name);
                    ESP_LOGI(TAG, "Description: %s", (*device_data).channels[i].description);
                    ESP_LOGI(TAG, "Enable: %s", (*device_data).channels[i].enable ? "true" : "false");
                    ESP_LOGI(TAG, "Frequency: %d", (int)(*device_data).channels[i].frequency);

                    //manage watering event data
                    //short bugfix
                    (*device_data).channels[i].events[0].amount = 0;
                    (*device_data).channels[i].events[1].amount = 0;
                    (*device_data).channels[i].events[2].amount = 0;
                    (*device_data).channels[i].events[3].amount = 0;
                    (*device_data).channels[i].events[4].amount = 0;
                    
                    cJSON *events = cJSON_GetObjectItem(channel, "events");
                    if (cJSON_IsArray(events)) {
                    int num_events = cJSON_GetArraySize(events);

                        for (int j = 0; j < num_events; j++) {
                            cJSON *event = cJSON_GetArrayItem(events, j);
                            (*device_data).channels[i].events[j].amount = cJSON_GetObjectItem(event, "amount")->valueint;
                            (*device_data).channels[i].events[j].time = cJSON_GetObjectItem(event, "time")->valueint;

                            //Printing event data
                            ESP_LOGI(TAG, "Amount: %d", (int)(*device_data).channels[i].events[j].amount);
                            ESP_LOGI(TAG, "Time: %d", (int)(*device_data).channels[i].events[j].time);
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

    esp_err_t err = ESP_OK;

    char* string = NULL;

    // Create cJSON object for the device
    cJSON *device = cJSON_CreateObject();
    cJSON_AddItemToObject(device,"id", cJSON_CreateNumber(device_data.id));
    cJSON_AddItemToObject(device,"name", cJSON_CreateString(device_data.name));
    cJSON_AddItemToObject(device,"status", cJSON_CreateString(device_data.status));
    cJSON_AddItemToObject(device,"battery", cJSON_CreateNumber(device_data.battery));
    cJSON_AddItemToObject(device,"temperature", cJSON_CreateNumber(device_data.temperature));

    // Create cJSON object for a new channel
    cJSON *channels = cJSON_AddArrayToObject(device,"channels");

    for (int32_t channelIndex = 0; channelIndex < CHANNELCOUNT; ++channelIndex)
    {
        cJSON *newChannel = cJSON_CreateObject();
        cJSON_AddItemToObject(newChannel, "id", cJSON_CreateNumber(device_data.channels[channelIndex].id));
        cJSON_AddItemToObject(newChannel, "name", cJSON_CreateString(device_data.channels[channelIndex].name));
        cJSON_AddItemToObject(newChannel, "description", cJSON_CreateString(device_data.channels[channelIndex].description));
        if (device_data.channels[channelIndex].enable){
           cJSON_AddItemToObject(newChannel, "enable", cJSON_CreateTrue());
        }
        else{
            cJSON_AddItemToObject(newChannel, "enable", cJSON_CreateFalse());
        }
        // cJSON_AddItemToObject(newChannel, "duration", cJSON_CreateNumber(device_data.channels[index].duration));
        cJSON_AddItemToObject(newChannel, "frequency", cJSON_CreateNumber(device_data.channels[channelIndex].frequency));

        // Create cJSON object for a new channel
        cJSON *events = cJSON_AddArrayToObject(newChannel,"events");
        for (int32_t eventIndex = 0; eventIndex < EVENTCOUNT; ++eventIndex)
        {
            if ((eventIndex == 0) || (device_data.channels[channelIndex].events[eventIndex].amount > 0))
            {
                cJSON *newEvent = cJSON_CreateObject();
                cJSON_AddItemToObject(newEvent, "amount", cJSON_CreateNumber(device_data.channels[channelIndex].events[eventIndex].amount));
                cJSON_AddItemToObject(newEvent, "time", cJSON_CreateNumber(device_data.channels[channelIndex].events[eventIndex].time));
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
    cJSON_Delete(device);
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