/*
 * watering.c
 *
 *  Created on: 23.08.2023
 *      Author: tobby
 */

#include "watering.h"

deviceData_t g_sDeviceData;
static deviceData_t deviceDataOld;
static wateringData_t s_eWateringData;
RTC_DATA_ATTR static time_t s_tLastServerUpdateUnix = 0;

const char * jsonData = "{\"device\":{\"id\":12345,\"name\":\"DeviceName\",\"status\":\"active\",\"battery\":45,\"temperature\":25.5,\"channels\":[{\"id\":1,\"name\":\"Channel 1\",\"description\":\"This is Channel 1\",\"enable\":true,\"duration\":10,\"frequency\":-3,\"events\":[{\"time\":20,\"amount\":20},{\"time\":2,\"amount\":30},{\"time\":5,\"amount\":25}]},{\"id\":2,\"name\":\"Channel 2\",\"description\":\"This is Channel 2\",\"enable\":true,\"duration\":15,\"frequency\":-3,\"events\":[{\"time\":22,\"amount\":10},{\"time\":6,\"amount\":15},{\"time\":12,\"amount\":20}]}]}}";
const char * jsonDataOld = "{\"device\":{\"id\":12345,\"name\":\"DeviceName\",\"status\":\"active\",\"battery\":45,\"temperature\":25.5,\"channels\":[{\"id\":1,\"name\":\"Channel 1\",\"description\":\"This is Channel 1\",\"enable\":true,\"duration\":10,\"frequency\":-2,\"events\":[{\"time\":22,\"amount\":20},{\"time\":5,\"amount\":25}]},{\"id\":2,\"name\":\"Channel 2\",\"description\":\"This is Channel 2\",\"enable\":false,\"duration\":15,\"frequency\":-3,\"events\":[{\"time\":22,\"amount\":10},{\"time\":6,\"amount\":15},{\"time\":12,\"amount\":20}]}]}}";

void button_task();
void setWateringTime(deviceData_t * deviceData, deviceData_t * deviceDataOld, wateringData_t* wateringData);
void wateringData2deviceData(deviceData_t * deviceData,  wateringData_t* wateringData);
void wateringTimeFirst(eventData_t * wateringData, wateringTime_t* wateringTime, time_t actTime);
void wateringTimeRecalc(eventData_t * wateringData, wateringTime_t* wateringTime, time_t actTime);
time_t setTimeToNextEvent(wateringData_t* wateringData);
esp_err_t erWatering(wateringData_t* wateringData);
esp_err_t erWateringMoisture(wateringData_t* wateringData, deviceData_t* channels);
esp_err_t erUpdateServerData(deviceData_t* psDeviceData);

void watering_init(void)
{
    button_sleep_init();
    xTaskCreate(button_task, "button1_event_task", 16384, NULL, 5, NULL);    										//1024 Create a task to handler button event from button state
}

deviceData_t eDeviceData_get(void){
    return g_sDeviceData;
}

void button_task(void)
{
    //uncomment for normal operaption - only for demo purposes
    led_set(1,LED_ON);
    char *data = (char*) malloc(4000*sizeof(char));
    ESP_LOGI("BTN_START", "Read dev storage data: %s", esp_err_to_name(storage_readDeviceJson(data)));
    data_convert_read(&g_sDeviceData, data);    
    ESP_LOGI("BTN_START", "Read wat storage data: %s", esp_err_to_name(storage_readWatering(&s_eWateringData)));
    //check watering data vs device data and repair if corrupted
    wateringData2deviceData(&deviceDataOld, &s_eWateringData);
    setWateringTime(&g_sDeviceData, &deviceDataOld, &s_eWateringData);
    ESP_LOGI("JSON", "Write wat storage data: %s", esp_err_to_name(storage_writeWatering(&s_eWateringData)));

    // ESP_LOGI("JSON", "Read data: %s",data);   
    free(data);
    //sync esp rtc with external rtc
    RTCExt_getUnixTime();        //uncommented
  
    // bt_prov(&deviceData); 

    while(1){
    ESP_LOGI("BUTTON_TASK", "Wakeup state: %d", (int)eWakeup_state());
    switch(eWakeup_state())
	{
	case WAKEUP_BTN_PRESSED_SHORT:
		{
            // led_switch(1, 1);
            // RTC_updateTime();
            // time_t tTimeAct= 1711481173;      //26.03.2024 20:26;        
            // settimeofday(&tTimeAct, NULL);
            // data_convert_read(&deviceData, jsonData);
            // data_convert_read(&deviceDataOld, jsonDataOld);
            // char *p = (char*) malloc(1000*sizeof(char));
            // ESP_LOGI("JSON", "Parse data: %s", data_convert_write( deviceData));
            // free(p);
            // setWateringTime(&deviceData, &deviceDataOld, &s_eWateringData);
            // erWatering(&s_eWateringData);
            // led_switch(1, 0);
            //RTCExt_setTime();

            deviceData_t deviceDataOld = g_sDeviceData;     
            g_sDeviceData.battery = ui32BattVolt_read();
            g_sDeviceData.temperature = (uint32_t)fTemp_read()*10;

            ESP_LOGI("BTN_SHORT", "Start BT Provisioning");
            bt_prov(&g_sDeviceData); 

            //demo mode
            // selector_setPos(1);
            // vTaskDelay(500 / portTICK_PERIOD_MS);
            // pump_runTime(3000, MOTOR_DIR_UP);    
            // vTaskDelay(500 / portTICK_PERIOD_MS);
            // selector_setPos(0);

            // led_switch(1, 0);
            //set unix time
            RTCExt_setUnixTime();
            // save data if changed - actual static data save
            setWateringTime(&g_sDeviceData, &deviceDataOld, &s_eWateringData);
            ESP_LOGI("BTN_SHORT", "Write dev storage data: %s", esp_err_to_name(storage_writeDeviceJson(data_convert_write(g_sDeviceData))));
            ESP_LOGI("JSON", "Write wat storage data: %s", esp_err_to_name(storage_writeWatering(&s_eWateringData)));
            deepSleep_activate(setTimeToNextEvent(&s_eWateringData) * 1000000);  //in µs - 100s  100000000              
		}
        break;

	case WAKEUP_BTN_PRESSED_MID:
		{
            led_set(1,LED_ON);
            // bt_prov(&deviceData);
            //manipulate time for testing purpose
            struct timeval sTime;
            sTime.tv_sec = s_eWateringData.wateringNextUnix - 10;
            sTime.tv_usec = 0;
            ESP_LOGI("BTN_MID", "Mid Press Btn WateringNextUnix: %lld",s_eWateringData.wateringNextUnix );
            ESP_LOGI("BTN_MID", "sTime: %lld",sTime.tv_sec);
            settimeofday(&sTime, NULL);
            RTCExt_setUnixTime();
            led_set(1,LED_OFF);
            time_t tTimeAct;        
            time(&tTimeAct);        //act unix time
            ESP_LOGI("BTN_MID", "Read time: %lld",tTimeAct);
            deepSleep_activate(setTimeToNextEvent(&s_eWateringData) * 1000000);  //in µs - 100s  100000000  
		}
        break;

 	case WAKEUP_TIMER:
		{
            led_set(1,LED_ON);     

            //time based watering event 
            // erWatering(&s_eWateringData);
            // deepSleep_activate(setTimeToNextEvent(&s_eWateringData) * 1000000);  //in µs - 100s   

            //moisture based watering event
            //update data sync with server every 1h
            static const time_t sc_tServerUpdateIntervalUnix =  1 * 60 * 60;       //1h in s -> 1 * 60 * 60
            time_t tTimeAct;
            time(&tTimeAct);
            if ((s_tLastServerUpdateUnix == 0) || ((tTimeAct - s_tLastServerUpdateUnix) >= sc_tServerUpdateIntervalUnix))
            {
                deviceData_t sDeviceDataOld = g_sDeviceData;
                ESP_LOGI("FSTR", "Starte Server-Update, letztes Update: %lld", (long long)s_tLastServerUpdateUnix);
                erUpdateServerData(&g_sDeviceData);
                //new data arrived
                if(memcmp(&g_sDeviceData, &sDeviceDataOld, sizeof(deviceData_t)) != 0)
                {
                    ESP_LOGI("FSTR", "Server-Update: neue Daten empfangen");        //save data
                    setWateringTime(&g_sDeviceData, &sDeviceDataOld, &s_eWateringData);
                    ESP_LOGI("BTN_SHORT", "Write dev storage data: %s", esp_err_to_name(storage_writeDeviceJson(data_convert_write(g_sDeviceData))));
                    ESP_LOGI("JSON", "Write wat storage data: %s", esp_err_to_name(storage_writeWatering(&s_eWateringData)));
                }
                ESP_LOGI("WAKE_TIMER", "Compare memory: %d", memcmp(&g_sDeviceData, &sDeviceDataOld, sizeof(deviceData_t)));   

                s_tLastServerUpdateUnix = tTimeAct;

            }

            erWateringMoisture(&s_eWateringData, &g_sDeviceData);  //watering
	        deepSleep_activate(20*60*1000000);		//log every 20 min data  
            led_set(1,LED_OFF);
            //proof watering data changed
                  
		}
        break;    

    case WAKEUP_IDLE:
        {
            led_set(1,LED_OFF);
            //go sleeping
            // deepSleep_activate(setTimeToNextEvent(&s_eWateringData) * 1000000);  //in µs - 100s  
            deepSleep_activate(10 * 1000000);  //in µs - 10s  
        }
        break;

    case WAKEUP_BTN_PRESSED_ACT:
        {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            //device reset
            // esp_restart();
        }
        break;

    default:
        //check wakeup time and go sleep
        deepSleep_activate(10 * 1000000);  //in µs - 10s  
        // deepSleep_activate(setTimeToNextEvent(&s_eWateringData) * 1000000);  //in µs - time to next event   
        break;
    }
    vTaskDelay(250);        
    }
}

// deepSleep_activate(uint64_t sleepTimeus);

void setWateringTime(deviceData_t * deviceData, deviceData_t * deviceDataOld, wateringData_t* wateringData)
{
    const time_t tDayUnix = 60*60*24;       //min, hour, day
    time_t tTimeAct;        
    time(&tTimeAct);        //act unix time
    // tTimeAct = 1711481173;      //26.03.2024 20:26
    setenv("TZ", TIMEZONE, 1);
	tzset();
    struct tm tmTimeAct;        //act tm time
    localtime_r(&tTimeAct, &tmTimeAct);
    ESP_LOGI("setWateringTime: ", "The current date/time in Germany is: %lld", tTimeAct);
    for (uint32_t i32ChanelCount = 0; i32ChanelCount < CHANNELCOUNT; i32ChanelCount++)
    {
        //if channel enabled
            for (uint32_t i32EventCount = 0; i32EventCount < EVENTCOUNT; i32EventCount++)
            {
                enum {
                WATERING_RECALC = 1,
                WATERING_CALCFIRST = 2,
                WATERING_DISABLE = 4, 
                };
                uint32_t  ui32changeState = 0;

                //set Watering amount
                (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringAmount = (*deviceData).channels[i32ChanelCount].events[i32EventCount].amount;

                //channel enable changed
                if((*deviceData).channels[i32ChanelCount].enable != (*deviceDataOld).channels[i32ChanelCount].enable)
                {
                    ESP_LOGI("setWateringTime: ", "Channel enable changed");
                    //switch on
                    if(((*deviceData).channels[i32ChanelCount].enable == true) && ((*deviceData).channels[i32ChanelCount].events[i32EventCount].amount > 0))
                    {
                        //calc first watering
                        ui32changeState |= WATERING_CALCFIRST;
                    }
                    //switch off
                    else{
                        //disable watering event
                        ui32changeState |= WATERING_DISABLE;
                    
                    }
                }

                //watering time changed
                if(((*deviceData).channels[i32ChanelCount].events[i32EventCount].hour != (*deviceDataOld).channels[i32ChanelCount].events[i32EventCount].hour) ||
                   ((*deviceData).channels[i32ChanelCount].events[i32EventCount].minute != (*deviceDataOld).channels[i32ChanelCount].events[i32EventCount].minute))
                {
                    ESP_LOGI("Time: ", "Watering time changed");
                    //recalc watering event
                    ui32changeState |= WATERING_RECALC;
                }
                //add watering event
                if(((*deviceData).channels[i32ChanelCount].events[i32EventCount].amount != 0) && ((*deviceDataOld).channels[i32ChanelCount].events[i32EventCount].amount == 0))
                {
                    ESP_LOGI("Time: ", "Add watering event");
                    //calc first watering
                    ui32changeState |= WATERING_CALCFIRST;
                }
                //delete watering event
                if(((*deviceData).channels[i32ChanelCount].events[i32EventCount].amount == 0) && ((*deviceDataOld).channels[i32ChanelCount].events[i32EventCount].amount != 0))
                {
                    ESP_LOGI("Time: ", "Delete watering event");
                    //disable watering event
                    ui32changeState |= WATERING_DISABLE;
                }

                //frequency changed
                if((*deviceData).channels[i32ChanelCount].frequency != (*deviceDataOld).channels[i32ChanelCount].frequency)
                {
                    ESP_LOGI("Time: ", "Frequency changed");
                    //recalc watering event
                    ui32changeState |= WATERING_RECALC;
                }

                //set watering state
                // if ((*deviceData).channels[i32ChanelCount].events[i32EventCount].amount > 0)
                // {
                //     //(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable = true;
                //     if((*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix == 0 || (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringLastUnix == 0 )   
                //     {
                //         ui32changeState |= WATERING_CALCFIRST;
                //     }     
                // }
                // else
                // {
                //     (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable = false;   
                // }

                //ESP_LOGI("setWateringTime", "Channel: %d - Event: %d - State: %d", (int)i32ChanelCount, (int)i32EventCount, (int)ui32changeState);
                //state changed
                if (ui32changeState != 0)
                {
                    //set watering frequency
                    (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringFreqUnix = (*deviceData).channels[i32ChanelCount].frequency * tDayUnix;
                    if ((*deviceData).channels[i32ChanelCount].frequency <= 0) (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringFreqUnix = tDayUnix; 
                    
                    //disable watering event
                    if (ui32changeState >= WATERING_DISABLE)
                    {
                        (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable = false;
                        (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringFreqUnix = 0;
                        (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringLastUnix = 0;
                        (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix = 0;                        
                    }
                    //calc first watering
                    else if(ui32changeState >= WATERING_CALCFIRST)
                    {
                        wateringTimeFirst(&((*deviceData).channels[i32ChanelCount].events[i32EventCount]), &(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount], tTimeAct);        
                        (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringLastUnix = 0;
                        (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable = true;               
                    }
                    //recalc watering
                    else if (ui32changeState >= WATERING_RECALC)
                    {
                        //recalc function
                        if ((*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringLastUnix != 0)
                        {
                            wateringTimeRecalc(&((*deviceData).channels[i32ChanelCount].events[i32EventCount]), &(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount], tTimeAct);
                        }
                        {
                            wateringTimeFirst(&((*deviceData).channels[i32ChanelCount].events[i32EventCount]), &(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount], tTimeAct);        
                        }

                    }
                }
                wateringTime_t wateringDataHelper = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount];
                ESP_LOGI("setWateringTime: ", "%ld %ld LastWatering: %lld NextWatering: %lld Frequency: %lld Amount: %ld Enable: %s", i32ChanelCount,i32EventCount,wateringDataHelper.wateringLastUnix, wateringDataHelper.wateringNextUnix,wateringDataHelper.wateringFreqUnix,wateringDataHelper.wateringAmount,wateringDataHelper.wateringEnable?"true":"false");

                if(!(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable)
                {
                    (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable = false;
                    // (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringFreqUnix = 0;
                    // (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringLastUnix = 0;
                    // (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix = 0;  
                }
            }                               
    }
}

void wateringData2deviceData(deviceData_t * deviceData,  wateringData_t* wateringData)
{
    const time_t tDayUnix = 60*60*24;       //min, hour, day

    setenv("TZ", TIMEZONE, 1);
	tzset();

    for (uint32_t i32ChanelCount = 0; i32ChanelCount < CHANNELCOUNT; i32ChanelCount++)
    {
        //if channel enabled
            bool bChannelEnable = false;
            
            for (uint32_t i32EventCount = 0; i32EventCount < EVENTCOUNT; i32EventCount++)
            {
                //set watering amount
                (*deviceData).channels[i32ChanelCount].events[i32EventCount].amount = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringAmount;
                
                //set watering enable
                if ((*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable)
                {
                    bChannelEnable = true;

                    //frequency
                    if (i32EventCount == 0)//first channel
                    {
                        (*deviceData).channels[i32ChanelCount].frequency = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringFreqUnix/tDayUnix;
                    }
                    else
                    {
    	                (*deviceData).channels[i32ChanelCount].frequency = 0;
                    }
                }  
                //set time
                struct tm tmWatTime;        //act tm time
                localtime_r(&((*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix), &tmWatTime);
                // (*deviceData).channels[i32ChanelCount].events[i32EventCount].hour = tmWatTime.tm_hour;      //not sure if this is correct - maybe better to save hour and minute in watering data struct
                // (*deviceData).channels[i32ChanelCount].events[i32EventCount].minute = tmWatTime.tm_min;
                ESP_LOGI("wateringData2deviceData: ", "wateringNext: %lld",(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix);
                ESP_LOGI("wateringData2deviceData: ", "channel: %ld event: %ld time: %ld:%ld amount: %ldml", i32ChanelCount, i32EventCount,(*deviceData).channels[i32ChanelCount].events[i32EventCount].hour, (*deviceData).channels[i32ChanelCount].events[i32EventCount].minute, (*deviceData).channels[i32ChanelCount].events[i32EventCount].amount);
            }
            (*deviceData).channels[i32ChanelCount].enable = bChannelEnable;
            ESP_LOGI("wateringData2deviceData: ", "channel: %ld frquency: %ld Enable: %s", i32ChanelCount,(*deviceData).channels[i32ChanelCount].frequency,(*deviceData).channels[i32ChanelCount].enable?"true":"false");
    }
}

void wateringTimeFirst(eventData_t * wateringData, wateringTime_t* wateringTime, time_t actTime)
{   
    // ESP_LOGI("wateringTimeFirst: ", "function called");
    struct tm tmTimeAct;    //act tm time
    localtime_r(&actTime, &tmTimeAct);
    // ESP_LOGI("wateringTimeFirst: ","actTime: %lld ", actTime);

    struct tm tmFirstWatering = tmTimeAct;
    tmFirstWatering.tm_sec = 0;
    tmFirstWatering.tm_min = (*wateringData).minute;
    tmFirstWatering.tm_hour = (*wateringData).hour;
    (*wateringTime).wateringNextUnix = mktime(&tmFirstWatering);
    // ESP_LOGI("wateringTimeFirst: ","wateringNextUnix: %lld ", (*wateringTime).wateringNextUnix);
    ESP_LOGI("wateringTimeFirst: ","WateringTime h: %d - timeAct h: %d ", tmFirstWatering.tm_hour, tmTimeAct.tm_hour);
    if (tmTimeAct.tm_hour >= (*wateringData).hour) (*wateringTime).wateringNextUnix +=  60*60*24;              //first watering event tomorrow / plus one day
    // ESP_LOGI("wateringTimeFirst: ","wateringNextUnix: %lld ", (*wateringTime).wateringNextUnix);
}

void wateringTimeRecalc(eventData_t * wateringData, wateringTime_t* wateringTime, time_t actTime)
{   
    //calc next theoretical watering event
    time_t tWateringNext = (*wateringTime).wateringLastUnix + (*wateringTime).wateringFreqUnix;
    //if next watering event is in the past
    if (tWateringNext < actTime + (60*5))
    {
        struct tm tmTimeAct;    //act tm time
        localtime_r(&actTime, &tmTimeAct);

        tmTimeAct.tm_sec = 0;
        tmTimeAct.tm_min = (*wateringData).minute;
        tmTimeAct.tm_hour = (*wateringData).hour;
        tWateringNext = mktime(&tmTimeAct) + (60*60*24);   //watering next day     
    }
    (*wateringTime).wateringNextUnix = tWateringNext;
}

//return time to next watering in s
time_t setTimeToNextEvent(wateringData_t* wateringData)
{
    time_t tTimeAct;        
    time(&tTimeAct);                //act unix time
    time_t tNextWatering = 0xFFFFFFFF;
    uint32_t ui32NextPos= 20;
    for (uint32_t i32ChanelCount = 0; i32ChanelCount < CHANNELCOUNT; i32ChanelCount++)
    {
        for (uint32_t i32EventCount = 0; i32EventCount < EVENTCOUNT; i32EventCount++)
        {
            //calc next watering timepoint
            wateringTime_t wateringDataHelper = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount];
            ESP_LOGI("setTimeToNextEvent", "%ld %ld - LastWatering: %lld NextWatering: %lld Frequency: %lld Enable: %s", i32ChanelCount,i32EventCount,wateringDataHelper.wateringLastUnix, wateringDataHelper.wateringNextUnix,wateringDataHelper.wateringFreqUnix,wateringDataHelper.wateringEnable?"true":"false");
            if((*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable)
            {
                // wateringTime_t wateringDataHelper = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount];
                // ESP_LOGI("setTimeToNextEvent", "%ld %ld - LastWatering: %lld NextWatering: %lld Frequency: %lld Enable: %s", i32ChanelCount,i32EventCount,wateringDataHelper.wateringLastUnix, wateringDataHelper.wateringNextUnix,wateringDataHelper.wateringFreqUnix,wateringDataHelper.wateringEnable?"true":"false");
                if((*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix < tNextWatering)
                {
                    tNextWatering = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix;
                    ui32NextPos = i32ChanelCount;
                }        
            }
        }
    }

    (*wateringData).wateringLastUnix = (*wateringData).wateringNextUnix;
    ESP_LOGI("setTimeToNextEvent: ","ui32NextPos %ld", ui32NextPos);
    ESP_LOGI("setTimeToNextEvent: ","tNextWatering %lld", tNextWatering);
    ESP_LOGI("setTimeToNextEvent: ","tTimeAct %lld", tTimeAct);

    //no watering set
    if (tNextWatering == 0xFFFFFFFF)
    {
        //sleep one day
        ESP_LOGI("setTimeToNextEvent: ","Next watering in 1day");
        (*wateringData).wateringNextUnix = tTimeAct + (60*60*24);
        ESP_LOGI("setTimeToNextEvent", "Write wat storage data: %s", esp_err_to_name(storage_writeWatering(&s_eWateringData)));  
        return 60*20; //log every 20 min data 
        //return 60*60*24;   //wait 1 day
    }

    //watering event pending
    if (tNextWatering <= (tTimeAct + 10))
    {
        ESP_LOGI("setTimeToNextEvent: ","Next watering in %ds", (10));
        (*wateringData).wateringNextUnix = tTimeAct + 10;
        ESP_LOGI("setTimeToNextEvent", "Write wat storage data: %s", esp_err_to_name(storage_writeWatering(&s_eWateringData)));      
        return 10;     //wake up in 10s an watering
    }
    //normal operation
    else
    {
        ESP_LOGI("setTimeToNextEvent: ","Next watering in %llds", (tNextWatering - tTimeAct));
        //next watering changed
        if(tNextWatering != (*wateringData).wateringNextUnix)
        {
           (*wateringData).wateringNextUnix = tNextWatering;
            ESP_LOGI("setTimeToNextEvent", "Write wat storage data: %s", esp_err_to_name(storage_writeWatering(&s_eWateringData)));      
        }
        return tNextWatering - tTimeAct;
        //  return 60*20; //log every 20 min data
    }

}

//watering time based
esp_err_t erWatering(wateringData_t* wateringData)
{
    //log the periphery data
    miflora_data_t paFloraData[CHANNELCOUNT];
    log_peripherieData(paFloraData);

    const bool bAvoidWaterlogging = true;       //enable to avoid watering if water is in pot
    bool bOverstepEvent = false;                //overstep watering event in case of water in pot
    esp_err_t error = ESP_OK;                   

    
    return error;       //delete after test

    //init powerstage
    bPowerstage_init();

    time_t tTimeAct;        
    time(&tTimeAct);        //act unix time
    
    ESP_LOGI("erWatering: ","wateringNextUnix: %lld", (*wateringData).wateringNextUnix);
    ESP_LOGI("erWatering: "," tTimeAct: %lld", tTimeAct);
    
    //check if watering event pending - time deviation < 10min
    if (tTimeAct > ((*wateringData).wateringNextUnix - (10*60)))
    {
        //if position > channel 5
            for (uint32_t i32ChanelCount = 0; i32ChanelCount < CHANNELCOUNT; i32ChanelCount++)
            {
                for (uint32_t i32EventCount = 0; i32EventCount < EVENTCOUNT; i32EventCount++)
                {
                    //if watering enable
                    if((*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringEnable)
                    {
                        //if event timepoint fits +-10min
                        if((*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix < (tTimeAct + (10*60)))
                        {
                            ESP_LOGI("erWatering: "," Channel: %d Event: %d", (int)i32ChanelCount, (int)i32EventCount);
                            
                            //set Positioning
                            esp_err_sel_t selError = ERR_SEL_OK;
                            if (bAvoidWaterlogging)
                            {
                                if (!bHumidity_check(i32ChanelCount + 1)) //check for no water in pot
                                {
                                    selError = selector_setPos(i32ChanelCount + 1);
                                }
                                else{
                                    bOverstepEvent = true;
                                    ESP_LOGI("erWatering: ","Overstep watering event - water in pot - Channel: %d Event: %d", (int)i32ChanelCount, (int)i32EventCount);
                                }
                            }
                            else
                            {
                                selError = selector_setPos(i32ChanelCount + 1);
                            }
                            
                            if(selError == ERR_SEL_OK)
                            {
                                volatile time_t tNextWatering;
                                if(!bOverstepEvent)
                                {
                                    //calc watering time quantity [ml/20] * pumpTimeFact
                                    const uint32_t ui32PumpTimeFact = (uint32_t)(0.4*1000);      //[ms/ml]
                                    const uint32_t ui32PumpPulseDuration = 5 * ui32PumpTimeFact + 500;     //5[ml] * ui32PumpTimeFact[ms/ml] + watering dead time (500ms)-> [ms] - pulse duration of watering cycles
                                    uint32_t ui32WaterTime = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringAmount * 20 * ui32PumpTimeFact;   //[ms]

                                    uint32_t ui32WateringDuration = 0;
                                    for (uint32_t i = 0; i < (ui32WaterTime/ui32PumpPulseDuration); i++)
                                    {
                                        //check for no water in pot
                                        if(!bHumidity_check(i32ChanelCount + 1) || !bAvoidWaterlogging)
                                        {
                                            pump_runTime(ui32WaterTime/(ui32WaterTime/ui32PumpPulseDuration), MOTOR_DIR_UP, 1);    
                                            ESP_LOGI("erWatering: ","No water in pot");
                                            ui32WateringDuration += ui32PumpPulseDuration;
                                            // vTaskDelay(10000);
                                            esp_sleep_enable_timer_wakeup(30 * 1000 * 1000);    //wait 30s
                                            esp_light_sleep_start();

                                        }
                                        else
                                        {
                                            ESP_LOGI("erWatering: ","Water in pot");
                                            break;
                                        }
                                    }
                                    ESP_LOGI("erWatering: "," ui32WaterTime: %ld", ui32WateringDuration); 
                                    

                                    //watering succeed? set last watering data
                                    (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringLastUnix = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix;                                  //set last watering data
                                    tNextWatering = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix + (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringFreqUnix;   //calc theoretical next event
                                }
                                else
                                {
                                    #ifndef CONFIG_PERIPHERY_VARIANT_LG
                                    log_wateringData(i32ChanelCount + 1, i32EventCount + 1, 0, ui32AdcTouch_readPwmMux(i32ChanelCount + 9, 100));
                                    #else
                                    log_wateringData(i32ChanelCount + 1, i32EventCount + 1, 0, 0);
                                    #endif  
                                    //if water in pot -> time delay for next watering 1d
                                    tNextWatering = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix + (24 * 60 * 60);      
                                }
                                                             
                                //check for watering event overstepped
                                ESP_LOGI("erWatering: ","tNextWatering: %lld", tNextWatering);
                                ESP_LOGI("erWatering: ","tTimeAct: %lld", tTimeAct);
                                if(tNextWatering  < (tTimeAct + (10*60)))
                                {
                                    wateringTimeFirst(&(g_sDeviceData.channels[i32ChanelCount].events[i32EventCount]), &(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount], tTimeAct); 
                                }
                                else
                                {
                                    (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix = tNextWatering;
                                }
                                ESP_LOGI("erWatering: ","wateringNextUnix: %lld", (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix);

                            }
                            else
                            {
                                ESP_LOGI("erWatering","Error code: %d", (int)selError);
                                vTaskDelay(500);
                                //do error stuff
                            }
                                                        //optional: run air
                            //error management
                            //if OK? set next watering timepoint
                        }                        
                    }
                    

                }
            }
        // else
        // {
        //     for (uint32_t i32ChanelCount = CHANNELCOUNT - 1; i32ChanelCount >= 0; i32ChanelCount--)
        //     {
        //         for (uint32_t i32EventCount = EVENTCOUNT - 1; i32EventCount >= EVENTCOUNT; i32EventCount--)
        //         {
                    
        //         }
        //     }
        // }

        //get selector position and set selector position to interstep
        selector_setPos(0);
        ESP_LOGI("erWatering: ","Watering event(s) processed");
        vTaskDelay(200);
    }

    return error;
}

//watering moisture based 
esp_err_t erWateringMoisture(wateringData_t* wateringData, deviceData_t* devData)
{
    esp_err_t err = ESP_OK;

    //log data
    miflora_data_t paFloraData[CHANNELCOUNT];
    log_peripherieData(paFloraData);       //todo check for valid data

    //check for moisture
    for (uint32_t ui32ChanelCount = 0; ui32ChanelCount < CHANNELCOUNT; ui32ChanelCount++)
    {
        channelData_t channelData = devData->channels[ui32ChanelCount];
        //check if channel enabled for watering
        if(channelData.enable)     
        {
            ESP_LOGI("erWateringMoisture: ","Channel:%d Enabled", (int)ui32ChanelCount);
            //check for valid humidity data
            ESP_LOGI("erWateringMoisture: ","Channel: %d Moisture:%d MaxMoisture: %d MinMoisture: %d", (int)ui32ChanelCount, (int)paFloraData[ui32ChanelCount].moisture, (int)channelData.moisture.maxMoisture, (int)channelData.moisture.minMoisture);
            if(paFloraData[ui32ChanelCount].valid)   
            { 
                //check if moisture is under defined threshold
                if(paFloraData[ui32ChanelCount].moisture < channelData.moisture.minMoisture)
                {
                    //set water output for channel
                    esp_err_t errSelector = selector_setPos(ui32ChanelCount + 1);

                    if(errSelector == ERR_SEL_OK)
                    {
                        //calc watering time quantity [ml/20] * pumpTimeFact
                        #if CONFIG_PERIPHERY_VARIANT_LG
                        const uint32_t ui32PumpTimeFact = (uint32_t)(0.6*1000);      //[ms/ml]
                        #else
                        const uint32_t ui32PumpTimeFact = (uint32_t)(0.4*1000);      //[ms/ml]
                        #endif
                        const uint32_t ui32PumpPulseDuration = 5 * ui32PumpTimeFact + 500;     //5[ml] * ui32PumpTimeFact[ms/ml] + watering dead time (500ms)-> [ms] - pulse duration of watering cycles
                        uint32_t ui32WateringDuration = 0;
                        bool bVertilizing = true;

                        while((paFloraData[ui32ChanelCount].moisture < channelData.moisture.maxMoisture))
                        {
                            //limit watering to max 100ml
                            if (ui32WateringDuration >= (100 * ui32PumpTimeFact))
                            {
                                ESP_LOGI("erWatering: ","Max watering time reached - Channel: %d", (int)ui32ChanelCount);
                                log_errorData(LOG_ERR_TYPE_WATERING, "Max watering time reached");           //log error - max watering time reached
                                break;
                            }

                            #ifdef VERTILIZING_ENABLE 
                            if(bVertilizing)
                            {
                                const uint32_t c_aui32VertAmount[3] = {0, 50, 100};            //[µl/l] - vertilizing amount for each channel
                                const uint32_t ui32SoilTimeFact = (uint32_t)(2);      //[ms/µl]

                                //make soil wet
                                pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_DOWN, 1);          //run pump for defined time in vertilizing direction
                                esp_sleep_enable_timer_wakeup(30 * 1000 * 1000);    //wait 30s
                                esp_light_sleep_start();
                                pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_DOWN, 1);          //run pump for defined time in vertilizing direction
                                esp_sleep_enable_timer_wakeup(30 * 1000 * 1000);    //wait 30s
                                esp_light_sleep_start();     

                                //vertilize and flush out with clean water
                                pump_runTimeOpenLoop(ui32SoilTimeFact * c_aui32VertAmount[ui32ChanelCount], MOTOR_DIR_UP, 2, 3000);          //run pump for defined time in vertilizing direction
                                ui32WateringDuration += ui32PumpPulseDuration * 2;

                                bVertilizing = false;
                            }
                            #endif

                            pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_UP, 1);          //run pump for defined time
                            ui32WateringDuration += ui32PumpPulseDuration;
                            esp_sleep_enable_timer_wakeup(30 * 1000 * 1000);    //wait 30s
                            esp_light_sleep_start();
                            log_peripherieData(paFloraData);       //todo check for valid data

                            if (!paFloraData[ui32ChanelCount].valid || (paFloraData[ui32ChanelCount].moisture == 0))
                            {
                                ESP_LOGI("MiData: ","No valid humidity data - Channel: %d", (int)ui32ChanelCount);
                                log_errorData(LOG_ERR_TYPE_WATERING, "No valid MIMoisture data");           //log error - no valid humidity data
                                break;                        
                            }
                        }
                        //log watering data
                        log_wateringData(ui32ChanelCount + 1, 0, ui32WateringDuration/ui32PumpTimeFact, paFloraData[ui32ChanelCount].moisture);
                    }
                    else
                    {
                        ESP_LOGI("Selector","Positioning error Channel: %d", (int)ui32ChanelCount);
                        log_errorData(LOG_ERR_TYPE_WATERING, "Selector positioning error");           //log error - selector positioning error
                    }

                    //set selector position to 0
                    selector_setPos(0);
                    ESP_LOGI("erWatering: ","Watering event(s) processed");
                    vTaskDelay(200);
                } 
            }
            else
            {
                ESP_LOGI("MiData: ","No valid humidity data - Channel: %d", (int)ui32ChanelCount);
                log_errorData(LOG_ERR_TYPE_WATERING, "No valid MIMoisture data");           //log error - no valid humidity data
            }
        }
    }
    return err;
}

//read/write server data - every 4h
esp_err_t erUpdateServerData(deviceData_t* psDeviceData)
{
    esp_err_t err = ESP_OK;

    // credentials_t sCredentials = {"FRITZ!Box 6660 Cable CR", "98543695872303115150", "tobbyb@gmx.net", "TobiObi"};
	// if (storage_writeCredentials(&sCredentials)== ESP_OK) 
	// {
	// 	ESP_LOGI("CRED", "Anmeldedaten gespeichert");
	// }
	// else
	// {
	// 	ESP_LOGW("CRED", "Schreiben der Anmeldedaten fehlgeschlagen");
	// }

	//get credentials from NVS
	credentials_t sCredentials = {};
    err = storage_readCredentials(&sCredentials);
	if (err == ESP_OK) 
	{
	/* ---- Firebase Realtime Database Upload ---- */
        err = firestore_wifiConnect(sCredentials.wifiSsid, sCredentials.wifiPassword);
		if (err == ESP_OK)
		{

			// Firebase Authentication (Email/Passwort) – ID-Token holen
            err = firestore_authenticate(sCredentials.firebaseEmail, sCredentials.firebasePassword);
			if (err == ESP_OK)
			{
				/* Zeit vom Firebase Server synchronisieren */
				firestore_syncTimeFromServer();

				/* Gerätedaten aus NVS laden und senden */
				deviceData_t sDevice = {0};
				char acJson[2048];

				
				if (data_getDeviceData(acJson, sizeof(acJson)) == ESP_OK)
				{
                    char acFirestorePath[128];

                    //write device data to firestore
                    snprintf(acFirestorePath, sizeof(acFirestorePath), "%s/DEVICE/DATA", sCredentials.deviceId);
					ESP_LOGI("FSTR", "Gelesene Gerätedaten: %s", acJson);
                    firestore_writeString(acFirestorePath, acJson);

                    // //write channel data to firestore
                    // if (data_getChannelData(sDevice.channels, acJson, sizeof(acJson)) == ESP_OK)
                    // {
                    // 	ESP_LOGI("FSTR", "Gelesene Kanaldaten: %s", acJson);
                    // 	firestore_writeString("LH_0000002/CHANNELS/", acJson);
                    // }
                    
                    //append peripherie log data to firestore from NVS and clear NVS log after successful upload
                    uint32_t ui32LineIdx = 0;
                    uint32_t ui32Count = 4; //number of log lines to read
                    esp_err_t eFirestoreErr = ESP_OK;
                    snprintf(acFirestorePath, sizeof(acFirestorePath), "%s/LOG/PERIPHERIE", sCredentials.deviceId);
                    while (data_getPeripherieLogData(acJson, sizeof(acJson), ui32LineIdx, ui32Count) == ESP_OK)
                    {	
                            ESP_LOGI("FSTR", "Gelesene Peripherie-Log-Daten: %s", acJson);
                            eFirestoreErr = firestore_patchString(acFirestorePath, acJson);
                            ui32LineIdx += ui32Count;
                    }

                    // //clear Periphery NVS log after successful upload
                    if (eFirestoreErr == ESP_OK)
                    {
                    	log_clearData(LOG_TYPE_PERIPHERY);
                    }

                    //append watering log data to firestore from NVS and clear NVS log after successful upload
                    ui32LineIdx = 0;
                    snprintf(acFirestorePath, sizeof(acFirestorePath), "%s/LOG/WATERING", sCredentials.deviceId);
                    while (data_getWateringLogData(acJson, sizeof(acJson), ui32LineIdx, ui32Count) == ESP_OK)
                    {	
                            ESP_LOGI("FSTR", "Gelesene Bewässerungs-Log-Daten: %s", acJson);
                            eFirestoreErr = firestore_patchString(acFirestorePath, acJson);
                            ui32LineIdx += ui32Count;
                    }

                    // //clear Watering NVS log after successful upload
                    if (eFirestoreErr == ESP_OK)
                    {
                    	log_clearData(LOG_TYPE_WATERING);
                    }

                    //append error log data to firestore from NVS and clear NVS log after successful upload
                    ui32LineIdx = 0;
                    snprintf(acFirestorePath, sizeof(acFirestorePath), "%s/LOG/ERROR", sCredentials.deviceId);
                    while (data_getErrorLogData(acJson, sizeof(acJson), ui32LineIdx, ui32Count) == ESP_OK)
                    {	
                            ESP_LOGI("FSTR", "Gelesene Fehler-Log-Daten: %s", acJson);
                            eFirestoreErr = firestore_patchString(acFirestorePath, acJson);
                            ui32LineIdx += ui32Count;
                    }

                    // //clear Error NVS log after successful upload
                    if (eFirestoreErr == ESP_OK)
                    {
                    	log_clearData(LOG_TYPE_ERROR);
                    }

                    //read channel data from firestore and write to NVS if changed
                    snprintf(acFirestorePath, sizeof(acFirestorePath), "%s/CHANNELS", sCredentials.deviceId);
                    if (firestore_readString(acFirestorePath, acJson, sizeof(acJson)) == ESP_OK)
                    {
                        data_setChannelData(acJson, psDeviceData->channels);
                        ble_miflora_setChannelData(psDeviceData);
                        data_logDeviceData(psDeviceData);
                    }
				}
			}
            else
            {
                ESP_LOGW("FSTR", "Firebase Authentifizierung fehlgeschlagen");
                return err;
            }

			firestore_wifiDisconnect();
		}
        else{
            ESP_LOGW("WIFI", "Verbindung zum WLAN fehlgeschlagen");
            return err;
        }
	}
	else
	{
		ESP_LOGW("CRED", "Lesen der Credentials fehlgeschlagen");
        return err;
	}

    return err;
}