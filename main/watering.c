/*
 * watering.c
 *
 *  Created on: 23.08.2023
 *      Author: tobby
 */

#include "watering.h"

static deviceData_t deviceData;
static deviceData_t deviceDataOld;
static wateringData_t s_eWateringData;

const char * jsonData = "{\"device\":{\"id\":12345,\"name\":\"DeviceName\",\"status\":\"active\",\"battery\":45,\"temperature\":25.5,\"channels\":[{\"id\":1,\"name\":\"Channel 1\",\"description\":\"This is Channel 1\",\"enable\":true,\"duration\":10,\"frequency\":-3,\"events\":[{\"time\":20,\"amount\":20},{\"time\":2,\"amount\":30},{\"time\":5,\"amount\":25}]},{\"id\":2,\"name\":\"Channel 2\",\"description\":\"This is Channel 2\",\"enable\":true,\"duration\":15,\"frequency\":-3,\"events\":[{\"time\":22,\"amount\":10},{\"time\":6,\"amount\":15},{\"time\":12,\"amount\":20}]}]}}";
const char * jsonDataOld = "{\"device\":{\"id\":12345,\"name\":\"DeviceName\",\"status\":\"active\",\"battery\":45,\"temperature\":25.5,\"channels\":[{\"id\":1,\"name\":\"Channel 1\",\"description\":\"This is Channel 1\",\"enable\":true,\"duration\":10,\"frequency\":-2,\"events\":[{\"time\":22,\"amount\":20},{\"time\":5,\"amount\":25}]},{\"id\":2,\"name\":\"Channel 2\",\"description\":\"This is Channel 2\",\"enable\":false,\"duration\":15,\"frequency\":-3,\"events\":[{\"time\":22,\"amount\":10},{\"time\":6,\"amount\":15},{\"time\":12,\"amount\":20}]}]}}";

void button_task();
void setWateringTime(deviceData_t * deviceData, deviceData_t * deviceDataOld, wateringData_t* wateringData);
void wateringData2deviceData(deviceData_t * deviceData,  wateringData_t* wateringData);
void wateringTimeFirst(eventData_t * wateringData, wateringTime_t* wateringTime, time_t actTime);
void wateringTimeRecalc(eventData_t * wateringData, wateringTime_t* wateringTime, time_t actTime);
time_t setTimeToNextEvent(wateringData_t* wateringData);
esp_err_t erWatering(wateringData_t* wateringData);

void watering_init(void)
{
    button_sleep_init();
    xTaskCreate(button_task, "button1_event_task", 10000, NULL, 5, NULL);    										//1024 Create a task to handler button event from button state
}

deviceData_t eDeviceData_get(void){
    return deviceData;
}

void button_task(void)
{
    led_set(1,LED_ON);
    char *data = (char*) malloc(4000*sizeof(char));
    ESP_LOGI("BTN_START", "Read dev storage data: %s", esp_err_to_name(device_read(data)));
    data_convert_read(&deviceData, data);    
    ESP_LOGI("BTN_START", "Read wat storage data: %s", esp_err_to_name(nvs_readWatering(&s_eWateringData)));
    //check watering data vs device data and repair if corrupted
    wateringData2deviceData(&deviceDataOld, &s_eWateringData);
    setWateringTime(&deviceData, &deviceDataOld, &s_eWateringData);
    ESP_LOGI("JSON", "Write wat storage data: %s", esp_err_to_name(nvs_writeWatering(&s_eWateringData)));

    // ESP_LOGI("JSON", "Read data: %s",data);   
    free(data);
    //sync esp rtc with external rtc
    RTCExt_getUnixTime();        //uncommented

    while(1){
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

            deviceData_t deviceDataOld = deviceData;
            deviceData.battery = ui32BattVolt_read();
            deviceData.temperature = (uint32_t)fTemp_read()*10;
            bt_prov(&deviceData);
            // led_switch(1, 0);
            //set unix time
            RTCExt_setUnixTime();
            // save data if changed - actual static data save
            setWateringTime(&deviceData, &deviceDataOld, &s_eWateringData);
            ESP_LOGI("BTN_SHORT", "Write dev storage data: %s", esp_err_to_name(device_write(data_convert_write(deviceData))));
            ESP_LOGI("JSON", "Write wat storage data: %s", esp_err_to_name(nvs_writeWatering(&s_eWateringData)));
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
            //watering event
            led_set(1,LED_ON);
            erWatering(&s_eWateringData);
            led_set(1,LED_OFF);
            //proof watering data changed
      
            //go sleeping
            deepSleep_activate(setTimeToNextEvent(&s_eWateringData) * 1000000);  //in µs - 100s   
		}
        break;    

    case WAKEUP_IDLE:
        {
            led_set(1,LED_OFF);
            //go sleeping
            deepSleep_activate(setTimeToNextEvent(&s_eWateringData) * 1000000);  //in µs - 100s  
        }
        break;

    default:
        //check wakeup time and go sleep
        deepSleep_activate(setTimeToNextEvent(&s_eWateringData) * 1000000);  //in µs - time to next event   
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
                if((*deviceData).channels[i32ChanelCount].events[i32EventCount].time != (*deviceDataOld).channels[i32ChanelCount].events[i32EventCount].time)
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
                (*deviceData).channels[i32ChanelCount].events[i32EventCount].time = tmWatTime.tm_hour; 
                ESP_LOGI("wateringData2deviceData: ", "wateringNext: %lld",(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix);
                ESP_LOGI("wateringData2deviceData: ", "channel: %ld event: %ld time: %ld amount: %ld", i32ChanelCount, i32EventCount,(*deviceData).channels[i32ChanelCount].events[i32EventCount].time,(*deviceData).channels[i32ChanelCount].events[i32EventCount].amount);
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
    tmFirstWatering.tm_min = 0;
    tmFirstWatering.tm_hour = (*wateringData).time;
    (*wateringTime).wateringNextUnix = mktime(&tmFirstWatering);
    // ESP_LOGI("wateringTimeFirst: ","wateringNextUnix: %lld ", (*wateringTime).wateringNextUnix);
    ESP_LOGI("wateringTimeFirst: ","WateringTime h: %d - timeAct h: %d ", tmFirstWatering.tm_hour, tmTimeAct.tm_hour);
    if (tmTimeAct.tm_hour >= (*wateringData).time) (*wateringTime).wateringNextUnix +=  60*60*24;              //first watering event tomorrow / plus one day
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
        tmTimeAct.tm_min = 0;
        tmTimeAct.tm_hour = (*wateringData).time;
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
        ESP_LOGI("setTimeToNextEvent", "Write wat storage data: %s", esp_err_to_name(nvs_writeWatering(&s_eWateringData)));  
        return 60*20; //log every 20 min data 
        //return 60*60*24;   //wait 1 day
    }

    //watering event pending
    if (tNextWatering <= (tTimeAct + 10))
    {
        ESP_LOGI("setTimeToNextEvent: ","Next watering in %ds", (10));
        (*wateringData).wateringNextUnix = tTimeAct + 10;
        ESP_LOGI("setTimeToNextEvent", "Write wat storage data: %s", esp_err_to_name(nvs_writeWatering(&s_eWateringData)));      
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
            ESP_LOGI("setTimeToNextEvent", "Write wat storage data: %s", esp_err_to_name(nvs_writeWatering(&s_eWateringData)));      
        }
        // return tNextWatering - tTimeAct;
         return 60*20; //log every 20 min data
    }

}

//watering
esp_err_t erWatering(wateringData_t* wateringData)
{
    //log the periphery data
    log_peripherieData();

    const bool bAvoidWaterlogging = false;       //enable to avoid watering if water is in pot
    bool bOverstepEvent = false;                //overstep watering event in case of water in pot
    esp_err_t error = ESP_OK;                   

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
                                if (bHumidity_check(i32ChanelCount)) //check for no water in pot
                                {
                                    selError = selector_setPos(i32ChanelCount + 1 ,  SELECTOR_1);
                                }
                                else{
                                    bOverstepEvent = true;
                                    ESP_LOGI("erWatering: ","Overstep watering event - water in pot - Channel: %d Event: %d", (int)i32ChanelCount, (int)i32EventCount);
                                }
                            }
                            else
                            {
                                selError = selector_setPos(i32ChanelCount + 1 ,  SELECTOR_1);
                            }
                            
                            if(selError == ERR_SEL_OK)
                            {
                                volatile time_t tNextWatering;
                                if(!bOverstepEvent)
                                {

                                    //calc watering time quantity [ml/20] * pumpTimeFact
                                    const uint32_t ui32PumpTimeFact = (uint32_t)(0.25*1000);      //[ms/ml]
                                    const uint32_t ui32PumpPulseDuration = 5 * ui32PumpTimeFact;     //10[ml] * ui32PumpTimeFact[ms/ml]-> [ms] - pulse duration of watering cycles
                                    uint32_t ui32WaterTime = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringAmount * 20 * ui32PumpTimeFact;   //[ms]

                                    uint32_t ui32WateringDuration = 0;
                                    for (uint32_t i = 0; i < (ui32WaterTime/ui32PumpPulseDuration); i++)
                                    {
                                        //check for no water in pot
                                        if(bHumidity_check(i32ChanelCount) || !bAvoidWaterlogging)
                                        {
                                            pump_runTime(ui32WaterTime/(ui32WaterTime/ui32PumpPulseDuration), MOTOR_DIR_UP);    
                                            ESP_LOGI("erWatering: ","No water in pot");
                                            ui32WateringDuration += ui32PumpPulseDuration;
                                            vTaskDelay(10000);
                                        }
                                        else
                                        {
                                            ESP_LOGI("erWatering: ","Water in pot");
                                            break;
                                        }
                                    }
                                    ESP_LOGI("erWatering: "," ui32WaterTime: %ld", ui32WateringDuration); 
                                    
                                    //log watering data
                                    log_wateringData(i32ChanelCount + 1, i32EventCount + 1, ui32WateringDuration/ui32PumpTimeFact);

                                    //watering succeed? set last watering data
                                    (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringLastUnix = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix;                                  //set last watering data
                                    tNextWatering = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix + (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringFreqUnix;   //calc theoretical next event
                                }
                                else
                                {
                                    //if water in pot -> time delay for next watering 1d
                                    tNextWatering = (*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount].wateringNextUnix + (24 * 60 * 60);      
                                }
                                                             
                                //check for watering event overstepped
                                ESP_LOGI("erWatering: ","tNextWatering: %lld", tNextWatering);
                                ESP_LOGI("erWatering: ","tTimeAct: %lld", tTimeAct);
                                if(tNextWatering  < (tTimeAct + (10*60)))
                                {
                                    wateringTimeFirst(&(deviceData.channels[i32ChanelCount].events[i32EventCount]), &(*wateringData).wateringChannel[i32ChanelCount].wateringEvent[i32EventCount], tTimeAct); 
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
        selector_setPos(0, SELECTOR_1);
        ESP_LOGI("erWatering: ","Watering event(s) processed");
        vTaskDelay(200);
    }

    //deinit powerstage
    // powerstage_deinit();

    // for (uint32_t i32AnalyserCount = 0; i32AnalyserCount < 512; i32AnalyserCount++)
    // {
    //     ESP_LOGI("Analyser","%d",(int)g_ai32Analyser[i32AnalyserCount]);
    // }

    return error;
}
