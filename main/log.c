/*
 * log.c
 *
 *  Created on: 13.12.2021
 *      Author: tobby
 */


#include "log.h"

static const char *TAG = "LOG";
static const char *sc_acFilePaths[] = {
    "/spiffs/log_periphery.csv",
    "/spiffs/log_watering.csv",
    "/spiffs/log_error.csv",
};

#ifdef MIFLORA_ENABLE
static const char *sc_acHeadline[] = {
    "time, message, logicVoltage, battVoltage, solVoltage, temperature, humidity 0, humidity 1, humidity 2, waterLevel, waterEmpty, chargeStatus, mifloraTemperature1, mifloraIlluminance1, mifloraMoisture1, mifloraConductivity1, mifloraTemperature2, mifloraIlluminance2, mifloraMoisture2, mifloraConductivity2, mifloraTemperature3, mifloraIlluminance3, mifloraMoisture3, mifloraConductivity3", 
    "time, wateringChannel, wateringEvent, wateringAmount, humidity",
    "time, errorTag, errorMessage",
};
#else
static const char *sc_acHeadline[] = {
    "time, message, logicVoltage, battVoltage, solVoltage, temperature, humidity 0, humidity 1, humidity 2, waterLevel, waterEmpty, chargeStatus", 
    "time, wateringChannel, wateringEvent, wateringAmount, humidity",
    "time, errorTag, errorMessage",
};
#endif

static const char *sc_acMessageTypes[] = {
    "P",
    "W",
    "E"
};

static const char *sc_acErrorTypes[] = {
    "Example 1",
    "Example 2",
    "Example 3"
};

void log_packData(char *acData);

void log_saveData(char acData[], log_type_t eType)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
        esp_spiffs_format(conf.partition_label);
        return;
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Check consistency of reported partition size info.
    if (used > total) {
        ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
        ret = esp_spiffs_check(conf.partition_label);
        // Could be also used to mend broken files, to clean unreferenced pages, etc.
        // More info at https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
            return;
        } else {
            ESP_LOGI(TAG, "SPIFFS_check() successful");
        }
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file and append data
    ESP_LOGI(TAG, "Opening file");

    FILE *f = fopen(sc_acFilePaths[eType], "a");

    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for appending");
        return;
    }

    fprintf(f, "%s", acData);

    fclose(f);
    ESP_LOGI(TAG, "File written");

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "SPIFFS unmounted");

}

//clear logging data
void log_clearData(log_type_t eType)
{
     ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
        esp_spiffs_format(conf.partition_label);
        return;
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Check consistency of reported partition size info.
    if (used > total) {
        ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
        ret = esp_spiffs_check(conf.partition_label);
        // Could be also used to mend broken files, to clean unreferenced pages, etc.
        // More info at https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
            return;
        } else {
            ESP_LOGI(TAG, "SPIFFS_check() successful");
        }
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file and append data
    ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen(sc_acFilePaths[eType], "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    fprintf(f, "%s\n", sc_acHeadline[eType]);

    fclose(f);
    ESP_LOGI(TAG, "File written");

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}

void log_readData(log_type_t eType)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
        esp_spiffs_format(conf.partition_label);
        return;
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Check consistency of reported partition size info.
    if (used > total) {
        ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
        ret = esp_spiffs_check(conf.partition_label);
        // Could be also used to mend broken files, to clean unreferenced pages, etc.
        // More info at https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
            return;
        } else {
            ESP_LOGI(TAG, "SPIFFS_check() successful");
        }
    }

    // Read data
    ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen(sc_acFilePaths[eType], "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }

    char line[256];
    ESP_LOGI(TAG, "Read line: $%s-START$", sc_acMessageTypes[eType]);
    while (fgets(line, sizeof(line), f) != NULL) 
    {
       // Process each line        
        line[strlen(line) - 1] = '\0';
        ESP_LOGI(TAG, "Read line: $%s$", line);
    }
    ESP_LOGI(TAG, "Read line: $%s-END$", sc_acMessageTypes[eType]);
    fclose(f);

    // All done, unmount partition and disable SPIFFS
    esp_vfs_spiffs_unregister(conf.partition_label);
    ESP_LOGI(TAG, "SPIFFS unmounted");
}

//pack peripherie data for logging
#ifdef MIFLORA_ENABLE
void log_peripherieData(void)
#else
void log_peripherieData(void)
#endif
{
    // char data[64];
    // char acdataString[64];

    // //first entry -> time
    // time_t tNow = time(NULL);        //64bits -> 8char
    // data[0] = tNow >> 56;
    // data[1] = tNow >> 48;
    // data[2] = tNow >> 40;
    // data[3] = tNow >> 32;
    // data[4] = tNow >> 24;
    // data[5] = tNow >> 16;
    // data[6] = tNow >> 8;
    // data[7] = tNow;

    // //second entry -> message tag
    // data[8] = 'P';  //L for log

    // //second entry -> esp voltage
    // uint32_t ui32LogicVoltage = ui32EspVolt_read();             //[mV]
    // data[10] = ui32LogicVoltage >> 8;
    // data[11] = ui32LogicVoltage;

    // //third entry -> battery voltage
    // uint32_t ui32BatteryVoltage = ui32BattVolt_read();             //[mV]
    // data[12] = ui32BatteryVoltage >> 8;
    // data[13] = ui32BatteryVoltage;

    // //fourth entry -> solar voltage
    // uint32_t ui32SolarVoltage = ui32SolarVolt_read();             //[mV]
    // data[14] = ui32SolarVoltage >> 8;
    // data[15] = ui32SolarVoltage;

    // //fifth entry -> temperature sensing
    // float fTemperature = fTemp_read() * 100;             //[°C/100]
    // uint32_t ui32Temperature = (uint32_t)fTemperature;             //convert to int
    // data[16] = ui32Temperature >> 8;
    // data[17] = ui32Temperature;

    // //sixth entry -> humidity sensing
    // data[18] = ((ui32Humidity_count(0) - 9500 ) / 10) >> 24;
    // data[19] = ((ui32Humidity_count(1) - 9500 ) / 10) >> 24;
    // data[20] = ((ui32Humidity_count(2) - 9500 ) / 10) >> 24;

    // //seventh entry -> water level
    // uint32_t ui32WaterLevel = ui32Level_read();             //read water level
    // data[21] = ui32WaterLevel >> 8;
    // data[22] = ui32WaterLevel;

    // sprintf(acdataString, "%s", data);
    // ESP_LOGI(TAG, "Packed data: %s", acdataString);

    char acData[512] = {NULL};
    char acAppendData[32] = {NULL};

    // //first entry -> time
    sprintf(acAppendData, "%lld,", (long long)time(NULL));
    strcat(acData, acAppendData);

    // //second entry -> message tag
    char cMessageTag = 'P';  //L for log
    sprintf(acAppendData, "%c,", cMessageTag);
    strcat(acData, acAppendData);

    //second entry -> esp voltage
    sprintf(acAppendData, "%d,", (int)ui32EspVolt_read());
    strcat(acData, acAppendData);

     //third entry -> battery voltage
    sprintf(acAppendData, "%d,", (int)ui32BattVolt_read());
    strcat(acData, acAppendData);

    //fourth entry -> solar voltage
    sprintf(acAppendData, "%d,", (int)ui32SolarVolt_read());
    strcat(acData, acAppendData);

    //fifth entry -> temperature sensing
    sprintf(acAppendData, "%.1f,", fTemp_read());
    strcat(acData, acAppendData);

    //sixth entry -> humidity sensing
    sprintf(acAppendData, "%d,%d,%d,", (int)ui32AdcTouch_readPwmMux(PWM_MUX_HUM1,100), (int)ui32AdcTouch_readPwmMux(PWM_MUX_HUM2,100), (int)ui32AdcTouch_readPwmMux(PWM_MUX_HUM3,100));
    // sprintf(acAppendData, "%d,%d,%d,", (int)FDC_getCap(1)/5243, (int)FDC_getCap(2)/5243, (int)FDC_getCap(3)/5243);
    strcat(acData, acAppendData);

    //seventh entry -> water level
    sprintf(acAppendData, "%d,", (int)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL,100));
    strcat(acData, acAppendData);

    //eighth entry -> water empty
    sprintf(acAppendData, "%d,", (int)ui32AdcTouch_readPwmMux(PWM_MUX_TANKETY,100));
    strcat(acData, acAppendData);

    // ninth entry -> charge status
    sprintf(acAppendData, "%d,", (int)ui32Charge_read());
    strcat(acData, acAppendData);

    #ifdef MIFLORA_ENABLE

    miflora_data_t pFloraData;
    ble_miflora_init();
    for (int i=0; i<3; i++)
    {
        ble_miflora_read(i, &pFloraData);

        sprintf(acAppendData, "%.1f,", pFloraData.temperature);
        strcat(acData, acAppendData);

        sprintf(acAppendData, "%d,", (int)pFloraData.illuminance);
        strcat(acData, acAppendData);

        sprintf(acAppendData, "%d,", (int)pFloraData.moisture);
        strcat(acData, acAppendData);

        if (i<2){
            sprintf(acAppendData, "%d,", (int)pFloraData.conductivity);
        }
        else{
             sprintf(acAppendData, "%d\n", (int)pFloraData.conductivity);
        }
        strcat(acData, acAppendData);
    }
    ble_miflora_deinit();

    #endif

    // Pack data for logging
    ESP_LOGI(TAG, "Pack periphery data: %s", acData);

    // Save packed data to file
    log_saveData(acData, LOG_TYPE_PERIPHERY);
}

//pack watering data for logging
void log_wateringData(uint32_t ui32WateringChannel, uint32_t ui32WateringEvent, uint32_t ui32WateringAmount, uint32_t ui32humidity)
{
    char acData[128] = {0};
    char acAppendData[32] = {0};

    sprintf(acAppendData, "%lld,", (long long)time(NULL));
    strcat(acData, acAppendData);

    sprintf(acAppendData, "%d,", (int)ui32WateringChannel);
    strcat(acData, acAppendData);

    sprintf(acAppendData, "%d,", (int)ui32WateringEvent);
    strcat(acData, acAppendData);

    sprintf(acAppendData, "%d,", (int)ui32WateringAmount);
    strcat(acData, acAppendData);

    sprintf(acAppendData, "%d\n", (int)ui32humidity);
    strcat(acData, acAppendData);

     // Save packed data to file
    log_saveData(acData, LOG_TYPE_WATERING);
}

//pack error data for logging
void log_errorData(log_error_type_t eErrorType, char* pacErrorMessage)
{
    char acData[128] = {0};;
    char acAppendData[64] = {0};

    // Pack data for logging
    sprintf(acAppendData, "%lld,", (long long)time(NULL));
    strcat(acData, acAppendData);

    sprintf(acAppendData, "%s,", sc_acErrorTypes[eErrorType]);
    strcat(acData, acAppendData);

    sprintf(acAppendData, "%s\n", pacErrorMessage);
    strcat(acData, acAppendData);

    // Save packed data to file
    log_saveData(acData, LOG_TYPE_ERROR);
}
