/*
 * log.c
 *
 *  Created on: 13.12.2021
 *      Author: tobby
 */


#include "log.h"
#include <stdarg.h>

static const char *TAG = "LOG";
static const char *sc_acFilePaths[] = {
    "/spiffs/log_periphery.csv",
    "/spiffs/log_watering.csv",
    "/spiffs/log_error.csv",
};

/* Compose periphery CSV headline from feature defines in periphery headers. */
#ifdef CAPHUMSENSE_ENABLE
#define LOG_HDR_HUMIDITY ", humidity 0, humidity 1, humidity 2"
#define LOG_HDR_WATER_EMPTY ", waterEmpty"
#define LOG_HDR_WATER_LEVEL ", waterLevel"
#define LOG_HDR_SEL ", selTouch"
#define LOG_HDR_PUMP ", pumpTouch"
#else
#define LOG_HDR_HUMIDITY ""
#define LOG_HDR_WATER_EMPTY ""
#define LOG_HDR_WATER_LEVEL ""
#define LOG_HDR_SEL ""
#define LOG_HDR_PUMP ""
#endif

#ifdef MIFLORA_ENABLE
#define LOG_HDR_MIFLORA \
    ", mifloraTemperature1, mifloraIlluminance1, mifloraMoisture1, mifloraConductivity1" \
    ", mifloraTemperature2, mifloraIlluminance2, mifloraMoisture2, mifloraConductivity2" \
    ", mifloraTemperature3, mifloraIlluminance3, mifloraMoisture3, mifloraConductivity3"
#else
#define LOG_HDR_MIFLORA ""
#endif

#ifdef AHT20_ENABLE
#define LOG_HDR_AHT20 ", airHumidity, airTemperature"
#else
#define LOG_HDR_AHT20 ""
#endif

static const char *sc_acHeadline[] = {
    "time, message, logicVoltage, battLevel, solVoltage, temperature"
    LOG_HDR_HUMIDITY
    LOG_HDR_WATER_LEVEL
    LOG_HDR_WATER_EMPTY
    LOG_HDR_SEL
    LOG_HDR_PUMP
    ", chargeStatus"
    LOG_HDR_MIFLORA
    LOG_HDR_AHT20,
    "time, wateringChannel, wateringEvent, wateringAmount, humidity",
    "time, errorTag, errorMessage",
};

#undef LOG_HDR_HUMIDITY
#undef LOG_HDR_WATER_EMPTY
#undef LOG_HDR_WATER_LEVEL
#undef LOG_HDR_SEL
#undef LOG_HDR_PUMP
#undef LOG_HDR_MIFLORA
#undef LOG_HDR_AHT20

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
void log_peripherieData(miflora_data_t paFloraData[])
{
    char acData[1024] = {0};
    size_t uiPos = 0u;
    bool bOverflow = false;

#define APPEND_LOG(_fmt, ...) \
    do { \
        if (!bOverflow && uiPos < sizeof(acData)) { \
            int _n = snprintf(acData + uiPos, sizeof(acData) - uiPos, (_fmt), __VA_ARGS__); \
            if (_n < 0 || (size_t)_n >= (sizeof(acData) - uiPos)) { \
                bOverflow = true; \
            } else { \
                uiPos += (size_t)_n; \
            } \
        } \
    } while (0)

    //time
    APPEND_LOG("%lld,", (long long)time(NULL));

    //message tag
    APPEND_LOG("%c,", 'P');

    //esp voltage
    APPEND_LOG("%d,", (int)ui32EspVolt_read());

    //battery level
    APPEND_LOG("%d,", (int)ui32BattLevel_read());

    //solar voltage
    APPEND_LOG("%d,", (int)ui32SolarVolt_read());

    //temperature sensing
    APPEND_LOG("%.1f,", fTemp_read());

    #ifdef CAPHUMSENSE_ENABLE
    //humidity sensing
    APPEND_LOG("%d,%d,%d,",
               (int)ui32AdcTouch_readPwmMux(PWM_MUX_HUM1,100),
               (int)ui32AdcTouch_readPwmMux(PWM_MUX_HUM2,100),
               (int)ui32AdcTouch_readPwmMux(PWM_MUX_HUM3,100));
    //water level
    APPEND_LOG("%d,", (int)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL,100));
    //water empty
    APPEND_LOG("%d,", (int)ui32AdcTouch_readPwmMux(PWM_MUX_TANKETY,100));
    //selector current
    APPEND_LOG("%d,", (int)ui32AdcTouch_readPwmMux(PWM_MUX_SEL,100));
    //pump current
    APPEND_LOG("%d,", (int)ui32AdcTouch_readPwmMux(PWM_MUX_PUMP,100));
    #endif

    //charge status
    APPEND_LOG("%d,", (int)ui32Charge_read());

    #ifdef MIFLORA_ENABLE

    miflora_data_t pFloraData;
    ble_miflora_init();
    for (int i=0; i<3; i++)
    {
        ble_miflora_read(i, &pFloraData);

        APPEND_LOG("%.1f,", pFloraData.temperature);
        APPEND_LOG("%d,", (int)pFloraData.illuminance);
        APPEND_LOG("%d,", (int)pFloraData.moisture);
        APPEND_LOG("%d,", (int)pFloraData.conductivity);
        paFloraData[i] = pFloraData;
    }
    ble_miflora_deinit();

    #endif

    // air humidity and temperature from AHT20
    #ifdef AHT20_ENABLE
    ahtData_t sAhtData;
    if (AHT20_read(&sAhtData) == ESP_OK)
    {
        APPEND_LOG("%.1f,", sAhtData.humidity);
        APPEND_LOG("%.1f", sAhtData.temperature);
    }
    else
    {
        APPEND_LOG("%s", "0,0");
    }
    #endif

    if (!bOverflow && uiPos > 0u)
    {
        if (acData[uiPos - 1u] == ',')
        {
            uiPos--;
            acData[uiPos] = '\0';
        }

        if (uiPos + 1u >= sizeof(acData))
        {
            bOverflow = true;
        }
        else
        {
            acData[uiPos++] = '\n';
            acData[uiPos] = '\0';
        }
    }

#undef APPEND_LOG

    if (bOverflow)
    {
        ESP_LOGE(TAG, "Pack periphery data overflow (buffer too small)");
        return;
    }

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
void log_errorData(log_error_type_t eErrorType, const char *pacErrorFormat, ...)
{
    char acMessage[160] = {0};
    char acData[256] = {0};
    va_list args;

    if (pacErrorFormat == NULL)
    {
        pacErrorFormat = "";
    }

    va_start(args, pacErrorFormat);
    (void)vsnprintf(acMessage, sizeof(acMessage), pacErrorFormat, args);
    va_end(args);

    const char *pacErrorTag = "Unknown";
    if ((int)eErrorType >= 0 && (int)eErrorType < (int)(sizeof(sc_acErrorTypes) / sizeof(sc_acErrorTypes[0])))
    {
        pacErrorTag = sc_acErrorTypes[eErrorType];
    }

    (void)snprintf(acData,
                   sizeof(acData),
                   "%lld,%s,%s\n",
                   (long long)time(NULL),
                   pacErrorTag,
                   acMessage);

    // Save packed data to file
    log_saveData(acData, LOG_TYPE_ERROR);
}
