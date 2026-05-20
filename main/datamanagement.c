/*
 * datamanagement.c
 *
 *  Created on: 07.10.2023
 *      Author: tobby
 */

#include "datamanagement.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static const char *TAG = "DMGMT";

static bool prv_parseMacString(const char *pacMac, uint8_t aui8Mac[6])
{
    if (pacMac == NULL || aui8Mac == NULL)
    {
        return false;
    }

    unsigned int auiMac[6];
    int i32Cnt = sscanf(pacMac, "%2x:%2x:%2x:%2x:%2x:%2x",
                        &auiMac[0], &auiMac[1], &auiMac[2],
                        &auiMac[3], &auiMac[4], &auiMac[5]);
    if (i32Cnt != 6)
    {
        i32Cnt = sscanf(pacMac, "%2x-%2x-%2x-%2x-%2x-%2x",
                        &auiMac[0], &auiMac[1], &auiMac[2],
                        &auiMac[3], &auiMac[4], &auiMac[5]);
    }
    if (i32Cnt != 6)
    {
        i32Cnt = sscanf(pacMac, "%2x%2x%2x%2x%2x%2x",
                        &auiMac[0], &auiMac[1], &auiMac[2],
                        &auiMac[3], &auiMac[4], &auiMac[5]);
    }
    if (i32Cnt != 6)
    {
        return false;
    }

    for (size_t uiI = 0u; uiI < 6u; uiI++)
    {
        aui8Mac[uiI] = (uint8_t)auiMac[uiI];
    }
    return true;
}

static bool prv_parseMacJson(cJSON *psMac, uint8_t aui8Mac[6])
{
    if (psMac == NULL || aui8Mac == NULL)
    {
        return false;
    }

    if (cJSON_IsString(psMac))
    {
        return prv_parseMacString(psMac->valuestring, aui8Mac);
    }

    if (cJSON_IsArray(psMac) && cJSON_GetArraySize(psMac) == 6)
    {
        for (size_t uiI = 0u; uiI < 6u; uiI++)
        {
            cJSON *psByte = cJSON_GetArrayItem(psMac, (int)uiI);
            if (!cJSON_IsNumber(psByte) || psByte->valueint < 0 || psByte->valueint > 255)
            {
                return false;
            }
            aui8Mac[uiI] = (uint8_t)psByte->valueint;
        }
        return true;
    }

    return false;
}

static void prv_formatMacString(const uint8_t aui8Mac[6], char *pacBuf, size_t uiBufSize)
{
    if (pacBuf == NULL || uiBufSize == 0u)
    {
        return;
    }

    snprintf(pacBuf, uiBufSize,
             "%02X:%02X:%02X:%02X:%02X:%02X",
             aui8Mac[0], aui8Mac[1], aui8Mac[2],
             aui8Mac[3], aui8Mac[4], aui8Mac[5]);
}

/* =========================================================================
 * data_getDeviceData
 * ========================================================================= */

esp_err_t data_getDeviceData(char *pacBuf, size_t uiBufSize)
{
    time_t   tTime      = time(NULL);
    float    fTemp      = fTemp_read();
    float    fBatt      = (float)ui32BattVolt_read();
    int32_t i32WatLev;
    erLevel_readPerc(&i32WatLev);

    int i32Len = snprintf(pacBuf, uiBufSize,
        "{"
          "\"TIME\":%lld,"
          "\"TEMP\":%.1f,"
          "\"BATT\":%.2f,"
          "\"WATLEV\":%lu"
        "}",
        (long long)tTime,
        fTemp,
        fBatt / 1000.0f,
        (unsigned long)i32WatLev);

    if (i32Len < 0 || (size_t)i32Len >= uiBufSize)
    {
        ESP_LOGE(TAG, "data_getDeviceData: Puffer zu klein");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGD(TAG, "DeviceData: %s", pacBuf);
    return ESP_OK;
}

/* =========================================================================
 * Header-gesteuertes CSV-Parsing für Periphery-Log
 * ========================================================================= */

typedef enum
{
    COL_TYPE_TIME,    /* Zeitstempel → wird JSON-Key          */
    COL_TYPE_CHAR,    /* erstes Zeichen → Zeichenwert         */
    COL_TYPE_SKIP,    /* Spalte ignorieren                    */
    COL_TYPE_MV,      /* int / 1000 → float, "%.3f"          */
    COL_TYPE_FLOAT1,  /* atof → "%.1f"                       */
    COL_TYPE_INT,     /* atoi → "%d"                         */
    COL_TYPE_BOOL,    /* atoi != 0 → "true"/"false"          */
} prv_col_type_t;

typedef struct
{
    const char     *pac_csv;   /* Spaltenname im CSV-Header              */
    const char     *pac_json;  /* JSON-Schlüssel (NULL bei TIME / SKIP)  */
    prv_col_type_t  e_type;
} prv_col_def_t;

static const prv_col_def_t sc_asColDefs[] = {
    { "time",                 NULL,      COL_TYPE_TIME   },
    { "message",              "EVENT",   COL_TYPE_CHAR   },
    { "logicVoltage",         NULL,      COL_TYPE_SKIP   },
    { "battVoltage",          "BATT",    COL_TYPE_MV     },
    { "solVoltage",           "SOL",     COL_TYPE_MV     },
    { "temperature",          "TEMP",    COL_TYPE_FLOAT1 },
    { "humidity 0",           "HUM1",    COL_TYPE_INT    },
    { "humidity 1",           "HUM2",    COL_TYPE_INT    },
    { "humidity 2",           "HUM3",    COL_TYPE_INT    },
    { "waterLevel",           "LEV",     COL_TYPE_INT    },
    { "waterEmpty",           "ETY",     COL_TYPE_INT    },
    { "selTouch",             "SEL",     COL_TYPE_INT    },
    { "pumpTouch",            "PUMP",    COL_TYPE_INT    },
    { "chargeStatus",         "CHRG",    COL_TYPE_BOOL   },
    { "mifloraTemperature1",  "MITEMP1", COL_TYPE_FLOAT1 },
    { "mifloraIlluminance1",  "MIILL1",  COL_TYPE_INT    },
    { "mifloraMoisture1",     "MIMOIS1", COL_TYPE_INT    },
    { "mifloraConductivity1", "MICOND1", COL_TYPE_INT    },
    { "mifloraTemperature2",  "MITEMP2", COL_TYPE_FLOAT1 },
    { "mifloraIlluminance2",  "MIILL2",  COL_TYPE_INT    },
    { "mifloraMoisture2",     "MIMOIS2", COL_TYPE_INT    },
    { "mifloraConductivity2", "MICOND2", COL_TYPE_INT    },
    { "mifloraTemperature3",  "MITEMP3", COL_TYPE_FLOAT1 },
    { "mifloraIlluminance3",  "MIILL3",  COL_TYPE_INT    },
    { "mifloraMoisture3",     "MIMOIS3", COL_TYPE_INT    },
    { "mifloraConductivity3", "MICOND3", COL_TYPE_INT    },
    { "airHumidity",          "AIRHUM",  COL_TYPE_FLOAT1 },
    { "airTemperature",       "AIRTEMP", COL_TYPE_FLOAT1 },
};

#define PRV_COL_DEF_COUNT  (sizeof(sc_asColDefs) / sizeof(sc_asColDefs[0]))
#define PRV_MAX_CSV_COLS   32u

/* Baut Map: sc_asColDefs[i] → CSV-Feldindex (-1 wenn Spalte fehlt).
 * pacHeader wird durch strtok modifiziert. */
static void prv_buildColMap(char *pacHeader, int8_t *pai8Map)
{
    for (size_t uiI = 0u; uiI < PRV_COL_DEF_COUNT; uiI++) { pai8Map[uiI] = -1; }

    char    *pacFields[PRV_MAX_CSV_COLS];
    uint8_t  ui8Cnt = 0u;
    char    *pacTok = strtok(pacHeader, ",");
    while (pacTok != NULL && ui8Cnt < PRV_MAX_CSV_COLS)
    {
        while (*pacTok == ' ') { pacTok++; }
        size_t uiLen = strlen(pacTok);
        while (uiLen > 0u && (pacTok[uiLen - 1u] == ' '  ||
                               pacTok[uiLen - 1u] == '\r' ||
                               pacTok[uiLen - 1u] == '\n'))
        {
            pacTok[--uiLen] = '\0';
        }
        pacFields[ui8Cnt++] = pacTok;
        pacTok = strtok(NULL, ",");
    }

    for (size_t uiDef = 0u; uiDef < PRV_COL_DEF_COUNT; uiDef++)
    {
        for (uint8_t uiCol = 0u; uiCol < ui8Cnt; uiCol++)
        {
            if (strcmp(sc_asColDefs[uiDef].pac_csv, pacFields[uiCol]) == 0)
            {
                pai8Map[uiDef] = (int8_t)uiCol;
                break;
            }
        }
    }
}

/* Wandelt eine CSV-Datenzeile anhand der Spalten-Map in ein JSON-Objekt um. */
static size_t prv_csvLineToJson(const char *pacLine, const int8_t *pai8Map,
                                char *pacBuf, size_t uiBufSize)
{
    char acCopy[512];
    strncpy(acCopy, pacLine, sizeof(acCopy) - 1u);
    acCopy[sizeof(acCopy) - 1u] = '\0';
    acCopy[strcspn(acCopy, "\r\n")] = '\0';

    char    *pacFields[PRV_MAX_CSV_COLS];
    uint8_t  ui8Cnt = 0u;
    char    *pacTok = strtok(acCopy, ",");
    while (pacTok != NULL && ui8Cnt < PRV_MAX_CSV_COLS)
    {
        while (*pacTok == ' ') { pacTok++; }
        pacFields[ui8Cnt++] = pacTok;
        pacTok = strtok(NULL, ",");
    }

    /* Zeitstempel (TIME-Spalte) ermitteln */
    long long llTime = 0LL;
    for (size_t uiDef = 0u; uiDef < PRV_COL_DEF_COUNT; uiDef++)
    {
        if (sc_asColDefs[uiDef].e_type == COL_TYPE_TIME &&
            pai8Map[uiDef] >= 0 && (uint8_t)pai8Map[uiDef] < ui8Cnt)
        {
            llTime = atoll(pacFields[(uint8_t)pai8Map[uiDef]]);
            break;
        }
    }
    if (llTime == 0LL) { return 0u; }

    size_t uiPos     = 0u;
    bool   bOverflow = false;
    bool   bFirst    = true;

#define APND(_fmt, ...) \
    do { \
        if (!bOverflow) { \
            int _n = snprintf(pacBuf + uiPos, uiBufSize - uiPos, (_fmt), ##__VA_ARGS__); \
            if (_n < 0 || (size_t)_n >= (uiBufSize - uiPos)) { bOverflow = true; } \
            else { uiPos += (size_t)_n; } \
        } \
    } while (0)

    APND("\"%lld\":{", llTime);

    for (size_t uiDef = 0u; uiDef < PRV_COL_DEF_COUNT; uiDef++)
    {
        const prv_col_def_t *psDef = &sc_asColDefs[uiDef];
        if (psDef->e_type == COL_TYPE_TIME || psDef->e_type == COL_TYPE_SKIP) { continue; }
        if (pai8Map[uiDef] < 0 || (uint8_t)pai8Map[uiDef] >= ui8Cnt)         { continue; }

        const char *pacVal = pacFields[(uint8_t)pai8Map[uiDef]];
        if (!bFirst) { APND(","); }
        bFirst = false;

        switch (psDef->e_type)
        {
            case COL_TYPE_CHAR:
                APND("\"%s\":\"%c\"", psDef->pac_json, pacVal[0]);
                break;
            case COL_TYPE_MV:
                APND("\"%s\":%.3f", psDef->pac_json, (float)atoi(pacVal) / 1000.0f);
                break;
            case COL_TYPE_FLOAT1:
                APND("\"%s\":%.1f", psDef->pac_json, atof(pacVal));
                break;
            case COL_TYPE_INT:
                APND("\"%s\":%d", psDef->pac_json, atoi(pacVal));
                break;
            case COL_TYPE_BOOL:
                APND("\"%s\":%s", psDef->pac_json, atoi(pacVal) != 0 ? "true" : "false");
                break;
            default:
                break;
        }
    }

    APND("}");

#undef APND

    if (bOverflow) { return 0u; }
    return uiPos;
}

static size_t prv_csvWateringLineToJson(const char *pacLine, char *pacBuf, size_t uiBufSize)
{
    char acCopy[256];
    strncpy(acCopy, pacLine, sizeof(acCopy) - 1u);
    acCopy[sizeof(acCopy) - 1u] = '\0';
    acCopy[strcspn(acCopy, "\r\n")] = '\0';

    char   *pacFields[5] = {NULL};
    uint8_t ui8Cnt       = 0u;
    char   *pacToken     = strtok(acCopy, ",");
    while (pacToken != NULL && ui8Cnt < 5u)
    {
        while (*pacToken == ' ') { pacToken++; }
        pacFields[ui8Cnt++] = pacToken;
        pacToken = strtok(NULL, ",");
    }

    if (ui8Cnt < 5u) { return 0u; }

    long long llTime      = atoll(pacFields[0]);
    int       i32Channel  = atoi(pacFields[1]);
    int       i32Event    = atoi(pacFields[2]);
    int       i32Amount   = atoi(pacFields[3]);
    int       i32Humidity = atoi(pacFields[4]);

    int i32Len = snprintf(pacBuf, uiBufSize,
        "\"%lld\":{"
          "\"CHANNEL\":%d,"
          "\"EVENT\":%d,"
          "\"AMOUNT\":%d,"
          "\"HUMIDITY\":%d"
        "}",
        llTime, i32Channel, i32Event, i32Amount, i32Humidity);

    if (i32Len < 0 || (size_t)i32Len >= uiBufSize) { return 0u; }
    return (size_t)i32Len;
}

static size_t prv_csvErrorLineToJson(const char *pacLine, char *pacBuf, size_t uiBufSize)
{
    char acCopy[256];
    strncpy(acCopy, pacLine, sizeof(acCopy) - 1u);
    acCopy[sizeof(acCopy) - 1u] = '\0';
    acCopy[strcspn(acCopy, "\r\n")] = '\0';

    char *pacFirstComma = strchr(acCopy, ',');
    if (pacFirstComma == NULL) { return 0u; }
    *pacFirstComma = '\0';

    char *pacSecondComma = strchr(pacFirstComma + 1, ',');
    if (pacSecondComma == NULL) { return 0u; }
    *pacSecondComma = '\0';

    char *pacTime = acCopy;
    char *pacTag  = pacFirstComma + 1;
    char *pacMsg  = pacSecondComma + 1;

    while (*pacTag == ' ') { pacTag++; }
    while (*pacMsg == ' ') { pacMsg++; }

    long long llTime = atoll(pacTime);

    int i32Len = snprintf(pacBuf, uiBufSize,
        "\"%lld\":{"
          "\"TAG\":\"%s\","
          "\"MESSAGE\":\"%s\""
        "}",
        llTime, pacTag, pacMsg);

    if (i32Len < 0 || (size_t)i32Len >= uiBufSize) { return 0u; }
    return (size_t)i32Len;
}

/* =========================================================================
 * data_getPeripherieLogData
 * ========================================================================= */

esp_err_t data_getPeripherieLogData(char *pacBuf, size_t uiBufSize,
                                    uint32_t ui32LineIdx, uint32_t ui32Count)
{
    if (ui32Count == 0u) { return ESP_ERR_INVALID_ARG; }

    esp_vfs_spiffs_conf_t sConf = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,
        .max_files              = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t eRet = esp_vfs_spiffs_register(&sConf);
    if (eRet != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIFFS mount fehlgeschlagen: %s", esp_err_to_name(eRet));
        return ESP_FAIL;
    }

    FILE *f = fopen("/spiffs/log_periphery.csv", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Log-Datei nicht gefunden");
        esp_vfs_spiffs_unregister(NULL);
        return ESP_FAIL;
    }

    char   acLine[512];
    char   acHeader[512];
    int8_t ai8ColMap[PRV_COL_DEF_COUNT];

    /* Header lesen und Spalten-Map aufbauen */
    if (fgets(acHeader, sizeof(acHeader), f) == NULL)
    {
        ESP_LOGE(TAG, "Log-Datei leer");
        fclose(f);
        esp_vfs_spiffs_unregister(NULL);
        return ESP_FAIL;
    }
    prv_buildColMap(acHeader, ai8ColMap);

    /* Bis Startzeile vorspulen */
    for (uint32_t ui32I = 0u; ui32I < ui32LineIdx; ui32I++)
    {
        if (fgets(acLine, sizeof(acLine), f) == NULL)
        {
            ESP_LOGW(TAG, "Startzeile %lu nicht vorhanden", (unsigned long)ui32LineIdx);
            fclose(f);
            esp_vfs_spiffs_unregister(NULL);
            return ESP_ERR_NOT_FOUND;
        }
    }

    /* JSON-Objekt aufbauen: {"T1":{...},"T2":{...},...} */
    size_t uiPos    = 0u;
    pacBuf[uiPos++] = '{';

    for (uint32_t ui32I = 0u; ui32I < ui32Count; ui32I++)
    {
        if (fgets(acLine, sizeof(acLine), f) == NULL)
        {
            if (ui32I == 0u)
            {
                ESP_LOGW(TAG, "Keine Log-Einträge ab Zeile %lu", (unsigned long)ui32LineIdx);
                fclose(f);
                esp_vfs_spiffs_unregister(NULL);
                return ESP_ERR_NOT_FOUND;
            }
            break;
        }

        if (ui32I > 0u)
        {
            if (uiPos + 1u >= uiBufSize) { break; }
            pacBuf[uiPos++] = ',';
        }

        size_t uiWritten = prv_csvLineToJson(acLine,
                                             ai8ColMap,
                                             pacBuf + uiPos,
                                             uiBufSize - uiPos - 1u);
        if (uiWritten == 0u)
        {
            ESP_LOGW(TAG, "Zeile %lu: Parse-Fehler oder Puffer zu klein",
                     (unsigned long)(ui32LineIdx + ui32I));
            break;
        }
        uiPos += uiWritten;
    }

    fclose(f);
    esp_vfs_spiffs_unregister(NULL);

    if (uiPos + 1u > uiBufSize)
    {
        ESP_LOGE(TAG, "data_getPeripherieLogData: Puffer zu klein");
        return ESP_ERR_NO_MEM;
    }
    pacBuf[uiPos++] = '}';
    pacBuf[uiPos]   = '\0';

    ESP_LOGD(TAG, "PeripherieLog (%lu Einträge): %s", (unsigned long)ui32Count, pacBuf);
    return ESP_OK;
}

esp_err_t data_getWateringLogData(char *pacBuf, size_t uiBufSize,
                                  uint32_t ui32LineIdx, uint32_t ui32Count)
{
    if (ui32Count == 0u) { return ESP_ERR_INVALID_ARG; }

    esp_vfs_spiffs_conf_t sConf = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,
        .max_files              = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t eRet = esp_vfs_spiffs_register(&sConf);
    if (eRet != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIFFS mount fehlgeschlagen: %s", esp_err_to_name(eRet));
        return ESP_FAIL;
    }

    FILE *f = fopen("/spiffs/log_watering.csv", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Watering-Log-Datei nicht gefunden");
        esp_vfs_spiffs_unregister(NULL);
        return ESP_FAIL;
    }

    char acLine[256];

    if (fgets(acLine, sizeof(acLine), f) == NULL)
    {
        ESP_LOGE(TAG, "Watering-Log-Datei leer");
        fclose(f);
        esp_vfs_spiffs_unregister(NULL);
        return ESP_FAIL;
    }

    for (uint32_t ui32I = 0u; ui32I < ui32LineIdx; ui32I++)
    {
        if (fgets(acLine, sizeof(acLine), f) == NULL)
        {
            ESP_LOGW(TAG, "Startzeile %lu nicht vorhanden", (unsigned long)ui32LineIdx);
            fclose(f);
            esp_vfs_spiffs_unregister(NULL);
            return ESP_ERR_NOT_FOUND;
        }
    }

    size_t uiPos    = 0u;
    pacBuf[uiPos++] = '{';

    for (uint32_t ui32I = 0u; ui32I < ui32Count; ui32I++)
    {
        if (fgets(acLine, sizeof(acLine), f) == NULL)
        {
            if (ui32I == 0u)
            {
                ESP_LOGW(TAG, "Keine Watering-Log-Eintraege ab Zeile %lu", (unsigned long)ui32LineIdx);
                fclose(f);
                esp_vfs_spiffs_unregister(NULL);
                return ESP_ERR_NOT_FOUND;
            }
            break;
        }

        if (ui32I > 0u)
        {
            if (uiPos + 1u >= uiBufSize) { break; }
            pacBuf[uiPos++] = ',';
        }

        size_t uiWritten = prv_csvWateringLineToJson(acLine,
                                                     pacBuf + uiPos,
                                                     uiBufSize - uiPos - 1u);
        if (uiWritten == 0u)
        {
            ESP_LOGW(TAG, "Zeile %lu: Parse-Fehler oder Puffer zu klein",
                     (unsigned long)(ui32LineIdx + ui32I));
            break;
        }
        uiPos += uiWritten;
    }

    fclose(f);
    esp_vfs_spiffs_unregister(NULL);

    if (uiPos + 1u > uiBufSize)
    {
        ESP_LOGE(TAG, "data_getWateringLogData: Puffer zu klein");
        return ESP_ERR_NO_MEM;
    }
    pacBuf[uiPos++] = '}';
    pacBuf[uiPos]   = '\0';

    ESP_LOGD(TAG, "WateringLog (%lu Eintraege): %s", (unsigned long)ui32Count, pacBuf);
    return ESP_OK;
}

esp_err_t data_getErrorLogData(char *pacBuf, size_t uiBufSize,
                               uint32_t ui32LineIdx, uint32_t ui32Count)
{
    if (ui32Count == 0u) { return ESP_ERR_INVALID_ARG; }

    esp_vfs_spiffs_conf_t sConf = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,
        .max_files              = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t eRet = esp_vfs_spiffs_register(&sConf);
    if (eRet != ESP_OK)
    {
        ESP_LOGE(TAG, "SPIFFS mount fehlgeschlagen: %s", esp_err_to_name(eRet));
        return ESP_FAIL;
    }

    FILE *f = fopen("/spiffs/log_error.csv", "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Error-Log-Datei nicht gefunden");
        esp_vfs_spiffs_unregister(NULL);
        return ESP_FAIL;
    }

    char acLine[256];

    if (fgets(acLine, sizeof(acLine), f) == NULL)
    {
        ESP_LOGE(TAG, "Error-Log-Datei leer");
        fclose(f);
        esp_vfs_spiffs_unregister(NULL);
        return ESP_FAIL;
    }

    for (uint32_t ui32I = 0u; ui32I < ui32LineIdx; ui32I++)
    {
        if (fgets(acLine, sizeof(acLine), f) == NULL)
        {
            ESP_LOGW(TAG, "Startzeile %lu nicht vorhanden", (unsigned long)ui32LineIdx);
            fclose(f);
            esp_vfs_spiffs_unregister(NULL);
            return ESP_ERR_NOT_FOUND;
        }
    }

    size_t uiPos    = 0u;
    pacBuf[uiPos++] = '{';

    for (uint32_t ui32I = 0u; ui32I < ui32Count; ui32I++)
    {
        if (fgets(acLine, sizeof(acLine), f) == NULL)
        {
            if (ui32I == 0u)
            {
                ESP_LOGW(TAG, "Keine Error-Log-Eintraege ab Zeile %lu", (unsigned long)ui32LineIdx);
                fclose(f);
                esp_vfs_spiffs_unregister(NULL);
                return ESP_ERR_NOT_FOUND;
            }
            break;
        }

        if (ui32I > 0u)
        {
            if (uiPos + 1u >= uiBufSize) { break; }
            pacBuf[uiPos++] = ',';
        }

        size_t uiWritten = prv_csvErrorLineToJson(acLine,
                                                  pacBuf + uiPos,
                                                  uiBufSize - uiPos - 1u);
        if (uiWritten == 0u)
        {
            ESP_LOGW(TAG, "Zeile %lu: Parse-Fehler oder Puffer zu klein",
                     (unsigned long)(ui32LineIdx + ui32I));
            break;
        }
        uiPos += uiWritten;
    }

    fclose(f);
    esp_vfs_spiffs_unregister(NULL);

    if (uiPos + 1u > uiBufSize)
    {
        ESP_LOGE(TAG, "data_getErrorLogData: Puffer zu klein");
        return ESP_ERR_NO_MEM;
    }
    pacBuf[uiPos++] = '}';
    pacBuf[uiPos]   = '\0';

    ESP_LOGD(TAG, "ErrorLog (%lu Eintraege): %s", (unsigned long)ui32Count, pacBuf);
    return ESP_OK;
}

/* =========================================================================
 * data_setChannelData
 * ========================================================================= */

esp_err_t data_setChannelData(const char *pacJson, channelData_t *psChannels)
{
    if (pacJson == NULL || psChannels == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *psRoot = cJSON_Parse(pacJson);
    if (psRoot == NULL)
    {
        ESP_LOGE(TAG, "data_setChannelData: JSON-Parse fehlgeschlagen");
        return ESP_FAIL;
    }

    memset(psChannels, 0, sizeof(channelData_t) * CHANNELCOUNT);

    for (uint8_t ui8I = 0u; ui8I < CHANNELCOUNT; ui8I++)
    {
        char acKey[6];
        snprintf(acKey, sizeof(acKey), "CH%d", (int)(ui8I + 1u));

        cJSON *psCh = cJSON_GetObjectItem(psRoot, acKey);
        if (psCh == NULL) { continue; }

        channelData_t *psC = &psChannels[ui8I];

        cJSON *psName = cJSON_GetObjectItem(psCh, "NAME");
        if (cJSON_IsString(psName))
        {
            strncpy(psC->name, psName->valuestring, sizeof(psC->name) - 1u);
        }

        cJSON *psEn = cJSON_GetObjectItem(psCh, "EN");
        psC->enable = cJSON_IsTrue(psEn);

        cJSON *psFreq = cJSON_GetObjectItem(psCh, "FREQ");
        if (cJSON_IsNumber(psFreq))
        {
            psC->frequency = (int32_t)psFreq->valueint;
        }

        cJSON *psHum = cJSON_GetObjectItem(psCh, "HUM");
        if (cJSON_IsObject(psHum))
        {
            cJSON *psMax  = cJSON_GetObjectItem(psHum, "MAX");
            cJSON *psMin  = cJSON_GetObjectItem(psHum, "MIN");
            cJSON *psSens = cJSON_GetObjectItem(psHum, "SENS");
            cJSON *psMac  = cJSON_GetObjectItem(psHum, "MAC");

            if (cJSON_IsNumber(psMax)) { psC->moisture.maxMoisture = (int32_t)psMax->valueint; }
            if (cJSON_IsNumber(psMin)) { psC->moisture.minMoisture = (int32_t)psMin->valueint; }
            psC->moisture.senseEnable = cJSON_IsTrue(psSens);
            (void)prv_parseMacJson(psMac, psC->moisture.macTable);
        }

        cJSON *psEvents = cJSON_GetObjectItem(psCh, "EVENTS");
        if (cJSON_IsObject(psEvents))
        {
            int32_t i32EvIdx = 0;
            cJSON  *psEvt    = NULL;
            cJSON_ArrayForEach(psEvt, psEvents)
            {
                if (i32EvIdx >= (int32_t)EVENTCOUNT) { break; }

                cJSON *psAmount = cJSON_GetObjectItem(psEvt, "AMOUNT");
                cJSON *psTime   = cJSON_GetObjectItem(psEvt, "TIME");

                if (cJSON_IsNumber(psAmount))
                {
                    psC->events[i32EvIdx].amount = (int32_t)psAmount->valueint;
                }

                if (cJSON_IsString(psTime))
                {
                    int i32H = 0, i32M = 0;
                    sscanf(psTime->valuestring, "%d:%d", &i32H, &i32M);
                    psC->events[i32EvIdx].hour   = (int32_t)i32H;
                    psC->events[i32EvIdx].minute = (int32_t)i32M;
                }
                else if (cJSON_IsNumber(psTime))
                {
                    psC->events[i32EvIdx].hour   = (int32_t)psTime->valueint;
                    psC->events[i32EvIdx].minute = 0;
                }

                i32EvIdx++;
            }
        }

        ESP_LOGD(TAG, "CH%d: name=%s en=%d freq=%d hum[min=%ld max=%ld sens=%d]", (int)(ui8I + 1u),
             psC->name, (int)psC->enable, (int)psC->frequency,
             (long)psC->moisture.minMoisture,
             (long)psC->moisture.maxMoisture,
             (int)psC->moisture.senseEnable);
    }

    cJSON_Delete(psRoot);
    return ESP_OK;
}

/* =========================================================================
 * data_logDeviceData
 * ========================================================================= */

void data_logDeviceData(const deviceData_t *psData)
{
    if (psData == NULL) { return; }

    ESP_LOGI(TAG, "--- deviceData_t ---");
    ESP_LOGI(TAG, "  id=%ld  name=%s  status=%s",
             (long)psData->id, psData->name, psData->status);
    ESP_LOGI(TAG, "  battery=%ldmV  temperature=%.1f°C  wateringtype=%ld",
             (long)psData->battery,
             (float)psData->temperature / 10.0f,
             (long)psData->wateringtype);

    for (int i = 0; i < CHANNELCOUNT; i++)
    {
        const channelData_t *psC = &psData->channels[i];
        char acMac[18];
        prv_formatMacString(psC->moisture.macTable, acMac, sizeof(acMac));
        ESP_LOGI(TAG, "  CH%d: name=%s  en=%d  freq=%d  moisture(min=%ld max=%ld sens=%d)",
                 i + 1, psC->name, (int)psC->enable, (int)psC->frequency,
                 (long)psC->moisture.minMoisture, (long)psC->moisture.maxMoisture,
                 (int)psC->moisture.senseEnable);
        ESP_LOGI(TAG, "       hum-mac=%s", acMac);

        for (int j = 0; j < EVENTCOUNT; j++)
        {
            const eventData_t *psE = &psC->events[j];
            if (psE->amount == 0 && psE->hour == 0 && psE->minute == 0) { continue; }
            ESP_LOGI(TAG, "    EVT%d: %02ld:%02ld  %ldml",
                     j + 1, (long)psE->hour, (long)psE->minute, (long)psE->amount);
        }
    }
}

/* =========================================================================
 * data_getChannelData
 * ========================================================================= */

esp_err_t data_getChannelData(const channelData_t *psChannels, char *pacBuf, size_t uiBufSize)
{
    if (psChannels == NULL || pacBuf == NULL || uiBufSize == 0u)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *psRoot = cJSON_CreateObject();
    if (psRoot == NULL) { return ESP_FAIL; }

    for (int i = 0; i < CHANNELCOUNT; i++)
    {
        const channelData_t *psC = &psChannels[i];

        cJSON *psCh = cJSON_CreateObject();
        if (psCh == NULL) { cJSON_Delete(psRoot); return ESP_FAIL; }

        cJSON_AddStringToObject(psCh, "NAME",  psC->name);
        cJSON_AddBoolToObject  (psCh, "EN",    psC->enable);
        cJSON_AddNumberToObject(psCh, "FREQ",  (double)psC->frequency);

        /* HUM */
        cJSON *psHum = cJSON_CreateObject();
        if (psHum != NULL)
        {
            char acMac[18];
            prv_formatMacString(psC->moisture.macTable, acMac, sizeof(acMac));

            cJSON_AddNumberToObject(psHum, "MAX",  (double)psC->moisture.maxMoisture);
            cJSON_AddNumberToObject(psHum, "MIN",  (double)psC->moisture.minMoisture);
            cJSON_AddBoolToObject  (psHum, "SENS", psC->moisture.senseEnable);
            cJSON_AddStringToObject(psHum, "MAC",  acMac);
            cJSON_AddItemToObject  (psCh,  "HUM",  psHum);
        }

        /* EVENTS – nur Einträge mit gesetzten Werten */
        cJSON *psEvents  = cJSON_CreateObject();
        bool   bHasEvts  = false;
        if (psEvents != NULL)
        {
            for (int j = 0; j < EVENTCOUNT; j++)
            {
                const eventData_t *psE = &psC->events[j];
                if (psE->amount == 0 && psE->hour == 0 && psE->minute == 0) { continue; }

                char acEvtKey[8];
                snprintf(acEvtKey, sizeof(acEvtKey), "EVT%d", j + 1);

                cJSON *psEvt = cJSON_CreateObject();
                if (psEvt != NULL)
                {
                    char acTime[6];
                    snprintf(acTime, sizeof(acTime), "%02ld:%02ld",
                             (long)psE->hour, (long)psE->minute);

                    cJSON_AddNumberToObject(psEvt, "AMOUNT", (double)psE->amount);
                    cJSON_AddStringToObject(psEvt, "TIME",   acTime);
                    cJSON_AddItemToObject  (psEvents, acEvtKey, psEvt);
                    bHasEvts = true;
                }
            }

            if (bHasEvts) { cJSON_AddItemToObject(psCh, "EVENTS", psEvents); }
            else          { cJSON_Delete(psEvents); }
        }

        char acKey[6];
        snprintf(acKey, sizeof(acKey), "CH%d", i + 1);
        cJSON_AddItemToObject(psRoot, acKey, psCh);
    }

    char *pacJson = cJSON_PrintUnformatted(psRoot);
    cJSON_Delete(psRoot);

    if (pacJson == NULL)
    {
        ESP_LOGE(TAG, "data_getChannelData: cJSON_Print fehlgeschlagen");
        return ESP_FAIL;
    }

    size_t uiLen = strlen(pacJson);
    if (uiLen >= uiBufSize)
    {
        free(pacJson);
        ESP_LOGE(TAG, "data_getChannelData: Puffer zu klein (%u bytes needed)", (unsigned)uiLen + 1u);
        return ESP_ERR_NO_MEM;
    }

    memcpy(pacBuf, pacJson, uiLen + 1u);
    free(pacJson);

    ESP_LOGD(TAG, "ChannelData: %s", pacBuf);
    return ESP_OK;
}
