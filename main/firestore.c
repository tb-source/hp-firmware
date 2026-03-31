/*
 * firestore.c
 *
 *  Created on: 31.03.2026
 *      Author: tobby
 *
 *  Firestore REST API Client (HTTPS POST)
 *  Dokumentation: https://firebase.google.com/docs/firestore/reference/rest
 *
 *  Authentifizierung: Firebase Web-API-Key als URL-Parameter.
 *  Sicherheit: Zugriff über Firebase Security Rules einschränken.
 */

#include "firestore.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

/* --------------------------------------------------------------------------
 * Interne Konstanten
 * -------------------------------------------------------------------------- */
static const char *sc_pacTAG = "FIRESTORE";

/** Basis-URL der Firestore REST API */
#define FIRESTORE_BASE_URL \
    "https://firestore.googleapis.com/v1/projects/" \
    FIRESTORE_PROJECT_ID \
    "/databases/(default)/documents"

/** Größe des JSON-Sendepuffers */
#define FIRESTORE_TX_BUF_SIZE   2048u

/** Größe des HTTP-Antwortpuffers */
#define FIRESTORE_RX_BUF_SIZE   512u

/** HTTP-Statuscode für erfolgreiches Anlegen eines Dokuments */
#define FIRESTORE_HTTP_OK       200

/* --------------------------------------------------------------------------
 * Interne Hilfsfunktionen
 * -------------------------------------------------------------------------- */

/**
 * Aktuellen Zeitstempel als ISO-8601-String schreiben (UTC).
 * Format: "2024-01-01T12:00:00Z"
 * @param pacBuf  Zielpuffer (mindestens 21 Zeichen).
 * @param uiBufLen Puffergröße.
 */
static void prv_getTimestampISO8601(char *pacBuf, size_t uiBufLen)
{
    time_t tNow = time(NULL);
    struct tm sUtc;
    gmtime_r(&tNow, &sUtc);
    strftime(pacBuf, uiBufLen, "%Y-%m-%dT%H:%M:%SZ", &sUtc);
}

/**
 * HTTP-Event-Handler – puffert nur den HTTP-Statuscode für die Auswertung.
 * Antwort-Body wird verworfen, da nur der Status relevant ist.
 */
static esp_err_t prv_httpEventHandler(esp_http_client_event_t *psEvent)
{
    switch (psEvent->event_id)
    {
        case HTTP_EVENT_ERROR:
            ESP_LOGW(sc_pacTAG, "HTTP Fehler");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(sc_pacTAG, "Verbunden");
            break;
        case HTTP_EVENT_ON_DATA:
            /* Antwort-Body ignorieren – kein Logging um Log-Spam zu vermeiden */
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(sc_pacTAG, "Getrennt");
            break;
        default:
            break;
    }
    return ESP_OK;
}

/**
 * Generischer HTTPS-POST an eine Firestore-Collection.
 *
 * @param pacCollection  Collection-Name (z. B. "devices").
 * @param pacBody        JSON-Body im Firestore-Dokumentformat.
 * @return               ESP_OK bei HTTP 200, sonst Fehler.
 */
static esp_err_t prv_firestorePost(const char *pacCollection, const char *pacBody)
{
    char acUrl[256];
    snprintf(acUrl, sizeof(acUrl),
             FIRESTORE_BASE_URL "/%s?key=" FIRESTORE_API_KEY,
             pacCollection);

    esp_http_client_config_t sConfig = {
        .url                = acUrl,
        .method             = HTTP_METHOD_POST,
        .event_handler      = prv_httpEventHandler,
        .crt_bundle_attach  = esp_crt_bundle_attach,   /* Root-CA-Bundle verwenden */
        .timeout_ms         = 10000,
        .buffer_size        = FIRESTORE_RX_BUF_SIZE,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        ESP_LOGE(sc_pacTAG, "HTTP-Client-Init fehlgeschlagen");
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "Content-Type", "application/json");
    esp_http_client_set_post_field(hClient, pacBody, (int)strlen(pacBody));

    esp_err_t eErr = esp_http_client_perform(hClient);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(sc_pacTAG, "HTTP-Request fehlgeschlagen: %s", esp_err_to_name(eErr));
    }
    else
    {
        int i32Status = esp_http_client_get_status_code(hClient);
        if (i32Status == FIRESTORE_HTTP_OK || i32Status == 200)
        {
            ESP_LOGI(sc_pacTAG, "Dokument gespeichert [%s] HTTP %d", pacCollection, i32Status);
        }
        else
        {
            ESP_LOGW(sc_pacTAG, "Unerwarteter HTTP-Status %d [%s]", i32Status, pacCollection);
            eErr = ESP_FAIL;
        }
    }

    esp_http_client_cleanup(hClient);
    return eErr;
}

/* --------------------------------------------------------------------------
 * Öffentliche Funktionen
 * -------------------------------------------------------------------------- */

esp_err_t firestore_writeDeviceData(const deviceData_t *psData)
{
    if (psData == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char acTimestamp[24];
    prv_getTimestampISO8601(acTimestamp, sizeof(acTimestamp));

    /* Temperatur: intern in [°C/10] gespeichert → Umrechnung in °C */
    float fTemperature = (float)psData->temperature / 10.0f;

    char acBody[FIRESTORE_TX_BUF_SIZE];
    int i32Len = snprintf(acBody, sizeof(acBody),
        "{"
          "\"fields\":{"
            "\"id\":{\"integerValue\":\"%ld\"},"
            "\"name\":{\"stringValue\":\"%s\"},"
            "\"status\":{\"stringValue\":\"%s\"},"
            "\"battery_mv\":{\"integerValue\":\"%ld\"},"
            "\"temperature_c\":{\"doubleValue\":%.1f},"
            "\"timestamp\":{\"timestampValue\":\"%s\"}"
          "}"
        "}",
        (long)psData->id,
        psData->name,
        psData->status,
        (long)psData->battery,
        fTemperature,
        acTimestamp);

    if (i32Len < 0 || i32Len >= (int)sizeof(acBody))
    {
        ESP_LOGE(sc_pacTAG, "JSON-Puffer zu klein (deviceData)");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(sc_pacTAG, "Sende Gerätedaten: id=%ld temp=%.1f°C batt=%ldmV",
             (long)psData->id, fTemperature, (long)psData->battery);

    return prv_firestorePost(FIRESTORE_COL_DEVICE, acBody);
}

/* -------------------------------------------------------------------------- */

esp_err_t firestore_writeMifloraData(const miflora_data_t *psData, uint32_t ui32SensorNr)
{
    if (psData == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!psData->valid)
    {
        ESP_LOGW(sc_pacTAG, "MiFlora-Daten ungültig – kein Upload");
        return ESP_ERR_INVALID_STATE;
    }

    char acTimestamp[24];
    prv_getTimestampISO8601(acTimestamp, sizeof(acTimestamp));

    char acBody[FIRESTORE_TX_BUF_SIZE];
    int i32Len = snprintf(acBody, sizeof(acBody),
        "{"
          "\"fields\":{"
            "\"sensor_nr\":{\"integerValue\":\"%lu\"},"
            "\"temperature_c\":{\"doubleValue\":%.1f},"
            "\"illuminance_lux\":{\"integerValue\":\"%lu\"},"
            "\"moisture_pct\":{\"integerValue\":\"%u\"},"
            "\"conductivity_us\":{\"integerValue\":\"%u\"},"
            "\"battery_pct\":{\"integerValue\":\"%u\"},"
            "\"firmware\":{\"stringValue\":\"%s\"},"
            "\"timestamp\":{\"timestampValue\":\"%s\"}"
          "}"
        "}",
        (unsigned long)ui32SensorNr,
        psData->temperature,
        (unsigned long)psData->illuminance,
        (unsigned)psData->moisture,
        (unsigned)psData->conductivity,
        (unsigned)psData->battery,
        psData->firmware,
        acTimestamp);

    if (i32Len < 0 || i32Len >= (int)sizeof(acBody))
    {
        ESP_LOGE(sc_pacTAG, "JSON-Puffer zu klein (mifloraData)");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(sc_pacTAG, "Sende MiFlora #%lu: temp=%.1f°C feuchte=%u%% licht=%lu lux",
             (unsigned long)ui32SensorNr,
             psData->temperature,
             (unsigned)psData->moisture,
             (unsigned long)psData->illuminance);

    return prv_firestorePost(FIRESTORE_COL_MIFLORA, acBody);
}

/* -------------------------------------------------------------------------- */

esp_err_t firestore_writeWateringData(const wateringData_t *psData)
{
    if (psData == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char acTimestamp[24];
    prv_getTimestampISO8601(acTimestamp, sizeof(acTimestamp));

    esp_err_t eErrRet = ESP_OK;

    /* Jeden Kanal und jedes Ereignis einzeln speichern */
    for (int i32Ch = 0; i32Ch < CHANNELCOUNT; i32Ch++)
    {
        for (int i32Ev = 0; i32Ev < EVENTCOUNT; i32Ev++)
        {
            const wateringTime_t *psEvent =
                &psData->wateringChannel[i32Ch].wateringEvent[i32Ev];

            if (!psEvent->wateringEnable)
            {
                continue; /* Deaktivierte Ereignisse überspringen */
            }

            /* Bewässerungsmenge: intern in [ml * 20] → Umrechnung in ml */
            float fAmountMl     = (float)psEvent->wateringAmount     / 20.0f;
            float fAmountLastMl = (float)psEvent->wateringAmountLast / 20.0f;

            char acBody[FIRESTORE_TX_BUF_SIZE];
            int i32Len = snprintf(acBody, sizeof(acBody),
                "{"
                  "\"fields\":{"
                    "\"channel\":{\"integerValue\":\"%d\"},"
                    "\"event\":{\"integerValue\":\"%d\"},"
                    "\"last_unix\":{\"integerValue\":\"%lld\"},"
                    "\"next_unix\":{\"integerValue\":\"%lld\"},"
                    "\"freq_unix\":{\"integerValue\":\"%lld\"},"
                    "\"amount_ml\":{\"doubleValue\":%.2f},"
                    "\"amount_last_ml\":{\"doubleValue\":%.2f},"
                    "\"global_last_unix\":{\"integerValue\":\"%lld\"},"
                    "\"global_next_unix\":{\"integerValue\":\"%lld\"},"
                    "\"timestamp\":{\"timestampValue\":\"%s\"}"
                  "}"
                "}",
                i32Ch,
                i32Ev,
                (long long)psEvent->wateringLastUnix,
                (long long)psEvent->wateringNextUnix,
                (long long)psEvent->wateringFreqUnix,
                fAmountMl,
                fAmountLastMl,
                (long long)psData->wateringLastUnix,
                (long long)psData->wateringNextUnix,
                acTimestamp);

            if (i32Len < 0 || i32Len >= (int)sizeof(acBody))
            {
                ESP_LOGE(sc_pacTAG, "JSON-Puffer zu klein (wateringData ch%d ev%d)", i32Ch, i32Ev);
                eErrRet = ESP_ERR_NO_MEM;
                continue;
            }

            ESP_LOGI(sc_pacTAG, "Sende Bewässerung ch=%d ev=%d amount=%.1fml",
                     i32Ch, i32Ev, fAmountMl);

            esp_err_t eErr = prv_firestorePost(FIRESTORE_COL_WATERING, acBody);
            if (eErr != ESP_OK)
            {
                eErrRet = eErr; /* Fehler merken, aber restliche Einträge noch senden */
            }
        }
    }

    return eErrRet;
}
