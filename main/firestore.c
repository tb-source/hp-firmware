/*
 * firestore.c
 *
 *  Created on: 31.03.2026
 *      Author: tobby
 *
 *  Firebase Realtime Database REST Client (HTTPS POST)
 *  Dokumentation: https://firebase.google.com/docs/database/rest/start
 *
 *  Authentifizierung: Firebase Email/Passwort (identitytoolkit.googleapis.com)
 *  Der ESP32 meldet sich mit Email + Passwort an und übergibt das ID-Token
 *  als ?auth=<token> URL-Parameter bei jedem Datenbank-Request.
 */

#include "firestore.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <sys/time.h>

/* --------------------------------------------------------------------------
 * Interne Konstanten
 * -------------------------------------------------------------------------- */
static const char *sc_pacTAG = "RTDB";

/** Basis-URL der Realtime Database */
#define RTDB_BASE_URL \
    "https://" FIRESTORE_PROJECT_ID "-default-rtdb.europe-west1.firebasedatabase.app"

/** Firebase Email/Passwort Auth Endpoint */
#define RTDB_AUTH_URL \
    "https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=" \
    FIRESTORE_API_KEY

/** Größe des JSON-Sendepuffers */
#define RTDB_TX_BUF_SIZE        1024u

/** Größe des HTTP-Antwortpuffers */
#define RTDB_RX_BUF_SIZE        4096u

/** Maximale Länge eines Firebase ID-Tokens */
#define RTDB_TOKEN_SIZE         2048u

/** Vorlaufzeit vor Token-Ablauf in Sekunden */
#define RTDB_TOKEN_RENEW_SECS   60u

/** Timeout für WiFi-Verbindungsaufbau in Millisekunden */
#define RTDB_WIFI_TIMEOUT_MS    15000u

/* --------------------------------------------------------------------------
 * WiFi-Verbindung – interne State-Verwaltung
 * -------------------------------------------------------------------------- */
static EventGroupHandle_t           s_hWifiEventGroup = NULL;
static esp_event_handler_instance_t s_hInstanceAnyId  = NULL;
static esp_event_handler_instance_t s_hInstanceGotIp  = NULL;
static bool                         s_bWifiInitDone   = false;

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

/* --------------------------------------------------------------------------
 * Auth-Token – interne State-Verwaltung
 * -------------------------------------------------------------------------- */
static char   s_acAuthToken[RTDB_TOKEN_SIZE] = {0};
static time_t s_tTokenExpiry                 = 0;
static char   s_acUid[64]                    = {0};  /**< Firebase User-UID (localId) */

/* --------------------------------------------------------------------------
 * Hilfsstruktur: HTTP-Antwort-Body puffern
 * -------------------------------------------------------------------------- */
typedef struct {
    char   *pacBuf;
    size_t  uiBufSize;
    size_t  uiLen;
} prv_httpRxCtx_t;

/* --------------------------------------------------------------------------
 * WiFi-Event-Handler
 * -------------------------------------------------------------------------- */
static void prv_wifiEventHandler(void *pvArg,
                                 esp_event_base_t eEventBase,
                                 int32_t i32EventId,
                                 void *pvEventData)
{
    if (eEventBase == WIFI_EVENT && i32EventId == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(sc_pacTAG, "WiFi getrennt");
        xEventGroupSetBits(s_hWifiEventGroup, WIFI_FAIL_BIT);
    }
    else if (eEventBase == IP_EVENT && i32EventId == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *psEv = (ip_event_got_ip_t *)pvEventData;
        ESP_LOGI(sc_pacTAG, "WiFi verbunden – IP: " IPSTR, IP2STR(&psEv->ip_info.ip));
        xEventGroupSetBits(s_hWifiEventGroup, WIFI_CONNECTED_BIT);
    }
}

/* --------------------------------------------------------------------------
 * HTTP-Event-Handler
 * Wenn user_data gesetzt ist, wird der Response-Body in den Puffer kopiert.
 * -------------------------------------------------------------------------- */
static esp_err_t prv_httpEventHandler(esp_http_client_event_t *psEvent)
{
    switch (psEvent->event_id)
    {
        case HTTP_EVENT_ERROR:
            ESP_LOGW(sc_pacTAG, "HTTP Fehler");
            break;

        case HTTP_EVENT_ON_DATA:
            if (psEvent->user_data != NULL)
            {
                prv_httpRxCtx_t *psCtx = (prv_httpRxCtx_t *)psEvent->user_data;
                size_t uiCopy = (size_t)psEvent->data_len;
                if (psCtx->uiLen + uiCopy >= psCtx->uiBufSize)
                {
                    uiCopy = psCtx->uiBufSize - psCtx->uiLen - 1u;
                }
                if (uiCopy > 0u)
                {
                    memcpy(psCtx->pacBuf + psCtx->uiLen, psEvent->data, uiCopy);
                    psCtx->uiLen += uiCopy;
                    psCtx->pacBuf[psCtx->uiLen] = '\0';
                }
            }
            break;

        default:
            break;
    }
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Öffentliche WiFi-Funktionen
 * -------------------------------------------------------------------------- */

esp_err_t firestore_wifiConnect(const char *pacSsid, const char *pacPassword)
{
    if (s_bWifiInitDone)
    {
        return ESP_OK;
    }

    esp_err_t eErr = nvs_flash_init();
    if (eErr == ESP_ERR_NVS_NO_FREE_PAGES || eErr == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        eErr = nvs_flash_init();
    }
    if (eErr != ESP_OK)
    {
        ESP_LOGE(sc_pacTAG, "NVS-Init fehlgeschlagen: %s", esp_err_to_name(eErr));
        return eErr;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t sCfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&sCfg));

    s_hWifiEventGroup = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &prv_wifiEventHandler, NULL, &s_hInstanceAnyId));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &prv_wifiEventHandler, NULL, &s_hInstanceGotIp));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (pacSsid != NULL && pacSsid[0] != '\0')
    {
        wifi_config_t sWifiCfg = {0};
        strncpy((char *)sWifiCfg.sta.ssid,    pacSsid,    sizeof(sWifiCfg.sta.ssid)     - 1u);
        strncpy((char *)sWifiCfg.sta.password, pacPassword != NULL ? pacPassword : "",
                sizeof(sWifiCfg.sta.password) - 1u);
        ESP_LOGI(sc_pacTAG, "Verbinde mit SSID: %s", pacSsid);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sWifiCfg));
    }
    else
    {
        ESP_LOGI(sc_pacTAG, "Verbinde mit NVS-Credentials");
    }

    ESP_ERROR_CHECK(esp_wifi_connect());

    EventBits_t eBits = xEventGroupWaitBits(
        s_hWifiEventGroup,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(RTDB_WIFI_TIMEOUT_MS));

    if (eBits & WIFI_CONNECTED_BIT)
    {
        s_bWifiInitDone = true;
        return ESP_OK;
    }

    ESP_LOGE(sc_pacTAG, "%s", (eBits & WIFI_FAIL_BIT) ?
             "WiFi-Verbindung fehlgeschlagen" : "WiFi-Timeout");
    return ESP_FAIL;
}

esp_err_t firestore_wifiDisconnect(void)
{
    if (!s_bWifiInitDone)
    {
        return ESP_OK;
    }

    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_hInstanceAnyId);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_hInstanceGotIp);
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();
    vEventGroupDelete(s_hWifiEventGroup);

    s_hWifiEventGroup = NULL;
    s_bWifiInitDone   = false;
    s_acAuthToken[0]  = '\0';
    s_tTokenExpiry    = 0;
    s_acUid[0]        = '\0';

    ESP_LOGI(sc_pacTAG, "WiFi getrennt");
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Firebase Email/Passwort Auth
 * -------------------------------------------------------------------------- */

static esp_err_t prv_rtdbGetAuthToken(const char *pacEmail, const char *pacPassword)
{
    static char acRxBuf[RTDB_RX_BUF_SIZE];
    memset(acRxBuf, 0, sizeof(acRxBuf));

    prv_httpRxCtx_t sRxCtx = {
        .pacBuf    = acRxBuf,
        .uiBufSize = sizeof(acRxBuf),
        .uiLen     = 0u,
    };

    char acAuthBody[256];
    snprintf(acAuthBody, sizeof(acAuthBody),
             "{\"email\":\"%s\",\"password\":\"%s\",\"returnSecureToken\":true}",
             pacEmail, pacPassword);

    esp_http_client_config_t sConfig = {
        .url               = RTDB_AUTH_URL,
        .method            = HTTP_METHOD_POST,
        .event_handler     = prv_httpEventHandler,
        .user_data         = &sRxCtx,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 10000,
        .buffer_size       = RTDB_RX_BUF_SIZE,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        ESP_LOGE(sc_pacTAG, "Auth: HTTP-Client-Init fehlgeschlagen");
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "Content-Type", "application/json");
    esp_http_client_set_post_field(hClient, acAuthBody, (int)strlen(acAuthBody));

    esp_err_t eErr   = esp_http_client_perform(hClient);
    int i32Status    = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK || i32Status != 200)
    {
        ESP_LOGE(sc_pacTAG, "Auth fehlgeschlagen: %s (HTTP %d)",
                 esp_err_to_name(eErr), i32Status);
        return ESP_FAIL;
    }

    /* "idToken":"<token>" aus JSON parsen */
    char *pacStart = strstr(acRxBuf, "\"idToken\":");
    if (pacStart == NULL)
    {
        ESP_LOGE(sc_pacTAG, "Auth: idToken nicht gefunden");
        return ESP_FAIL;
    }
    pacStart += strlen("\"idToken\":");
    while (*pacStart == ' ' || *pacStart == '\t') { pacStart++; }  /* Leerzeichen überspringen */
    if (*pacStart != '"') { ESP_LOGE(sc_pacTAG, "Auth: idToken-Format ungültig"); return ESP_FAIL; }
    pacStart++; /* öffnendes " überspringen */
    char *pacEnd = strchr(pacStart, '"');
    if (pacEnd == NULL)
    {
        ESP_LOGE(sc_pacTAG, "Auth: idToken-Ende fehlt");
        return ESP_FAIL;
    }

    size_t uiLen = (size_t)(pacEnd - pacStart);
    if (uiLen == 0u || uiLen >= RTDB_TOKEN_SIZE)
    {
        ESP_LOGE(sc_pacTAG, "Auth: idToken-Länge ungültig");
        return ESP_FAIL;
    }
    memcpy(s_acAuthToken, pacStart, uiLen);
    s_acAuthToken[uiLen] = '\0';

    /* Ablaufzeit auslesen (Default: 3600 s) */
    uint32_t ui32ExpiresIn = 3600u;
    char *pacExpiry = strstr(acRxBuf, "\"expiresIn\":");
    if (pacExpiry != NULL)
    {
        pacExpiry += strlen("\"expiresIn\":");
        while (*pacExpiry == ' ' || *pacExpiry == '"') { pacExpiry++; }
        ui32ExpiresIn = (uint32_t)atoi(pacExpiry);
    }
    s_tTokenExpiry = time(NULL) + (time_t)ui32ExpiresIn - (time_t)RTDB_TOKEN_RENEW_SECS;

    /* localId (UID) auslesen */
    s_acUid[0] = '\0';
    char *pacUid = strstr(acRxBuf, "\"localId\":");
    if (pacUid != NULL)
    {
        pacUid += strlen("\"localId\":");
        while (*pacUid == ' ' || *pacUid == '\t') { pacUid++; }
        if (*pacUid == '"')
        {
            pacUid++;
            char *pacUidEnd = strchr(pacUid, '"');
            if (pacUidEnd != NULL)
            {
                size_t uiUidLen = (size_t)(pacUidEnd - pacUid);
                if (uiUidLen > 0u && uiUidLen < sizeof(s_acUid))
                {
                    memcpy(s_acUid, pacUid, uiUidLen);
                    s_acUid[uiUidLen] = '\0';
                }
            }
        }
    }
    if (s_acUid[0] == '\0')
    {
        ESP_LOGW(sc_pacTAG, "Auth: localId nicht gefunden – UID unbekannt");
    }
    else
    {
        ESP_LOGI(sc_pacTAG, "Auth: UID = %s", s_acUid);
    }

    ESP_LOGI(sc_pacTAG, "Auth: Token erhalten (gültig %lu s)", (unsigned long)ui32ExpiresIn);
    return ESP_OK;
}

esp_err_t firestore_authenticate(const char *pacEmail, const char *pacPassword)
{
    if (s_acAuthToken[0] != '\0' && time(NULL) < s_tTokenExpiry)
    {
        ESP_LOGD(sc_pacTAG, "Auth: Token noch gültig");
        return ESP_OK;
    }
    ESP_LOGI(sc_pacTAG, "Auth: Hole neues Token...");
    return prv_rtdbGetAuthToken(pacEmail, pacPassword);
}

const char *firestore_getUid(void)
{
    return s_acUid;
}

/* --------------------------------------------------------------------------
 * Interne Hilfsfunktionen
 * -------------------------------------------------------------------------- */

static void prv_getTimestampISO8601(char *pacBuf, size_t uiBufLen)
{
    time_t tNow = time(NULL);
    struct tm sUtc;
    gmtime_r(&tNow, &sUtc);
    strftime(pacBuf, uiBufLen, "%Y-%m-%dT%H:%M:%SZ", &sUtc);
}

/**
 * HTTPS-POST an einen Realtime Database Pfad.
 * POST erzeugt einen neuen Eintrag mit auto-generiertem Push-Key.
 * Auth-Token wird als ?auth=<token> URL-Parameter übergeben.
 *
 * @param pacPath  Datenbankpfad, z.B. "devices" oder "miflora"
 * @param pacBody  JSON-Body als plain JSON-Objekt
 */
static esp_err_t prv_rtdbPost(const char *pacPath, const char *pacBody)
{
    if (s_acAuthToken[0] == '\0')
    {
        ESP_LOGE(sc_pacTAG, "Kein Auth-Token – firestore_authenticate() zuerst aufrufen");
        return ESP_ERR_INVALID_STATE;
    }

    /* URL: https://<project>-default-rtdb.firebaseio.com/<path>.json?auth=<token> */
    char acUrl[256 + RTDB_TOKEN_SIZE];
    snprintf(acUrl, sizeof(acUrl),
             RTDB_BASE_URL "/USERS/%s/%s.json?auth=%s",
             s_acUid, pacPath, s_acAuthToken);

    esp_http_client_config_t sConfig = {
        .url               = acUrl,
        .method            = HTTP_METHOD_POST,
        .event_handler     = prv_httpEventHandler,
        .user_data         = NULL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 10000,
        .buffer_size       = 1024u,
        .buffer_size_tx    = 2048u,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        ESP_LOGE(sc_pacTAG, "HTTP-Client-Init fehlgeschlagen");
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "Content-Type", "application/json");
    esp_http_client_set_post_field(hClient, pacBody, (int)strlen(pacBody));

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        ESP_LOGE(sc_pacTAG, "HTTP-Request fehlgeschlagen: %s", esp_err_to_name(eErr));
        return ESP_FAIL;
    }
    if (i32Status == 200)
    {
        ESP_LOGI(sc_pacTAG, "Gespeichert unter /%s/", pacPath);
        return ESP_OK;
    }

    ESP_LOGW(sc_pacTAG, "HTTP-Status %d [/%s/]", i32Status, pacPath);
    return ESP_FAIL;
}

/* --------------------------------------------------------------------------
 * Firebase Server-Zeit Synchronisation
 * -------------------------------------------------------------------------- */

/** Kontext zum Erfassen des HTTP Date-Headers */
typedef struct {
    char acDate[64];
    bool bGotDate;
} prv_dateSyncCtx_t;

static esp_err_t prv_dateHeaderEventHandler(esp_http_client_event_t *psEvent)
{
    if (psEvent->event_id == HTTP_EVENT_ON_HEADER && psEvent->user_data != NULL)
    {
        prv_dateSyncCtx_t *psCtx = (prv_dateSyncCtx_t *)psEvent->user_data;
        if (strcasecmp(psEvent->header_key, "Date") == 0)
        {
            strncpy(psCtx->acDate, psEvent->header_value, sizeof(psCtx->acDate) - 1u);
            psCtx->acDate[sizeof(psCtx->acDate) - 1u] = '\0';
            psCtx->bGotDate = true;
        }
    }
    return ESP_OK;
}

/**
 * @brief Synchronisiert die ESP32-Systemzeit anhand des HTTP Date-Headers.
 *        Der Date-Header ist in jeder HTTPS-Antwort enthalten (RFC 7231).
 *        Kein Auth nötig – nutzt einen anonymen GET auf die RTDB-Root.
 *
 * @return ESP_OK bei Erfolg, ESP_FAIL bei Fehler.
 */
esp_err_t firestore_syncTimeFromServer(void)
{
    ESP_LOGI(sc_pacTAG, "Zeit-Sync: Hole Server-Zeit via HTTP Date-Header...");

    prv_dateSyncCtx_t sDateCtx = {0};

    esp_http_client_config_t sConfig = {
        .url               = RTDB_BASE_URL "/.json?print=silent",
        .method            = HTTP_METHOD_GET,
        .event_handler     = prv_dateHeaderEventHandler,
        .user_data         = &sDateCtx,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 5000,
        .buffer_size       = 1024u,
        .buffer_size_tx    = 512u,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        return ESP_FAIL;
    }

    esp_http_client_perform(hClient);   /* HTTP-Status egal – nur Date-Header nötig */
    esp_http_client_cleanup(hClient);

    if (!sDateCtx.bGotDate)
    {
        ESP_LOGE(sc_pacTAG, "Zeit-Sync: Date-Header nicht empfangen");
        return ESP_FAIL;
    }

    /* RFC 1123 parsen: "Mon, 01 Apr 2026 14:23:45 GMT" (immer UTC) */
    struct tm sTime = {0};
    if (strptime(sDateCtx.acDate, "%a, %d %b %Y %H:%M:%S", &sTime) == NULL)
    {
        ESP_LOGE(sc_pacTAG, "Zeit-Sync: Datum-Parse fehlgeschlagen: %s", sDateCtx.acDate);
        return ESP_FAIL;
    }

    /* struct tm (UTC) → time_t: TZ kurz auf UTC setzen damit mktime korrekt rechnet */
    setenv("TZ", "UTC0", 1);
    tzset();
    time_t tServer = mktime(&sTime);

    /* TZ auf konfigurierte Zeitzone zurücksetzen */
    setenv("TZ", TIMEZONE, 1);
    tzset();

    struct timeval sNow = { .tv_sec = tServer, .tv_usec = 0 };
    settimeofday(&sNow, NULL);

    char acBuf[64];
    struct tm sLocal;
    localtime_r(&tServer, &sLocal);
    strftime(acBuf, sizeof(acBuf), "%Y-%m-%d %H:%M:%S", &sLocal);
    ESP_LOGI(sc_pacTAG, "Zeit-Sync OK → %s", acBuf);

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Öffentliche Write-Funktionen
 * -------------------------------------------------------------------------- */

esp_err_t firestore_writeDeviceData(const deviceData_t *psData)
{
    if (psData == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char acTimestamp[24];
    prv_getTimestampISO8601(acTimestamp, sizeof(acTimestamp));

    float fTemperature = (float)psData->temperature / 10.0f;

    char acBody[RTDB_TX_BUF_SIZE];
    int i32Len = snprintf(acBody, sizeof(acBody),
        "{"
          "\"id\":%ld,"
          "\"name\":\"%s\","
          "\"status\":\"%s\","
          "\"battery_mv\":%ld,"
          "\"temperature_c\":%.1f,"
          "\"timestamp\":\"%s\""
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

    return prv_rtdbPost("devices", acBody);
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

    char acBody[RTDB_TX_BUF_SIZE];
    int i32Len = snprintf(acBody, sizeof(acBody),
        "{"
          "\"sensor_nr\":%lu,"
          "\"temperature_c\":%.1f,"
          "\"illuminance_lux\":%lu,"
          "\"moisture_pct\":%u,"
          "\"conductivity_us\":%u,"
          "\"battery_pct\":%u,"
          "\"firmware\":\"%s\","
          "\"timestamp\":\"%s\""
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
             (unsigned long)ui32SensorNr, psData->temperature,
             (unsigned)psData->moisture, (unsigned long)psData->illuminance);

    return prv_rtdbPost("miflora", acBody);
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

    for (int i32Ch = 0; i32Ch < CHANNELCOUNT; i32Ch++)
    {
        for (int i32Ev = 0; i32Ev < EVENTCOUNT; i32Ev++)
        {
            const wateringTime_t *psEvent =
                &psData->wateringChannel[i32Ch].wateringEvent[i32Ev];

            if (!psEvent->wateringEnable)
            {
                continue;
            }

            float fAmountMl     = (float)psEvent->wateringAmount     / 20.0f;
            float fAmountLastMl = (float)psEvent->wateringAmountLast / 20.0f;

            char acBody[RTDB_TX_BUF_SIZE];
            int i32Len = snprintf(acBody, sizeof(acBody),
                "{"
                  "\"channel\":%d,"
                  "\"event\":%d,"
                  "\"last_unix\":%lld,"
                  "\"next_unix\":%lld,"
                  "\"freq_unix\":%lld,"
                  "\"amount_ml\":%.2f,"
                  "\"amount_last_ml\":%.2f,"
                  "\"global_last_unix\":%lld,"
                  "\"global_next_unix\":%lld,"
                  "\"timestamp\":\"%s\""
                "}",
                i32Ch, i32Ev,
                (long long)psEvent->wateringLastUnix,
                (long long)psEvent->wateringNextUnix,
                (long long)psEvent->wateringFreqUnix,
                fAmountMl, fAmountLastMl,
                (long long)psData->wateringLastUnix,
                (long long)psData->wateringNextUnix,
                acTimestamp);

            if (i32Len < 0 || i32Len >= (int)sizeof(acBody))
            {
                ESP_LOGE(sc_pacTAG, "JSON-Puffer zu klein (watering ch%d ev%d)", i32Ch, i32Ev);
                eErrRet = ESP_ERR_NO_MEM;
                continue;
            }

            ESP_LOGI(sc_pacTAG, "Sende Bewässerung ch=%d ev=%d amount=%.1fml",
                     i32Ch, i32Ev, fAmountMl);

            esp_err_t eErr = prv_rtdbPost("watering", acBody);
            if (eErr != ESP_OK)
            {
                eErrRet = eErr;
            }
        }
    }

    return eErrRet;
}

/* -------------------------------------------------------------------------- */

static esp_err_t prv_rtdbPut(const char *pacPath, const char *pacBody)
{
    if (s_acAuthToken[0] == '\0')
    {
        ESP_LOGE(sc_pacTAG, "Kein Auth-Token – firestore_authenticate() zuerst aufrufen");
        return ESP_ERR_INVALID_STATE;
    }

    char acUrl[256 + RTDB_TOKEN_SIZE];
    snprintf(acUrl, sizeof(acUrl),
             RTDB_BASE_URL "/USERS/%s/%s.json?auth=%s",
             s_acUid, pacPath, s_acAuthToken);

    esp_http_client_config_t sConfig = {
        .url               = acUrl,
        .method            = HTTP_METHOD_PUT,
        .event_handler     = prv_httpEventHandler,
        .user_data         = NULL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 10000,
        .buffer_size       = 1024u,
        .buffer_size_tx    = 2048u,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        ESP_LOGE(sc_pacTAG, "HTTP-Client-Init fehlgeschlagen");
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "Content-Type", "application/json");
    esp_http_client_set_post_field(hClient, pacBody, (int)strlen(pacBody));

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        ESP_LOGE(sc_pacTAG, "HTTP-Request fehlgeschlagen: %s", esp_err_to_name(eErr));
        return ESP_FAIL;
    }
    if (i32Status == 200)
    {
        ESP_LOGI(sc_pacTAG, "Gespeichert unter /%s", pacPath);
        return ESP_OK;
    }

    ESP_LOGW(sc_pacTAG, "HTTP-Status %d [/%s]", i32Status, pacPath);
    return ESP_FAIL;
}

/* -------------------------------------------------------------------------- */

static esp_err_t prv_rtdbPatch(const char *pacPath, const char *pacBody)
{
    if (s_acAuthToken[0] == '\0')
    {
        ESP_LOGE(sc_pacTAG, "Kein Auth-Token – firestore_authenticate() zuerst aufrufen");
        return ESP_ERR_INVALID_STATE;
    }

    char acUrl[256 + RTDB_TOKEN_SIZE];
    snprintf(acUrl, sizeof(acUrl),
             RTDB_BASE_URL "/USERS/%s/%s.json?auth=%s",
             s_acUid, pacPath, s_acAuthToken);

    esp_http_client_config_t sConfig = {
        .url               = acUrl,
        .method            = HTTP_METHOD_PATCH,
        .event_handler     = prv_httpEventHandler,
        .user_data         = NULL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 10000,
        .buffer_size       = 1024u,
        .buffer_size_tx    = 2048u,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        ESP_LOGE(sc_pacTAG, "HTTP-Client-Init fehlgeschlagen");
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "Content-Type", "application/json");
    esp_http_client_set_post_field(hClient, pacBody, (int)strlen(pacBody));

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        ESP_LOGE(sc_pacTAG, "HTTP-Request fehlgeschlagen: %s", esp_err_to_name(eErr));
        return ESP_FAIL;
    }
    if (i32Status == 200)
    {
        ESP_LOGI(sc_pacTAG, "Gepatcht unter /%s", pacPath);
        return ESP_OK;
    }

    ESP_LOGW(sc_pacTAG, "HTTP-Status %d [/%s]", i32Status, pacPath);
    return ESP_FAIL;
}

/* -------------------------------------------------------------------------- */

esp_err_t firestore_writeString(const char *pacPath, const char *pacJson)
{
    if (pacPath == NULL || pacJson == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return prv_rtdbPut(pacPath, pacJson);
}

/* -------------------------------------------------------------------------- */

esp_err_t firestore_appendString(const char *pacPath, const char *pacJson)
{
    if (pacPath == NULL || pacJson == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_acAuthToken[0] == '\0')
    {
        ESP_LOGE(sc_pacTAG, "Kein Auth-Token – firestore_authenticate() zuerst aufrufen");
        return ESP_ERR_INVALID_STATE;
    }
    return prv_rtdbPost(pacPath, pacJson);
}

/* -------------------------------------------------------------------------- */

esp_err_t firestore_patchString(const char *pacPath, const char *pacJson)
{
    if (pacPath == NULL || pacJson == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return prv_rtdbPatch(pacPath, pacJson);
}

/* -------------------------------------------------------------------------- */

static esp_err_t prv_rtdbGet(const char *pacPath, char *pacRxBuf, size_t uiBufSize)
{
    char acUrl[256 + RTDB_TOKEN_SIZE];
    snprintf(acUrl, sizeof(acUrl),
             RTDB_BASE_URL "/USERS/%s/%s.json?auth=%s",
             s_acUid, pacPath, s_acAuthToken);

    prv_httpRxCtx_t sRxCtx = {
        .pacBuf    = pacRxBuf,
        .uiBufSize = uiBufSize,
        .uiLen     = 0u,
    };
    pacRxBuf[0] = '\0';

    esp_http_client_config_t sConfig = {
        .url               = acUrl,
        .method            = HTTP_METHOD_GET,
        .event_handler     = prv_httpEventHandler,
        .user_data         = &sRxCtx,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 10000,
        .buffer_size       = 8192u,
        .buffer_size_tx    = 2048u,  /* Auth-Token in URL kann >1000 Zeichen lang sein */
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        return ESP_FAIL;
    }

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        ESP_LOGE(sc_pacTAG, "GET fehlgeschlagen: %s", esp_err_to_name(eErr));
        return ESP_FAIL;
    }
    if (i32Status != 200)
    {
        ESP_LOGW(sc_pacTAG, "GET HTTP-Status %d [/%s]", i32Status, pacPath);
        return ESP_FAIL;
    }

    ESP_LOGI(sc_pacTAG, "GET /%s (%d Bytes)", pacPath, (int)sRxCtx.uiLen);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

esp_err_t firestore_readString(const char *pacPath, char *pacBuf, size_t uiBufSize)
{
    if (pacPath == NULL || pacBuf == NULL || uiBufSize == 0u)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_acAuthToken[0] == '\0')
    {
        ESP_LOGE(sc_pacTAG, "Kein Auth-Token – firestore_authenticate() zuerst aufrufen");
        return ESP_ERR_INVALID_STATE;
    }
    return prv_rtdbGet(pacPath, pacBuf, uiBufSize);
}
