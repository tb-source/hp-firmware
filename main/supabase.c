#include "supabase.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "mbedtls/base64.h"
#include "nvs_flash.h"
#include "datamanagement.h"
#include "helper.h"

static const char *TAG = "SUPABASE";
static const int HTTP_TIMEOUT_MS = 10000;
static const uint32_t WIFI_TIMEOUT_MS = 15000u;
static const size_t SUPABASE_POST_MAX_JSON_BYTES = 1800u;
static const uint32_t SUPABASE_LOG_READ_BATCH = 200u;
static const size_t SUPABASE_LOG_BUF_SIZE = 8192u;
static const int SUPABASE_LOG_DEBUG_MAX_ITEMS = 15;
static const long long SUPABASE_UNIX_MAX_2100 = 4102444800LL;

static esp_err_t http_perform_request(esp_http_client_method_t eMethod,
                                      const credentials_t *psCredentials,
                                      const char *pacBody,
                                      char *pacResponseBuf,
                                      size_t uiResponseBufSize,
                                      int *piHttpStatus);

static EventGroupHandle_t s_hWifiEventGroup = NULL;
static esp_event_handler_instance_t s_hInstanceAnyId = NULL;
static esp_event_handler_instance_t s_hInstanceGotIp = NULL;
static bool s_bWifiInitDone = false;

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static void wifi_event_handler(void *pvArg,
                               esp_event_base_t eEventBase,
                               int32_t i32EventId,
                               void *pvEventData)
{
    (void)pvArg;

    if (eEventBase == WIFI_EVENT && i32EventId == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW(TAG, "WiFi disconnected");
        xEventGroupSetBits(s_hWifiEventGroup, WIFI_FAIL_BIT);
    }
    else if (eEventBase == IP_EVENT && i32EventId == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *psEv = (ip_event_got_ip_t *)pvEventData;
        ESP_LOGI(TAG, "WiFi connected - IP: " IPSTR, IP2STR(&psEv->ip_info.ip));
        xEventGroupSetBits(s_hWifiEventGroup, WIFI_CONNECTED_BIT);
    }
}

esp_err_t supabase_wifiConnect(const char *pacSsid, const char *pacPassword)
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
    if (eErr != ESP_OK && eErr != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(eErr));
        return eErr;
    }

    eErr = esp_netif_init();
    if (eErr != ESP_OK && eErr != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(eErr));
        return eErr;
    }

    eErr = esp_event_loop_create_default();
    if (eErr != ESP_OK && eErr != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "event loop create failed: %s", esp_err_to_name(eErr));
        return eErr;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t sCfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&sCfg));

    s_hWifiEventGroup = xEventGroupCreate();
    if (s_hWifiEventGroup == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &s_hInstanceAnyId));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &s_hInstanceGotIp));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (pacSsid != NULL && pacSsid[0] != '\0')
    {
        wifi_config_t sWifiCfg = {0};
        strncpy((char *)sWifiCfg.sta.ssid, pacSsid, sizeof(sWifiCfg.sta.ssid) - 1u);
        strncpy((char *)sWifiCfg.sta.password,
                (pacPassword != NULL) ? pacPassword : "",
                sizeof(sWifiCfg.sta.password) - 1u);
        ESP_LOGI(TAG, "Connecting to SSID: %s", pacSsid);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sWifiCfg));
    }
    else
    {
        ESP_LOGI(TAG, "Connecting with stored WiFi credentials");
    }

    ESP_ERROR_CHECK(esp_wifi_connect());

    EventBits_t eBits = xEventGroupWaitBits(
        s_hWifiEventGroup,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(WIFI_TIMEOUT_MS));

    if ((eBits & WIFI_CONNECTED_BIT) != 0)
    {
        s_bWifiInitDone = true;
        return ESP_OK;
    }

    ESP_LOGE(TAG, "%s", ((eBits & WIFI_FAIL_BIT) != 0) ?
             "WiFi connection failed" : "WiFi timeout");

    (void)supabase_wifiDisconnect();
    return ESP_FAIL;
}

esp_err_t supabase_wifiDisconnect(void)
{
    if (!s_bWifiInitDone)
    {
        return ESP_OK;
    }

    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_hInstanceAnyId);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_hInstanceGotIp);
    (void)esp_wifi_disconnect();
    (void)esp_wifi_stop();
    (void)esp_wifi_deinit();

    if (s_hWifiEventGroup != NULL)
    {
        vEventGroupDelete(s_hWifiEventGroup);
        s_hWifiEventGroup = NULL;
    }

    s_bWifiInitDone = false;
    ESP_LOGI(TAG, "WiFi disconnected");
    return ESP_OK;
}

typedef struct {
    char *pacBuf;
    size_t uiBufSize;
    size_t uiLen;
} http_rx_ctx_t;

static esp_err_t http_event_handler(esp_http_client_event_t *psEvent)
{
    if (psEvent == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (psEvent->event_id == HTTP_EVENT_ON_DATA && psEvent->user_data != NULL)
    {
        http_rx_ctx_t *psCtx = (http_rx_ctx_t *)psEvent->user_data;
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

    return ESP_OK;
}

static const char *log_type_to_text(log_type_t eType)
{
    switch (eType)
    {
        case LOG_TYPE_PERIPHERY:
            return "PERIPHERY";
        case LOG_TYPE_WATERING:
            return "WATERING";
        case LOG_TYPE_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

static int append_logs_from_json_object(const char *pacLogJson,
                                        log_type_t eType,
                                        cJSON *psOutArray)
{
    cJSON *psRoot;
    cJSON *psNode;
    int iAdded = 0;

    if (pacLogJson == NULL || psOutArray == NULL)
    {
        return 0;
    }

    psRoot = cJSON_Parse(pacLogJson);
    if (!cJSON_IsObject(psRoot))
    {
        cJSON_Delete(psRoot);
        return 0;
    }

    psNode = psRoot->child;
    while (psNode != NULL)
    {
        if (cJSON_IsObject(psNode) && psNode->string != NULL)
        {
            long long llTime = atoll(psNode->string);
            if (llTime <= 0)
            {
                ESP_LOGW(TAG, "Skip log entry with invalid time key: %s", psNode->string);
                psNode = psNode->next;
                continue;
            }
            cJSON *psLog = cJSON_Duplicate(psNode, true);
            if (psLog == NULL)
            {
                break;
            }

            cJSON_AddNumberToObject(psLog, "time", (double)llTime);

            if (!cJSON_IsString(cJSON_GetObjectItem(psLog, "type")))
            {
                if (eType == LOG_TYPE_PERIPHERY)
                {
                    cJSON_AddStringToObject(psLog, "type", "data");
                }
                else if (eType == LOG_TYPE_WATERING)
                {
                    cJSON_AddStringToObject(psLog, "type", "watering");
                }
                else if (eType == LOG_TYPE_ERROR)
                {
                    cJSON_AddStringToObject(psLog, "type", "error");
                }
            }

            if (psLog != NULL)
            {
                if (iAdded < SUPABASE_LOG_DEBUG_MAX_ITEMS)
                {
                    char *pacItem = cJSON_PrintUnformatted(psLog);
                    if (pacItem != NULL)
                    {
                        ESP_LOGI(TAG,
                                 "Collected %s log[%d]: %s",
                                 log_type_to_text(eType),
                                 iAdded,
                                 pacItem);
                        cJSON_free(pacItem);
                    }
                }
                cJSON_AddItemToArray(psOutArray, psLog);
                iAdded++;
            }
        }
        psNode = psNode->next;
    }

    cJSON_Delete(psRoot);
    return iAdded;
}

static esp_err_t collect_logs_to_array(cJSON *psOutLogs)
{
    char *pacBuf;
    esp_err_t eErr;
    int iAddedPeriphery = 0;
    int iAddedWatering = 0;
    int iAddedError = 0;

    if (psOutLogs == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    pacBuf = (char *)calloc(1u, SUPABASE_LOG_BUF_SIZE);
    if (pacBuf == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    eErr = data_getPeripherieLogData(pacBuf, SUPABASE_LOG_BUF_SIZE, 0u, SUPABASE_LOG_READ_BATCH);
    if (eErr == ESP_OK)
    {
        iAddedPeriphery = append_logs_from_json_object(pacBuf, LOG_TYPE_PERIPHERY, psOutLogs);
        ESP_LOGI(TAG, "Collected PERIPHERY logs: %d", iAddedPeriphery);
    }
    else if (eErr != ESP_ERR_NOT_FOUND)
    {
        ESP_LOGW(TAG, "Periphery log read skipped: %s", esp_err_to_name(eErr));
    }

    eErr = data_getWateringLogData(pacBuf, SUPABASE_LOG_BUF_SIZE, 0u, SUPABASE_LOG_READ_BATCH);
    if (eErr == ESP_OK)
    {
        iAddedWatering = append_logs_from_json_object(pacBuf, LOG_TYPE_WATERING, psOutLogs);
        ESP_LOGI(TAG, "Collected WATERING logs: %d", iAddedWatering);
    }
    else if (eErr != ESP_ERR_NOT_FOUND)
    {
        ESP_LOGW(TAG, "Watering log read skipped: %s", esp_err_to_name(eErr));
    }

    eErr = data_getErrorLogData(pacBuf, SUPABASE_LOG_BUF_SIZE, 0u, SUPABASE_LOG_READ_BATCH);
    if (eErr == ESP_OK)
    {
        iAddedError = append_logs_from_json_object(pacBuf, LOG_TYPE_ERROR, psOutLogs);
        ESP_LOGI(TAG, "Collected ERROR logs: %d", iAddedError);
    }
    else if (eErr != ESP_ERR_NOT_FOUND)
    {
        ESP_LOGW(TAG, "Error log read skipped: %s", esp_err_to_name(eErr));
    }

    ESP_LOGI(TAG,
             "Collected logs total=%d (P=%d, W=%d, E=%d)",
             cJSON_GetArraySize(psOutLogs),
             iAddedPeriphery,
             iAddedWatering,
             iAddedError);

    free(pacBuf);
    return ESP_OK;
}

static char *build_status_payload(const deviceData_t *psDeviceData,
                                  const char *pacFirmwareVersion,
                                  cJSON *psLogsArray)
{
    cJSON *psPost;
    char *pacPayload;

    psPost = cJSON_CreateObject();
    if (psPost == NULL)
    {
        return NULL;
    }

    cJSON_AddNumberToObject(psPost, "temperature", ((double)psDeviceData->temperature) / 10.0);
    cJSON_AddNumberToObject(psPost, "batteryLevel", ui32BattLevel_read());
    cJSON_AddNumberToObject(psPost, "wateringLevel", 58);
    cJSON_AddStringToObject(psPost, "firmwareVersion", (pacFirmwareVersion != NULL) ? pacFirmwareVersion : "1.0.0");

    if (psLogsArray != NULL && cJSON_GetArraySize(psLogsArray) > 0)
    {
        cJSON *psCopy = cJSON_Duplicate(psLogsArray, true);
        if (psCopy != NULL)
        {
            cJSON_AddItemToObject(psPost, "logs", psCopy);
        }
    }

    pacPayload = cJSON_PrintUnformatted(psPost);
    cJSON_Delete(psPost);
    return pacPayload;
}

static esp_err_t send_status_payload(const credentials_t *psCredentials,
                                        const deviceData_t *psDeviceData,
                                     const char *pacFirmwareVersion,
                                     cJSON *psLogsArray,
                                     bool *pbUpdated,
                                     char *pacResponseBuf,
                                     size_t uiResponseBufSize,
                                     int *piHttpStatus)
{
    esp_err_t eErr;
    char *pacPostBody;
    cJSON *psResp;
    cJSON *psOk;
    cJSON *psUpdated;

    pacPostBody = build_status_payload(psDeviceData,
                                       pacFirmwareVersion,
                                       psLogsArray);
    if (pacPostBody == NULL)
    {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "POST body (first 256): %.256s", pacPostBody);

    eErr = http_perform_request(HTTP_METHOD_POST,
                                psCredentials,
                                pacPostBody,
                                pacResponseBuf,
                                uiResponseBufSize,
                                piHttpStatus);
    cJSON_free(pacPostBody);

    if (eErr != ESP_OK)
    {
        return eErr;
    }

    psResp = cJSON_Parse(pacResponseBuf);
    if (psResp == NULL)
    {
        ESP_LOGE(TAG, "POST JSON parse failed");
        return ESP_FAIL;
    }

    psOk = cJSON_GetObjectItem(psResp, "ok");
    psUpdated = cJSON_GetObjectItem(psResp, "updated");

    if (pbUpdated != NULL)
    {
        *pbUpdated = cJSON_IsTrue(psUpdated);
    }

    if (!cJSON_IsTrue(psOk))
    {
        cJSON_Delete(psResp);
        return ESP_FAIL;
    }

    cJSON_Delete(psResp);
    return ESP_OK;
}

static esp_err_t set_device_auth_headers(esp_http_client_handle_t hClient,
                                         const credentials_t *psCredentials)
{
    char acRawAuth[160];
    unsigned char acEncodedAuth[256];
    char acAuthorizationHeader[320];
    size_t uiOutLen = 0u;
    int iLen;
    int iHeaderLen;
    int iRet;

    if (hClient == NULL || psCredentials == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (psCredentials->deviceId[0] == '\0' || psCredentials->devicePW[0] == '\0')
    {
        ESP_LOGW(TAG, "deviceId/devicePW missing - cannot build Authorization header");
        return ESP_ERR_INVALID_STATE;
    }

    iLen = snprintf(acRawAuth,
                    sizeof(acRawAuth),
                    "%s:%s",
                    psCredentials->deviceId,
                    psCredentials->devicePW);
    if (iLen <= 0 || iLen >= (int)sizeof(acRawAuth))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    iRet = mbedtls_base64_encode(acEncodedAuth,
                                 sizeof(acEncodedAuth),
                                 &uiOutLen,
                                 (const unsigned char *)acRawAuth,
                                 (size_t)iLen);
    if (iRet != 0)
    {
        ESP_LOGW(TAG, "Base64 encode for Authorization failed: %d", iRet);
        return ESP_FAIL;
    }

    iHeaderLen = snprintf(acAuthorizationHeader,
                          sizeof(acAuthorizationHeader),
                          "Basic %s",
                          (const char *)acEncodedAuth);
    if (iHeaderLen <= 0 || iHeaderLen >= (int)sizeof(acAuthorizationHeader))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_http_client_set_header(hClient, "Authorization", acAuthorizationHeader);
    esp_http_client_set_header(hClient, "x-device-id", psCredentials->deviceId);
    esp_http_client_set_header(hClient, "x-device-secret", psCredentials->devicePW);

    return ESP_OK;
}

static esp_err_t http_perform_request(esp_http_client_method_t eMethod,
                                      const credentials_t *psCredentials,
                                      const char *pacBody,
                                      char *pacResponseBuf,
                                      size_t uiResponseBufSize,
                                      int *piHttpStatus)
{
    http_rx_ctx_t sRxCtx;
    esp_http_client_config_t sConfig;
    esp_http_client_handle_t hClient;
    esp_err_t eErr;
    int iStatus;

    if (psCredentials == NULL || pacResponseBuf == NULL || uiResponseBufSize == 0u)
    {
        return ESP_ERR_INVALID_ARG;
    }

    pacResponseBuf[0] = '\0';

    sRxCtx.pacBuf = pacResponseBuf;
    sRxCtx.uiBufSize = uiResponseBufSize;
    sRxCtx.uiLen = 0u;

    memset(&sConfig, 0, sizeof(sConfig));
    sConfig.url = SUPABASE_EDGE_URL;
    sConfig.method = eMethod;
    sConfig.event_handler = http_event_handler;
    sConfig.user_data = &sRxCtx;
    sConfig.crt_bundle_attach = esp_crt_bundle_attach;
    sConfig.timeout_ms = HTTP_TIMEOUT_MS;
    sConfig.buffer_size = 2048;
    sConfig.buffer_size_tx = 2048;

    hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        return ESP_FAIL;
    }

    eErr = set_device_auth_headers(hClient, psCredentials);
    if (eErr != ESP_OK)
    {
        esp_http_client_cleanup(hClient);
        return eErr;
    }

    esp_http_client_set_header(hClient, "Accept", "application/json");

    if (eMethod == HTTP_METHOD_POST)
    {
        if (pacBody == NULL)
        {
            esp_http_client_cleanup(hClient);
            return ESP_ERR_INVALID_ARG;
        }

        esp_http_client_set_header(hClient, "Content-Type", "application/json");
        esp_http_client_set_post_field(hClient, pacBody, (int)strlen(pacBody));
    }

    eErr = esp_http_client_perform(hClient);
    iStatus = esp_http_client_get_status_code(hClient);

    ESP_LOGI(TAG, "HTTP %s status=%d", (eMethod == HTTP_METHOD_GET) ? "GET" : "POST", iStatus);
    ESP_LOGI(TAG, "HTTP response body: %s", pacResponseBuf);

    if (piHttpStatus != NULL)
    {
        *piHttpStatus = iStatus;
    }

    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        ESP_LOGW(TAG, "HTTP perform failed: %s", esp_err_to_name(eErr));
        return eErr;
    }

    if (iStatus < 200 || iStatus >= 300)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t supabase_get_config(const credentials_t *psCredentials,
                              deviceData_t *psDeviceData,
                              char *pacResponseBuf,
                              size_t uiResponseBufSize,
                              int *piHttpStatus)
{
    esp_err_t eErr;

    if (psCredentials == NULL || psDeviceData == NULL || pacResponseBuf == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    eErr = http_perform_request(HTTP_METHOD_GET,
                                psCredentials,
                                NULL,
                                pacResponseBuf,
                                uiResponseBufSize,
                                piHttpStatus);
    if (eErr != ESP_OK)
    {
        return eErr;
    }

    eErr = deviceDataJson_parse(psDeviceData, pacResponseBuf, true);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "GET config parse failed: %s", esp_err_to_name(eErr));
    }

    return eErr;
}

esp_err_t supabase_post_status(const credentials_t *psCredentials,
                               const deviceData_t *psDeviceData,
                               const char *pacFirmwareVersion,
                               bool *pbUpdated,
                               char *pacResponseBuf,
                               size_t uiResponseBufSize,
                               int *piHttpStatus)
{
    esp_err_t eErr;
    cJSON *psAllLogs;
    cJSON *psChunk;
    bool bUpdatedAny = false;
    bool bSentLogs = false;
    int iLogCount;

    if (psCredentials == NULL || psDeviceData == NULL || pacResponseBuf == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    psAllLogs = cJSON_CreateArray();
    if (psAllLogs == NULL)
    {
        return ESP_FAIL;
    }

    (void)collect_logs_to_array(psAllLogs);
    iLogCount = cJSON_GetArraySize(psAllLogs);
    ESP_LOGI(TAG, "supabase_post_status preparing %d log entries", iLogCount);

    if (iLogCount <= 0)
    {
        bool bUpdated = false;
        eErr = send_status_payload(psCredentials,
                                   psDeviceData,
                                   pacFirmwareVersion,
                                   NULL,
                                   &bUpdated,
                                   pacResponseBuf,
                                   uiResponseBufSize,
                                   piHttpStatus);
        if (pbUpdated != NULL)
        {
            *pbUpdated = bUpdated;
        }
        cJSON_Delete(psAllLogs);
        return eErr;
    }

    psChunk = cJSON_CreateArray();
    if (psChunk == NULL)
    {
        cJSON_Delete(psAllLogs);
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < iLogCount; i++)
    {
        cJSON *psItem = cJSON_GetArrayItem(psAllLogs, i);
        char *pacBody;
        size_t uiBodyLen;

        if (psItem == NULL)
        {
            continue;
        }

        if (!cJSON_IsObject(psItem))
        {
            continue;
        }

        {
            cJSON *psTime = cJSON_GetObjectItem(psItem, "time");
            if (!cJSON_IsNumber(psTime) ||
                psTime->valuedouble <= 0.0 ||
                psTime->valuedouble > (double)SUPABASE_UNIX_MAX_2100)
            {
                ESP_LOGW(TAG, "Skip chunk item with invalid time field");
                continue;
            }
        }

        {
            cJSON *psItemCopy = cJSON_Duplicate(psItem, true);
            if (psItemCopy == NULL)
            {
                ESP_LOGW(TAG, "Skip chunk item due to OOM while duplicating log object");
                continue;
            }
            cJSON_AddItemToArray(psChunk, psItemCopy);
        }

        pacBody = build_status_payload(psDeviceData,
                                       pacFirmwareVersion,
                                       psChunk);
        if (pacBody == NULL)
        {
            continue;
        }
        uiBodyLen = strlen(pacBody);
        cJSON_free(pacBody);

        if (uiBodyLen > SUPABASE_POST_MAX_JSON_BYTES && cJSON_GetArraySize(psChunk) > 1)
        {
            cJSON *psCarry;
            bool bUpdatedChunk = false;
            int iLast;

            iLast = cJSON_GetArraySize(psChunk) - 1;
            psCarry = cJSON_DetachItemFromArray(psChunk, iLast);

            eErr = send_status_payload(psCredentials,
                                       psDeviceData,
                                       pacFirmwareVersion,
                                       psChunk,
                                       &bUpdatedChunk,
                                       pacResponseBuf,
                                       uiResponseBufSize,
                                       piHttpStatus);
            if (eErr != ESP_OK)
            {
                cJSON_Delete(psCarry);
                cJSON_Delete(psChunk);
                cJSON_Delete(psAllLogs);
                return eErr;
            }

            bSentLogs = true;
            bUpdatedAny = bUpdatedAny || bUpdatedChunk;

            cJSON_Delete(psChunk);
            psChunk = cJSON_CreateArray();
            if (psChunk == NULL)
            {
                cJSON_Delete(psCarry);
                cJSON_Delete(psAllLogs);
                return ESP_ERR_NO_MEM;
            }
            if (psCarry != NULL)
            {
                cJSON_AddItemToArray(psChunk, psCarry);
            }
        }
    }

    if (cJSON_GetArraySize(psChunk) > 0)
    {
        bool bUpdatedChunk = false;
        eErr = send_status_payload(psCredentials,
                                   psDeviceData,
                                   pacFirmwareVersion,
                                   psChunk,
                                   &bUpdatedChunk,
                                   pacResponseBuf,
                                   uiResponseBufSize,
                                   piHttpStatus);
        if (eErr != ESP_OK)
        {
            cJSON_Delete(psChunk);
            cJSON_Delete(psAllLogs);
            return eErr;
        }
        bSentLogs = true;
        bUpdatedAny = bUpdatedAny || bUpdatedChunk;
    }

    if (bSentLogs)
    {
        log_clearData(LOG_TYPE_PERIPHERY);
        log_clearData(LOG_TYPE_WATERING);
        log_clearData(LOG_TYPE_ERROR);
    }

    if (pbUpdated != NULL)
    {
        *pbUpdated = bUpdatedAny;
    }

    cJSON_Delete(psChunk);
    cJSON_Delete(psAllLogs);
    return ESP_OK;
}
