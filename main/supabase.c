#include "supabase.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

/* --------------------------------------------------------------------------
 * Interne Konstanten
 * -------------------------------------------------------------------------- */
static const char *sc_pacTAG = "SUPABASE";

/** Basis-URL der Supabase REST API */
#define SUPABASE_BASE_URL           "https://your-supabase-url.supabase.co/rest/v1"

/** Supabase API Key */
#define SUPABASE_API_KEY            "your-supabase-api-key"

/** Größe des JSON-Sendepuffers */
#define SUPABASE_TX_BUF_SIZE        1024u

/** Größe des HTTP-Antwortpuffers */
#define SUPABASE_RX_BUF_SIZE        4096u

/* --------------------------------------------------------------------------
 * Hilfsstruktur: HTTP-Antwort-Body puffern
 * -------------------------------------------------------------------------- */
typedef struct {
    char   *pacBuf;
    size_t  uiBufSize;
    size_t  uiLen;
} prv_httpRxCtx_t;

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
 * Öffentliche Funktionen - WiFi & Authentication
 * -------------------------------------------------------------------------- */

int server_init(const char *pacUrl, const char *pacApiKey)
{
    if (pacUrl == NULL || pacApiKey == NULL)
    {
        return -1;
    }

    esp_http_client_config_t sConfig = {
        .url               = pacUrl,
        .method            = HTTP_METHOD_GET,
        .event_handler     = prv_httpEventHandler,
        .user_data         = NULL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 5000,
        .buffer_size       = 1024u,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        return -2;
    }

    esp_http_client_set_header(hClient, "apikey", pacApiKey);
    esp_http_client_set_header(hClient, "Content-Type", "application/json");

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK || i32Status < 200 || i32Status >= 300)
    {
        return -3;
    }

    return 0;
}

/* --------------------------------------------------------------------------
 * Interne HTTP-Helper-Funktionen
 * -------------------------------------------------------------------------- */

static esp_err_t prv_supabasePost(const char *pacTable, const char *pacBody, char *pacRxBuf, size_t uiBufSize)
{
    if (pacTable == NULL || pacBody == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char acUrl[512];
    snprintf(acUrl, sizeof(acUrl), "%s/%s", SUPABASE_BASE_URL, pacTable);

    prv_httpRxCtx_t sRxCtx = {
        .pacBuf    = pacRxBuf,
        .uiBufSize = uiBufSize,
        .uiLen     = 0u,
    };
    if (pacRxBuf != NULL)
    {
        pacRxBuf[0] = '\0';
    }

    esp_http_client_config_t sConfig = {
        .url               = acUrl,
        .method            = HTTP_METHOD_POST,
        .event_handler     = prv_httpEventHandler,
        .user_data         = (pacRxBuf != NULL) ? &sRxCtx : NULL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = 10000,
        .buffer_size       = SUPABASE_RX_BUF_SIZE,
        .buffer_size_tx    = SUPABASE_TX_BUF_SIZE,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "apikey", SUPABASE_API_KEY);
    esp_http_client_set_header(hClient, "Content-Type", "application/json");
    esp_http_client_set_post_field(hClient, pacBody, (int)strlen(pacBody));

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        return ESP_FAIL;
    }

    if (i32Status != 201 && i32Status != 200)
    {
        ESP_LOGW(sc_pacTAG, "POST HTTP-Status %d [/%s]", i32Status, pacTable);
        return ESP_FAIL;
    }

    ESP_LOGI(sc_pacTAG, "Daten gespeichert in /%s", pacTable);
    return ESP_OK;
}

static esp_err_t prv_supabaseGet(const char *pacTable, const char *pacQuery, char *pacRxBuf, size_t uiBufSize)
{
    if (pacTable == NULL || pacRxBuf == NULL || uiBufSize == 0u)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char acUrl[512];
    if (pacQuery != NULL && pacQuery[0] != '\0')
    {
        snprintf(acUrl, sizeof(acUrl), "%s/%s?%s", SUPABASE_BASE_URL, pacTable, pacQuery);
    }
    else
    {
        snprintf(acUrl, sizeof(acUrl), "%s/%s", SUPABASE_BASE_URL, pacTable);
    }

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
        .buffer_size_tx    = 2048u,
    };

    esp_http_client_handle_t hClient = esp_http_client_init(&sConfig);
    if (hClient == NULL)
    {
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "apikey", SUPABASE_API_KEY);
    esp_http_client_set_header(hClient, "Content-Type", "application/json");

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        return ESP_FAIL;
    }

    if (i32Status != 200)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t prv_supabaseUpdate(const char *pacTable, const char *pacQuery, const char *pacBody)
{
    if (pacTable == NULL || pacBody == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char acUrl[512];
    if (pacQuery != NULL && pacQuery[0] != '\0')
    {
        snprintf(acUrl, sizeof(acUrl), "%s/%s?%s", SUPABASE_BASE_URL, pacTable, pacQuery);
    }
    else
    {
        snprintf(acUrl, sizeof(acUrl), "%s/%s", SUPABASE_BASE_URL, pacTable);
    }

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
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "apikey", SUPABASE_API_KEY);
    esp_http_client_set_header(hClient, "Content-Type", "application/json");
    esp_http_client_set_post_field(hClient, pacBody, (int)strlen(pacBody));

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        return ESP_FAIL;
    }

    if (i32Status != 200)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t prv_supabaseDelete(const char *pacTable, const char *pacQuery)
{
    if (pacTable == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char acUrl[512];
    if (pacQuery != NULL && pacQuery[0] != '\0')
    {
        snprintf(acUrl, sizeof(acUrl), "%s/%s?%s", SUPABASE_BASE_URL, pacTable, pacQuery);
    }
    else
    {
        snprintf(acUrl, sizeof(acUrl), "%s/%s", SUPABASE_BASE_URL, pacTable);
    }

    esp_http_client_config_t sConfig = {
        .url               = acUrl,
        .method            = HTTP_METHOD_DELETE,
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
        return ESP_FAIL;
    }

    esp_http_client_set_header(hClient, "apikey", SUPABASE_API_KEY);
    esp_http_client_set_header(hClient, "Content-Type", "application/json");

    esp_err_t eErr  = esp_http_client_perform(hClient);
    int i32Status   = esp_http_client_get_status_code(hClient);
    esp_http_client_cleanup(hClient);

    if (eErr != ESP_OK)
    {
        return ESP_FAIL;
    }

    if (i32Status != 204 && i32Status != 200)
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * Öffentliche API-Funktionen
 * -------------------------------------------------------------------------- */

int server_add_data(const char *table, const char *data)
{
    if (table == NULL || data == NULL)
    {
        return -1;
    }

    esp_err_t eErr = prv_supabasePost(table, data, NULL, 0);
    return (eErr == ESP_OK) ? 0 : -1;
}

int server_write_data(const char *table, const char *json_data)
{
    if (table == NULL || json_data == NULL)
    {
        return -1;
    }

    esp_err_t eErr = prv_supabasePost(table, json_data, NULL, 0);
    return (eErr == ESP_OK) ? 0 : -1;
}

char *server_get_data(const char *table, const char *query)
{
    if (table == NULL)
    {
        return NULL;
    }

    static char acRxBuf[SUPABASE_RX_BUF_SIZE];
    memset(acRxBuf, 0, sizeof(acRxBuf));

    esp_err_t eErr = prv_supabaseGet(table, query, acRxBuf, sizeof(acRxBuf));
    if (eErr == ESP_OK && acRxBuf[0] != '\0')
    {
        return acRxBuf;
    }

    return NULL;
}

int server_read_data(const char *table, const char *query, char *response, size_t response_size)
{
    if (table == NULL || response == NULL || response_size == 0u)
    {
        return -1;
    }

    esp_err_t eErr = prv_supabaseGet(table, query, response, response_size);
    return (eErr == ESP_OK) ? 0 : -1;
}

int server_update_data(const char *table, const char *query, const char *new_data)
{
    if (table == NULL || new_data == NULL)
    {
        return -1;
    }

    esp_err_t eErr = prv_supabaseUpdate(table, query, new_data);
    return (eErr == ESP_OK) ? 0 : -1;
}

int server_delete_data(const char *table, const char *query)
{
    if (table == NULL)
    {
        return -1;
    }

    esp_err_t eErr = prv_supabaseDelete(table, query);
    return (eErr == ESP_OK) ? 0 : -1;
}

int server_authenticate(const char *email, const char *password)
{
    if (email == NULL || password == NULL)
    {
        return -1;
    }

    char acUrl[256];
    snprintf(acUrl, sizeof(acUrl), "%s/auth/v1/token?grant_type=password", SUPABASE_BASE_URL);

    char acBody[256];
    snprintf(acBody, sizeof(acBody), "{\"email\":\"%s\",\"password\":\"%s\"}", email, password);

    char acRxBuf[SUPABASE_RX_BUF_SIZE];
    memset(acRxBuf, 0, sizeof(acRxBuf));

    esp_err_t eErr = prv_supabasePost("auth/v1/token", acBody, acRxBuf, sizeof(acRxBuf));
    if (eErr == ESP_OK)
    {
        return 0;
    }

    return -1;
}