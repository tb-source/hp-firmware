/*
 * provisioning.c
 *
 *  Created on: 21.12.2023
 *      Author: tobby
 */
#include "provisioning.h"
#include "storage.h"
#include "ble_miflora.h"

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <nvs_flash.h>
#include <esp_bt.h>


#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>

static const char *TAG = "app";

void bt_prov_reset(void);

static deviceData_t s_peDevice_data;
static volatile bool s_bProvRunning = false;

static uint8_t s_aui8Sec2Salt[BT_SALT_LEN];
static uint8_t s_aui8Sec2Verifier[BT_VERIFIER_LEN];

static esp_err_t prv_prepare_bt_controller_for_provisioning(void)
{
    esp_bt_controller_status_t eStatus = esp_bt_controller_get_status();

    if (eStatus == ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
        esp_err_t eErr = esp_bt_controller_disable();
        if (eErr != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_bt_controller_disable failed: %s", esp_err_to_name(eErr));
            return eErr;
        }
        eStatus = esp_bt_controller_get_status();
    }

    if (eStatus == ESP_BT_CONTROLLER_STATUS_INITED)
    {
        esp_err_t eErr = esp_bt_controller_deinit();
        if (eErr != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_bt_controller_deinit failed: %s", esp_err_to_name(eErr));
            return eErr;
        }
    }

    return ESP_OK;
}

static esp_err_t example_get_sec2_salt(const char **salt, uint16_t *salt_len) {
    if (salt == NULL || salt_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t eErr = storage_readBtSalt(s_aui8Sec2Salt, sizeof(s_aui8Sec2Salt));
    if (eErr != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sec2 salt from NVS: %s", esp_err_to_name(eErr));
        return eErr;
    }

    *salt = (const char *)s_aui8Sec2Salt;
    *salt_len = (uint16_t)sizeof(s_aui8Sec2Salt);
    return ESP_OK;
}

static esp_err_t example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len) {
    if (verifier == NULL || verifier_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t eErr = storage_readBtVerifier(s_aui8Sec2Verifier, sizeof(s_aui8Sec2Verifier));
    if (eErr != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sec2 verifier from NVS: %s", esp_err_to_name(eErr));
        return eErr;
    }

    *verifier = (const char *)s_aui8Sec2Verifier;
    *verifier_len = (uint16_t)sizeof(s_aui8Sec2Verifier);
    return ESP_OK;
}


/* Signal Wi-Fi events on this event-group */
const int WIFI_CONNECTED_EVENT = BIT0;
const int BLE_DISCONNECTED = BIT1;
static EventGroupHandle_t wifi_event_group;
static volatile int iTimeKeeper = 0; 

#define PROV_TRANSPORT_SOFTAP   "softap"
#define PROV_TRANSPORT_BLE      "ble"

/* Event handler for catching system events */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    static int retries;
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG, "Provisioning started");
                break;
            case WIFI_PROV_CRED_RECV: {
                wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
                ESP_LOGI(TAG, "Received Wi-Fi credentials"
                         "\n\tSSID     : %s\n\tPassword : %s",
                         (const char *) wifi_sta_cfg->ssid,
                         (const char *) wifi_sta_cfg->password);
                break;
            }
            case WIFI_PROV_CRED_FAIL: {
                wifi_prov_sta_fail_reason_t *reason = (wifi_prov_sta_fail_reason_t *)event_data;
                ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                         "\n\tPlease reset to factory and retry provisioning",
                         (*reason == WIFI_PROV_STA_AUTH_ERROR) ?
                         "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
                retries++;
                if (retries >= 3) {
                    ESP_LOGI(TAG, "Failed to connect with provisioned AP, reseting provisioned credentials");
                    wifi_prov_mgr_reset_sm_state_on_failure();
                    retries = 0;
                }
                break;
            }
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG, "Provisioning successful");
                retries = 0;
                break;
            case WIFI_PROV_END:
                /* Manager deinit is handled in bt_prov_cleanup */
                break;
            default:
                break;
        }
    } else if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
                esp_wifi_connect();
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
        
        /* Save WiFi credentials to storage */
        wifi_config_t wifi_cfg;
        esp_err_t eErr = esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);
        if (eErr == ESP_OK)
        {
            credentials_t sCredentials = {0};
            
            /* Read existing credentials */
            eErr = storage_readCredentials(&sCredentials);
            if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
            {
                ESP_LOGW(TAG, "Failed to read existing credentials: %s", esp_err_to_name(eErr));
            }
            
            /* Update WiFi credentials */
            strncpy(sCredentials.wifiSsid, (const char *)wifi_cfg.sta.ssid, sizeof(sCredentials.wifiSsid) - 1u);
            sCredentials.wifiSsid[sizeof(sCredentials.wifiSsid) - 1u] = '\0';
            
            strncpy(sCredentials.wifiPassword, (const char *)wifi_cfg.sta.password, sizeof(sCredentials.wifiPassword) - 1u);
            sCredentials.wifiPassword[sizeof(sCredentials.wifiPassword) - 1u] = '\0';
            
            /* Write updated credentials back to storage */
            eErr = storage_writeCredentials(&sCredentials);
            if (eErr == ESP_OK)
            {
                ESP_LOGI(TAG, "WiFi credentials saved to storage");
            }
            else
            {
                ESP_LOGW(TAG, "Failed to save WiFi credentials: %s", esp_err_to_name(eErr));
            }
        }
        else
        {
            ESP_LOGW(TAG, "Failed to get WiFi config: %s", esp_err_to_name(eErr));
        }
        
        /* Signal main application to continue execution */
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
    } else if (event_base == PROTOCOMM_TRANSPORT_BLE_EVENT) {
        switch (event_id) {
            case PROTOCOMM_TRANSPORT_BLE_CONNECTED:
                ESP_LOGI(TAG, "BLE transport: Connected!");
                led_set(1,LED_ON);
                break;
            case PROTOCOMM_TRANSPORT_BLE_DISCONNECTED:
                ESP_LOGI(TAG, "BLE transport: Disconnected!");
                xEventGroupSetBits(wifi_event_group, BLE_DISCONNECTED);
                break;
            default:
                break;
        }
    } else if (event_base == PROTOCOMM_SECURITY_SESSION_EVENT) {
        switch (event_id) {
            case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
                ESP_LOGI(TAG, "Secured session established!");
                break;
            case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
                ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
                break;
            case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
                ESP_LOGE(TAG, "Received incorrect username and/or PoP for establishing secure session!");
                break;
            default:
                break;
        }
    }
}

static void wifi_init_sta(void)
{
    /* Start Wi-Fi in station mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void get_device_service_name(char *service_name, size_t max)
{
    // *service_name = "PROV_84EAE0"; // Example service name, replace with actual logic to generate unique name
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    const char *ssid_suffix = "FDC6DC";
    snprintf(service_name, max, "%s%s", ssid_prefix, ssid_suffix);
    // esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    // snprintf(service_name, max, "%s%02X%02X%02X",
    //          ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
             
}

/* Handler for the optional provisioning endpoint registered by the application.
 * The data format can be chosen by applications. Here, we are using plain ascii text.
 * Applications can choose to use other formats like protobuf, JSON, XML, etc.
 */
esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    (void)session_id;
    (void)priv_data;

    *outbuf = NULL;
    *outlen = 0;

    if (inbuf && inlen > 0) {
        char acInputString[500];
        size_t uiInputLen = (size_t)inlen;
        if (uiInputLen >= sizeof(acInputString)) {
            uiInputLen = sizeof(acInputString) - 1U;
            ESP_LOGW(TAG, "Provisioning payload truncated from %d to %d bytes", (int)inlen, (int)uiInputLen);
        }

        memcpy(acInputString, inbuf, uiInputLen);
        acInputString[uiInputLen] = 0;

        ESP_LOGI(TAG, "Received data: %.*s", (int)uiInputLen, acInputString);
        *outbuf = (uint8_t*)strdup(pacData_send_receive(acInputString, &s_peDevice_data));
        iTimeKeeper = 0;
    }
    else
    {
        *outbuf = (uint8_t *)strdup("Error");
        ESP_LOGE(TAG, "Received empty string");
    }

    if (*outbuf == NULL) {
        ESP_LOGE(TAG, "System out of memory");
        return ESP_ERR_NO_MEM;
    }
    *outlen = strlen((char *)*outbuf) + 1; /* +1 for NULL terminating byte */

    return ESP_OK;
}

void bt_prov(deviceData_t* peDevice_data)
{
    bool bProvMgrInit = false;
    esp_err_t eErr;

    if (s_bProvRunning)
    {
        ESP_LOGW(TAG, "Provisioning already running, ignoring trigger");
        return;
    }
    s_bProvRunning = true;

    led_set(1,LED_BLINK_SLOW);
    //set data 
    s_peDevice_data = *peDevice_data;

    /* Ensure any optional MiFlora NimBLE lifecycle is stopped before
     * starting BLE provisioning. */
    esp_err_t eMifloraStop = ble_miflora_deinit();
    if (eMifloraStop != ESP_OK)
    {
        ESP_LOGW(TAG, "ble_miflora_deinit before provisioning failed: %s", esp_err_to_name(eMifloraStop));
    }

    eErr = prv_prepare_bt_controller_for_provisioning();
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to prepare BT controller for provisioning: %s", esp_err_to_name(eErr));
        goto bt_prov_cleanup;
    }

    /* NVS bereits durch storage_init() initialisiert */

    /* Initialize TCP/IP (idempotent) */
    eErr = esp_netif_init();
    if ((eErr != ESP_OK) && (eErr != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(eErr));
        goto bt_prov_cleanup;
    }

    /* Initialize the event loop (ignoriere Fehler falls bereits erstellt) */
    eErr = esp_event_loop_create_default();
    if ((eErr != ESP_OK) && (eErr != ESP_ERR_INVALID_STATE))
    {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(eErr));
        goto bt_prov_cleanup;
    }
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create wifi event group");
        goto bt_prov_cleanup;
    }

    /* Register our event handler for Wi-Fi, IP and Provisioning related events */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_TRANSPORT_BLE_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    /* Release Classic BT memory before WiFi init.
     * This MUST be done before esp_wifi_init() to prevent heap corruption of
     * the interrupt allocator's vector_desc_t linked list (seen as LoadProhibited
     * in find_desc_for_source during hli_queue_setup when BT controller inits).
     * Official ESP-IDF BLE+WiFi examples always release Classic BT memory first. */
    {
        esp_err_t eMemRel = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        if (eMemRel != ESP_OK && eMemRel != ESP_ERR_INVALID_STATE)
        {
            ESP_LOGW(TAG, "esp_bt_controller_mem_release: %s", esp_err_to_name(eMemRel));
        }
    }

    /* Initialize Wi-Fi including netif with default config */
    static bool s_bNetifCreated = false;
    if (!s_bNetifCreated)
    {
        esp_netif_create_default_wifi_sta();
        s_bNetifCreated = true;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Configuration for the provisioning manager */
    wifi_prov_mgr_config_t config = {
        /* What is the Provisioning Scheme that we want ?
         * wifi_prov_scheme_softap or wifi_prov_scheme_ble */
        .scheme = wifi_prov_scheme_ble,

        /* Any default scheme specific event handler that you would
         * like to choose. Since our example application requires
         * neither BT nor BLE, we can choose to release the associated
         * memory once provisioning is complete, or not needed
         * (in case when device is already provisioned). Choosing
         * appropriate scheme specific event handler allows the manager
         * to take care of this automatically. This can be set to
         * WIFI_PROV_EVENT_HANDLER_NONE when using wifi_prov_scheme_softap*/

        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE
    };

    /* Initialize provisioning manager with the
     * configuration parameters set above */
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));
    bProvMgrInit = true;


    bool provisioned = false;
    wifi_prov_mgr_reset_provisioning();

    /* Let's find out if the device is provisioned */
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    /* If device is not yet provisioned start provisioning service */
    if (!provisioned) {
        ESP_LOGI(TAG, "Starting provisioning");

        /* What is the Device Service Name that we want
         * This translates to :
         *     - Wi-Fi SSID when scheme is wifi_prov_scheme_softap
         *     - device name when scheme is wifi_prov_scheme_ble
         */
        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));

        wifi_prov_security_t security = WIFI_PROV_SECURITY_2;
        /* The username must be the same one, which has been used in the generation of salt and verifier */

        /* This is the structure for passing security parameters
         * for the protocomm security 2.
         * If dynamically allocated, sec2_params pointer and its content
         * must be valid till WIFI_PROV_END event is triggered.
         */
        wifi_prov_security2_params_t sec2_params = {};
        const void *pvSecParams = &sec2_params;

        eErr = example_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len);
        if (eErr == ESP_OK)
        {
            eErr = example_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len);
        }

        if (eErr != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to load Security2 params: %s", esp_err_to_name(eErr));
            ESP_LOGE(TAG, "Security2 credentials are mandatory. Set $btSalt and $btVerifi via UART.");
            wifi_prov_mgr_deinit();
            bProvMgrInit = false;
            goto bt_prov_cleanup;
        }

        /* What is the service key (could be NULL)
         * This translates to :
         *     - Wi-Fi password when scheme is wifi_prov_scheme_softap
         *          (Minimum expected length: 8, maximum 64 for WPA2-PSK)
         *     - simply ignored when scheme is wifi_prov_scheme_ble
         */
        const char *service_key = NULL;


        /* This step is only useful when scheme is wifi_prov_scheme_ble. This will
         * set a custom 128 bit UUID which will be included in the BLE advertisement
         * and will correspond to the primary GATT service that provides provisioning
         * endpoints as GATT characteristics. Each GATT characteristic will be
         * formed using the primary service UUID as base, with different auto assigned
         * 12th and 13th bytes (assume counting starts from 0th byte). The client side
         * applications must identify the endpoints by reading the User Characteristic
         * Description descriptor (0x2901) for each characteristic, which contains the
         * endpoint name of the characteristic */
        uint8_t custom_service_uuid[] = {
            /* LSB <---------------------------------------
             * ---------------------------------------> MSB */
            0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf,
            0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
        };

        wifi_prov_scheme_ble_set_service_uuid(custom_service_uuid);

        /* An optional endpoint that applications can create if they expect to
         * get some additional custom data during provisioning workflow.
         * The endpoint name can be anything of your choice.
         * This call must be made before starting the provisioning.
         */
        wifi_prov_mgr_endpoint_create("custom-data");
        /* Do not stop and de-init provisioning even after success,
         * so that we can restart it later. */
        wifi_prov_mgr_disable_auto_stop(1000);
        /* Start provisioning service */
        eErr = wifi_prov_mgr_start_provisioning(security, pvSecParams, service_name, service_key);
        if (eErr != ESP_OK)
        {
            ESP_LOGE(TAG, "wifi_prov_mgr_start_provisioning failed: %s", esp_err_to_name(eErr));
            goto bt_prov_cleanup;
        }
        /* The handler for the optional endpoint created above.
         * This call must be made after starting the provisioning, and only if the endpoint
         * has already been created above.
         */
        eErr = wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);
        if (eErr != ESP_OK)
        {
            ESP_LOGE(TAG, "wifi_prov_mgr_endpoint_register failed: %s", esp_err_to_name(eErr));
            goto bt_prov_cleanup;
        }

        /* Uncomment the following to wait for the provisioning to finish and then release
         * the resources of the manager. Since in this case de-initialization is triggered
         * by the default event loop handler, we don't need to call the following */
        // wifi_prov_mgr_wait();
        // wifi_prov_mgr_deinit();
        ESP_LOGI(TAG, "Provisioning started");
     } 
     else 
     {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

        /* We don't need the manager as device is already provisioned,
         * so let's release it's resources */
        wifi_prov_mgr_deinit();
        bProvMgrInit = false;

        /* Start Wi-Fi station */
        wifi_init_sta();
    }

    /* Wait for Wi-Fi connection */
    // xEventGroupWaitBits(wifi_event_group, BLE_DISCONNECTED, true, true, portMAX_DELAY);
    while(1)
    {
        if(xEventGroupGetBits(wifi_event_group) & BLE_DISCONNECTED)
        {
            break;
        }
        //timeout after 1min
        if(iTimeKeeper > 60)
        {
            break;
        }
        vTaskDelay(1000);       //wait 1s
        iTimeKeeper++;
    }

    bt_prov_cleanup:
    
    led_set(1,LED_OFF);
    ESP_LOGI("PROV", "Disconnect");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (bProvMgrInit)
    {
        wifi_prov_mgr_deinit();
    }

    /* Event-Handler abmelden */
    esp_event_handler_unregister(WIFI_PROV_EVENT,                  ESP_EVENT_ANY_ID,    &event_handler);
    esp_event_handler_unregister(PROTOCOMM_TRANSPORT_BLE_EVENT,    ESP_EVENT_ANY_ID,    &event_handler);
    esp_event_handler_unregister(PROTOCOMM_SECURITY_SESSION_EVENT, ESP_EVENT_ANY_ID,    &event_handler);
    esp_event_handler_unregister(WIFI_EVENT,                       ESP_EVENT_ANY_ID,    &event_handler);
    esp_event_handler_unregister(IP_EVENT,                         IP_EVENT_STA_GOT_IP, &event_handler);

    if (wifi_event_group != NULL)
    {
        vEventGroupDelete(wifi_event_group);
        wifi_event_group = NULL;
    }

    /* WiFi stoppen damit firestore_wifiConnect() sauber neu starten kann */
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    // wifi_prov_mgr_reset_sm_state_for_reprovision(); 
    // xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);
    *peDevice_data = s_peDevice_data;
    s_bProvRunning = false;
}

// void bt_prov_state()
// {

// }

void bt_prov_reset(void)
{
    /* Resetting provisioning state machine to enable re-provisioning */
    wifi_prov_mgr_reset_sm_state_for_reprovision();    
    /* Wait for Wi-Fi connection */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);  
}
