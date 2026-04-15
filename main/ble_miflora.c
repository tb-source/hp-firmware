/*
 * ble_miflora.c
 *
 *  Created on: 07.10.2023
 *      Author: tobby
 *

 * ┌──────────────────────────────────────────────────────────────┐
 * │  Protokollablauf                                             │
 * │  1. nimble_port_init + ble_hs_cfg konfigurieren             │
 * │  2. GAP-Scan starten (active scan)                           │
 * │  3. BLE_GAP_EVENT_DISC: Gerätename prüfen → connect         │
 * │  4. BLE_GAP_EVENT_CONNECT: GATT Service-Discovery starten   │
 * │  5. ble_gattc_disc_all_svcs → ble_gattc_disc_all_chrs       │
 * │  6. Write 0xA01F auf Handle 0x0033 (Echtzeit-Modus)         │
 * │  7. Read Handle 0x0035 → Sensor-Daten parsen                │
 * │  8. Read Handle 0x0038 → Batterie + Firmware parsen         │
 * │  9. ble_gap_terminate → Verbindung sauber trennen           │
 * └──────────────────────────────────────────────────────────────┘
 *
 * sdkconfig (menuconfig):
 *   Component config → Bluetooth → NimBLE Options → [*] NimBLE host
 *   CONFIG_BT_NIMBLE_ENABLED=y
 *   CONFIG_BT_NIMBLE_ROLE_CENTRAL=y
 *
 * CMakeLists.txt REQUIRES:
 *   bt nvs_flash esp_event
 */
 

 

#include "ble_miflora.h"
#include "watering.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  KONFIGURATION  –  hier anpassen
 * ═══════════════════════════════════════════════════════════════════════════ */
 
#define MIFLORA_USE_FIXED_MAC

/**
 * Sensor-Tabelle: MAC-Adressen aller bekannten Sensoren.
 *
 * Sensor-Nummer (ui32SensorNb) entspricht dem Index in diesem Array (0-basiert).
 * MAC-Format: genau so wie in nRF Connect angezeigt,
 *   z.B. "5C:85:7E:14:39:D3" -> { 0x5C, 0x85, 0x7E, 0x14, 0x39, 0xD3 }
 *
 * Adresstyp:
 *   BLE_ADDR_PUBLIC  - Standard bei Mi Flora (steht in nRF Connect neben der Adresse)
 *   BLE_ADDR_RANDOM  - falls nRF Connect "Random" anzeigt
 */
#define MIFLORA_ADDR_TYPE   BLE_ADDR_PUBLIC
 
/* ─── Gemeinsame Konstanten ──────────────────────────────────────────────── */
 
#define TAG                 "MI_FLORA"
 
#define HANDLE_WRITE_MODE   0x0033   /**< Echtzeit-Modus aktivieren  */
#define HANDLE_SENSOR_DATA  0x0035   /**< Temp / Lux / Moisture / EC */
#define HANDLE_DEVICE_INFO  0x0038   /**< Batterie + Firmware-String */
 
#define CONNECT_TIMEOUT_MS  30000    /**< Verbindungs-Timeout        */
 
static const uint8_t REALTIME_CMD[] = {0xA0, 0x1F};
 

/* ═══════════════════════════════════════════════════════════════════════════
 *  GLOBALER ZUSTAND
 * ═══════════════════════════════════════════════════════════════════════════ */
 
static miflora_data_t    g_data;
static miflora_phase_t   g_phase      = PHASE_IDLE;
static uint16_t          g_conn_hdl   = BLE_HS_CONN_HANDLE_NONE;
static SemaphoreHandle_t g_done_sem   = NULL;
static volatile bool     g_stack_ready = false; /**< true sobald on_ble_sync gefeuert hat */
static bool              g_nimble_started = false;
static bool              g_sensor_enabled[CHANNELCOUNT] = { false, false, false };
static uint8_t           g_mac_table[CHANNELCOUNT][6] = {};

static bool prv_anySensorEnabled(void)
{
    for (uint32_t uiI = 0u; uiI < CHANNELCOUNT; uiI++)
    {
        if (g_sensor_enabled[uiI])
        {
            return true;
        }
    }
    return false;
}


static bool prv_isMacValid(const uint8_t aui8Mac[6])
{
    bool bAnyNonZero = false;
    bool bAnyNonFF   = false;

    for (uint32_t uiI = 0; uiI < 6u; uiI++)
    {
        if (aui8Mac[uiI] != 0u) { bAnyNonZero = true; }
        if (aui8Mac[uiI] != 0xFFu) { bAnyNonFF = true; }
    }

    return bAnyNonZero && bAnyNonFF;
}

void ble_miflora_setChannelData(const deviceData_t *psDevData)
{
    if (psDevData == NULL)
    {
        return;
    }

    for (uint32_t uiI = 0u; uiI < CHANNELCOUNT; uiI++)
    {
        const channelData_t *psChannel = &psDevData->channels[uiI];
        g_sensor_enabled[uiI] = psChannel->moisture.senseEnable;

        const uint8_t *pui8Mac = psChannel->moisture.macTable;
        if (prv_isMacValid(pui8Mac))
        {
            memcpy(g_mac_table[uiI], pui8Mac, 6u);
        }
    }
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 *  HILFSFUNKTIONEN: ROHDATEN PARSEN
 * ═══════════════════════════════════════════════════════════════════════════ */
 
/**
 * Sensor-Payload (Handle 0x0035) parsen.
 * Byte-Layout:
 *   [0-1]  Temperatur x10, little-endian int16
 *   [2]    reserviert
 *   [3-6]  Beleuchtungsstaerke, little-endian uint32
 *   [7]    Bodenfeuchtigkeit uint8
 *   [8-9]  Leitfaehigkeit, little-endian uint16
 */
static void parse_sensor_data(const uint8_t *buf, uint16_t len)
{
    if (len < 10) {
        ESP_LOGE(TAG, "Sensor-Payload zu kurz (%u Byte)", len);
        return;
    }
    int16_t raw_t       = (int16_t)((buf[1] << 8) | buf[0]);
    g_data.temperature  = raw_t / 10.0f;
    g_data.illuminance  = (uint32_t) buf[3]
                        | ((uint32_t)buf[4] <<  8)
                        | ((uint32_t)buf[5] << 16)
                        | ((uint32_t)buf[6] << 24);
    g_data.moisture     = buf[7];
    g_data.conductivity = (uint16_t)((buf[9] << 8) | buf[8]);
}
 
/**
 * DeviceInfo-Payload (Handle 0x0038) parsen.
 * Byte-Layout:
 *   [0]    Batteriestand %
 *   [1]    0x20 (Trennzeichen)
 *   [2..]  Firmware-Version ASCII
 */
static void parse_device_info(const uint8_t *buf, uint16_t len)
{
    if (len < 3) {
        ESP_LOGE(TAG, "DeviceInfo-Payload zu kurz (%u Byte)", len);
        return;
    }
    g_data.battery = buf[0];
    uint16_t fw_len = (len - 2) < (sizeof(g_data.firmware) - 1)
                    ? (len - 2)
                    : (uint16_t)(sizeof(g_data.firmware) - 1);
    memcpy(g_data.firmware, &buf[2], fw_len);
    g_data.firmware[fw_len] = '\0';
}
 
static void print_sensor_data(void)
{
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  Temperatur    : %.1f C",    g_data.temperature);
    ESP_LOGI(TAG, "  Helligkeit    : %lu lux",   (unsigned long)g_data.illuminance);
    ESP_LOGI(TAG, "  Bodenfeucht.  : %u %%",     g_data.moisture);
    ESP_LOGI(TAG, "  Leitfaehigkeit: %u uS/cm",  g_data.conductivity);
    ESP_LOGI(TAG, "  Batterie      : %u %%",      g_data.battery);
    ESP_LOGI(TAG, "  Firmware      : %s",         g_data.firmware);
    ESP_LOGI(TAG, "======================================");
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 *  FORWARD-DEKLARATIONEN
 * ═══════════════════════════════════════════════════════════════════════════ */
 
static int on_read_sensor(uint16_t conn_hdl,
                          const struct ble_gatt_error *error,
                          struct ble_gatt_attr *attr,
                          void *arg);
 
static int on_read_device_info(uint16_t conn_hdl,
                               const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr,
                               void *arg);
 
static int gap_event_handler(struct ble_gap_event *event, void *arg);
static int sniff_gap_event_handler(struct ble_gap_event *event, void *arg);
 
/* ═══════════════════════════════════════════════════════════════════════════
 *  GATT-CALLBACKS  (identisch in beiden Modi)
 * ═══════════════════════════════════════════════════════════════════════════ */
 
static int on_write_mode(uint16_t conn_hdl,
                         const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr,
                         void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Write-Mode fehlgeschlagen: status=%d", error->status);
        g_phase = PHASE_ERROR;
        xSemaphoreGive(g_done_sem);
        return 0;
    }
    ESP_LOGI(TAG, "Echtzeit-Modus aktiviert, lese Sensor-Daten ...");
    g_phase = PHASE_READ_SENSOR;
 
    int rc = ble_gattc_read(conn_hdl, HANDLE_SENSOR_DATA, on_read_sensor, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_read(SensorData) rc=%d", rc);
        g_phase = PHASE_ERROR;
        xSemaphoreGive(g_done_sem);
    }
    return 0;
}
 
static int on_read_sensor(uint16_t conn_hdl,
                          const struct ble_gatt_error *error,
                          struct ble_gatt_attr *attr,
                          void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Read-Sensor fehlgeschlagen: status=%d", error->status);
        g_phase = PHASE_ERROR;
        xSemaphoreGive(g_done_sem);
        return 0;
    }
    uint8_t buf[16] = {0};
    uint16_t len    = OS_MBUF_PKTLEN(attr->om);
    if (len > sizeof(buf)) len = sizeof(buf);
    ble_hs_mbuf_to_flat(attr->om, buf, len, NULL);
    ESP_LOG_BUFFER_HEX(TAG, buf, len);
    parse_sensor_data(buf, len);
 
    g_phase = PHASE_READ_INFO;
    int rc = ble_gattc_read(conn_hdl, HANDLE_DEVICE_INFO, on_read_device_info, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_read(DeviceInfo) rc=%d", rc);
        g_phase = PHASE_ERROR;
        xSemaphoreGive(g_done_sem);
    }
    return 0;
}
 
static int on_read_device_info(uint16_t conn_hdl,
                               const struct ble_gatt_error *error,
                               struct ble_gatt_attr *attr,
                               void *arg)
{
    if (error->status != 0) {
        ESP_LOGE(TAG, "Read-DeviceInfo fehlgeschlagen: status=%d", error->status);
        g_phase = PHASE_ERROR;
        xSemaphoreGive(g_done_sem);
        return 0;
    }
    uint8_t buf[16] = {0};
    uint16_t len    = OS_MBUF_PKTLEN(attr->om);
    if (len > sizeof(buf)) len = sizeof(buf);
    ble_hs_mbuf_to_flat(attr->om, buf, len, NULL);
    ESP_LOG_BUFFER_HEX(TAG, buf, len);
    parse_device_info(buf, len);
 
    g_data.valid = true;
    g_phase      = PHASE_DONE;
    print_sensor_data();
 
    ble_gap_terminate(conn_hdl, BLE_ERR_REM_USER_CONN_TERM);
    return 0;
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 *  GAP EVENT HANDLER  (identisch in beiden Modi)
 * ═══════════════════════════════════════════════════════════════════════════ */
 
static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
 
    /* ── Verbindung hergestellt ─────────────────────────────────────────── */
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "Verbindungsfehler: status=%d", event->connect.status);
            g_phase = PHASE_ERROR;
            xSemaphoreGive(g_done_sem);
            break;
        }
        g_conn_hdl = event->connect.conn_handle;
        ESP_LOGI(TAG, "Verbunden (conn_hdl=%d)", g_conn_hdl);
 
        g_phase = PHASE_WRITE_MODE;
        {
            int rc = ble_gattc_write_flat(g_conn_hdl,
                                          HANDLE_WRITE_MODE,
                                          REALTIME_CMD,
                                          sizeof(REALTIME_CMD),
                                          on_write_mode,
                                          NULL);
            if (rc != 0) {
                ESP_LOGE(TAG, "ble_gattc_write_flat rc=%d", rc);
                g_phase = PHASE_ERROR;
                xSemaphoreGive(g_done_sem);
            }
        }
        break;
 
    /* ── Verbindung getrennt ────────────────────────────────────────────── */
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Verbindung getrennt (reason=0x%02x)",
                 event->disconnect.reason);
        g_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
        if (g_phase != PHASE_DONE) {
            g_phase = PHASE_ERROR;
        }
        xSemaphoreGive(g_done_sem);
        break;
 
    default:
        break;
    }
    return 0;
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 *  on_ble_sync  –  reiner Stack-Ready-Callback (kein Connect hier)
 * ═══════════════════════════════════════════════════════════════════════════ */
 
static void on_ble_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr rc=%d", rc);
        return;
    }
    /* Nur Ready-Flag setzen. on_ble_sync kann mehrfach aufgerufen werden
     * (z.B. nach Host-Reset). Kein Connect hier – das macht miflora_read_sensor(). */
    if (!g_stack_ready) {
        g_stack_ready = true;
        ESP_LOGI(TAG, "BLE-Stack bereit");
    }
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 *  start_connect  –  Connect oder Scan starten (aus miflora_read_sensor)
 * ═══════════════════════════════════════════════════════════════════════════ */
 
static esp_err_t start_connect(uint32_t ui32SensorIdx)
{
    /* Laufenden Scan abbrechen falls aktiv */
    if (ble_gap_disc_active()) ble_gap_disc_cancel();
    /* Hinweis: ble_gap_conn_active() wird NICHT geprueft hier –
     * miflora_read_sensor() wartet bereits auf DISCONNECT bevor es
     * start_connect() erneut aufruft. */
 
    if (ui32SensorIdx >= CHANNELCOUNT) {
        ESP_LOGE(TAG, "Ungueltige Sensor-Nummer: %lu", (unsigned long)ui32SensorIdx);
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t *mac = g_mac_table[ui32SensorIdx];
 
    /* NimBLE speichert MAC in umgekehrter Reihenfolge (little-endian) */
    ble_addr_t peer = { .type = MIFLORA_ADDR_TYPE };
    for (int i = 0; i < 6; i++) peer.val[i] = mac[5 - i];
 
    g_phase = PHASE_CONNECTING;
    ESP_LOGI(TAG, "[FIXED MAC] Sensor %lu | %02X:%02X:%02X:%02X:%02X:%02X ...",
             (unsigned long)ui32SensorIdx,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
 
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &peer,
                             CONNECT_TIMEOUT_MS, NULL,
                             gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect rc=%d", rc);
        g_phase = PHASE_ERROR;
        return ESP_FAIL;
    }
 
    return ESP_OK;
}
 
static void on_ble_reset(int reason)
{
    ESP_LOGW(TAG, "BLE-Host zurueckgesetzt (reason=%d)", reason);
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 *  NIMBLE HOST-TASK
 * ═══════════════════════════════════════════════════════════════════════════ */
 
static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE Host-Task gestartet");
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
}
 
/* ═══════════════════════════════════════════════════════════════════════════
 *  OEFFENTLICHE API
 * ═══════════════════════════════════════════════════════════════════════════ */
 
/**
 * @brief NimBLE-Stack initialisieren (einmalig beim Start aufrufen).
 */
static void miflora_nimble_init(void)
{
    if (g_nimble_started)
    {
        return;
    }

    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.reset_cb        = on_ble_reset;
    ble_hs_cfg.sync_cb         = on_ble_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_svc_gap_device_name_set("esp32-miflora");
    nimble_port_freertos_init(nimble_host_task);
    g_nimble_started = true;
}
 
/**
 * @brief NimBLE-Stack und alle Ressourcen freigeben.
 *
 * Trennt eine laufende BLE-Verbindung, stoppt den NimBLE Host-Task,
 * deinitialisiert den BLE-Stack und gibt den internen Semaphore frei.
 * Nach diesem Aufruf muss miflora_nimble_init() erneut aufgerufen werden,
 * bevor miflora_read_sensor() wieder verwendbar ist.
 *
 * @return ESP_OK bei Erfolg, andernfalls ESP-Fehlercode
 */
esp_err_t miflora_nimble_deinit(void)
{
    if (!g_nimble_started)
    {
        g_stack_ready = false;
        g_phase = PHASE_IDLE;
        g_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
        if (g_done_sem != NULL)
        {
            vSemaphoreDelete(g_done_sem);
            g_done_sem = NULL;
        }
        return ESP_OK;
    }

    /* Laufende BLE-Verbindung sauber trennen */
    if (g_conn_hdl != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "Trenne aktive Verbindung (conn_hdl=%d) ...", g_conn_hdl);
        ble_gap_terminate(g_conn_hdl, BLE_ERR_REM_USER_CONN_TERM);
        /* Kurz warten damit der Stack die Trennung verarbeiten kann */
        vTaskDelay(pdMS_TO_TICKS(200));
        g_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
    }
 
    /* Laufenden Scan abbrechen */
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }
 
    /* Stack-Ready-Flag sofort loeschen damit read() nicht loslegt */
    g_stack_ready = false;
 
    /* NimBLE Host-Task stoppen (nimble_port_run() kehrt zurueck).
     * Fehler hier nur loggen – deinit trotzdem weiterfuehren. */
    esp_err_t ret = nimble_port_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_stop fehlgeschlagen: %s", esp_err_to_name(ret));
        return ret;
    } else {
        /* Kurz warten damit der Host-Task sauber beendet wird */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
 
    /* Stack deinitialisieren und Controller freigeben */
    ret = nimble_port_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_deinit fehlgeschlagen: %s", esp_err_to_name(ret));
        return ret;
    }
 
    /* Semaphore freigeben */
    if (g_done_sem) {
        vSemaphoreDelete(g_done_sem);
        g_done_sem = NULL;
    }
 
    /* Internen Zustand vollstaendig zuruecksetzen */
    g_phase    = PHASE_IDLE;
    g_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
    g_nimble_started = false;
 
    ESP_LOGI(TAG, "miflora_nimble_deinit abgeschlossen");
    return ESP_OK;
}
 
/**
 * @brief Sensor auslesen - blockiert bis Daten vorliegen oder Timeout.
 *
 * @param[in]  ui32SensorNb Sensor-Nummer (0-basiert).
 *                          Index in der konfigurierten MAC-Tabelle.
 * @param[out] out          Zielstruktur fuer Messwerte. Darf nicht NULL sein.
 * @param[in]  timeout_ms   Maximale Wartezeit in Millisekunden.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, ESP_ERR_TIMEOUT oder ESP_FAIL
 */
esp_err_t miflora_read_sensor(uint32_t ui32SensorNb, miflora_data_t *out, uint32_t timeout_ms)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    if (ui32SensorNb >= CHANNELCOUNT) {
        ESP_LOGE(TAG, "Sensor-Nummer %lu ungueltig (max %d)",
                 (unsigned long)ui32SensorNb, CHANNELCOUNT - 1);
        return ESP_ERR_INVALID_ARG;
    }

    if (!g_sensor_enabled[ui32SensorNb]) {
        memset(out, 0, sizeof(*out));
        out->valid = false;
        ESP_LOGI(TAG, "Sensor %lu deaktiviert (HUM.SENS=false)",
                 (unsigned long)ui32SensorNb);
        return ESP_OK;
    }
 
    memset(&g_data, 0, sizeof(g_data));
 
    /* Warten bis on_ble_sync() den Stack als bereit gemeldet hat.
     * Nach einem re-init kann das einige hundert ms dauern. */
    uint32_t wait_ms = 0;
    while (!g_stack_ready && wait_ms < 5000) {
        vTaskDelay(pdMS_TO_TICKS(50));
        wait_ms += 50;
    }
    if (!g_stack_ready) {
        ESP_LOGE(TAG, "BLE-Stack nicht bereit nach %lu ms", (unsigned long)wait_ms);
        return ESP_ERR_TIMEOUT;
    }
 
    /* Semaphore aus vorherigem Lauf leeren (defensiv) */
    xSemaphoreTake(g_done_sem, 0);
 
    /* Zustand fuer neuen Lauf zuruecksetzen */
    g_phase = PHASE_IDLE;
 
    /* Verbindung aufbauen (Scan oder direkt je nach Modus) */
    esp_err_t conn_err = start_connect(ui32SensorNb);
    if (conn_err != ESP_OK) {
        return conn_err;
    }
 
    /* Auf Abschluss warten */
    if (xSemaphoreTake(g_done_sem, pdMS_TO_TICKS(timeout_ms)) == pdFALSE) {
        /* Verbindungsversuch abbrechen */
        ble_gap_disc_cancel();
        if (g_conn_hdl != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(g_conn_hdl, BLE_ERR_REM_USER_CONN_TERM);
        } else {
            /* Pending connect (noch kein conn_hdl) abbrechen –
             * loest BLE_GAP_EVENT_CONNECT mit status=29 aus */
            ble_gap_conn_cancel();
        }
        /* Warten bis DISCONNECT/CONNECT-cancel den Semaphore gibt.
         * Ohne dieses Warten laeuft der naechste read() in einen
         * "conn_active"-Zustand und schlaegt fehl. */
        xSemaphoreTake(g_done_sem, pdMS_TO_TICKS(3000));
        g_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
        g_phase    = PHASE_IDLE;
        ESP_LOGW(TAG, "Timeout nach %lu ms", (unsigned long)timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
 
    if (g_phase == PHASE_DONE && g_data.valid) {
        memcpy(out, &g_data, sizeof(miflora_data_t));
        return ESP_OK;
    }
 
    ESP_LOGE(TAG, "Sensor-Auslese fehlgeschlagen (phase=%d)", g_phase);
    return ESP_FAIL;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  functions
 * ═══════════════════════════════════════════════════════════════════════════ */
 
void ble_miflora_init()
{
    //get mac data from deviceData structure
    ble_miflora_setChannelData(&g_sDeviceData);

    if (!prv_anySensorEnabled())
    {
        ESP_LOGI(TAG, "Alle Sensoren deaktiviert, BLE-Stack wird nicht gestartet");
        g_stack_ready = false;
        g_phase = PHASE_IDLE;
        g_conn_hdl = BLE_HS_CONN_HANDLE_NONE;
        return;
    }

    /* NVS initialisieren */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
 
    /* Semaphore anlegen */
    if (g_done_sem == NULL)
    {
        g_done_sem = xSemaphoreCreateBinary();
        assert(g_done_sem);
    }
 
    /* NimBLE initialisieren - on_ble_sync wird automatisch aufgerufen
     * sobald Host und Controller synchronisiert sind */
    miflora_nimble_init();

    ESP_LOGI(TAG, "Modus: FIXED MAC");
}

esp_err_t ble_miflora_deinit(void)
{
    return miflora_nimble_deinit();
}

void ble_miflora_read(uint32_t ui32SensorNb, miflora_data_t *out)
{
    /* Sensor auslesen – erster Versuch */
    esp_err_t err = miflora_read_sensor(ui32SensorNb, out, 5000);
 
    if (err == ESP_OK) {
        if (out != NULL && out->valid) {
            ESP_LOGI(TAG, "Initialer Read erfolgreich");
        } else {
            ESP_LOGI(TAG, "Read uebersprungen oder keine gueltigen Daten (Sensor %lu)",
                     (unsigned long)ui32SensorNb);
        }
        /* Weiterverarbeitung: MQTT, Display, NVS speichern … */
    } else {
        ESP_LOGE(TAG, "Initialer Read fehlgeschlagen: %s",
                 esp_err_to_name(err));
    }
}

static int sniff_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    char acName[32] = {0};

    if (event->type == BLE_GAP_EVENT_DISC)
    {
        const uint8_t *pui8Data = event->disc.data;
        uint8_t uiLen = event->disc.length_data;

        while (uiLen > 1u)
        {
            uint8_t uiFieldLen = pui8Data[0];
            if (uiFieldLen == 0u || (uint16_t)uiFieldLen + 1u > uiLen) { break; }

            uint8_t uiFieldType = pui8Data[1];
            if ((uiFieldType == 0x09u || uiFieldType == 0x08u) && uiFieldLen > 1u)
            {
                size_t uiNameLen = (size_t)(uiFieldLen - 1u);
                if (uiNameLen >= sizeof(acName)) { uiNameLen = sizeof(acName) - 1u; }
                memcpy(acName, &pui8Data[2], uiNameLen);
                acName[uiNameLen] = '\0';
                break;
            }

            pui8Data += (uint16_t)uiFieldLen + 1u;
            uiLen -= (uint16_t)uiFieldLen + 1u;
        }
    }

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
        ESP_LOGI(TAG,
                 "BLE_SNIFF,%02X:%02X:%02X:%02X:%02X:%02X,rssi=%d,type=%u,len=%u,name=%s",
                 event->disc.addr.val[5], event->disc.addr.val[4],
                 event->disc.addr.val[3], event->disc.addr.val[2],
                 event->disc.addr.val[1], event->disc.addr.val[0],
                 event->disc.rssi,
                 (unsigned)event->disc.addr.type,
                 (unsigned)event->disc.length_data,
                 acName[0] != '\0' ? acName : "-");
        break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGI(TAG, "BLE_SNIFF,DONE,status=%d", event->disc_complete.reason);
        xSemaphoreGive(g_done_sem);
        break;

    default:
        break;
    }

    return 0;
}

esp_err_t ble_miflora_sniff(uint32_t ui32DurationMs)
{
    if (ui32DurationMs == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t uiWaitMs = 0u;
    while (!g_stack_ready && uiWaitMs < 5000u) {
        vTaskDelay(pdMS_TO_TICKS(50));
        uiWaitMs += 50u;
    }
    if (!g_stack_ready) {
        ESP_LOGE(TAG, "BLE-Stack nicht bereit nach %lu ms", (unsigned long)uiWaitMs);
        return ESP_ERR_TIMEOUT;
    }

    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }

    xSemaphoreTake(g_done_sem, 0);

    struct ble_gap_disc_params sDiscParams = {0};
    sDiscParams.passive = 0;
    sDiscParams.filter_duplicates = 1;

    ESP_LOGI(TAG, "BLE_SNIFF,START,duration_ms=%lu", (unsigned long)ui32DurationMs);
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC,
                          (int32_t)ui32DurationMs,
                          &sDiscParams,
                          sniff_gap_event_handler,
                          NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc rc=%d", rc);
        return ESP_FAIL;
    }

    if (xSemaphoreTake(g_done_sem, pdMS_TO_TICKS(ui32DurationMs + 2000u)) == pdFALSE) {
        ble_gap_disc_cancel();
        ESP_LOGW(TAG, "BLE_SNIFF,TIMEOUT");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}