/*
 * firestore.h
 *
 *  Created on: 31.03.2026
 *      Author: tobby
 *
 *  Firestore REST API Client für ESP32 / ESP-IDF
 *  Verwendet den ESP HTTP-Client für HTTPS-Requests.
 *
 *  Konfiguration (anpassen vor Verwendung):
 *    FIRESTORE_PROJECT_ID  – Firebase-Projekt-ID
 *    FIRESTORE_API_KEY     – Firebase Web-API-Key (aus Firebase Console)
 */

#ifndef MAIN_FIRESTORE_H_
#define MAIN_FIRESTORE_H_

#include "esp_err.h"
#include "types.h"
#include "ble_miflora.h"

/* --------------------------------------------------------------------------
 * Konfiguration – hier anpassen
 * -------------------------------------------------------------------------- */
#define FIRESTORE_PROJECT_ID        "dein-firebase-projekt-id"
#define FIRESTORE_API_KEY           "dein-firebase-api-key"

#define FIRESTORE_COL_DEVICE        "devices"   /**< Collection für Gerätedaten    */
#define FIRESTORE_COL_MIFLORA       "miflora"   /**< Collection für MiFlora-Sensor */
#define FIRESTORE_COL_WATERING      "watering"  /**< Collection für Bewässerung    */

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * @brief Geräte- und Sensordaten in Firestore speichern.
 *
 * Erzeugt ein neues Dokument in der Collection FIRESTORE_COL_DEVICE.
 * Felder: id, name, status, battery_mv, temperature_c, timestamp.
 * WiFi muss bereits verbunden sein.
 *
 * @param psData  Zeiger auf die Gerätedaten.
 * @return        ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_writeDeviceData(const deviceData_t *psData);

/**
 * @brief MiFlora-Sensordaten in Firestore speichern.
 *
 * Erzeugt ein neues Dokument in der Collection FIRESTORE_COL_MIFLORA.
 * Felder: sensor_nr, temperature_c, illuminance_lux, moisture_pct,
 *         conductivity_us, battery_pct, firmware, timestamp.
 * WiFi muss bereits verbunden sein.
 *
 * @param psData     Zeiger auf die MiFlora-Daten.
 * @param ui32SensorNr  Sensor-Nummer (0-basiert).
 * @return           ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_writeMifloraData(const miflora_data_t *psData, uint32_t ui32SensorNr);

/**
 * @brief Bewässerungsdaten in Firestore speichern.
 *
 * Erzeugt ein neues Dokument in der Collection FIRESTORE_COL_WATERING.
 * Felder: last_unix, next_unix, channel_index, event_index,
 *         amount_ml, amount_last_ml, timestamp.
 * WiFi muss bereits verbunden sein.
 *
 * @param psData        Zeiger auf die Bewässerungsdaten.
 * @return              ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_writeWateringData(const wateringData_t *psData);

#endif /* MAIN_FIRESTORE_H_ */
