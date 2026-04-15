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

/* --------------------------------------------------------------------------
 * Konfiguration – hier anpassen
 * -------------------------------------------------------------------------- */
#define FIRESTORE_PROJECT_ID        "lastingplants"   /**< Firebase-Projekt-ID */
#define FIRESTORE_API_KEY           "AIzaSyDEFPeIftMYP0GXZ_1MiQuXYi9qFcHN3tQ"  /**< Firebase Web-API-Key */

/** Timezone für SNTP (Mitteleuropäische Zeit mit Sommerzeit) */
#ifndef TIMEZONE
#define TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3"
#endif

#define FIRESTORE_COL_DEVICE        "devices"   /**< Collection für Gerätedaten    */
#define FIRESTORE_COL_MIFLORA       "miflora"   /**< Collection für MiFlora-Sensor */
#define FIRESTORE_COL_WATERING      "watering"  /**< Collection für Bewässerung    */

/** Firebase Email/Passwort-Login.
 *  Benutzer muss in Firebase Console unter Authentication → Users angelegt sein. */

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * @brief Firebase Email/Passwort Login – ID-Token holen (oder wiederverwenden).
 *
 * Muss nach firestore_wifiConnect() und vor den firestore_write*()-Funktionen
 * aufgerufen werden. Das Token ist 1 Stunde gültig; bei erneutem Aufruf wird
 * es nur dann neu geholt, wenn es abgelaufen ist.
 * Firestore Security Rules: allow write: if request.auth != null;
 *
 * @return ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_authenticate(const char *pacEmail, const char *pacPassword);

/**
 * @brief Gibt die Firebase User-UID (localId) zurück, die nach erfolgreicher
 *        Authentifizierung gespeichert wurde.
 *        Alle RTDB-Pfade werden automatisch unter /users/{uid}/ abgelegt.
 *
 * @return  Null-terminierter UID-String, oder "" wenn nicht authentifiziert.
 */
const char *firestore_getUid(void);

/**
 * @brief Synchronisiert die ESP32-Systemzeit mit der Firebase Server-Zeit.
 *        Nutzt Firebase /.info/serverTimeOffset (kein Auth nötig).
 *        Sollte nach firestore_wifiConnect() aufgerufen werden.
 *
 * @return ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_syncTimeFromServer(void);

/**
 * @brief WiFi verbinden (STA-Modus, Credentials aus NVS via Provisioning).
 *
 * Initialisiert NVS, TCP/IP-Stack, WiFi-Treiber und wartet auf eine IP-Adresse.
 * Muss vor allen firestore_write*()-Funktionen aufgerufen werden.
 * Idempotent: Mehrfachaufrufe sind sicher.
 *
 * @return ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_wifiConnect(const char *pacSsid, const char *pacPassword);

/**
 * @brief WiFi trennen und Ressourcen freigeben.
 *
 * Sinnvoll vor dem Deep-Sleep, um Stromverbrauch zu minimieren.
 *
 * @return ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_wifiDisconnect(void);

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

/**
 * @brief Beliebigen JSON-String direkt an einen RTDB-Pfad schreiben (PUT).
 *        Überschreibt den Pfad direkt – kein auto-generierter Push-Key.
 *
 * @param pacPath  Datenbankpfad, z.B. "users/LH_0000002"
 * @param pacJson  JSON-String, z.B. "{\"key\":\"value\"}"
 * @return         ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_writeString(const char *pacPath, const char *pacJson);

/**
 * @brief Beliebigen JSON-String an einen RTDB-Pfad anhängen (POST).
 *        Erzeugt einen neuen Eintrag mit auto-generiertem Push-Key unter dem Pfad.
 *
 * @param pacPath  Datenbankpfad, z.B. "devices" oder "log"
 * @param pacJson  JSON-String, z.B. "{\"key\":\"value\"}"
 * @return         ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_appendString(const char *pacPath, const char *pacJson);

/**
 * @brief Beliebigen JSON-String mit PATCH an einen RTDB-Pfad schreiben.
 *        Führt einen Merge durch – vorhandene Keys bleiben erhalten,
 *        nur die im JSON enthaltenen Keys werden aktualisiert/ergänzt.
 *
 * @param pacPath  Datenbankpfad, z.B. "users/LH_0000002/log"
 * @param pacJson  JSON-String, z.B. "{\"1743513825\":{\"TEMP\":19.0}}"
 * @return         ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_patchString(const char *pacPath, const char *pacJson);

/**
 * @brief Daten von einem RTDB-Pfad lesen (GET).
 *        Das Ergebnis ist ein null-terminierter JSON-String im übergebenen Puffer.
 *
 * @param pacPath    Datenbankpfad, z.B. "users/LH_0000002"
 * @param pacBuf     Puffer für die Antwort
 * @param uiBufSize  Größe des Puffers in Bytes
 * @return           ESP_OK bei Erfolg, sonst Fehlercode.
 */
esp_err_t firestore_readString(const char *pacPath, char *pacBuf, size_t uiBufSize);

#endif /* MAIN_FIRESTORE_H_ */
