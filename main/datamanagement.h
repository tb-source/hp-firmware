/*
 * datamanagement.h
 *
 *  Created on: 07.10.2023
 *      Author: tobby
 */

#ifndef MAIN_DATAMANAGEMENT_H_
#define MAIN_DATAMANAGEMENT_H_

#include "types.h"
#include <esp_err.h>
#include "cJSON.h"
#include "firestore.h"
#include "storage.h"
#include "log.h"
#include "periphery.h"

/**
 * @brief Liest aktuelle Sensordaten (Zeit, Temperatur, Batterie, Wasserstand)
 *        und schreibt sie als JSON-String in den übergebenen Puffer.
 *
 * @param pacBuf     Puffer für den JSON-String.
 * @param uiBufSize  Größe des Puffers in Bytes (mind. 80 empfohlen).
 * @return           ESP_OK bei Erfolg, ESP_ERR_NO_MEM wenn Puffer zu klein.
 */
esp_err_t data_getDeviceData(char *pacBuf, size_t uiBufSize);

/**
 * @brief Liest eine Zeile aus dem Peripherie-Log (SPIFFS) und
 *        gibt sie als JSON-String zurück.
 *
 * @param pacBuf       Ausgabepuffer (mind. 256 Bytes empfohlen).
 * @param uiBufSize    Größe des Puffers in Bytes.
 * @param ui32LineIdx  0-basierter Index der Datenzeile (0 = älteste).
 * @return             ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_NO_MEM, ESP_FAIL.
 */
esp_err_t data_getPeripherieLogData(char *pacBuf, size_t uiBufSize,
                                    uint32_t ui32LineIdx, uint32_t ui32Count);

/**
 * @brief Liest mehrere Zeilen aus dem Watering-Log (SPIFFS) und
 *        gibt sie als JSON-Objekt zurueck.
 *
 * @param pacBuf       Ausgabepuffer.
 * @param uiBufSize    Groesse des Puffers in Bytes.
 * @param ui32LineIdx  0-basierter Index der Datenzeile (0 = aelteste).
 * @param ui32Count    Anzahl der zu lesenden Zeilen.
 * @return             ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_NO_MEM, ESP_FAIL.
 */
esp_err_t data_getWateringLogData(char *pacBuf, size_t uiBufSize,
                                  uint32_t ui32LineIdx, uint32_t ui32Count);

/**
 * @brief Liest mehrere Zeilen aus dem Error-Log (SPIFFS) und
 *        gibt sie als JSON-Objekt zurueck.
 *
 * @param pacBuf       Ausgabepuffer.
 * @param uiBufSize    Groesse des Puffers in Bytes.
 * @param ui32LineIdx  0-basierter Index der Datenzeile (0 = aelteste).
 * @param ui32Count    Anzahl der zu lesenden Zeilen.
 * @return             ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_NO_MEM, ESP_FAIL.
 */
esp_err_t data_getErrorLogData(char *pacBuf, size_t uiBufSize,
                               uint32_t ui32LineIdx, uint32_t ui32Count);

/**
 * @brief Parst einen JSON-String mit Channel-Daten und füllt ein channelData_t-Array.
 *
 * @param pacJson     JSON-String im Format {"CH1":{...},"CH2":{...},"CH3":{...}}.
 * @param psChannels  Zeiger auf channelData_t-Array (mind. ui8Count Einträge).
 * @param ui8Count    Anzahl zu füllender Kanäle (üblicherweise CHANNELCOUNT = 3).
 * @return            ESP_OK, ESP_FAIL, ESP_ERR_INVALID_ARG.
 */
esp_err_t data_setChannelData(const char *pacJson, channelData_t *psChannels);

/**
 * @brief Serialisiert ein channelData_t-Array in einen JSON-String.
 *        Inverse von data_setChannelData().
 *
 * Ausgabe-Format: {"CH1":{"EN":true,"FREQ":1,"NAME":"...","HUM":{"MIN":0,"MAX":0,"SENS":false,"MAC":"00:00:00:00:00:00"},"EVENTS":{...}},...}
 *
 * @param psChannels  Zeiger auf channelData_t-Array (CHANNELCOUNT Einträge).
 * @param pacBuf      Ausgabepuffer.
 * @param uiBufSize   Größe des Puffers in Bytes.
 * @return            ESP_OK, ESP_ERR_NO_MEM, ESP_ERR_INVALID_ARG, ESP_FAIL.
 */
esp_err_t data_getChannelData(const channelData_t *psChannels, char *pacBuf, size_t uiBufSize);

/**
 * @brief Gibt den Inhalt einer deviceData_t-Struktur über ESP_LOGI aus.
 *
 * @param psData  Zeiger auf die Gerätedaten.
 */
void data_logDeviceData(const deviceData_t *psData);

#endif /* MAIN_DATAMANAGEMENT_H_ */