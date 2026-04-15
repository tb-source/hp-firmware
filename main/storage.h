/*
 * storage.h
 *
 *  Created on: 13.12.2021
 *      Author: tobby
 */

#ifndef MAIN_STORAGE_H_
#define MAIN_STORAGE_H_

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "types.h"

#define STORAGE_NAMESPACE        "NVSData"
#define STORAGE_DEVICE           "dev_data"
#define STORAGE_SECURE_DEVICE    "safe_data"
#define STORAGE_SECURE_NAMESPACE "secure"

#define BT_SALT_LEN     16
#define BT_VERIFIER_LEN 384

//storage data structure for credentials
typedef struct {
    char deviceId[64];
    char wifiSsid[64];
    char wifiPassword[64];
    char firebaseEmail[64];
    char firebasePassword[64];
} credentials_t;

//storage data for bt credentials
typedef struct {
    char btSalt[BT_SALT_LEN];
    char btVerifier[BT_VERIFIER_LEN];
} bt_credentials_t;

//storage data structure for selector properties
typedef struct
{
  uint32_t ui32Angle[4];
} sel_prop_t;

extern esp_err_t storage_init(void);
extern esp_err_t storage_read(char* pacIdentifier, uint32_t *pui32Data);
extern esp_err_t storage_write(char* pacIdentifier, uint32_t ui32Data);
extern esp_err_t storage_readDeviceJson(char* pacData);
extern esp_err_t storage_writeDeviceJson(char* pacData);
extern esp_err_t storage_readWatering(wateringData_t* sWateringData);
extern esp_err_t storage_writeWatering(wateringData_t* psWateringData);
extern esp_err_t storage_writeCredentials(const credentials_t *psCredentials);
extern esp_err_t storage_readCredentials(credentials_t *psCredentials);


extern esp_err_t storage_readBtSalt(uint8_t *pui8Salt, size_t uiLen);
extern esp_err_t storage_writeBtSalt(const uint8_t *pui8Salt, size_t uiLen);
extern esp_err_t storage_readBtVerifier(uint8_t *pui8Verifier, size_t uiLen);
extern esp_err_t storage_writeBtVerifier(const uint8_t *pui8Verifier, size_t uiLen);
extern esp_err_t storage_writeSelectorProperty(uint32_t ui32Channel, uint32_t ui32Angle);
extern esp_err_t storage_readSelectorProperty(sel_prop_t *psSelProp);

#endif /* MAIN_STORAGE_H_ */
