/*
 * storage.c
 *
 *  Created on: 13.12.2021
 *      Author: tobby
 */

#include "storage.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "STORAGE";

esp_err_t storage_init(void)
{
    static const char *apacPartitions[] = {
        NVS_DEFAULT_PART_NAME, STORAGE_DEVICE, STORAGE_SECURE_DEVICE
    };

    for (int i = 0; i < 3; i++)
    {
        esp_err_t eErr = nvs_flash_init_partition(apacPartitions[i]);
        if (eErr == ESP_ERR_NVS_NO_FREE_PAGES || eErr == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            nvs_flash_erase_partition(apacPartitions[i]);
            eErr = nvs_flash_init_partition(apacPartitions[i]);
        }
        if (eErr != ESP_OK)
        {
            ESP_LOGE(TAG, "NVS init fehlgeschlagen [%s]: %s", apacPartitions[i], esp_err_to_name(eErr));
            return eErr;
        }
    }

    ESP_LOGI(TAG, "NVS initialisiert");
    return ESP_OK;
}

esp_err_t storage_read(char* pacIdentifier, uint32_t *pui32Data)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) 
    {
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
            if (err != ESP_OK) return err;
            nvs_close(my_handle);
            err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
            if (err != ESP_OK) return err;
        }
        else{
            return err;            
        }
    }

    // Read
    err = nvs_get_u32(my_handle, pacIdentifier , pui32Data);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t storage_write(char* pacIdentifier, uint32_t ui32Data)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write
    err = nvs_set_u32(my_handle, pacIdentifier, ui32Data);
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t storage_readDeviceJson(char* pacData)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;
    if (err != ESP_OK) 
    {
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            err = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READWRITE, &my_handle);
            if (err != ESP_OK) return err;
            nvs_close(my_handle);
            err = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READONLY, &my_handle);
            if (err != ESP_OK) return err;
        }
        else{
            return err;            
        }
    }
    size_t strSize = 4000;
    err = nvs_get_str(my_handle, "deviceString", pacData, &strSize);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t storage_writeDeviceJson(char* pacData)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READWRITE, &my_handle);

    if (err != ESP_OK) return err;

    // Write
    err = nvs_set_str(my_handle, "deviceString", pacData);
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t storage_readWatering(wateringData_t *sWateringData)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;
    if (err != ESP_OK) 
    {
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            err = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READWRITE, &my_handle);
            if (err != ESP_OK) return err;
            nvs_close(my_handle);
            err = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READONLY, &my_handle);
            if (err != ESP_OK) return err;
        }
        else{
            return err;            
        }
    }
    size_t required_size = sizeof(wateringData_t);
    err = nvs_get_blob(my_handle, "wateringString", sWateringData, &required_size);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t storage_writeWatering(wateringData_t* sWateringData)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READWRITE, &my_handle);

    if (err != ESP_OK) return err;

    // Write
    err = nvs_set_blob(my_handle, "wateringString", sWateringData, sizeof(wateringData_t));
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}

esp_err_t storage_readCredentials(credentials_t *psCredentials)
{
    if (psCredentials == NULL) { return ESP_ERR_INVALID_ARG; }

    nvs_handle_t hNvs;
    esp_err_t eErr = nvs_open_from_partition(STORAGE_SECURE_DEVICE,
                                             STORAGE_SECURE_NAMESPACE,
                                             NVS_READONLY, &hNvs);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_readCredentials: open fehlgeschlagen: %s", esp_err_to_name(eErr));
        return eErr;
    }

    size_t uiLen;

    uiLen = sizeof(psCredentials->deviceId);
    eErr = nvs_get_str(hNvs, "devId", psCredentials->deviceId, &uiLen);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "storage_readCredentials: deviceId lesen fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    uiLen = sizeof(psCredentials->devicePW);
    eErr = nvs_get_str(hNvs, "devPW", psCredentials->devicePW, &uiLen);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "storage_readCredentials: devicePW lesen fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    uiLen = sizeof(psCredentials->wifiSsid);
    eErr = nvs_get_str(hNvs, "wifiSsid", psCredentials->wifiSsid, &uiLen);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "storage_readCredentials: wifiSsid lesen fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    uiLen = sizeof(psCredentials->wifiPassword);
    eErr = nvs_get_str(hNvs, "wifiPassword", psCredentials->wifiPassword, &uiLen);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "storage_readCredentials: wifiPassword lesen fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    uiLen = sizeof(psCredentials->firebaseEmail);
    eErr = nvs_get_str(hNvs, "fbEmail", psCredentials->firebaseEmail, &uiLen);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "storage_readCredentials: firebaseEmail lesen fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    uiLen = sizeof(psCredentials->firebasePassword);
    eErr = nvs_get_str(hNvs, "fbPassword", psCredentials->firebasePassword, &uiLen);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "storage_readCredentials: firebasePassword lesen fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    nvs_close(hNvs);
    return ESP_OK;
}

esp_err_t storage_writeCredentials(const credentials_t *psCredentials)
{
    if (psCredentials == NULL) { return ESP_ERR_INVALID_ARG; }

    nvs_handle_t hNvs;
    esp_err_t eErr = nvs_open_from_partition(STORAGE_SECURE_DEVICE,
                                             STORAGE_SECURE_NAMESPACE,
                                             NVS_READWRITE, &hNvs);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_writeCredentials: open fehlgeschlagen: %s", esp_err_to_name(eErr));
        return eErr;
    }

    eErr = nvs_set_str(hNvs, "devId", psCredentials->deviceId);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_writeCredentials: deviceId schreiben fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    eErr = nvs_set_str(hNvs, "devPW", psCredentials->devicePW);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_writeCredentials: devicePW schreiben fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    eErr = nvs_set_str(hNvs, "wifiSsid", psCredentials->wifiSsid);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_writeCredentials: wifiSsid schreiben fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    eErr = nvs_set_str(hNvs, "wifiPassword", psCredentials->wifiPassword);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_writeCredentials: wifiPassword schreiben fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    eErr = nvs_set_str(hNvs, "fbEmail", psCredentials->firebaseEmail);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_writeCredentials: firebaseEmail schreiben fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    eErr = nvs_set_str(hNvs, "fbPassword", psCredentials->firebasePassword);
    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_writeCredentials: firebasePassword schreiben fehlgeschlagen: %s", esp_err_to_name(eErr));
        nvs_close(hNvs);
        return eErr;
    }

    eErr = nvs_commit(hNvs);
    nvs_close(hNvs);

    if (eErr != ESP_OK)
    {
        ESP_LOGE(TAG, "storage_writeCredentials: commit fehlgeschlagen: %s", esp_err_to_name(eErr));
    }
    return eErr;
}

esp_err_t storage_readBtSalt(uint8_t *pui8Salt, size_t uiLen)
{
    if (pui8Salt == NULL || uiLen < BT_SALT_LEN) { return ESP_ERR_INVALID_ARG; }

    bt_credentials_t sBtCred;
    memset(&sBtCred, 0, sizeof(sBtCred));

    nvs_handle_t hNvs;
    esp_err_t eErr = nvs_open_from_partition(STORAGE_SECURE_DEVICE, STORAGE_SECURE_NAMESPACE,
                                             NVS_READONLY, &hNvs);
    if (eErr != ESP_OK) { return eErr; }

    size_t uiStoredLen = sizeof(sBtCred.btSalt);
    eErr = nvs_get_blob(hNvs, "btSalt", sBtCred.btSalt, &uiStoredLen);
    nvs_close(hNvs);

    if (eErr != ESP_OK) { return eErr; }
    if (uiStoredLen != sizeof(sBtCred.btSalt)) { return ESP_ERR_NVS_INVALID_LENGTH; }

    memcpy(pui8Salt, sBtCred.btSalt, BT_SALT_LEN);
    return ESP_OK;
}

esp_err_t storage_writeBtSalt(const uint8_t *pui8Salt, size_t uiLen)
{
    if (pui8Salt == NULL || uiLen != BT_SALT_LEN) { return ESP_ERR_INVALID_ARG; }

    bt_credentials_t sBtCred;
    memset(&sBtCred, 0, sizeof(sBtCred));

    nvs_handle_t hNvs;
    esp_err_t eErr = nvs_open_from_partition(STORAGE_SECURE_DEVICE, STORAGE_SECURE_NAMESPACE,
                                             NVS_READWRITE, &hNvs);
    if (eErr != ESP_OK) { return eErr; }

    size_t uiStoredLen = sizeof(sBtCred.btVerifier);
    eErr = nvs_get_blob(hNvs, "btVerifier", sBtCred.btVerifier, &uiStoredLen);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(hNvs);
        return eErr;
    }
    if (eErr == ESP_OK && uiStoredLen != sizeof(sBtCred.btVerifier))
    {
        nvs_close(hNvs);
        return ESP_ERR_NVS_INVALID_LENGTH;
    }

    memcpy(sBtCred.btSalt, pui8Salt, BT_SALT_LEN);

    eErr = nvs_set_blob(hNvs, "btSalt", sBtCred.btSalt, sizeof(sBtCred.btSalt));
    if (eErr == ESP_OK) { eErr = nvs_commit(hNvs); }
    nvs_close(hNvs);
    return eErr;
}

esp_err_t storage_readBtVerifier(uint8_t *pui8Verifier, size_t uiLen)
{
    if (pui8Verifier == NULL || uiLen < BT_VERIFIER_LEN) { return ESP_ERR_INVALID_ARG; }

    bt_credentials_t sBtCred;
    memset(&sBtCred, 0, sizeof(sBtCred));

    nvs_handle_t hNvs;
    esp_err_t eErr = nvs_open_from_partition(STORAGE_SECURE_DEVICE, STORAGE_SECURE_NAMESPACE,
                                             NVS_READONLY, &hNvs);
    if (eErr != ESP_OK) { return eErr; }

    size_t uiStoredLen = sizeof(sBtCred.btVerifier);
    eErr = nvs_get_blob(hNvs, "btVerifier", sBtCred.btVerifier, &uiStoredLen);
    nvs_close(hNvs);

    if (eErr != ESP_OK) { return eErr; }
    if (uiStoredLen != sizeof(sBtCred.btVerifier)) { return ESP_ERR_NVS_INVALID_LENGTH; }

    memcpy(pui8Verifier, sBtCred.btVerifier, BT_VERIFIER_LEN);
    return ESP_OK;
}

esp_err_t storage_writeBtVerifier(const uint8_t *pui8Verifier, size_t uiLen)
{
    if (pui8Verifier == NULL || uiLen != BT_VERIFIER_LEN) { return ESP_ERR_INVALID_ARG; }

    bt_credentials_t sBtCred;
    memset(&sBtCred, 0, sizeof(sBtCred));

    nvs_handle_t hNvs;
    esp_err_t eErr = nvs_open_from_partition(STORAGE_SECURE_DEVICE, STORAGE_SECURE_NAMESPACE,
                                             NVS_READWRITE, &hNvs);
    if (eErr != ESP_OK) { return eErr; }

    size_t uiStoredLen = sizeof(sBtCred.btSalt);
    eErr = nvs_get_blob(hNvs, "btSalt", sBtCred.btSalt, &uiStoredLen);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(hNvs);
        return eErr;
    }
    if (eErr == ESP_OK && uiStoredLen != sizeof(sBtCred.btSalt))
    {
        nvs_close(hNvs);
        return ESP_ERR_NVS_INVALID_LENGTH;
    }

    memcpy(sBtCred.btVerifier, pui8Verifier, BT_VERIFIER_LEN);

    eErr = nvs_set_blob(hNvs, "btVerifier", sBtCred.btVerifier, sizeof(sBtCred.btVerifier));
    if (eErr == ESP_OK) { eErr = nvs_commit(hNvs); }
    nvs_close(hNvs);
    return eErr;
}

esp_err_t storage_writeSelectorProperty(uint32_t ui32Channel, uint32_t ui32Angle)
{
    if (ui32Channel >= (SELCOUNT * 4)) { return ESP_ERR_INVALID_ARG; }

    nvs_handle_t hNvs;
    esp_err_t eErr = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READWRITE, &hNvs);
    if (eErr != ESP_OK) { return eErr; }

    sel_prop_t sSelProp = {0};
    size_t uiRequiredSize = sizeof(sSelProp);

    eErr = nvs_get_blob(hNvs, "selectorProps", &sSelProp, &uiRequiredSize);
    if (eErr != ESP_OK && eErr != ESP_ERR_NVS_NOT_FOUND)
    {
        nvs_close(hNvs);
        return eErr;
    }
    if (eErr == ESP_OK && uiRequiredSize != sizeof(sSelProp))
    {
        nvs_close(hNvs);
        return ESP_ERR_NVS_INVALID_LENGTH;
    }

    sSelProp.ui32Angle[ui32Channel] = ui32Angle;

    eErr = nvs_set_blob(hNvs, "selectorProps", &sSelProp, sizeof(sSelProp));
    if (eErr == ESP_OK) { eErr = nvs_commit(hNvs); }

    nvs_close(hNvs);
    return eErr;
}

esp_err_t storage_readSelectorProperty(sel_prop_t *psSelProp)
{
    if (psSelProp == NULL) { return ESP_ERR_INVALID_ARG; }

    nvs_handle_t hNvs;
    esp_err_t eErr = nvs_open_from_partition(STORAGE_DEVICE, "device", NVS_READONLY, &hNvs);
    if (eErr != ESP_OK) { return eErr; }

    size_t uiRequiredSize = sizeof(*psSelProp);

    eErr = nvs_get_blob(hNvs, "selectorProps", psSelProp, &uiRequiredSize);
    nvs_close(hNvs);

    if (eErr != ESP_OK) { return eErr; }
    if (uiRequiredSize != sizeof(*psSelProp)) { return ESP_ERR_NVS_INVALID_LENGTH; }

    return ESP_OK;
}
