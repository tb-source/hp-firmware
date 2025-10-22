/*
 * storage.c
 *
 *  Created on: 13.12.2021
 *      Author: tobby
 */

#include "storage.h"

esp_err_t storage_init(void)
{
	esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) return err;

    err = nvs_flash_init_partition(STORAGE_DEVICE);
	return err;
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

esp_err_t device_read(char* pacData)
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

esp_err_t device_write(char* pacData)
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

esp_err_t nvs_readWatering(wateringData_t *sWateringData)
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

esp_err_t nvs_writeWatering(wateringData_t* sWateringData)
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