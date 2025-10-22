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

#define STORAGE_NAMESPACE "NVSData"
#define STORAGE_DEVICE "dev_data"

extern esp_err_t storage_init(void);
extern esp_err_t storage_read(char* pacIdentifier, uint32_t *pui32Data);
extern esp_err_t storage_write(char* pacIdentifier, uint32_t ui32Data);
extern esp_err_t device_read(char* pacData);
extern esp_err_t device_write(char* pacData);
extern esp_err_t nvs_readWatering(wateringData_t* sWateringData);
extern esp_err_t nvs_writeWatering(wateringData_t* psWateringData);

#endif /* MAIN_STORAGE_H_ */
