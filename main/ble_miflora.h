/*
 * ble_miflora.h
 *
 *  Created on: 07.10.2023
 *      Author: tobby
 *
 */

#ifndef MAIN_BLE_MIFLORA_H_
#define MAIN_BLE_MIFLORA_H_

#define MIFLORA_ENABLE

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "types.h"
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
 
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
 
/* NimBLE-Header */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"


typedef enum {
    PHASE_IDLE = 0,
    PHASE_SCAN,
    PHASE_CONNECTING,
    PHASE_WRITE_MODE,
    PHASE_READ_SENSOR,
    PHASE_READ_INFO,
    PHASE_DONE,
    PHASE_ERROR,
} miflora_phase_t;

extern void ble_miflora_init(void);
extern void ble_miflora_read(uint32_t ui32SensorNb, miflora_data_t *out);
extern void ble_miflora_setChannelData(const deviceData_t *psDevData);
extern esp_err_t ble_miflora_deinit(void);
extern esp_err_t ble_miflora_sniff(uint32_t ui32DurationMs);

#endif /* MAIN_BLE_MIFLORA_H_ */