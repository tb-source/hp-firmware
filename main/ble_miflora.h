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

typedef struct {
    float    temperature;   /**< °C, resolution 0,1 °C          */
    uint32_t illuminance;   /**< Lux                            */
    uint8_t  moisture;      /**< humidity %            */
    uint16_t conductivity;  /**< µS/cm                          */
    uint8_t  battery;       /**< battery %                */
    char     firmware[8];   /**< e.g. "3.2.1"                   */
    bool     valid;         /**< true if all fields are valid  */
} miflora_data_t;
 
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
extern esp_err_t ble_miflora_deinit(void);

#endif /* MAIN_BLE_MIFLORA_H_ */