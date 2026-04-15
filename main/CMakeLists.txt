idf_component_register(SRCS "firestore.c" "log.c" "watering.c" "main.c" "periphery.c" "communication.c" "storage.c" "provisioning.c" "helper.c" "peripherie/button_sleep.c" "peripherie/wifi_time.c" "peripherie/iic.c" "peripherie/tca6408.c" "ble_miflora.c" "datamanagement.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client esp_wifi esp_event esp_netif nvs_flash driver esp_timer bt spiffs wifi_provisioning protocomm esp_adc json mbedtls)

                    