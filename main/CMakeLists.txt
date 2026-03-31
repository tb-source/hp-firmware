idf_component_register(SRCS "firestore.c" "log.c" "watering.c" "main.c" "periphery.c" "communication.c" "storage.c" "provisioning.c" "helper.c" "peripherie/button_sleep.c" "peripherie/wifi_time.c" "peripherie/iic.c" "peripherie/tca6408.c" "ble_miflora.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client esp_tls mbedtls)

                    