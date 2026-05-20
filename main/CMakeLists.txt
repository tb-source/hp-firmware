if(CONFIG_PERIPHERY_VARIANT_LH)
    set(PERIPHERY_SRC "periphery_lh.c")
    set(PERIPHERY_VARIANT_DEFS "")
else()
    set(PERIPHERY_SRC "periphery_lg.c")
    set(PERIPHERY_VARIANT_DEFS "PERIPHERY_VARIANT_LG=1")
endif()

idf_component_register(SRCS "supabase.c" "firestore.c" "log.c" "watering.c" "main.c" ${PERIPHERY_SRC} "communication.c" "storage.c" "provisioning.c" "helper.c" "peripherie/button_sleep.c" "peripherie/wifi_time.c" "peripherie/iic.c" "peripherie/tca6408.c" "ble_miflora.c" "datamanagement.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_http_client esp_wifi esp_event esp_netif nvs_flash driver esp_timer bt spiffs wifi_provisioning protocomm esp_adc json mbedtls)

target_compile_definitions(${COMPONENT_LIB} PRIVATE ${PERIPHERY_VARIANT_DEFS})

