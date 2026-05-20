//#include <bluetoothLE.h>
#include "periphery.h"
#include "communication.h"
#include "watering.h"
#include "peripherie/iic.h"
#include "peripherie/tca6408.h"
#include "ble_miflora.h"
#include "storage.h"
#include "helper.h"
#include "firestore.h"
#include "log.h"
#include "datamanagement.h"

/*
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

static void prv_wifiBasicScanTest(void)
{
	const char *TAG_WIFI_TEST = "WIFI_TEST";
	esp_err_t eErr;

	eErr = nvs_flash_init();
	if (eErr == ESP_ERR_NVS_NO_FREE_PAGES || eErr == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		eErr = nvs_flash_init();
	}
	if (eErr != ESP_OK && eErr != ESP_ERR_INVALID_STATE)
	{
		ESP_LOGE(TAG_WIFI_TEST, "NVS init failed: %s", esp_err_to_name(eErr));
		return;
	}

	eErr = esp_netif_init();
	if (eErr != ESP_OK && eErr != ESP_ERR_INVALID_STATE)
	{
		ESP_LOGE(TAG_WIFI_TEST, "esp_netif_init failed: %s", esp_err_to_name(eErr));
		return;
	}

	bool bEventLoopCreated = false;
	eErr = esp_event_loop_create_default();
	if (eErr == ESP_OK)
	{
		bEventLoopCreated = true;
	}
	else if (eErr != ESP_ERR_INVALID_STATE)
	{
		ESP_LOGE(TAG_WIFI_TEST, "event loop create failed: %s", esp_err_to_name(eErr));
		return;
	}

	esp_netif_t *psStaNetif = esp_netif_create_default_wifi_sta();
	if (psStaNetif == NULL)
	{
		ESP_LOGE(TAG_WIFI_TEST, "create default STA netif failed");
		return;
	}

	wifi_init_config_t sCfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&sCfg));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_start());

	wifi_scan_config_t sScanCfg = {0};
	sScanCfg.show_hidden = true;

	ESP_LOGI(TAG_WIFI_TEST, "Starting WiFi scan...");
	ESP_ERROR_CHECK(esp_wifi_scan_start(&sScanCfg, true));

	uint16_t ui16ApCount = 0u;
	ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ui16ApCount));
	ESP_LOGI(TAG_WIFI_TEST, "Scan done. AP count: %u", (unsigned)ui16ApCount);

	wifi_ap_record_t asApRecords[20];
	uint16_t ui16ToRead = ui16ApCount;
	if (ui16ToRead > (uint16_t)(sizeof(asApRecords) / sizeof(asApRecords[0])))
	{
		ui16ToRead = (uint16_t)(sizeof(asApRecords) / sizeof(asApRecords[0]));
	}

	if (ui16ToRead > 0u)
	{
		ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ui16ToRead, asApRecords));
		for (uint16_t uiI = 0u; uiI < ui16ToRead; uiI++)
		{
			ESP_LOGI(TAG_WIFI_TEST,
					 "AP[%u]: SSID='%s' RSSI=%d CH=%u AUTH=%d",
					 (unsigned)uiI,
					 (const char *)asApRecords[uiI].ssid,
					 (int)asApRecords[uiI].rssi,
					 (unsigned)asApRecords[uiI].primary,
					 (int)asApRecords[uiI].authmode);
		}
	}

	ESP_ERROR_CHECK(esp_wifi_stop());
	ESP_ERROR_CHECK(esp_wifi_deinit());
	esp_netif_destroy(psStaNetif);

	if (bEventLoopCreated)
	{
		ESP_ERROR_CHECK(esp_event_loop_delete_default());
	}
}
*/

void app_main(void)
{
	
	ESP_LOGE("NVS", "%s", esp_err_to_name(storage_init()));
	led_init();
	adcTouch_init();
	uart_init();
	i2c_init();	
	bPowerstage_init();



	// humidity_init();
	// touch_init1();

	#if CONFIG_PERIPHERY_VARIANT_LG
	debug_init();
	if (gpio_get_level(PIN_DEBUG) == 1)
	{
		watering_init();
	}
	#else
	watering_init();	
	#endif

  	// RTCExt_getUnixTime();        //uncommented
	// log_peripherieData();
	// deepSleep_activate(1000000*60*20);		//log every 20 min data

	// ble_miflora_init();
	// ble_miflora_sniff(5000);		//sniff for 10s
	// ble_miflora_deinit();

	// WakeUpCause_test();

	// struct tm tm = {0};
	// time_t tv = 0;
	// tm.tm_sec = 0;
	// tm.tm_min = 10;
	// tm.tm_hour = 15;
	// tm.tm_mday = 19;
	// tm.tm_mon = 8 - 1;
	// tm.tm_year = 2025 - 1900;

    // setenv("TZ", "GMT0", 1);
	// tzset();
	// tv = mktime(&tm);
	// ESP_LOGI("RTC", "RTCExt time: %lld", tv);
	// settimeofday(&tv, NULL);
	// RTCExt_setUnixTime();
	// time_t time;
	// gettimeofday(&time, NULL);
	// ESP_LOGI("RTC", "RTCInt time: %lld", time);
	// button_sleep_init();
	

	// led_set(2,LED_BLINK_SLOW);
	// led_set(1,LED_BLINK_FAST);
	int32_t i32Value;
	int32_t i32InitValue;
	// ESP_ERROR_CHECK(ui32HX710_read(&i32InitValue));

    while (true)
    {
		// ESP_ERROR_CHECK(ui32HX710_read(&i32Value));
		// ESP_LOGI("HX710", "Value: %d", (int)(i32Value - i32InitValue)/ 500);
		
		// ui32Charge_read();
		// ESP_LOGI("FDC", "Capacitance Hum1: %d", (int)FDC_getCap(1)/5243);
		// ESP_LOGI("FDC", "Capacitance Hum2: %d", (int)FDC_getCap(2)/5243);
		// ESP_LOGI("FDC", "Capacitance Hum3: %d", (int)FDC_getCap(3)/5243);
    	vTaskDelay(500);
	
		// ESP_LOGI("Pump Speed: ", "DATA: %d", (int)gpio_get_level(PIN_PUMP_SPEED));
		// led_set(2,LED_ON);
		// ESP_LOGI("Humidity", "Pulse count 1: %d", (int)ui32Humidity_count(0)); // Log the pulse count
		// ESP_LOGI("Humidity", "Pulse count 2: %d", (int)ui32Humidity_count(1)); // Log the pulse count
		// ESP_LOGI("Touch: ", "DATA: %d", (int)ui32Level_read());
		// vTaskDelay(500);
		// ESP_LOGI("Pump Speed: ", "DATA: %d", (int)gpio_get_level(PIN_PUMP_SPEED));
		// led_set(2,LED_OFF);
		// ESP_LOGI("Touch: ", "DATA: %d", (int)ui32Level_read());

		// deepSleep_activate(100*10000000);		//100s
		// ESP_LOGI("Humidity", "Pulse count 1: %d, Pulse count 2: %d", (int)ui32Humidity_count(0), (int)ui32Humidity_count(1));
//    	if (gpio_get_level(PIN_CHRG) == 1)
//    	{
//    		led_switch(1,0);
//    	}
//    	else
//    	{
//    		led_switch(1,1);
//    	}
   	// ESP_LOGI("MagX: ", "DATA: %d", i16TLV_getX());
//    	selector_pos(1);
		// ESP_LOGI("Touch: ", "Value: %d", (int)ui32Level_read());

    }
}
