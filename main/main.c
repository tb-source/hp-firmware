//#include <bluetoothLE.h>
#include "periphery.h"
#include "communication.h"
#include "watering.h"
#include "peripherie/iic.h"
#include "peripherie/tca6408.h"
#include "log.h"

void app_main(void)
{
	ESP_LOGE("NVS", "%s", esp_err_to_name(storage_init()));	

	led_init();
	adc_init();
	uart_init();
	i2c_init();	

	// bPowerstage_init();					
	// humidity_init();
	touch_init1();

	// watering_init();

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
	// deepSleep_activate(1000*10000000);		//1000s

    while (true)
    {
		// ui32Charge_read();
    	vTaskDelay(500);
	
		// ESP_LOGI("Pump Speed: ", "DATA: %d", (int)gpio_get_level(PIN_PUMP_SPEED));
		// led_set(2,LED_ON);
		// ESP_LOGI("Humidity", "Pulse count 1: %d", (int)ui32Humidity_count(0)); // Log the pulse count
		// ESP_LOGI("Humidity", "Pulse count 2: %d", (int)ui32Humidity_count(1)); // Log the pulse count
		ESP_LOGI("Touch: ", "DATA: %d", (int)ui32Level_read());
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
