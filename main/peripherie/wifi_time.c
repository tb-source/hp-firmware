/*
 * wifi_time.c
 *
 *  Created on: 23.08.2023
 *      Author: tobby
 */

#include "iic.h"
#include "periphery.h"

#include "time.h"
#include "stdio.h"
#include "string.h"
#include "esp_log.h"
// #include "esp_sntp.h"
// #include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"

#include "lwip/err.h"
#include "lwip/sys.h"

// #define ESP_WIFI_SSID      	"FRITZ!Box 6660 Cable CX"
// #define ESP_WIFI_PASSWD      "18562668959760717487"
// #define EXAMPLE_ESP_MAXIMUM_RETRY  5

// /* FreeRTOS event group to signal when we are connected*/
// static EventGroupHandle_t s_wifi_event_group;

// /* The event group allows multiple bits for each event, but we only care about two events:
//  * - we are connected to the AP with an IP
//  * - we failed to connect after the maximum amount of retries */
// #define WIFI_CONNECTED_BIT BIT0
// #define WIFI_FAIL_BIT      BIT1

// static const char *TAG = "wifi station";

// static int s_retry_num = 0;

// static void event_handler(void* arg, esp_event_base_t event_base,
//                                 int32_t event_id, void* event_data)
// {
//     if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
//         esp_wifi_connect();
//     } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
//         if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
//             esp_wifi_connect();
//             s_retry_num++;
//             ESP_LOGI(TAG, "retry to connect to the AP");
//         } else {
//             xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
//         }
//         ESP_LOGI(TAG,"connect to the AP fail");
//     } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
//         ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
//         ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
//         s_retry_num = 0;
//         xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
//     }
// }

// void wifi_init_sta(void)
// {
//     s_wifi_event_group = xEventGroupCreate();

//     ESP_ERROR_CHECK(esp_netif_init());

//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     esp_netif_create_default_wifi_sta();

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     esp_event_handler_instance_t instance_any_id;
//     esp_event_handler_instance_t instance_got_ip;
//     ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
//                                                         ESP_EVENT_ANY_ID,
//                                                         &event_handler,
//                                                         NULL,
//                                                         &instance_any_id));
//     ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
//                                                         IP_EVENT_STA_GOT_IP,
//                                                         &event_handler,
//                                                         NULL,
//                                                         &instance_got_ip));

//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = ESP_WIFI_SSID,
//             .password = ESP_WIFI_PASSWD,
//         },
//     };
//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
//     ESP_ERROR_CHECK(esp_wifi_start() );

//     ESP_LOGI(TAG, "wifi_init_sta finished.");

//     /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
//      * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
//     EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
//             WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
//             pdFALSE,
//             pdFALSE,
//             portMAX_DELAY);

//     /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
//      * happened. */
//     if (bits & WIFI_CONNECTED_BIT) {
//         ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
//                  ESP_WIFI_SSID, ESP_WIFI_PASSWD);
//     } else if (bits & WIFI_FAIL_BIT) {
//         ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
//         		ESP_WIFI_SSID, ESP_WIFI_PASSWD);
//     } else {
//         ESP_LOGE(TAG, "UNEXPECTED EVENT");
//     }
// }

// time_t RTC_getTime(void)
// {
// 	time_t now;
// 	char strftime_buf[64];
// 	struct tm timeinfo;

// 	time(&now);
// 	// Set timezone to China Standard Time
// 	setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
// 	tzset();

// 	localtime_r(&now, &timeinfo);
// 	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
// 	ESP_LOGI(TAG, "The current date/time in Germany is: %s", strftime_buf);
//     return time_t;
// }

// void RTC_updateTime(void)
// {
// 	led_switch(1, 1);
// 	wifi_init_sta();

// 	// Obtain the current time from the SNTP server
//     ESP_LOGI(TAG, "Initializing SNTP");
//     esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
//     esp_sntp_setservername(0, "pool.ntp.org");
//     esp_sntp_init();

//     int retry = 0;
//     const int retry_count = 10;
//     while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
//         ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }

// //	 configTzTime(timeZone,  ntpServer);
//     esp_sntp_stop();
//     vTaskDelay(100);
//     esp_wifi_disconnect(); // disconnect the Wi-Fi connectivity.
//     vTaskDelay(100);
//     esp_wifi_stop(); // stop the Wi-Fi driver.
//     vTaskDelay(100);
//     esp_wifi_deinit(); // unload the Wi-Fi driver.
//     vTaskDelay(100);
// 	led_switch(1, 0);
// }

//external RTC iic 
static esp_err_t last_i2c_err = ESP_OK; 
static bool s_bIICInit = false;

// void iic_set_up()
// {
//     if (!s_bIICInit)
//     {
//         i2c_config_t conf = {0};
//         gpio_reset_pin(PIN_I2C_SDA);    //reset jtag pins
//         gpio_reset_pin(PIN_I2C_SCL);    //reset jtag pins
//         conf.mode = I2C_MODE_MASTER;
//         conf.sda_io_num = PIN_I2C_SDA;
//         conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//         conf.scl_io_num = PIN_I2C_SCL;
//         conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//         conf.master.clk_speed = 100000;
//         esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
//         assert(ret == ESP_OK);
//         ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
//         assert(ret == ESP_OK);  
//         s_bIICInit = true;      
//     }

// }

// esp_err_t PCF_Write(uint8_t addr, uint8_t *data, size_t count) {

// 	last_i2c_err = ESP_OK;
// 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// 	i2c_master_start(cmd);
// 	i2c_master_write_byte(cmd, PCF8563_WRITE_ADDR, true);
// 	i2c_master_write_byte(cmd, addr, true);
// 	i2c_master_write(cmd, data, count, true);
// 	i2c_master_stop(cmd);
// 	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
// 	last_i2c_err = ret;
// 	// printf("last_i2c_err %d\n", last_i2c_err);
//     return ret;
// }

// esp_err_t PCF_Read(uint8_t addr, uint8_t *data, size_t count) {

// 	last_i2c_err = ESP_OK;
// 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
// 	i2c_master_start(cmd);
// 	i2c_master_write_byte(cmd, PCF8563_WRITE_ADDR, true);
// 	i2c_master_write_byte(cmd, addr, true);
// 	i2c_master_start(cmd);
// 	i2c_master_write_byte(cmd, PCF8563_READ_ADDR, true);
// 	i2c_master_read(cmd, data, count, I2C_MASTER_LAST_NACK);
// 	i2c_master_stop(cmd);
// 	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
// 	last_i2c_err = ret;
// 	// printf("last_i2c_err %d\n", last_i2c_err);
//     return ret;
// }

// // int PCF_GetAndClearFlags(){
// // 	uint8_t flags;

// // 	esp_err_t ret = PCF_Read(0x01, &flags, 1);
// // 	if (ret != ESP_OK){
// // 		return -1;
// // 	}
// // 	uint8_t cleared = flags & 0b00010011;
// // 	ret = PCF_Write(0x01, &cleared, 1);
// // 	if (ret != ESP_OK){
// // 		return -1;
// // 	}

// // 	return flags & 0x0C;
// // }

// esp_err_t PCF_GetLastError(){
// 	return last_i2c_err;
// }

// #define BinToBCD(bin) ((((bin) / 10) << 4) + ((bin) % 10))

// int PCF_Init(uint8_t mode){
// 	static bool init = false;
// 	if(!init){
// 		uint8_t tmp = 0b00000000;
// 		esp_err_t ret = PCF_Write(0x00, &tmp, 1);
// 		if (ret != ESP_OK){
// 			return -1;
// 		}
// 		mode &= 0b00010011;
// 		ret = PCF_Write(0x01, &mode, 1);
// 		if (ret != ESP_OK){
// 			return -2;
// 		}
// 		init = true;
// 	}
// 	return 0;
// }

// int PCF_SetDateTime(PCF_DateTime *dateTime) {
// 	if (dateTime->second >= 60 || dateTime->minute >= 60 || dateTime->hour >= 24 || dateTime->day > 32 || dateTime->weekday > 6 || dateTime->month > 12 || dateTime->year < 1900 || dateTime->year >= 2100)
// 	{
// 		return -2;
// 	}

// 	uint8_t buffer[7];

// 	buffer[0] = BinToBCD(dateTime->second) & 0x7F;
// 	buffer[1] = BinToBCD(dateTime->minute) & 0x7F;
// 	buffer[2] = BinToBCD(dateTime->hour) & 0x3F;
// 	buffer[3] = BinToBCD(dateTime->day) & 0x3F;
// 	buffer[4] = BinToBCD(dateTime->weekday) & 0x07;
// 	buffer[5] = BinToBCD(dateTime->month) & 0x1F;

// 	if (dateTime->year >= 2000)
// 	{
// 		buffer[5] |= 0x80;
// 		buffer[6] = BinToBCD(dateTime->year - 2000);
// 	}
// 	else
// 	{
// 		buffer[6] = BinToBCD(dateTime->year - 1900);
// 	}

// 	esp_err_t ret = PCF_Write(0x02, buffer, sizeof(buffer));
// 	if (ret != ESP_OK) {
// 		return -1;
// 	}

// 	return 0;
// }

// int PCF_GetDateTime(PCF_DateTime *dateTime) {
// 	uint8_t buffer[7];
// 	esp_err_t ret;

// 	ret = PCF_Read(0x02, buffer, sizeof(buffer));
// 	if (ret != ESP_OK) {
// 		return -1;
// 	}

// 	dateTime->second = (((buffer[0] >> 4) & 0x07) * 10) + (buffer[0] & 0x0F);
// 	dateTime->minute = (((buffer[1] >> 4) & 0x07) * 10) + (buffer[1] & 0x0F);
// 	dateTime->hour = (((buffer[2] >> 4) & 0x03) * 10) + (buffer[2] & 0x0F);
// 	dateTime->day = (((buffer[3] >> 4) & 0x03) * 10) + (buffer[3] & 0x0F);
// 	dateTime->weekday = (buffer[4] & 0x07);
// 	dateTime->month = ((buffer[5] >> 4) & 0x01) * 10 + (buffer[5] & 0x0F);
// 	dateTime->year = 1900 + ((buffer[6] >> 4) & 0x0F) * 10 + (buffer[6] & 0x0F);

//     ESP_LOGI("RTC", "Second: %d", dateTime->second);

// 	if (buffer[5] &  0x80)
// 	{
// 		dateTime->year += 100;
// 	}

// 	if (buffer[0] & 0x80) //Clock integrity not guaranted
// 	{
// 		return 1;
// 	}

// 	return 0;
// }

// time_t RTCExt_getUnixTime(void)
// {
//     iic_set_up();
// 	int ret;
// 	PCF_DateTime date = {0};
// 	struct tm tm = {0};
// 	time_t tv = 0;
	
// 	ret = PCF_Init(0);
// 	if (ret != 0) {
// 		goto fail;
// 	}
//     ret = PCF_GetDateTime(&date);
// 	// printf("PCF_GetDateTime %d\n", ret);
//     if (ret != 0) {
// 		goto fail;
//     }
// 	tm.tm_sec = date.second;
// 	tm.tm_min = date.minute;
// 	tm.tm_hour = date.hour;
// 	tm.tm_mday = date.day;
// 	tm.tm_mon = date.month - 1;
// 	tm.tm_year = date.year - 1900;

//     setenv("TZ", "GMT0", 1);
// 	tzset();
// 	tv = mktime(&tm);
// 	ESP_LOGI("RTC", "RTCExt time: %lld", tv);
// 	settimeofday(&tv, NULL);
// 	return tv;
// fail:
// 	return ret;
// }

// int RTCExt_setUnixTime(void)
// {
//     iic_set_up();
// 	int ret;
// 	PCF_DateTime date = {0};
// 	struct tm tm = {0};

// 	ret = PCF_Init(0);
// 	if (ret != 0) {
// 		goto fail;
// 	}

// 	time_t now = time(NULL);
// 	gmtime_r(&now, &tm);
// 	date.second = tm.tm_sec;
// 	date.minute = tm.tm_min;
// 	date.hour = tm.tm_hour;
// 	date.day = tm.tm_mday;
// 	date.month = tm.tm_mon + 1;
// 	date.year = tm.tm_year + 1900;
// 	date.weekday = tm.tm_wday;

// 	ret = PCF_SetDateTime(&date);

// fail:
// 	return ret;
// }

// void RTC_syncTime(void)
// {
// 	time_t tv = RTCExt_getUnixTime();
// 	//valid time from rtc
// 	if (tv > 100)
// 	{
// 		time_t tTimeAct;
// 		time(&tTimeAct);        //act unix time

// 	}
// }
	
    