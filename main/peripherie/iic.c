/*
 * iic.c
 *
 *  Created on: 02.02.2024
 *      Author: T.Bretzke
 */

#include "iic.h"

static bool s_bIICInit = false;
static esp_err_t last_i2c_err = ESP_OK; 
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t tlv_dev_handle;
static i2c_master_dev_handle_t pcf_dev_handle;

//init i2c 
void i2c_init()
{
    if (!s_bIICInit)
    {
        ESP_LOGI("I2C", "I2C_init");

        i2c_master_bus_config_t i2c_mst_config = 
        {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = -1,
            .scl_io_num = PIN_I2C_SCL,
            .sda_io_num = PIN_I2C_SDA,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = false,
        };

        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));   

        //init tlv dev handle
        i2c_device_config_t dev_cfg = 
        {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = TLV493_ADDR,
            .scl_speed_hz = 100000,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &tlv_dev_handle));

        //init pcf dev handle
        dev_cfg.device_address = PCF8563_ADDR;
        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &pcf_dev_handle));

        s_bIICInit = true;      
    }
}

//deinit i2c
void i2c_deinit()
{
    if (s_bIICInit)
    {
        ESP_ERROR_CHECK(i2c_master_bus_rm_device(tlv_dev_handle));
        ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
        s_bIICInit = false;
    }
}

//init tlv493
void TLV_init(void)
{
        const uint8_t lpm[]  = { 0b00000000, 0b00000001, 0b00000000, 0b01000000 };         // low power mode

        uint8_t aui8WriteData[4];               // low power mode
        uint8_t aui8ReadData[10];

        ESP_ERROR_CHECK(i2c_master_receive(tlv_dev_handle, &aui8ReadData, 10, 100));

        // for (int i = 0; i < 10; i++)
        // {
        //     ESP_LOGI("TLV", "DATA: %i", aui8ReadData[i]);
        // }

        aui8WriteData[0] = 0b00000000;                      //write to register 0
        aui8WriteData[1] = aui8ReadData[7] & 0b01111000;    //clear bits 2,3,4,5
        aui8WriteData[2] = aui8ReadData[8];
        aui8WriteData[3] = aui8ReadData[9] & 0b00001111;    //clear bit 7

        // Set Power Mode (ulpm, lpm, fm, pd)
        for(int i=0; i < 4; i++)
        {
            aui8WriteData[i] |= lpm[i];
        }

        ESP_ERROR_CHECK(i2c_master_transmit(tlv_dev_handle, aui8WriteData, 4, 100));
        vTaskDelay(50);	
}

//deinit tlv493
// void TLV_deinit(void)
// {
//         // ESP_ERROR_CHECK(i2c_master_bus_rm_device(tlv_dev_handle));
// }

int32_t i32TLV_getAngle(void)
{
    uint8_t buffer[3] = {0, 0, 0};
    int8_t i8MagData[3] = {0, 0, 0};

    ESP_ERROR_CHECK(i2c_master_receive(tlv_dev_handle, &buffer, 3, -1));
    for (int i = 0; i < 3; i++)
    {
        // ESP_LOGI("TLV", "DATA: %i", (int8_t)buffer[i]);
        i8MagData[i] = (int8_t)buffer[i];
    }
    float result;
    result = atan2f((float)i8MagData[2], (float)i8MagData[1]);     //atangent of x/z
    result *= 180.0/3.14159;                                        //convert to degrees
    result += 180.0;

    // ESP_LOGI("TLV", "DEG: %f", result);

    return (int32_t) result;
}

//PCF init
#define BinToBCD(bin) ((((bin) / 10) << 4) + ((bin) % 10))

void PCF_init(void){
	static bool init = false;
	i2c_init();
	if(!init)
    {
		uint8_t aui8Data[] = {0b00000000, 0b00000000};
        ESP_ERROR_CHECK(i2c_master_transmit(pcf_dev_handle, aui8Data, 2, 100));

        uint8_t buffer[] = {0, 0};
        ESP_ERROR_CHECK(i2c_master_receive(pcf_dev_handle, buffer, 2, 100));
        ESP_LOGI("PCF", "Init data: %i %i", (int)buffer[0], (int)buffer[1]);

		init = true;
	}
}

//PCF get actual date time
void PCF_GetDateTime(pcfData_t *dateTime) {
	uint8_t buffer[7];
    uint8_t writeBuffer = 0x02;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(pcf_dev_handle, &writeBuffer, 1, buffer, 7, 300));

	dateTime->second = (((buffer[0] >> 4) & 0x07) * 10) + (buffer[0] & 0x0F);
	dateTime->minute = (((buffer[1] >> 4) & 0x07) * 10) + (buffer[1] & 0x0F);
	dateTime->hour = (((buffer[2] >> 4) & 0x03) * 10) + (buffer[2] & 0x0F);
	dateTime->day = (((buffer[3] >> 4) & 0x03) * 10) + (buffer[3] & 0x0F);
	dateTime->weekday = (buffer[4] & 0x07);
	dateTime->month = ((buffer[5] >> 4) & 0x01) * 10 + (buffer[5] & 0x0F);
	dateTime->year = 1900 + ((buffer[6] >> 4) & 0x0F) * 10 + (buffer[6] & 0x0F);

    ESP_LOGI("RTC", "Second: %d", dateTime->second);

	if (buffer[5] &  0x80)
	{
		dateTime->year += 100;
	}

	if (buffer[0] & 0x80) //Clock integrity not guaranted
	{
		ESP_LOGE("PCF", "Clock integrity error");
	}
}

//get unix time from external rtc
time_t RTCExt_getUnixTime(void)
{
	pcfData_t date = {0};
	struct tm tm = {0};
	time_t tv = 0;
	
	PCF_init();

    PCF_GetDateTime(&date);

	tm.tm_sec = date.second;
	tm.tm_min = date.minute;
	tm.tm_hour = date.hour;
	tm.tm_mday = date.day;
	tm.tm_mon = date.month - 1;
	tm.tm_year = date.year - 1900;

    setenv("TZ", "GMT0", 1);
	tzset();
	tv = mktime(&tm);
	ESP_LOGI("RTC", "RTCExt time: %lld", tv);
	settimeofday(&tv, NULL);
	return tv;
}

//set unix time from external rtc
void RTCExt_setUnixTime(void)
{
	pcfData_t date;
	struct tm tm;

	PCF_init();

	time_t now = time(NULL);
    ESP_LOGI("RTC", "RTCExt_setUnixTime: %lld", now);
	gmtime_r(&now, &tm);
	date.second = tm.tm_sec;
	date.minute = tm.tm_min;
	date.hour = tm.tm_hour;
	date.day = tm.tm_mday;
	date.month = tm.tm_mon + 1;
	date.year = tm.tm_year + 1900;
	date.weekday = tm.tm_wday;

    ESP_LOGI("RTC", "Second: %d", date.second);
    ESP_LOGI("RTC", "Minute: %d", date.minute);
    ESP_LOGI("RTC", "Hour: %d", date.hour);
    ESP_LOGI("RTC", "Day: %d", date.day);
    ESP_LOGI("RTC", "Month: %d", date.month);
    ESP_LOGI("RTC", "Year: %d", date.year);

	if (date.second >= 60 || date.minute >= 60 || date.hour >= 24 || date.day > 32 || date.weekday > 6 || date.month > 12 || date.year < 1900 || date.year >= 2100)
	{
        ESP_LOGE("PCF", "Invalid date/time values");
	}

	uint8_t buffer[8];
    buffer[0] = 0x02;  //Register address
	buffer[1] = BinToBCD(date.second) & 0x7F;
	buffer[2] = BinToBCD(date.minute) & 0x7F;
	buffer[3] = BinToBCD(date.hour) & 0x3F;
	buffer[4] = BinToBCD(date.day) & 0x3F;
	buffer[5] = BinToBCD(date.weekday) & 0x07;
	buffer[6] = BinToBCD(date.month) & 0x1F;

	if (date.year >= 2000)
	{
		buffer[6] |= 0x80;
		buffer[7] = BinToBCD(date.year - 2000);
	}
	else
	{
		buffer[7] = BinToBCD(date.year - 1900);
	}

    ESP_ERROR_CHECK(i2c_master_transmit(pcf_dev_handle, buffer, sizeof(buffer), 300));
}