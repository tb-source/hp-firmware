/*
 * tca6408.c
 *
 *  Created on: 02.02.2024
 *      Author: T.Bretzke
 */

#include "tca6408.h"

static bool s_bIICInit = false;
static esp_err_t last_i2c_err = ESP_OK; 
i2c_master_dev_handle_t dev_handle;

//init tca6408
void tca_init(void)
{
    if (!s_bIICInit)
    {
        i2c_master_bus_config_t i2c_mst_config = 
        {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = I2C_NUM_0,
            .scl_io_num = PIN_I2C_SCL,
            .sda_io_num = PIN_I2C_SDA,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = false,
        };

        i2c_master_bus_handle_t bus_handle;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

        i2c_device_config_t dev_cfg = 
        {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = TCA6408_ADDR,
            .scl_speed_hz = 10000,
        };

        ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
        vTaskDelay(1);	

        for (int i = 0; i < 125; i++)
        {
            ESP_LOGI("TLC", "Adress 0x%02X: %s", 0x00 + i, esp_err_to_name(i2c_master_probe(bus_handle, 0x00 + i, -1)));
            vTaskDelay(10);
        }
        // ESP_LOGI("TLC", "Adress 0x10: %s", esp_err_to_name(i2c_master_probe(bus_handle, 0x10, -1)));
        // vTaskDelay(10);
        // ESP_LOGI("TLC", "Adress 0x11: %s", esp_err_to_name(i2c_master_probe(bus_handle, 0x11, -1)));
        // vTaskDelay(10);
        // ESP_LOGI("TLC", "Adress 0x20: %s", esp_err_to_name(i2c_master_probe(bus_handle, 0x20, -1)));
        // vTaskDelay(10);
        // ESP_LOGI("TLC", "Adress 0x21: %s", esp_err_to_name(i2c_master_probe(bus_handle, 0x21, -1)));
        // vTaskDelay(10);


        uint8_t aui8WriteCommandI2C[]  = { 0x00, 0x01, 0x02, 0x03 }; // Register addresses for TCA6408

        uint8_t aui8WriteDataI2C[]  = { 
            0b00000000,             // Register 0: ignored
            0b10000000,             //Register 1: Output port register
            0b00000000,             //Register 2: Polarity inversion 0->not inverted, 1->inverted
            0b00000000};             //Register 3: Configuration register 0-> output, 1->input
             
        uint8_t aui8WriteData[4];               // low power mode
        uint8_t aui8ReadData[10];

        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, aui8WriteCommandI2C, 2, aui8ReadData, 2, -1));
        ESP_LOGI("TLC", "DATA: %i", aui8ReadData[0]);

        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, aui8WriteCommandI2C + 1, 1, aui8ReadData, 1, -1));
        ESP_LOGI("TLC", "DATA: %i", aui8ReadData[0]);

        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, aui8WriteCommandI2C + 2, 1, aui8ReadData, 1, -1));
        ESP_LOGI("TLC", "DATA: %i", aui8ReadData[0]);

        for (int i = 0; i < 4; i++)
        {
            uint8_t aui8WriteI2C[] = {aui8WriteCommandI2C[i], aui8WriteDataI2C[i]};
            ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, aui8WriteI2C, 2, -1));
            vTaskDelay(1);	
            // ESP_LOGI("TLV", "DATA: %i", aui8ReadData[i]);
        }

        // aui8WriteI2C[1] = 0b10000000;                      //set output values

        // ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, aui8WriteData, 4, -1));
        

        s_bIICInit = true;      
    }
}

int32_t i32TLV_getAngle(void)
{
    uint8_t buffer[3] = {0, 0, 0};
    int8_t i8MagData[3] = {0, 0, 0};

    ESP_ERROR_CHECK(i2c_master_receive(dev_handle, &buffer, 3, -1));
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

