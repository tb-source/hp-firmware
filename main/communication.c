/*
 * communication.c
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 */

#include "communication.h"

#define BUF_SIZE (1024)
static QueueHandle_t uart0_queue;
static const char* TAG = "UART0";
static const char* TAGA = "Voltage";
static const char* TAGS = "Storage";

static void uart_event_task(void *pvParameters);
void testFunction(uint8_t* apData);

void uart_init(void)
{
	uart_config_t uart_config =
	{
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0));
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	ESP_LOGI(TAG, "INIT");
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);    //2048 Create a task to handler UART event from ISR
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    for(;;) {
        if(xQueueReceive(uart0_queue, (void * )&event, portMAX_DELAY))         //Waiting for UART event.
        {
            bzero(dtmp, BUF_SIZE);															//delete buffer

            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
//                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);
//                    ESP_LOGI(TAG, "[DATA EVT]:");
                    uart_write_bytes(UART_NUM_0, (const char*) dtmp, event.size);
                    testFunction(dtmp);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_NUM_0, &buffered_size);
                    int pos = uart_pattern_pop_pos(UART_NUM_0);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void testFunction(uint8_t* pacData)
{
	//LED test
	switch (*pacData)
	{

		case 'l':
		{
			log_type_t eType = LOG_TYPE_UNKNOWN;

			//check for valid Indentifier
			if (*(pacData + 2) == 'P') eType = LOG_TYPE_PERIPHERY;
			if (*(pacData + 2) == 'W') eType = LOG_TYPE_WATERING;
			if (*(pacData + 2) == 'E') eType = LOG_TYPE_ERROR;

			if (eType != LOG_TYPE_UNKNOWN)
			{
				switch (*(pacData + 1))
				{
					case 'r':
						log_readData(eType);
						break;
					case 'c':
						log_clearData(eType);
						break;
					default:
						break;
				}
			}
		}
		break;

		case 'u':
		{
	        ESP_LOGI(TAGA, "DATA: %i", (int)ui32BattVolt_read());
		}
		break;

		case 'v':
		{
	        ESP_LOGI(TAGA, "DATA: %i", (int)ui32EspVolt_read());
		}
		break;

		case 'o':
		{
			// ESP_ERROR_CHECK(gpio_set_level(PIN_SEL1_EN, 1));        		//enable driver
			// vTaskDelay(100);		//wait for capacitors loaded
			// TLV_init();				//init TLV sensor
			selector_setPos((*(pacData + 1)), SELECTOR_1);
			// ESP_ERROR_CHECK(gpio_set_level(PIN_SEL1_EN, 0));        		//enable driver
			// TLV_deinit();
		}
		break;

		case 'i':
		{
			// ESP_ERROR_CHECK(gpio_set_level(PIN_SEL2_EN, 1));        		//enable driver
			// vTaskDelay(100);		//wait for capacitors loaded
			// TLV_init();				//init TLV sensor
			selector_setPos((*(pacData + 1)), SELECTOR_2);
			// ESP_ERROR_CHECK(gpio_set_level(PIN_SEL2_EN, 0));        		//enable driver
			// TLV_deinit();
		}
		break;
	

		case 't':
		{
	        ESP_LOGI(TAGA, "DATA: %f", fTemp_read());
		}
		break;

		case 'w':
		{
			uint32_t ui32Position = (uint32_t)(*(pacData + 1));
//	        ESP_LOGI(TAGA, "DATA: %d", ui32Position_read());
			// selector_set(ui32Position);
		}
		break;

		case 'y':
		{
			selector_caliPos(SELECTOR_2);
		}
		break;

		case 'z':
		{
//			RTC_updateTime();
		}
		break;

		case 'h':
		{
			ESP_LOGI("Humidity", "Frequency: %d", (int)ui32Humidity_count((uint32_t)(*(pacData + 1))));
		}
		break;

		case 'f':
		{
			// bPowerstage_init();
			// esp_err_t err;
			// uint32_t iRunTime = (uint32_t)(*(pacData + 1))*100;
			// //uint32_t iBackEMF = (uint32_t)(*(pacData + 2))*100;
			// err = pump_runTime(iRunTime, MOTOR_DIR_UP);
			// ESP_LOGI(TAG, "DATA: %s", esp_err_to_name(err));
			// ESP_LOGI(TAG, "DATA: %i", (int)ui32ReadVariable());

		}
		break;

		case 'r':
		{
			uint32_t ui32Time = (uint32_t)(*(pacData + 1))*100;
			uint32_t ui32Direction = (uint32_t)(*(pacData + 2));
			// selector_runTime(ui32Time, ui32Direction);
		}
		break;

		case 's':
		{
			// bPowerstage_init();
			uint32_t ui32Time = (uint32_t)(*(pacData + 1))*100;
			uint32_t ui32Direction = (uint32_t)(*(pacData + 2));;
			pump_runTime(ui32Time, ui32Direction);
		}
		break;

		case 'x':
		{
			// bPowerstage_init();
			// deepSleep_activate();
		}
		break;

		case '1':
		{
			esp_err_t err;
			err = storage_write("test", (uint32_t)(*(pacData + 1)));
			ESP_LOGI(TAGS, "DATA: %s", esp_err_to_name(err));
	        //ESP_LOGI(TAGA, "DATA: %d", storage_write("test", *(pacData + 1)));
		}
		break;

		case '2':
		{
			// uint32_t ui32Data;
			// storage_read("test", &ui32Data);
	        // ESP_LOGI(TAGS, "DATA: %c", (int)ui32Data);
		}
		break;

		case 'a':
		{
			esp_err_t  eEspError = ESP_OK;
			eEspError = adc_calibration_150mV();
	        ESP_LOGI(TAGS, "DATA: %s", esp_err_to_name(eEspError));
		}
		break;

		case 'b':
		{
			esp_err_t eEspError = ESP_OK;
			eEspError = adc_calibration_850mV();
	        ESP_LOGI(TAGS, "DATA: %s", esp_err_to_name(eEspError));
		}
		break;

		case 'd':
		{
			for(uint32_t ui32Count=0; ui32Count < 511; ui32Count++){
				ESP_LOGI(TAGA, "DATA: %i", (int)g_ai32Analyser[ui32Count]);
			}
		}
		break;
	}
}
