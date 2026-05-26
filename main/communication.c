/*
 * communication.c
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 * 
 * OPEN TOOL FOR COMMUNICATION WITH EXTERNAL DEVICES IN TERMINAL: py tools\communication_uart_ui.py
 */

#include "communication.h"
#include <stdlib.h>

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
    static char acDollarCmd[BT_VERIFIER_LEN * 2u + 64u];
    static size_t uiDollarLen = 0u;
    static bool bDollarInProgress = false;
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

                    if (bDollarInProgress || (event.size > 0 && dtmp[0] == '$'))
                    {
                        for (size_t uiI = 0u; uiI < event.size; uiI++)
                        {
                            char cByte = (char)dtmp[uiI];

                            if (!bDollarInProgress)
                            {
                                if (cByte == '$')
                                {
                                    bDollarInProgress = true;
                                    uiDollarLen = 0u;
                                    acDollarCmd[uiDollarLen++] = cByte;
                                }
                                continue;
                            }

                            if (cByte == '\r')
                            {
                                continue;
                            }

                            if (cByte == '\n')
                            {
                                acDollarCmd[uiDollarLen] = '\0';
                                testFunction((uint8_t *)acDollarCmd);
                                bDollarInProgress = false;
                                uiDollarLen = 0u;
                                continue;
                            }

                            if (uiDollarLen < (sizeof(acDollarCmd) - 1u))
                            {
                                acDollarCmd[uiDollarLen++] = cByte;
                            }
                            else
                            {
                                ESP_LOGE(TAG, "$ command overflow, dropping frame");
                                bDollarInProgress = false;
                                uiDollarLen = 0u;
                            }
                        }
                    }
                    else
                    {
                        testFunction(dtmp);
                    }
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

/* =========================================================================
 * $ Protocol Handler
 * Format: $<id>><value>  (Set)
 *         $<id><         (Get)
 * Response Set: <id>OK\n  |  <id>ERR\n
 * Response Get: <id>OK:<value>\n  |  <id>ERR\n
 * ========================================================================= */

static void prv_uartSend(const char *pacStr)
{
    uart_write_bytes(UART_NUM_0, pacStr, strlen(pacStr));
}

static void prv_hexEncode(const uint8_t *pui8Data, size_t uiLen, char *pacOut, size_t uiOutSize)
{
    static const char acHex[] = "0123456789abcdef";
    size_t uiI;
    for (uiI = 0; uiI < uiLen && (uiI * 2u + 2u) < uiOutSize; uiI++)
    {
        pacOut[uiI * 2u]      = acHex[(pui8Data[uiI] >> 4) & 0x0Fu];
        pacOut[uiI * 2u + 1u] = acHex[pui8Data[uiI] & 0x0Fu];
    }
    pacOut[uiI * 2u] = '\0';
}

static bool prv_hexDecode(const char *pacHex, uint8_t *pui8Out, size_t uiExpected)
{
    for (size_t uiI = 0u; uiI < uiExpected; uiI++)
    {
        char cH = pacHex[uiI * 2u];
        char cL = pacHex[uiI * 2u + 1u];
        if (cH == '\0' || cL == '\0') { return false; }
        uint8_t uiHi = (cH >= '0' && cH <= '9') ? (uint8_t)(cH - '0') :
                       (cH >= 'a' && cH <= 'f') ? (uint8_t)(cH - 'a' + 10) :
                       (cH >= 'A' && cH <= 'F') ? (uint8_t)(cH - 'A' + 10) : 0xFFu;
        uint8_t uiLo = (cL >= '0' && cL <= '9') ? (uint8_t)(cL - '0') :
                       (cL >= 'a' && cL <= 'f') ? (uint8_t)(cL - 'a' + 10) :
                       (cL >= 'A' && cL <= 'F') ? (uint8_t)(cL - 'A' + 10) : 0xFFu;
        if (uiHi == 0xFFu || uiLo == 0xFFu) { return false; }
        pui8Out[uiI] = (uint8_t)((uiHi << 4) | uiLo);
    }
    return true;
}

static void prv_handleDollarCmd(const uint8_t *pucData)
{
    const char *pacCmd = (const char *)(pucData + 1);  /* skip '$' */
    size_t      uiLen  = strlen(pacCmd);

    /* find direction marker */
    size_t uiDirIdx = 0u;
    char   cDir     = 0;
    for (size_t uiI = 0u; uiI < uiLen; uiI++)
    {
        if (pacCmd[uiI] == '>' || pacCmd[uiI] == '<')
        {
            uiDirIdx = uiI;
            cDir     = pacCmd[uiI];
            break;
        }
    }
    if (cDir == 0) { return; }

    /* extract identifier */
    char acId[32];
    if (uiDirIdx >= sizeof(acId)) { return; }
    memcpy(acId, pacCmd, uiDirIdx);
    acId[uiDirIdx] = '\0';

    /* extract value (Set only), strip \r\n */
    char acValue[BT_VERIFIER_LEN * 2u + 1u];
    acValue[0] = '\0';
    if (cDir == '>')
    {
        const char *pacVal = pacCmd + uiDirIdx + 1u;
        size_t      uiVLen = strnlen(pacVal, uiLen - uiDirIdx - 1u);
        while (uiVLen > 0u && (pacVal[uiVLen - 1u] == '\r' || pacVal[uiVLen - 1u] == '\n'))
        {
            uiVLen--;
        }
        if (uiVLen >= sizeof(acValue)) { uiVLen = sizeof(acValue) - 1u; }
        memcpy(acValue, pacVal, uiVLen);
        acValue[uiVLen] = '\0';
    }

    char acResp[128];

    /* --- selPos --- */
    if (strncmp(acId, "selPos", 6u) == 0)
    {
        const uint32_t ui32SelPosCount = (uint32_t)(SELCOUNT * 4u);

        if (cDir == '>')
        {
            char *pcEndChannel = NULL;
            char *pcEndAngle = NULL;
            unsigned long ulChannel = strtoul(acId + 6u, &pcEndChannel, 10);
            unsigned long ulAngle = strtoul(acValue, &pcEndAngle, 10);
            bool bOk = false;

            if (acId[6] != '\0' && pcEndChannel != NULL && *pcEndChannel == '\0' &&
                pcEndAngle != NULL && *pcEndAngle == '\0' && ulChannel < (unsigned long)ui32SelPosCount)
            {
                bOk = (storage_writeSelectorProperty((uint32_t)ulChannel, (uint32_t)ulAngle) == ESP_OK);
            }
            prv_uartSend(bOk ? "selPosOK\n" : "selPosERR\n");
        }
        else
        {
            sel_prop_t sSelProp;
            memset(&sSelProp, 0, sizeof(sSelProp));

            esp_err_t eErr = storage_readSelectorProperty(&sSelProp);
            if (eErr == ESP_OK || eErr == ESP_ERR_NVS_NOT_FOUND)
            {
                char acSelResp[16u + (SELCOUNT * 4u * 12u)];
                size_t uiPos = 0u;
                bool bOk = true;

                int i32Len = snprintf(acSelResp, sizeof(acSelResp), "selPosOK:");
                if (i32Len < 0 || (size_t)i32Len >= sizeof(acSelResp))
                {
                    bOk = false;
                }
                else
                {
                    uiPos = (size_t)i32Len;
                }

                for (uint32_t uiI = 0u; bOk && uiI < ui32SelPosCount; uiI++)
                {
                    i32Len = snprintf(acSelResp + uiPos,
                                      sizeof(acSelResp) - uiPos,
                                      "%lu%s",
                                      (unsigned long)sSelProp.ui32Angle[uiI],
                                      (uiI + 1u < ui32SelPosCount) ? "," : "\n");
                    if (i32Len < 0 || (size_t)i32Len >= (sizeof(acSelResp) - uiPos))
                    {
                        bOk = false;
                    }
                    else
                    {
                        uiPos += (size_t)i32Len;
                    }
                }

                prv_uartSend(bOk ? acSelResp : "selPosERR\n");
            }
            else
            {
                prv_uartSend("selPosERR\n");
            }
        }
        return;
    }

    /* --- devID --- */
    if (strcmp(acId, "devID") == 0)
    {
        credentials_t sCred;
        memset(&sCred, 0, sizeof(sCred));
        storage_readCredentials(&sCred);
        if (cDir == '>')
        {
            strncpy(sCred.deviceId, acValue, sizeof(sCred.deviceId) - 1u);
            bool bOk = (storage_writeCredentials(&sCred) == ESP_OK);
            prv_uartSend(bOk ? "devIDOK\n" : "devIDERR\n");
        }
        else
        {
            snprintf(acResp, sizeof(acResp), "devIDOK:%s\n", sCred.deviceId);
            prv_uartSend(acResp);
        }
        return;
    }

    /* --- devPW --- */
    if (strcmp(acId, "devPW") == 0)
    {
        credentials_t sCred;
        memset(&sCred, 0, sizeof(sCred));
        storage_readCredentials(&sCred);
        if (cDir == '>')
        {
            strncpy(sCred.devicePW, acValue, sizeof(sCred.devicePW) - 1u);
            bool bOk = (storage_writeCredentials(&sCred) == ESP_OK);
            prv_uartSend(bOk ? "devPWOK\n" : "devPWERR\n");
        }
        else
        {
            snprintf(acResp, sizeof(acResp), "devPWOK:%s\n", sCred.devicePW);
            prv_uartSend(acResp);
        }
        return;
    }

    /* --- wifiSsid / wifiPw / fbEmail / fbPw --- */
    if (strcmp(acId, "wifiSsid") == 0 || strcmp(acId, "wifiPw")  == 0 ||
        strcmp(acId, "fbEmail")  == 0 || strcmp(acId, "fbPw")    == 0)
    {
        credentials_t sCred;
        memset(&sCred, 0, sizeof(sCred));
        storage_readCredentials(&sCred);   /* read current values first (ignore error) */

        if (cDir == '>')
        {
            if      (strcmp(acId, "wifiSsid") == 0) { strncpy(sCred.wifiSsid,         acValue, sizeof(sCred.wifiSsid) - 1u); }
            else if (strcmp(acId, "wifiPw")   == 0) { strncpy(sCred.wifiPassword,     acValue, sizeof(sCred.wifiPassword) - 1u); }
            else if (strcmp(acId, "fbEmail")  == 0) { strncpy(sCred.firebaseEmail,    acValue, sizeof(sCred.firebaseEmail) - 1u); }
            else                                    { strncpy(sCred.firebasePassword, acValue, sizeof(sCred.firebasePassword) - 1u); }
            bool bOk = (storage_writeCredentials(&sCred) == ESP_OK);
            snprintf(acResp, sizeof(acResp), "%s%s\n", acId, bOk ? "OK" : "ERR");
        }
        else
        {
            const char *pacFieldVal = "";
            if      (strcmp(acId, "wifiSsid") == 0) { pacFieldVal = sCred.wifiSsid; }
            else if (strcmp(acId, "wifiPw")   == 0) { pacFieldVal = sCred.wifiPassword; }
            else if (strcmp(acId, "fbEmail")  == 0) { pacFieldVal = sCred.firebaseEmail; }
            else                                    { pacFieldVal = sCred.firebasePassword; }
            snprintf(acResp, sizeof(acResp), "%sOK:%s\n", acId, pacFieldVal);
        }
        prv_uartSend(acResp);
        return;
    }

    /* --- btSalt --- */
    if (strcmp(acId, "btSalt") == 0)
    {
        uint8_t aui8Salt[BT_SALT_LEN];

        if (cDir == '>')
        {
            if (!prv_hexDecode(acValue, aui8Salt, BT_SALT_LEN))
            {
                prv_uartSend("btSaltERR\n");
            }
            else
            {
                bool bOk = (storage_writeBtSalt(aui8Salt, BT_SALT_LEN) == ESP_OK);
                prv_uartSend(bOk ? "btSaltOK\n" : "btSaltERR\n");
            }
        }
        else
        {
            if (storage_readBtSalt(aui8Salt, BT_SALT_LEN) == ESP_OK)
            {
                char acHex[BT_SALT_LEN * 2u + 1u];
                prv_hexEncode(aui8Salt, BT_SALT_LEN, acHex, sizeof(acHex));
                snprintf(acResp, sizeof(acResp), "btSaltOK:%s\n", acHex);
                prv_uartSend(acResp);
            }
            else { prv_uartSend("btSaltERR\n"); }
        }
        return;
    }

    /* --- btVerifi --- */
    if (strcmp(acId, "btVerifi") == 0)
    {
        uint8_t *pui8Ver = (uint8_t *)malloc(BT_VERIFIER_LEN);
        if (pui8Ver == NULL) { prv_uartSend("btVerifiERR\n"); return; }

        if (cDir == '>')
        {
            if (!prv_hexDecode(acValue, pui8Ver, BT_VERIFIER_LEN))
            {
                prv_uartSend("btVerifiERR\n");
            }
            else
            {
                bool bOk = (storage_writeBtVerifier(pui8Ver, BT_VERIFIER_LEN) == ESP_OK);
                prv_uartSend(bOk ? "btVerifiOK\n" : "btVerifiERR\n");
            }
        }
        else
        {
            if (storage_readBtVerifier(pui8Ver, BT_VERIFIER_LEN) == ESP_OK)
            {
                char acHex[BT_VERIFIER_LEN * 2u + 1u];
                prv_hexEncode(pui8Ver, BT_VERIFIER_LEN, acHex, sizeof(acHex));
                prv_uartSend("btVerifiOK:");
                prv_uartSend(acHex);
                prv_uartSend("\n");
            }
            else { prv_uartSend("btVerifiERR\n"); }
        }
        free(pui8Ver);
        return;
    }
}

void testFunction(uint8_t* pacData)
{
	if (*pacData == '$')
	{
		prv_handleDollarCmd(pacData);
		return;
	}
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

		case 'p':
		{
            miflora_data_t paFloraData[3];
	        log_peripherieData(paFloraData);
		}
		break;

        case 'n':
        {   
            ESP_LOGI(TAGA, "Battery Level: %d", (int)ui32BattLevel_read());
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
			selector_setAngle((*(pacData + 1))*2, false, (*(pacData + 2)));
			// ESP_ERROR_CHECK(gpio_set_level(PIN_SEL1_EN, 0));        		//enable driver
			// TLV_deinit();
		}
		break;

		case 'i':
		{
			// ESP_ERROR_CHECK(gpio_set_level(PIN_SEL2_EN, 1));        		//enable driver
			// vTaskDelay(100);		//wait for capacitors loaded
			// TLV_init();				//init TLV sensor
			selector_setPos((*(pacData + 1)));
			// ESP_ERROR_CHECK(gpio_set_level(PIN_SEL2_EN, 0));        		//enable driver
			// TLV_deinit();
		}
		break;
	

		case 'c':
		{
			selector_caliPos();
		}
		break;

		case 't':
		{
	        ESP_LOGI(TAGA, "DATA: %f", fTemp_read());
		}
		break;

		case 'w':
		{
            #if PWM_MUX_TANKLVL
			ESP_LOGI(TAGA, "DATA: %ld", ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100));
            #endif
			// uint32_t ui32Position = (uint32_t)(*(pacData + 1));
//	        ESP_LOGI(TAGA, "DATA: %d", ui32Position_read());
			// selector_set(ui32Position);
		}
		break;

		case 'y':
		{
            #ifdef CAPHUMSENSE_ENABLE
			ESP_LOGI(TAGA, "DATA: %ld", ui32AdcTouch_readPwmMux((adc_mux_t)(*(pacData + 1)), 500));
            #endif
		}
		break;

		case 'z':
		{
            #ifdef VERTILIZING_ENABLE 
            const uint32_t ui32PumpTimeFact = (uint32_t)(0.6*1000);      //[ms/ml]
            const uint32_t ui32PumpPulseDuration = 5 * ui32PumpTimeFact + 500;     //5[ml] * ui32PumpTimeFact[ms/ml] + watering dead time (500ms)-> [ms] - pulse duration of watering cycles

            const uint32_t ui32SoilTimeFact = (uint32_t)(2);      //[ms/µl]

            //make soil wet
            pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_DOWN, 1);          //run pump for defined time in vertilizing direction
            esp_sleep_enable_timer_wakeup(10 * 1000 * 1000);    //wait 10s
            esp_light_sleep_start();
            pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_DOWN, 1);          //run pump for defined time in vertilizing direction
            esp_sleep_enable_timer_wakeup(10 * 1000 * 1000);    //wait 10s
            esp_light_sleep_start();     

            //vertilize
            pump_runTimeOpenLoop(ui32SoilTimeFact * 50U, MOTOR_DIR_UP, 2, 3000);          //run pump for defined time in vertilizing direction

            //and flush with clear water
            pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_DOWN, 1);          //run pump for defined time in vertilizing direction
            #endif
		}
		break;

		case 'h':
		{
            #if FDC1004_ENABLE
			ESP_LOGI("Humidity", "Capacitance: %f", (float)FDC_getCap((uint32_t)(*(pacData + 1))) / 524288.0);
            #endif
		}
		break;

		case 'f':
		{
            #if PWM_MUX_TANKLVL
			ESP_LOGI("LEVEL", "ui32Level_readMl: %d", (int)ui32Level_readMl());
            #endif
		}
		break;

		case 'g':
		{
            int32_t i32LevelPerc;
            erLevel_readPerc(&i32LevelPerc);
			ESP_LOGI("LEVEL", "erLevel_readPerc: %d", (int)i32LevelPerc);
		}
		break;

		case 'r':
		{
			uint32_t ui32Time = (uint32_t)(*(pacData + 1))*100;
			uint32_t ui32Direction = (uint32_t)(*(pacData + 2));

			// select            
            #ifdef VERTILIZING_ENABLE 
            const uint32_t ui32PumpTimeFact = (uint32_t)(0.6*1000);      //[ms/ml]
            const uint32_t ui32PumpPulseDuration = 5 * ui32PumpTimeFact + 500;     //5[ml] * ui32PumpTimeFact[ms/ml] + watering dead time (500ms)-> [ms] - pulse duration of watering cycles

            const uint32_t ui32SoilTimeFact = (uint32_t)(2);      //[ms/µl]

            //make soil wet
            pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_DOWN, 1);          //run pump for defined time in vertilizing direction
            esp_sleep_enable_timer_wakeup(10 * 1000 * 1000);    //wait 10s
            esp_light_sleep_start();
            pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_DOWN, 1);          //run pump for defined time in vertilizing direction
            esp_sleep_enable_timer_wakeup(10 * 1000 * 1000);    //wait 10s
            esp_light_sleep_start();     

            //vertilize
            pump_runTimeOpenLoop(ui32SoilTimeFact * 50U, MOTOR_DIR_UP, 2, 3000);          //run pump for defined time in vertilizing direction

            //and flush with clear water
            pump_runTime(ui32PumpPulseDuration, MOTOR_DIR_DOWN, 1);          //run pump for defined time in vertilizing direction
            #endif
        }

        break;

		case 's':
		{
			// bPowerstage_init();
			uint32_t ui32Time = (uint32_t)(*(pacData + 1))*100;
			uint32_t ui32Direction = (uint32_t)(*(pacData + 2));
            uint32_t ui32PumpNb = (uint32_t)(*(pacData + 3));
			pump_runTime(ui32Time, ui32Direction, ui32PumpNb);
		}
		break;

		case 'x':
		{
            #ifdef HX710_ENABLE
            int32_t i32Pressure;
			ui32HX710_read(&i32Pressure);
            #else
            ESP_LOGI("ui32HX710_read", "Sensor not supported");
            #endif
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
            #ifdef VERTILIZING_ENABLE
            pump_runTimeOpenLoop(200, MOTOR_DIR_UP, 2, 3000);          //run pump for defined time in vertilizing direction
            #endif
		}
		break;

		case 'a':
		{
			esp_err_t  eEspError = ESP_OK;
			// eEspError = adc_calibration_150mV();
	        ESP_LOGI(TAGS, "DATA: %s", esp_err_to_name(eEspError));
		}
		break;

		case 'b':
		{
			esp_err_t eEspError = ESP_OK;
			// eEspError = adc_calibration_850mV();
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

		case 'm':
		{
			miflora_data_t pFloraData;
			ble_miflora_init();
			ble_miflora_read(0, &pFloraData);
			ble_miflora_read(1, &pFloraData);
			ble_miflora_read(2, &pFloraData);
			ble_miflora_deinit();
		}
		break;

        case 'e':
        {
            led_set((*(pacData + 1)), (led_status_t)(*(pacData + 2)));
        }
	}
}
