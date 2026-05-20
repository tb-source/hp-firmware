/*
 * periphery_lg.c
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 */

#include "periphery_lg.h"
#include "rom/ets_sys.h"		//for us delay function - delete further


static const uint32_t s_cui32LogicVoltageFact = (uint32_t)((100.0+33.0)/33.0 * 4096);          					//voltage divider (Rpu(100kR)+Rpd(33kR))/Rpd(33kR)) (16Q12)
static const uint32_t s_cui32BattVoltageFact = (uint32_t)((33.0+10.0)/10.0 * 4096);          					//voltage divider (Rpu(33kR)+Rpd(10kR))/Rpd(10kR)) (16Q12)
static const uint32_t s_cui32SolarVoltageFact = (uint32_t)((200.0+33.0)/33.0 * 4096);          					//voltage divider (Rpu(200kR)+Rpd(33kR))/Rpd(33kR)) (16Q12)


static uint32_t s_ui32SupplyVoltage = 0;																		//logic supply voltage 14Q0 [mV]
static uint32_t s_ui32BattVoltage = 0;																			//battery supply voltage 14Q0 [mV]

const float g_cafNTCTempValues[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};
const float g_cafNTCFactValues[] = {0.0264, 0.0346, 0.0449, 0.0575, 0.0727, 0.0909, 0.1123, 0.1371, 0.1652, 0.1968, 0.2315, 0.2691, 0.3090};


static uint32_t s_ui32LevelMin = 0;
static uint32_t s_ui32LevelMax = 0;

static motor_regulation_t s_motorRegulation = {.voltageRegKp = 2000,				//12Q12
											   .voltageRegKi = 500,				//12Q12
											   .motorResitance = 2118,			//14Q10 [R]
											   .motorSpeedFactor = 18392,		//16Q16 [1/min*mV]
											   .voltageEmfSet = 10000};			//16Q4

static sel_prop_t s_saSelProperties = {0};		//angle values for selector positions (16Q16) [°*65536]
static volatile motor_direction_t s_motorState;
static volatile int s_eMotorRegulationSM = MOTOR_REG_IDL;

static volatile int32_t s_i32CurSensOffsetVoltage;										//voltage offset of current sensor 16Q4
// static const int32_t s_ci32CurSensFact =  (int32_t)(8192 * 2.13);						//ADC factor 4.7/0.1R /4.7 15Q13 [mA/mV]
//static const float s_cfMotorResistance = 1;													//Motorresistance 1R


static QueueHandle_t speed_sens_evt_queue = NULL;						//queue for Speed sensing events
static volatile uint32_t s_ui32PumpNb = 0;								//pump number running 0...x
static volatile uint32_t s_i32MotorDuration = 60000000;

//analyser
int32_t g_ai32Analyser[512];


//functions
static esp_err_sel_t selector_findPos(void);
static void speed_sens_task(void *arg);
static void IRAM_ATTR speed_sens_isr_handler(void* arg);


//************************          LED           *******************************************/
static led_status_t s_aeLedState[] = {LED_OFF, LED_OFF};		//time for led blinking in ms
static void led_task();

void debug_init(void)
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	io_conf.mode = GPIO_MODE_INPUT;                                                           //set as input mode
	io_conf.pin_bit_mask = (1ULL << PIN_DEBUG);                                                 //bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                           //disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                                //disable pull-up mode
	gpio_config(&io_conf);
}

//initalise LEDs (red and yellow)
void led_init(void)
{
  //init leds
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
  io_conf.mode = GPIO_MODE_OUTPUT;                                                          //set as output mode
  io_conf.pin_bit_mask = ((1U << PIN_LEDS[0])|(1U << PIN_LEDS[1]));     					//bit mask of the pins
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE  ;                                           //disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                //disable pull-up mode
  gpio_config(&io_conf);

  gpio_set_level(PIN_LEDS[0], 0);                                                         	//disable LED1
  gpio_set_level(PIN_LEDS[1], 0);                                                           //disable LED2

  xTaskCreate(led_task, "led_task", 1024, NULL, 2, NULL);
}


//switch led  $input: uiLEDNumber - number of LED (1...4), uiLevel - state of led
void led_set(uint32_t ui32LEDNumber, led_status_t eLedStatus)
{
	if(ui32LEDNumber > 2 || ui32LEDNumber < 1)  	// Check if LED number is out of range
	{
		ESP_LOGE("LED", "LED number out of range");
		return;
	}

	switch(eLedStatus)
	{
		case LED_OFF:
			s_aeLedState[ui32LEDNumber - 1] = eLedStatus;
			gpio_set_level(PIN_LEDS[ui32LEDNumber - 1], 0);
			break;
		case LED_ON:
			s_aeLedState[ui32LEDNumber - 1] = eLedStatus;
			gpio_set_level(PIN_LEDS[ui32LEDNumber - 1], 1);
			break;
		case LED_BLINK_SLOW:
		case LED_BLINK_FAST:
			s_aeLedState[ui32LEDNumber - 1] = eLedStatus;
			/* Blink handling is performed in led_task(). */
			break;
		default:
			ESP_LOGE("LED", "Invalid LED status");
			return;
	}
}


//task for LED control
static void led_task()
{
	const int32_t ai32BlinkOnTime[] =  {2, 1};		//blink on time in ms	
	const int32_t ai32BlinkOffTime[] = {-2, -1};		//blink off time in ms	
	int32_t ai32LEDCounter[2] = {-1, -1};

	while(1)
	{
		//500ms delay for task switching
		for (uint32_t ui32LEDNumber = 0; ui32LEDNumber < 2; ui32LEDNumber++)
		{
			if (s_aeLedState[ui32LEDNumber] == LED_BLINK_SLOW || s_aeLedState[ui32LEDNumber] == LED_BLINK_FAST)
			{
				//LED on
				if(ai32LEDCounter[ui32LEDNumber] > 0)
				{
					if(ai32LEDCounter[ui32LEDNumber] >= ai32BlinkOnTime[s_aeLedState[ui32LEDNumber] - 2])
					{
						gpio_set_level(PIN_LEDS[ui32LEDNumber], 0);			//switch off
						ai32LEDCounter[ui32LEDNumber] = -1;
					}
					else
					{
						ai32LEDCounter[ui32LEDNumber]++;
					}
				}
				//LED off
				else
				{
					if(ai32LEDCounter[ui32LEDNumber] <= ai32BlinkOffTime[s_aeLedState[ui32LEDNumber] - 2])
					{
						gpio_set_level(PIN_LEDS[ui32LEDNumber], 1);			//switch on
						ai32LEDCounter[ui32LEDNumber] = 1;
					}
					else
					{
						ai32LEDCounter[ui32LEDNumber]--;
					}
				}
			}
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}


//initalise powerstage
bool bPowerstage_init(void)
{
	static bool s_bPowerstageInitDone = false;
	if (!s_bPowerstageInitDone)
	{
		s_bPowerstageInitDone = true;
	}
	else
	{
		return false;		//already done
	}
	
	ESP_LOGI("Powerstage", "bPowerstage_init");

	//init speed measurement (Hall sensor)
	speed_sens_evt_queue = xQueueCreate(1, sizeof(uint32_t));
	// xTaskCreate(speed_sens_task, "speed_sens_task", 2048, NULL, 10, NULL);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(PIN_PUMP_SPEED[0], speed_sens_isr_handler, (void*) PIN_PUMP_SPEED[0]);
	gpio_isr_handler_add(PIN_PUMP_SPEED[1], speed_sens_isr_handler, (void*) PIN_PUMP_SPEED[1]);

	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (1ULL << PIN_PUMP_SPEED[0]) | (1ULL << PIN_PUMP_SPEED[1]);
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpio_config(&io_conf);

	//init ledc timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,		//duty resolution 1023bit 
        .freq_hz          = 1000,  					// Set output frequency at 500 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	//init channel1 - Selector 1
	ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_HS,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));	
	ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE , LEDC_CHANNEL_0, 0));

	//init channel2 - Selector B
	ledc_channel_config_t ledc_channel2 = ledc_channel1;
	ledc_channel2.gpio_num = PIN_LS;
	ledc_channel2.channel = LEDC_CHANNEL_1;
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));	
	ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE , LEDC_CHANNEL_1, 0));

	s_motorState = MOTOR_DIR_IDLE;												//set direction to idle

	//Proof if battery voltage is measured
	if (s_ui32SupplyVoltage == 0)
	{
		ui32EspVolt_read();
	}
	if (s_ui32BattVoltage == 0)
	{
		ui32BattVolt_read();
	}

	//read selector position from nvs
	storage_readSelectorProperty(&s_saSelProperties);


	// ESP_LOGI("MCPWM: ", "ui32SelPos: %d", (int)s_ui32SelPos);
	return true;
}

void powerstage_deinit(void)
{
	// storage_write("sel_pos", s_ui32SelPos);
	// ESP_LOGE("NVS", "%s", esp_err_to_name(storage_write("sel_pos", 5)));
}

static void speed_sens_task(void *arg)
{
	uint32_t ui32SpeedDuration = 1;
	ESP_LOGI("Speed: ", "Task created");
	while (true)
	{
		if (xQueueReceive(speed_sens_evt_queue, &ui32SpeedDuration, portMAX_DELAY))
		{
			ESP_LOGI("Speed: ", "Pulse frequency: %d rpm", (int)(60000000U/ui32SpeedDuration));
			// if (ui32SpeedDuration > 0)
			// {
			// 	s_motorRegulation.speedAct = (uint32_t)(60000 / ui32SpeedDuration); // Convert to RPM
			// 	// ESP_LOGI("Speed: ", "Current speed: %d RPM", s_motorRegulation.speedAct);
			// }
		}
	}
}

static void IRAM_ATTR speed_sens_isr_handler(void* arg) 
{
	int64_t i64GetTime = esp_timer_get_time();
	gpio_num_t eIsrPin = (gpio_num_t)(uint32_t)arg;
	if (eIsrPin != PIN_PUMP_SPEED[s_ui32PumpNb])
	{
		return;
	}
	//check gpio level is "1"
	if (gpio_get_level(eIsrPin) == 1)
	{
		// ESP_LOGI("Speed: ", "OK");
		volatile static int64_t i64PulseStart = 0;
		volatile static int64_t i64PulseEnd = 0;
		volatile static bool pulseStarted = false;	
		uint32_t ui32SpeedPulse = 60000000;

		if (!pulseStarted) {
			i64PulseStart = i64GetTime;
			pulseStarted = true;
		} else {
			i64PulseEnd = i64GetTime;
			int64_t i64SpeedPulseDuration = i64PulseEnd - i64PulseStart;

			//ignore glitches smaller 3000us -> 20000rpm
			if(i64SpeedPulseDuration > 3000)
			{
				ui32SpeedPulse = (uint32_t)i64SpeedPulseDuration;
				i64PulseStart = i64PulseEnd;
				xQueueSendFromISR(speed_sens_evt_queue, &ui32SpeedPulse, NULL);
				s_i32MotorDuration = ui32SpeedPulse;			
			}
		}		
	}

}

//run selector motor $ui32Speedbool: dutycycle 10Q10 $eDirection: direction of motor,  $return: esp_err_t -> running motor sucessful
esp_err_t selector_run(uint32_t ui32Speedbool, motor_direction_t eDirection)
{
	esp_err_t err = ESP_OK;
	static motor_direction_t s_eMotorDirection = MOTOR_DIR_IDLE;

	//Proof if battery voltage is measured
	if (s_ui32SupplyVoltage == 0)
	{
		ui32EspVolt_read();
		if (err != ESP_OK) return err;
	}

	//check for direction change
	if (s_eMotorDirection != eDirection)
	{
		selector_stop();						//stop motor before start	
		s_eMotorDirection = eDirection;			//set motor state
	}


	switch(eDirection)
	{
		case MOTOR_DIR_UP:
			//activate pwm
			err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ui32Speedbool);
			// err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, fSpeedbool);
		    if (err != ESP_OK) return err;
			err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
		    // err = mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);   	//nmos 1 switching
		    if (err != ESP_OK) return err;
		    break;

		case MOTOR_DIR_DOWN:
			//activate pwm
			err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ui32Speedbool);
		    // err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, fSpeedbool);
		    if (err != ESP_OK) return err;
			err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
		    // err = mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);   	//pmos 1 switching
		    if (err != ESP_OK) return err;
			break;

		default:
			err = ESP_ERR_INVALID_ARG;
			return err;
			break;
	}

	return err;
}

//run pump motor $ui32Speedbool: dutycycle(10Q10) $eDirection: direction of motor $ui32PumpNb: pump number,  $return: esp_err_t -> running motor sucessful
esp_err_t pump_run(uint32_t ui32Speed, motor_direction_t eDirection)
{
	esp_err_t err = ESP_OK;

	switch(eDirection)
	{
		case MOTOR_DIR_UP:
			//activate pwm
			ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, ui32Speed));
			ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
		    break;

		case MOTOR_DIR_DOWN:
			//activate pwm
			ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, ui32Speed));
			ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
			break;

		default:
			err = ESP_ERR_INVALID_ARG;
			return err;
			break;
	}

	return err;
}

//stop selector motor  $return: esp_err_t -> running motor sucessful
esp_err_t selector_stop(void)
{
	esp_err_t err = ESP_OK;
	//brake motor
	ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE , LEDC_CHANNEL_0, 1));
	ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE , LEDC_CHANNEL_1, 1));
	vTaskDelay(1000);

	//switch motor off
	ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE , LEDC_CHANNEL_0, 0));
	ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE , LEDC_CHANNEL_1, 0));

	return err;
}

//stop pump motor  $return: esp_err_t -> running motor sucessful
esp_err_t pump_stop(void)
{
	esp_err_t err = ESP_OK;

	//switch motor off
	ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE , LEDC_CHANNEL_0, 0));
	ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE , LEDC_CHANNEL_1, 0));

	return err;
}

//positionate slector $i32SetAngleRaw: set angle of selector, $eSelNb: number of selector, $bInitTLV: TLV initialized before? $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_setAngle(int32_t i32SetAngleRaw, bool bInitTLV, int32_t i32SelNb)
{
	if(i32SelNb < 1 || i32SelNb > SELCOUNT)
	{
		ESP_LOGE("Selector: ", "Error selector number out of range");
		return ERR_SEL_POS_OUTOFRANGE;
	}

	if (i32SetAngleRaw > 359)
	{
		ESP_LOGE("Selector: ", "Error angle > 360°");
		return ERR_SEL_POS_OUTOFRANGE;
	}

	//enable driver/magnet sensor
	if (!bInitTLV)
	{	
		i2c_deinit();
		ui32AdcTouch_readAdcMux(ADC_MUX_CUR_SEL1 + i32SelNb - 1, 1);		//read once to enable selector
		// ESP_ERROR_CHECK(gpio_set_level(PIN_SEL_EN, 1));        		//enable driver
		// vTaskDelay(100);		//wait for capacitors loaded

		i2c_init();
		// ESP_LOGI("I2C: ", "Init done");
		TLV_init();				//init TLV sensor
		// ESP_LOGI("TLV: ", "Init done");

	}

	//init powerstage
	bPowerstage_init();

	esp_err_sel_t err = ERR_SEL_OK;
	motor_direction_t eMotorDir = MOTOR_DIR_IDLE;
	motor_direction_t eMotorDirLast = MOTOR_DIR_IDLE;
	uint32_t ui32TimeWatch = 0;
	uint32_t ui32DirectionCount = 0;
	static int32_t s_i32ActAngleLast;
	int32_t i32SetAngle = i32SetAngleRaw;
	s_i32ActAngleLast = i32TLV_getAngle();

	//positionate selector
	if(s_i32ActAngleLast != i32SetAngleRaw)
	{
		int32_t i32ActAngleOffset = 0;
		
		//calc shortest direction
		if (i32SetAngleRaw > s_i32ActAngleLast)
		{
			if ((i32SetAngleRaw - s_i32ActAngleLast) > 180)
			{
				// adapt offset value for angle overflow 0° to 360°
				i32ActAngleOffset = 360;
				eMotorDir = MOTOR_DIR_UP;		//move neg angle
			}
			else
			{
				eMotorDir = MOTOR_DIR_DOWN;		//move pos angle
			}
		}
		else
		{
			if ((s_i32ActAngleLast - i32SetAngleRaw) > 180)
			{
				// adapt offset value for angle overflow 360° to 0°
				i32SetAngle += 360;
				eMotorDir = MOTOR_DIR_DOWN;		//move pos angle
			}
			else
			{
				eMotorDir = MOTOR_DIR_UP;		//move neg angle
			}
		}

		//adapt value for overflow area +-30° 
		if ((i32SetAngleRaw > 330 || i32SetAngleRaw < 30) && (eMotorDir == MOTOR_DIR_UP))
		{
			i32ActAngleOffset = 360;
			if (i32SetAngle < 180)
			{
				i32SetAngle += 360;
			}
		}
		
		// ESP_LOGI("SEL", "SetAngle: %i", (int)i32SetAngle);
		// ESP_LOGI("SEL", "ActAngle: %i", (int)s_i32ActAngleLast);

		while((abs(s_i32ActAngleLast - i32SetAngleRaw) > 1))
		{
			vTaskDelay(5);
			ui32TimeWatch++;
			if (ui32TimeWatch > 500)		//300 - longer than 3s		
			{
				selector_stop();
				err = ERR_SEL_OT;
				ESP_LOGE("Selector: ", "Error Overtime");
				break;
			}

			// if (selector_motor_regulation(eMotorDir) == ERR_SEL_OC)
			// {
			// 	selector_stop();
			// 	err = ERR_SEL_OC;
			// 	ESP_LOGE("Selector: ", "Error Overcurrent");
			// 	break;
			// }
			//selector pos changed


			int32_t i32ActAngleRaw = i32TLV_getAngle();			//read new angle value
			// ESP_LOGI("SEL", "DEG: %i", (int)i32ActAngleRaw);

			//check for overflow
			if ((i32ActAngleRaw - s_i32ActAngleLast)  > 180)			//overflow 0° to 360°
			{
				i32ActAngleOffset = 0;
			}
			else if ((s_i32ActAngleLast - i32ActAngleRaw)  > 180)		//overflow 360° to 0°
			{
				i32ActAngleOffset = 360;
			}
	
			// ESP_LOGI("SEL", "OFFSET: %i", (int)i32ActAngleOffset);

			int32_t ui32ActAngle = i32ActAngleRaw + i32ActAngleOffset;	//calculate new angle value

			//Regulator
			int32_t i32DeltaAngle = (int32_t)i32SetAngle - (int32_t)ui32ActAngle;					//calculate angle difference
			int32_t i32Duty = i32DeltaAngle * 50;													//dutycylce 50% * 1023 - 10Q10											
			// ESP_LOGI("SEL", "Duty: %i", (int)i32Duty);

			if (i32Duty > 0)	
			{
				eMotorDir = MOTOR_DIR_DOWN;		//motor direction positive angle
			}
			else
			{
				eMotorDir = MOTOR_DIR_UP;		//motor direction negative angle
				i32Duty = -i32Duty;
			}

			if (i32Duty > 1023)																		//limit dutycycle to 100%
			{
				i32Duty = 1023;
			}

			if (eMotorDirLast != eMotorDir)
			{
				ui32DirectionCount++;
				if (ui32DirectionCount > 3)
				{
					selector_stop();
					err = ERR_SEL_POS_OUTOFRANGE;
					ESP_LOGE("Selector: ", "Error end position not found");
					break;
				}
			}
			
			// ESP_LOGI("SEL", "Duty: %i", (int)i32Duty);
			selector_run((uint32_t)i32Duty, eMotorDir);		//dutycylce xx% * 1023

			s_i32ActAngleLast = i32ActAngleRaw;			//save last angle value
			eMotorDirLast = eMotorDir;					//save last motor direction
			// err = ERR_SEL_POS_OUTOFRANGE;

		}
		selector_stop();
//		led_switch(1, 0);
	}

	if (!bInitTLV)
	{
		ui32AdcTouch_readAdcMux(ADC_MUX_VOLT_3V3, 1);		//read once to enable selector        		//disable driver
	}														//deinit TLV sensor

	return err;
}

//positionate slector $i32SetPos: set pos of selector, $eSelNb: number of selector $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_setPos(int32_t i32SetPos)
{
	esp_err_t err = ESP_OK;

	if (i32SetPos < 0 || i32SetPos > CHANNELCOUNT)
	{
		ESP_LOGE("selector_setPos: ", "Error position out of range");
		return ERR_SEL_POS_OUTOFRANGE;
	}

	switch(i32SetPos)
	{
		case 0:
			//set all selectors to position 0
			err = selector_setAngle(s_saSelProperties.ui32Angle[0], false, 1);
			err |= selector_setAngle(s_saSelProperties.ui32Angle[0 + 4], false, 2);
			break;
		case 1:
			err = selector_setAngle(s_saSelProperties.ui32Angle[1], false, 1);
			break;
		case 2:
			err = selector_setAngle(s_saSelProperties.ui32Angle[2], false, 1);
			break;
		case 3:
			err = selector_setAngle(s_saSelProperties.ui32Angle[3], false, 1);
			break;	
		case 4:
			err = selector_setAngle(s_saSelProperties.ui32Angle[7], false, 2);
			break;
		case 5:
			err = selector_setAngle(s_saSelProperties.ui32Angle[6], false, 2);
			break;
		case 6:
			err = selector_setAngle(s_saSelProperties.ui32Angle[5], false, 2);
			break;
		default:
			ESP_LOGE("selector_setPos: ", "Error position out of range");
			return ERR_SEL_POS_OUTOFRANGE;
			break;
	}

	return err;
}

//calibrate slectors position $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_caliPos(void)
{
	ESP_LOGI("Selector: ", "selector_caliPos()");
	uint32_t ui32TimeCount = 0;

	//enable driver/magnet sensor
	// gpio_set_level(PIN_SEL_EN, 1);        		//enable driver
	vTaskDelay(100);

	TLV_init();		//init TLV sensor
	int32_t i32LastAngle = 0;
	uint32_t ui32StuckCount = 0;

	//position points dir down
	for (int i = 0; i < 36; i++)
	{
		selector_setAngle(i * 10, true, 1);	//set angle +10° for next position

		uint32_t ui32Duty = 0;
		int32_t i32Angle = i32TLV_getAngle();			//read first angle value
		i32LastAngle = i32Angle;		
		
		// while(i32LastAngle == i32Angle)
		// {
		// 	ui32Duty+=10;		//increase dutycycle
		// 	selector_run(ui32Duty, MOTOR_DIR_DOWN);		//dutycylce xx% * 1023
		// 	vTaskDelay(20);			//wait 10ms
		// 	i32Angle = i32TLV_getAngle();			//read new angle value
		// 	if(ui32Duty > 1000)
		// 	{
		// 		selector_stop();
		// 		ESP_LOGE("Selector: ", "Error timeout");
		// 		break;
		// 	}
		// }

		// ESP_LOGI("Sel", "Pos: %ld Frequency: %ld", i32Angle*10, (uint32_t)ui32AdcTouch_readPwmMux(PWM_MUX_SEL, 100));
		// selector_stop();
		// i32LastAngle = i32Angle;

		// selector_run(300U, MOTOR_DIR_DOWN);		//dutycylce xx% * 1023

		// while (ui32TimeCount < 1000)		//wait for 1s
		// {
		// 	ui32TimeCount++;		
		// 	vTaskDelay(5);	//wait 5ms
		// 	int32_t i32Angle = i32TLV_getAngle();			//read new angle value
		// 	ESP_LOGI("Sel", "%i", (int)i32Angle);
		// 	if (i32Angle == i32LastAngle)
		// 	{
		// 		ui32StuckCount++;
		// 		if (ui32StuckCount > 10)		//wait for 100ms
		// 		{
		// 			// selector_stop();
		// 			ESP_LOGI("Sel", "AngleFinal: %i", (int)i32Angle);
		// 			break;
		// 		}
		// 	}
		// 	else{
		// 		ui32StuckCount = 0;
		// 	}
		// 	i32LastAngle = i32Angle;
		// }
		// ui32StuckCount = 0;

	}

	selector_stop();
	
	return ERR_SEL_OK;
}

//run pump for defined time $ui32Time: time in ms, $eDirection: direction of motor, $ui32PumpNb: pump number 1..x,  $return: esp_err_t -> running motor sucessful
esp_err_pump_t pump_runTime(uint32_t ui32Time, motor_direction_t eDirection, int32_t ui32PumpNb)
{
	const int32_t c_i32MotorSpeedSet = 2500;	//set motor speed in rpm
	const int32_t i32BattVoltage = (int32_t)ui32BattVolt_read();					//measrure battery voltage

	esp_err_pump_t err = ERR_PUMP_OK;

	if (ui32PumpNb > PUMPCOUNT || ui32PumpNb <= 0)
	{
		ESP_LOGE("Pump: ", "Pump number out of range");
		return ERR_PUMP_CNT;
	}

	int32_t i32PressureMbar;
	ui32HX710_read(&i32PressureMbar);		//read once to enable current sensor

    s_ui32PumpNb = ui32PumpNb - 1;		//set pump number for isr handler and speed sensing task
	ui32AdcTouch_readAdcMux(ADC_MUX_CUR_PUMP1 + s_ui32PumpNb, 1);		//read once to enable selector

	volatile uint32_t ui32ActTime = 0;

	int64_t i64EndTime = esp_timer_get_time() + (ui32Time * 1000);
	uint32_t ui32SpeedSet = 60000000;
	xQueueSend(speed_sens_evt_queue, &ui32SpeedSet, 0);					//reset speed value
	s_i32MotorDuration = ui32SpeedSet;	
	volatile uint32_t counter = 0;

	volatile int32_t i32SpeedVoltage = 500 * 1024; 		//initial speed voltage (first 100ms) 20Q10 [mV]

	static volatile int32_t s_i32SpeedDevSum = 0; 			//i32SpeedVoltage/1024*1000;
	volatile int32_t s_i32SpeedDevSumLast[] = {0,0,0,0,0};

	if(s_i32SpeedDevSum == 0)
	{
		ESP_ERROR_CHECK(storage_read("SpeedDevSum", &s_i32SpeedDevSum));
	}
	
	// s_i32SpeedDevSum = 44782;		//delete for later purpose

	// volatile int32_t i32CurrSet = 0;						//[mA]
	// volatile int32_t i32CurrDevSum = 0;
	// volatile int32_t i32CurrDevLast = 0;

	int32_t i32OCcount = 0;

	while(esp_timer_get_time() < i64EndTime)
	{		
		
		//************************************************* speed adaption  ************************************************/
		// static int32_t s_i32MotorSpeed = 60000000;
		// const int32_t i32SpeedKp = (int32_t)(.005 * 1024);			//10Q10
		// const int32_t i32SpeedKi = (int32_t)(.005 * 1024);			//10Q10

		// //wait for 50ms
		// if(counter > 5)
		// {
		// 	int32_t i32MotorSpeed = 1;
		// 	s_i32MotorSpeed = s_i32MotorDuration;

		// 	i32MotorSpeed = 60000000 / s_i32MotorSpeed;	
		// 	int32_t i32RegDev = c_i32MotorSpeedSet - i32MotorSpeed;								//calc deviation to set speed
		// 	ESP_LOGI("pump_runTime","i32RegDev: %d", (int)i32RegDev);
		// 	s_i32SpeedDevSum+=i32RegDev;
		// 	// if (s_i32SpeedDevSum > 50000) s_i32SpeedDevSum = 500000;		//limit integral part to 2,5V
		// 	// if (s_i32SpeedDevSum < -20000) s_i32SpeedDevSum = -200000;		//limit integral part to -1V
		// 	i32SpeedVoltage = (i32RegDev * i32SpeedKp) + (s_i32SpeedDevSum * i32SpeedKi);	//32Q10[mV]
		// 	s_i32SpeedDevSumLast[counter % 5] = s_i32SpeedDevSum;
		// 	// i32SpeedVoltage = 352000;
		// 	// i32SpeedVoltage = 500*1024;
		// }
		

		// //************************************************* current voltage regulation  ************************************************/
		// int32_t i32AdcReadVoltage = ui32AdcTouch_readAdcMux(ADC_MUX_CUR_PUMP, 2);		//read adc mux for current sensor 12Q0[mV]
		// //16Q4 -> 12Q0 * 12Q8 -> 24Q8 >> 4 -> 16Q4[mA]
		// int32_t i32SelCurrent = ((i32AdcReadVoltage - i32AdcReadVoltageOffset) * ((int32_t)(2.275 * 256)))>>4;		// 2.275 mA/mV sensitivity
		// //18Q4*10Q6 -> 28Q10
		// int32_t i32SelResVoltage = i32SelCurrent * (uint32_t)(1.65 * 64);	//[mV] = 15Q4[mA] * 10Q6[R] (Resistance motor 1,5R + driver ,15R)
		// // ESP_LOGI("pump_runTime","i32SelResVoltage: %ld",i32SelResVoltage>>10);
		// int32_t i32SelSetVoltage = i32SelResVoltage + i32SpeedVoltage;
		// // i32SelSetVoltage = 2500 * 1024;	//2V
		// int32_t i32DutyCyle = i32SelSetVoltage / i32BattVoltage;			//29Q10 / 14Q0 -> 15Q10
		// // int32_t i32DutyCyle = 1500*1024 / i32BattVoltage;	

		// ESP_LOGI("pump_runTime","Counter: %d, Speed: %drpm, i32SpeedVoltage: %d, CurrAct: %d, s_i32SpeedDevSum: %d", (int)counter, (int)(60000000 / s_i32MotorDuration), (int)i32SpeedVoltage, (int)(i32SelCurrent >> 4), (int)s_i32SpeedDevSum);
		//************************************************* speed regulation  ************************************************/
		// static int32_t s_i32MotorSpeed = 60000000;
		int32_t i32DutyCyle = 0;
		if ((counter%1)==0)
		{
			int32_t i32MotorSpeed = 1;
			// xQueueReceive(speed_sens_evt_queue, &s_i32MotorSpeed, 0);
			i32MotorSpeed = 60000000 / s_i32MotorDuration;											//calc time to frequency [rpm]
			int32_t i32SpeedKp = (int32_t)(0.01 * 1024);		//10Q10
			int32_t i32SpeedKi = (int32_t)(0.005 * 1024);		//10Q10;

			int32_t i32RegDev = c_i32MotorSpeedSet - i32MotorSpeed;								//calc deviation to set speed

			s_i32SpeedDevSum+=i32RegDev;
			i32DutyCyle  = ((i32RegDev * i32SpeedKp) + (s_i32SpeedDevSum * i32SpeedKi))>>10;											//calc dutycycle 22Q10
			// i32DutyCyle += 250;																//Vorsteuerung
			// i32CurrSet>>=10;																	//shift 20Q10 to 32Q0
			// i32CurrSet += 160;			//[160mA @ 40001/min] "vorsteuerung" speed control, current with no load
			ESP_LOGI("pump_runTime","counter: %d, i32MotorSpeed: %d, i32RegDev: %d, i32DutyCyle: %d, s_i32SpeedDevSum: %d", (int)counter, (int)i32MotorSpeed, (int)i32RegDev, (int)i32DutyCyle, (int)s_i32SpeedDevSum);
		}

		//save speed deviation
		s_i32SpeedDevSumLast[counter % 5] = s_i32SpeedDevSum;

		//************************************************* current regulation  ************************************************/
		// int32_t i32AdcValue = adc_read_mux(ADC_MUX_CUR_PUMP, 2);		//read adc mux for current sensor
		// i32AdcValue *= 5;												//200mR resitor -> V/A = 5
		// ESP_LOGI("pump_runTime","Counter: %d, Speed: %drpm, CurrSet: %d, CurrAct: %d", (int)counter, (int)(60000000 / s_i32MotorSpeed), (int)i32CurrSet, (int)i32AdcValue);
		// int32_t i32CurrKp = (int32_t)(0.5 * 1024);		//10Q10
		// int32_t i32CurrKi = (int32_t)(0.15 * 1024);		//10Q10;
		// int32_t i32CurrKd = (int32_t)(0.03 * 1024);		//10Q10;

		// int32_t i32CurrDev = i32CurrSet - i32AdcValue;								//calc deviation to set current
		
		// i32CurrDevSum+=i32CurrDev;
		// int32_t i32DutyCyle = (i32CurrDev * i32CurrKp) + (i32CurrDevSum * i32CurrKi) + ((i32CurrDev - i32CurrDevLast) * i32CurrKd);				//calc dutycycle 22Q10
		// i32DutyCyle>>=10;															//shift 20Q10 to 32Q0
		// i32DutyCyle += 	(500 * 1024 / i32BattVoltage);								//[0,5V @ 40001/min], (mv/mv * 100% PWM)"vorsteuerung" current control, speed with no load
		// i32CurrDevLast = i32CurrDev;


		ESP_LOGI("pump_runTime","i32DutyCyle: %ld",i32DutyCyle);					//

		//check for overcurrent 
		if (i32DutyCyle > 1000)
		{
			i32DutyCyle = 1000;
			i32OCcount++;
			if (i32OCcount > 30) //> 3s
			{
				err = ESP_ERR_INVALID_STATE;
				pump_stop();
				ESP_LOGE("pump_runTime","Error overcurrent");
				return err;
			}
		}
		else
		{
			i32OCcount = 0;
		}

		//check for negative values
		if (i32DutyCyle < 0)
		{
			i32DutyCyle = 0;
		}

		//switch motor direction
		switch(eDirection)
		{
			case MOTOR_DIR_UP:
				//activate pwm
				err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, i32DutyCyle);//i32DutyCyle
		    	if (err != ESP_OK) 
				{
					ESP_LOGE("pump_runTime","Error setting duty cycle");
					break;
				}
				err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
//				err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, fDutyCylce);
				break;

			case MOTOR_DIR_DOWN:
				//activate pwm
				err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, i32DutyCyle);//i32DutyCyle
		    			    	if (err != ESP_OK) 
				{
					ESP_LOGE("pump_runTime","Error setting duty cycle");
					break;
				}
				err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
//			    err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, fDutyCylce);
				break;

			default:
				break;
		}
		ui32ActTime++;
		counter++;
		vTaskDelay(20 / portTICK_PERIOD_MS);	
	}
	pump_stop();
	ui32AdcTouch_readAdcMux(ADC_MUX_VOLT_3V3, 1);		//read once to disable selector

	ui32HX710_read(&i32PressureMbar);		//read once to enable current sensor

	//save data permanent of speed controller
	int32_t i32SpeedDevSumMean = (s_i32SpeedDevSumLast[0] + s_i32SpeedDevSumLast[1] + s_i32SpeedDevSumLast[2] + s_i32SpeedDevSumLast[3] + s_i32SpeedDevSumLast[4]) / 5;
	uint32_t ui32SpeedDevSumMeanLast = 0;
	ESP_ERROR_CHECK(storage_read("SpeedDevSum", &ui32SpeedDevSumMeanLast));
	int32_t i32SpeedDevSumMeanLast = (int32_t)ui32SpeedDevSumMeanLast;
	int32_t i32SpeedDevSumMeanNext = ((i32SpeedDevSumMean - i32SpeedDevSumMeanLast)>>1) + i32SpeedDevSumMeanLast;
	//only save on deviation > 2000
	if ((abs(i32SpeedDevSumMeanLast - i32SpeedDevSumMeanNext) > 2000) && (i32SpeedDevSumMean > 0))
	{
		ESP_ERROR_CHECK(storage_write("SpeedDevSum", (uint32_t)i32SpeedDevSumMeanNext));
		ESP_LOGI("pump_runTime","saved: %d", (int)i32SpeedDevSumMean);
	}
	ESP_LOGI("pump_runTime","i32SpeedDevSumMeanLast: %d, i32SpeedDevSumMeanNext: %d", (int)i32SpeedDevSumMeanLast, (int)i32SpeedDevSumMeanNext);

	return err;
}

//run pump for defined time without regulation $ui32Time: time in ms, $eDirection: direction of motor, $ui32PumpNb: pump number 1..x,  $return: esp_err_t -> running motor sucessful
esp_err_pump_t pump_runTimeOpenLoop(uint32_t ui32Time, motor_direction_t eDirection, int32_t ui32PumpNb, int32_t i32EmfVoltage)
{
	const int32_t i32BattVoltage = (int32_t)ui32BattVolt_read();					//measrure battery voltage
	int32_t i32DutyCycle = i32EmfVoltage * 1024 / i32BattVoltage ;			//set motor speed in rpm
	if (i32DutyCycle > 1000)
	{
		i32DutyCycle = 1000;
	}

	esp_err_pump_t err = ERR_PUMP_OK;

	if (ui32PumpNb > PUMPCOUNT || ui32PumpNb <= 0)
	{
		ESP_LOGE("Pump: ", "Pump number out of range");
		return ERR_PUMP_CNT;
	}

	s_ui32PumpNb = ui32PumpNb - 1;		//set pump number for isr handler and speed sensing task
	ui32AdcTouch_readAdcMux(ADC_MUX_CUR_PUMP1 + s_ui32PumpNb, 1);		//read once to enable selector

	int64_t i64EndTime = esp_timer_get_time() + (ui32Time * 1000);

	//switch motor direction
	switch(eDirection)
	{
		case MOTOR_DIR_UP:
			//activate pwm
			err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, i32DutyCycle);//i32DutyCyle
			if (err != ESP_OK) 
			{
				ESP_LOGE("pump_runTime","Error setting duty cycle");
				break;
			}
			err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
//				err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, fDutyCylce);
			break;

		case MOTOR_DIR_DOWN:
			//activate pwm
			err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, i32DutyCycle);//i32DutyCyle
							if (err != ESP_OK) 
			{
				ESP_LOGE("pump_runTime","Error setting duty cycle");
				break;
			}
			err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
//			    err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, fDutyCylce);
			break;

		default:
			break;
	}

	while(esp_timer_get_time() < i64EndTime)
	{
		vTaskDelay(20 / portTICK_PERIOD_MS);	
	};

	pump_stop();
	ui32AdcTouch_readAdcMux(ADC_MUX_VOLT_3V3, 1);		//read once to disable selector

	return err;
}

//initialise adc function
static pcnt_unit_handle_t s_pcnt_unit = NULL;				//pulsecount unit handle
static volatile adc_mux_t s_eAdcChannel = ADC_MUX_VOLT_3V3;		//current adc channel
static adc_oneshot_unit_handle_t s_eAdc1Handle = NULL;		//adc1 handle
adc_cali_handle_t s_eCaliHandle = NULL;

void adcTouch_init()
{
	//init sensing and supply enable pin
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT;                                                          //set as output mode
	io_conf.pin_bit_mask = ((1ULL << PIN_SENS_EN));     		  		 //bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE  ;                                           //disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                //disable pull-up mode
	gpio_config(&io_conf);

	//init mux for adc expansion
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT;                                                          //set as output mode
	io_conf.pin_bit_mask = ((1ULL << PIN_ADCMUX1)|(1ULL << PIN_ADCMUX2)|(1ULL << PIN_ADCMUX3));     		  //bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE  ;                                           //disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                //disable pull-up mode
	gpio_config(&io_conf);


	//init charge pin
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	io_conf.mode = GPIO_MODE_INPUT;                                                          //set as output mode
	io_conf.pin_bit_mask = ((1ULL << PIN_CHRG));//|(1ULL << PIN_CUR_SENS_EN)|(1ULL << PIN_3V3_EN));     		  //bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE  ;                                           //disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                //disable pull-up mode
	gpio_config(&io_conf);

	//init adc of PIN_ADC_MUX and calibration
	adc_oneshot_unit_init_cfg_t init_config1 = 
	{
    	.unit_id = ADC_UNIT_1,
    	.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &s_eAdc1Handle));

	adc_oneshot_chan_cfg_t config = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten = ADC_ATTEN_DB_0,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_eAdc1Handle, PIN_ADC_MUX, &config));

	adc_cali_line_fitting_config_t cali_config = {			//calibration config
		.unit_id = ADC_UNIT_1,
		.atten = ADC_ATTEN_DB_0,
		.bitwidth = ADC_BITWIDTH_12,
	};
	ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_config, &s_eCaliHandle));

	//set pins
	gpio_set_level(PIN_SENS_EN, 1);                                                           //enable supply sensing
	gpio_set_level(PIN_ADCMUX1, 0);                                                           //disable MUX 
	gpio_set_level(PIN_ADCMUX2, 0);                                                           //
	gpio_set_level(PIN_ADCMUX3, 0);                                                           //

}


//read adc values from mux $eAdcChannel: enum of adc channel,  $ui32NbMean: quantities of adc measurements for mean value, $output: adc voltage 12Q0[mV]
uint32_t ui32AdcTouch_readAdcMux(adc_mux_t eAdcChannel, uint32_t ui32NbMean)
{
	if (s_eAdcChannel != eAdcChannel)
	{
		if (eAdcChannel > 7) {
			ESP_LOGE("ui32AdcTouch_readAdcMux", "Invalid ADC channel %d", (int)eAdcChannel);
			return 0;
		}
	
		//set mux channel
		gpio_set_level(PIN_ADCMUX1, eAdcChannel&1);        	//select Mux
		gpio_set_level(PIN_ADCMUX2, eAdcChannel&2);
		gpio_set_level(PIN_ADCMUX3, eAdcChannel&4);	

		vTaskDelay(50);                                 //wait for capacitors loaded

		s_eAdcChannel = eAdcChannel;						//save last channel		
	}
	int iAdcRaw = 0;
	int iVoltage = 0;
	ESP_ERROR_CHECK(adc_oneshot_read(s_eAdc1Handle, PIN_ADC_MUX, &iAdcRaw));
	ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_eCaliHandle, iAdcRaw, &iVoltage));

	uint32_t ui32AdcReadSum = (uint32_t)iVoltage;			//adc read sum
	// ESP_LOGI("ADC: ", "ui32AdcReadSum: %d", (int)ui32AdcReadSum);
	if (ui32NbMean > 1)
	{	
		for (uint32_t iCount = 1; iCount < ui32NbMean; iCount++)
		{
			ESP_ERROR_CHECK(adc_oneshot_read(s_eAdc1Handle, PIN_ADC_MUX, &iAdcRaw));
			ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_eCaliHandle, iAdcRaw, &iVoltage));
			ui32AdcReadSum += iVoltage;
		}
		ui32AdcReadSum /= ui32NbMean;		//calc mean value
	}

	return ui32AdcReadSum;
}

//read battery voltage $output: battery voltage [mV]
uint32_t ui32BattVolt_read(void)
{
	//16Q0 = 12Q0 * 16Q12 >> 12 -> 16Q0
	s_ui32BattVoltage =  ((ui32AdcTouch_readAdcMux(ADC_MUX_VOLT_BATT, 4) * s_cui32BattVoltageFact) >> 12);
	ESP_LOGI("ADC","ui32BattVolt_read: %d", (int)s_ui32BattVoltage);
	return s_ui32BattVoltage;
}

//read solar voltage $output: solar voltage [mV]
uint32_t ui32SolarVolt_read(void)
{
	//16Q0 = 12Q0 * 16Q12 >> 12 -> 16Q0
	uint32_t ui32SolarVoltage =  ((ui32AdcTouch_readAdcMux(ADC_MUX_VOLT_SOLAR, 4) * s_cui32SolarVoltageFact) >> 12);
	ESP_LOGI("ADC","ui32SolarVoltage: %d", (int)ui32SolarVoltage);
	return ui32SolarVoltage;
}

//read esp supply voltage $output: esp voltage [mV]
uint32_t ui32EspVolt_read(void)
{
	//16Q0 = 12Q0 * 16Q12 >> 12 -> 16Q0
	s_ui32SupplyVoltage = ((ui32AdcTouch_readAdcMux(ADC_MUX_VOLT_3V3, 4) * s_cui32LogicVoltageFact) >> 12);
	  return s_ui32SupplyVoltage;
}

//read temperature $output: temperature [�C]
float fTemp_read(void)
{
	if (s_ui32SupplyVoltage == 0)						//supply voltage not read
	{
		ui32EspVolt_read();								//read supply voltage
	}

	//interpolate temperatures
//	float fTemperatureFact = ((((float)iTempReadSum) * s_sAdcCalibration.adcValueFactor) + s_sAdcCalibration.adcValueOffset) / s_ui32SupplyVoltage;
	//(14Q2 * 15Q16 -> 29Q18 >> 12 -> 17Q6) + 14Q6 -> 17Q6)
	ESP_LOGI("TEMP","Raw Temp Value: %d", (int)ui32AdcTouch_readAdcMux(ADC_MUX_TEMP_PCB, 4));
	float fTemperatureFact = (float)(ui32AdcTouch_readAdcMux(ADC_MUX_TEMP_PCB, 4) / (float)s_ui32SupplyVoltage);
	for (int iTempCount = 0; iTempCount < 14; iTempCount ++)
	  {
		if (g_cafNTCFactValues[iTempCount] >  fTemperatureFact)
		{
		  return g_cafNTCTempValues[iTempCount - 1] + (5.0/(g_cafNTCFactValues[iTempCount] - g_cafNTCFactValues[iTempCount - 1])*(fTemperatureFact - g_cafNTCFactValues[iTempCount - 1]));
		}
	  }

	return 0;
}

uint32_t ui32Charge_read(void)
{
	uint32_t ui32ChargeValue = gpio_get_level(PIN_CHRG);				//read charge pin
	ESP_LOGI("CHRG","ui32Charge_read: %d", (int)ui32ChargeValue);
	return ui32ChargeValue;			
}

// //read water level in ml $output: water level [ml]
// uint32_t ui32Level_readMl(void)
// {
// 	// int32_t i32LevelFreq = (int32_t)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100);				//read water level sensor
// 	// i32LevelFreq += (int32_t)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100);				//read water level sensor
// 	// i32LevelFreq += (int32_t)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100);				//read water level sensor
// 	// i32LevelFreq += (int32_t)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100);				//read water level sensor
// 	// i32LevelFreq = i32LevelFreq >> 2;		//calc mean value

// 	int32_t i32LevelMl = 0;		

// 	//0ml -> 6000Hz, 600ml -> 5740Hz, 1750ml -> 4820Hz
// 	const int32_t c_i32LevelCurveFreq[] = {5980, 5740, 4820};
// 	const int32_t c_i32LevelCurveMl[] = {0, 600, 1750};

// 	if (i32LevelFreq > c_i32LevelCurveFreq[1])
// 	{
// 		i32LevelMl = (c_i32LevelCurveFreq[0] - i32LevelFreq) * (c_i32LevelCurveMl[1] - c_i32LevelCurveMl[0]) / (c_i32LevelCurveFreq[0] - c_i32LevelCurveFreq[1]);
// 	}
// 	else
// 	{
// 		i32LevelMl = (c_i32LevelCurveFreq[1] - i32LevelFreq) * (c_i32LevelCurveMl[2] - c_i32LevelCurveMl[1]) / (c_i32LevelCurveFreq[1] - c_i32LevelCurveFreq[2]);
// 		i32LevelMl += c_i32LevelCurveMl[0];
// 	}
// 	ESP_LOGI("ui32Level_readMl","i32LevelFreq: %d, i32LevelMl: %d", (int)i32LevelFreq, (int)i32LevelMl);

// 	if (i32LevelMl < 0) i32LevelMl = 0;

// 	return (uint32_t)i32LevelMl;			
// }

//read water level in ml $output: water level [%]
esp_err_t erLevel_readPerc(int32_t *value)
{
	*value = 0;
	return ESP_OK;

	if (value == NULL)
	{
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t err = ESP_OK;

	int32_t i32TankHight = 20;		//tank hight in cm
	int32_t i32TankPressure = 0;	//pressure in mbar
	uint32_t ui32LevelPerc = 0;

	//close all valves 
	selector_setPos(0);

	//open valve 6 
	selector_setPos(6);

	//read water level sensor
	err = ui32HX710_read(&i32TankPressure);		//read once to enable sensor

	//pump air for 2s 
	pump_runTime(2000, MOTOR_DIR_UP, 1);

	//read water level sensor
	err = ui32HX710_read(&i32TankPressure);		//read once to enable sensor

	if (err != ESP_OK)
	{
		ESP_LOGE("ui32Level_readPerc","Error reading pressure");
	}
	else if (i32TankPressure <= 0)
	{
		err = ESP_ERR_INVALID_STATE;
		ESP_LOGE("ui32Level_readPerc", "Pressure is zero, cannot calculate level");
	}
	else
	{
		*value = i32TankPressure * 100 / i32TankHight;
		ESP_LOGI("ui32Level_readPerc","ui32LevelPerc: %d", (int)*value);
	}
	
	selector_setPos(0);

	return err;
}

#ifdef HX710_ENABLE
void HX710_init(void)
{
	static bool s_bInitDone = false;
	if (s_bInitDone) return;
	s_bInitDone = true;

    gpio_config_t io_conf = {};

    // DOUT als Input konfigurieren
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_HX710_OUT);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // SCK als Output konfigurieren
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1ULL << PIN_HX710_SCK) | (1ULL << PIN_HX710_EN));
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // SCK initial LOW
    gpio_set_level(PIN_HX710_SCK, 0);
	gpio_set_level(PIN_HX710_EN, 1);
}

esp_err_t ui32HX710_read(int32_t *value)
{
	HX710_init();

	// wait for DOUT = LOW  (Data ready), Timeout ~200 ms
    uint32_t timeout = 300000; // in microseconds
	// esp_timer_get_time();
	gpio_set_level(PIN_HX710_EN, 1);		//enable sensor
	vTaskDelay(50 / portTICK_PERIOD_MS);	//wait for sensor ready
	
    while (gpio_get_level(PIN_HX710_OUT) == 1) {
        ets_delay_us(1);
        if (--timeout == 0) {
			gpio_set_level(PIN_HX710_EN, 0);
			ESP_LOGE("HX710A", "Timeout waiting for pressure");
            return ESP_ERR_TIMEOUT; // Timeout
        }
    }

    uint32_t raw = 0;

    // 24 Datenbits einlesen (MSB zuerst)
    for (int i = 0; i < 24; i++) {
        gpio_set_level(PIN_HX710_SCK, 1);
        ets_delay_us(1);
        raw = (raw << 1) | gpio_get_level(PIN_HX710_OUT);
        gpio_set_level(PIN_HX710_SCK, 0);
        ets_delay_us(1);
    }

    // // Pulse 25 + 26: Kanalauswahl → nächste Konversion = Temperatur
    // for (int i = 0; i < 2; i++) {
    //     gpio_set_level(PIN_HX710_SCK, 1);
    //     ets_delay_us(1);
    //     gpio_set_level(PIN_HX710_SCK, 0);
    //     ets_delay_us(1);
    // }

    // 24-Bit Two's Complement → int32_t (Vorzeichenerweiterung)
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }
	int32_t i32PressureRaw  = (int32_t)raw;

	// Theoretical scaling for XGZP167 40kPa @ 3.3V with HX710A gain 128.
	// Sensor FS at 3.3V: 65mV * 3.3/5 = 42.9mV (at 400mbar).
	// Usable range before ADC clipping is about 120mbar -> ~8388607 / 120 LSB/mbar.
	static const float c_fLsbPerMbar = 69905.06f;
	static bool s_bPressureOffsetSet = false;
	static int32_t s_i32PressureOffset = 0;

	if (!s_bPressureOffsetSet) {
		s_i32PressureOffset = i32PressureRaw;
		s_bPressureOffsetSet = true;
		ESP_LOGI("HX710A", "Pressure tare set: ADC_offset=%d", (int)s_i32PressureOffset);
	}

	float fPressureMbar = ((float)(i32PressureRaw - s_i32PressureOffset)) / c_fLsbPerMbar;
	if (fPressureMbar > 400.0f) fPressureMbar = 400.0f;
	int32_t i32PressureMbar = -(int32_t)(fPressureMbar + 0.5f);

	if (i32PressureRaw >= 8380000) {
		ESP_LOGW("HX710A", "Pressure ADC near saturation (raw=%d). Gain may be too high for full 400mbar range.", (int)i32PressureRaw);
	}

    // // --- Temperaturkanal lesen ---
    // timeout = 300000;
    // while (gpio_get_level(PIN_HX710_OUT) == 1) {
    //     ets_delay_us(1);
    //     if (--timeout == 0) {
    //         gpio_set_level(PIN_HX710_EN, 0);
    //         ESP_LOGW("HX710A", "Timeout waiting for temperature");
    //         *value = i32PressureMbar;
    //         return ESP_OK; // Druckwert trotzdem zurückgeben
    //     }
    // }

    // uint32_t rawTemp = 0;
    // for (int i = 0; i < 24; i++) {
    //     gpio_set_level(PIN_HX710_SCK, 1);
    //     ets_delay_us(1);
    //     rawTemp = (rawTemp << 1) | gpio_get_level(PIN_HX710_OUT);
    //     gpio_set_level(PIN_HX710_SCK, 0);
    //     ets_delay_us(1);
    // }

	// // Puls 25: Kanalauswahl → nächste Konversion = Druck
    // gpio_set_level(PIN_HX710_SCK, 1);
    // ets_delay_us(1);
    // gpio_set_level(PIN_HX710_SCK, 0);
    // ets_delay_us(1);


    // if (rawTemp & 0x800000) {
    //     rawTemp |= 0xFF000000;
    // // }
	// int32_t i32TempRaw = (int32_t)rawTemp;
	// // HX710A datasheet: T[degC] ~= (ADCraw - 1100000) / 20.4
	// float fTempC = ((float)i32TempRaw - 1100000.0f) / 20.4f;

	ESP_LOGI("HX710A", "Druck Raw: %d  Druck: %.1f mbar ", (int)i32PressureRaw, -fPressureMbar);
    *value = i32PressureMbar;
	gpio_set_level(PIN_HX710_EN, 0);
    return ESP_OK;
}
#endif

