/*
 * periphery.c
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 */

#include "periphery.h"
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

static sel_prop_t s_saSelProperties = {.ui32Angle = {0, 0, 0, 0}};		//angle values for selector positions (16Q16) [°*65536]
static volatile motor_direction_t s_motorState;
static volatile int s_eMotorRegulationSM = MOTOR_REG_IDL;

static volatile int32_t s_i32CurSensOffsetVoltage;										//voltage offset of current sensor 16Q4
// static const int32_t s_ci32CurSensFact =  (int32_t)(8192 * 2.13);						//ADC factor 4.7/0.1R /4.7 15Q13 [mA/mV]
//static const float s_cfMotorResistance = 1;													//Motorresistance 1R


static QueueHandle_t speed_sens_evt_queue = NULL;						//queue for Speed sensing events
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
	else{
		s_aeLedState[ui32LEDNumber - 1] = eLedStatus;
		switch(eLedStatus)
		{
			case LED_OFF:
				gpio_set_level(PIN_LEDS[ui32LEDNumber - 1], 0);
				break;
			case LED_ON:
				gpio_set_level(PIN_LEDS[ui32LEDNumber - 1], 1);
				break;
			default:
				ESP_LOGE("LED", "Invalid LED status");
				return;
		}
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

//check humidity state $return: bool , true->water in pot, false -> no water in pot $ui32Channel - channel number (1...3)
bool bHumidity_check(uint32_t ui32Channel)
{
	const uint32_t ui32FrequencyLimit = 5000;		//limit for deivce 1
	// const uint32_t ui32FrequencyLimit = 2950;		//limit for deivce 2

	if ((ui32Channel > 3) || (ui32Channel < 1)) {
		ESP_LOGE("bHumidity_check", "Channel out of range");
		return false;
	}

	if (ui32AdcTouch_readPwmMux(ui32Channel + 8, 100) > ui32FrequencyLimit)		//no/less water in pot 
	// if ((FDC_getCap(ui32Channel)/5243) > ui32FrequencyLimit)		//no/less water in pot 
	{
		return false;
	}
	else
	{
		return true;									//water in pot
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

	//init driver enable
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    			//disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT;                                                          			//set as output mode
	io_conf.pin_bit_mask = ((1ULL << PIN_SEL_EN)|(1ULL << PIN_PUMP_EN));   	  	//bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE  ;                                           			//disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                			//disable pull-up mode
	gpio_config(&io_conf);
	gpio_set_level(PIN_SEL_EN, 0);
	gpio_set_level(PIN_PUMP_EN, 0);


	//init speed measurement (Hall sensor)
	speed_sens_evt_queue = xQueueCreate(1, sizeof(uint32_t));
	// xTaskCreate(speed_sens_task, "speed_sens_task", 2048, NULL, 10, NULL);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(PIN_PUMP_SPEED, speed_sens_isr_handler, (void*) PIN_PUMP_SPEED);

	io_conf.intr_type = GPIO_INTR_POSEDGE;//GPIO_INTR_POSEDGE;                                                    //interrupt pos edge
	io_conf.mode = GPIO_MODE_INPUT;                                                          	//set as output mode
	io_conf.pin_bit_mask = (1ULL << PIN_PUMP_SPEED);     											//bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                           	//disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                                //disable pull-up mode
	gpio_config(&io_conf);
	
	//  //init position input pin
	//  io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	//  io_conf.mode = GPIO_MODE_INPUT;                                                          	//set as output mode
	//  io_conf.pin_bit_mask = ((1U << PIN_SEL_POS));     											//bit mask of the pins
	//  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                           	//disable pull-down mode
	//  io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                //disable pull-up mode
	//  gpio_config(&io_conf);

	// //init current sens adc
	// adc1_config_channel_atten(PIN_CURPUMP_SENS, ADC_ATTEN_DB_0);
	// adc1_config_channel_atten(PIN_CURSEL_SENS, ADC_ATTEN_DB_0);

	//init ledc timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,		//duty resolution 1023bit 
        .freq_hz          = 1000,  					// Set output frequency at 500 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

	//init channel1 - Selector 1
	ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_HS,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));	
	ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_0, 0));

	//init channel2 - Selector B
	ledc_channel_config_t ledc_channel2 = ledc_channel1;
	ledc_channel2.gpio_num = PIN_LS;
	ledc_channel2.channel = LEDC_CHANNEL_1;
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));	
	ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_1, 0));

	// //init channel3 - Pump A
	// ledc_channel_config_t ledc_channel3 = ledc_channel1;
	// ledc_channel3.gpio_num = PIN_PUMP_A;	
	// ledc_channel3.channel = LEDC_CHANNEL_2;
	// ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel3));	
	// ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_2, 0));

	// //init channel4 - Pump B
	// ledc_channel_config_t ledc_channel4 = ledc_channel1;
	// ledc_channel4.gpio_num = PIN_PUMP_B;
	// ledc_channel4.channel = LEDC_CHANNEL_3;
	// ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel4));	
	// ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_3, 0));

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
	//check gpio level is "1"
	if (gpio_get_level(PIN_PUMP_SPEED) == 1)
	{
		// ESP_LOGI("Speed: ", "OK");
		volatile static int64_t i64PulseStart = 0;
		volatile static int64_t i64PulseEnd = 0;
		volatile static bool pulseStarted = false;	

		uint32_t gpio_num = (uint32_t) arg;
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


// static void motor_stop(void)
// {
	
//   mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);    //set to idle - nmos 1 low
//   mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);   //set to idle - pmos 1 high
// //  mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);    //set to idle - nmos 2 low
// //  mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);   //set to idle - pmos 2 high
//   //timer_pause(TIMER_GROUP_0, TIMER_0);    //stop timer0
// }

// //motor regulation
// void motorRegulation(void)
// {
// 	//16Q4 = ((12Q0 * 15Q16 -> 27Q16 >> 10 -> 17Q6) + 14Q6 -> 17Q6) >> 2 -> 15Q4
// 	int32_t i32AdcReadVoltage = (int32_t)(((adc1_get_raw(PIN_CURPUMP_SENS) * s_sAdcCalibration.adcValueFactor) >> 10) + s_sAdcCalibration.adcValueOffset) >> 2;
// 	//14Q4 * 15Q13 = 29Q17 >> 13 -> 16Q4 [mA]
// 	s_motorRegulation.currentAct = ((i32AdcReadVoltage - s_i32CurSensOffsetVoltage) * s_ci32CurSensFact) >> 13;
// 	//16Q4 * 14Q10 = 30Q14 >> 10 = 20Q4
// 	int32_t i32TorqueVoltage = (s_motorRegulation.currentAct * s_motorRegulation.motorResitance) >> 10;
// 	if (i32TorqueVoltage > 65535)i32TorqueVoltage = 65535;																//limit to 16Q4
// 	int32_t i32EMFVoltage = s_motorRegulation.voltageSet - i32TorqueVoltage;			// 16Q4 - 16Q4 = 16Q4
// 	int32_t i32deltaV = s_motorRegulation.voltageEmfSet - i32EMFVoltage;				//16Q4
// 	s_motorRegulation.voltageRegISum = s_motorRegulation.voltageRegISum + (i32deltaV * s_motorRegulation.voltageRegKi);		//28Q16 = 16Q4 * 12Q12
// 	if(s_motorRegulation.voltageRegISum > (3000 * 65535)) s_motorRegulation.voltageRegISum = (3000 * 65535);				//limit i part to 3000mV
// 	// 16Q4 * 12Q12 = 28Q16
// 	int32_t i32VoltageSet = i32deltaV * s_motorRegulation.voltageRegKp + s_motorRegulation.voltageRegISum;		// 16Q4 * 12Q12 = 28Q16 + 28Q16
// 	if (i32VoltageSet < 0)
// 	{
// 		s_motorRegulation.voltageSet = 0;
// 	}
// 	else
// 	{
// 		s_motorRegulation.voltageSet = i32VoltageSet >> 12;			//28Q16 >> 12 -> 16Q4
// 	}
// 	s_motorRegulation.pwmSet  =  ((float)(i32VoltageSet / s_ui32BattVoltage)) * (1.0/655.35);						//28Q16 / 14Q0 -> 28Q16 * (Q16->65535 * 100) =
// }

// ////run motor $iRunTime: run time in ms $iBackEMF: equivalent ot speed $eDirection: direction of motor $output: bool -> running motor sucessful
// //bool motor_run(uint32_t iRunTime, uint32_t iBackEMF, motor_direction_t eDirection)
// //{
// //  //proof motor not running
// //  if (s_motorState == MOTOR_DIR_IDLE)f
// //  {
// ////    s_motorRegulation.voltageSetValue = iBackEMF;           //set value
// //
// ////    s_motorRegulation.pwmSetValue = 20.0;                   //set start PWM
// //    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 20.0);
// //    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 20.0);
// //    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 20.0);
// //    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 20.0);
// //
// //    s_eMotorRegulationSM = MOTOR_REG_FOR_READCUR;           //set regulation state
// //    s_motorState = eDirection;            		 			//set motor state
// //
// //    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);     //reset timer value
// //    timer_start(TIMER_GROUP_0, TIMER_0);                    //start timer for speed regulation
// //
// //    //read current sensor offset missing
// //
// //    switch(eDirection)
// //    {
// //    	case MOTOR_DIR_FORWARD:
// //		{
// //			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);                     	//nmos 2 inactive
// //			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);                     	//pmos 2 active
// //			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);   	//nmos 1 switching
// //			mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);   	//pmos 1 switching
// //		}
// //		break;
// //
// //    	case MOTOR_DIR_BACKWARD:
// //		{
// //		    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);                     	//nmos 1 inactive
// //		    mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);                     	//pmos 1 active
// //		    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);   	//start pwm signal
// //		    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);   	//start pwm signal
// //		}
// //		break;
// //
// //    	default:
// //		{
// //			return false;
// //		}
// //    	break;
// //    }
// //
// //    vTaskDelay(iRunTime);									//wait motor running time
// ////    ets_delay_us(iRunTime);
// //    s_eMotorRegulationSM = MOTOR_REG_IDL;                 	//set regualtion state to idle
// //    s_motorState = MOTOR_DIR_IDLE;              			//set motor state idle
// //    motor_stop();											//set powerstage pins idle/disable timer0
// //
// //  }
// //  return true;
// //}

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
			err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ui32Speedbool);
			// err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, fSpeedbool);
		    if (err != ESP_OK) return err;
			err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
		    // err = mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);   	//nmos 1 switching
		    if (err != ESP_OK) return err;
		    break;

		case MOTOR_DIR_DOWN:
			//activate pwm
			err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, ui32Speedbool);
		    // err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, fSpeedbool);
		    if (err != ESP_OK) return err;
			err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
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

//run pump motor $ui32Speedbool: dutycycle(10Q10) $eDirection: direction of motor,  $return: esp_err_t -> running motor sucessful
esp_err_t pump_run(uint32_t ui32Speedbool, motor_direction_t eDirection)
{
	esp_err_t err = ESP_OK;

	switch(eDirection)
	{
		case MOTOR_DIR_UP:
			//activate pwm
			ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, ui32Speedbool));
			ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0));
		    break;

		case MOTOR_DIR_DOWN:
			//activate pwm
			ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, ui32Speedbool));
			ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1));
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
	ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_0, 1));
	ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_1, 1));
	vTaskDelay(1000);

	//switch motor off
	ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_0, 0));
	ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_1, 0));

	return err;
}


//stop pump motor  $return: esp_err_t -> running motor sucessful
esp_err_t pump_stop(void)
{
	esp_err_t err = ESP_OK;

	//switch motor off
	ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_0, 0));
	ESP_ERROR_CHECK(ledc_stop(LEDC_HIGH_SPEED_MODE , LEDC_CHANNEL_1, 0));

	return err;
}


//positionate slector $i32SetAngleRaw: set angle of selector, $eSelNb: number of selector, $bInitTLV: TLV initialized before? $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_setAngle(int32_t i32SetAngleRaw, bool bInitTLV)
{
	if (i32SetAngleRaw > 359)
	{
		ESP_LOGE("Selector: ", "Error angle > 360°");
		return ERR_SEL_POS_OUTOFRANGE;
	}

	//enable driver/magnet sensor
	if (!bInitTLV)
	{	
		i2c_deinit();
		ESP_ERROR_CHECK(gpio_set_level(PIN_SEL_EN, 1));        		//enable driver
		vTaskDelay(100);		//wait for capacitors loaded
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
		gpio_set_level(PIN_SEL_EN, 0);        		//disable driver
	}														//deinit TLV sensor

	return err;
}


//positionate slector $i32SetPos: set pos of selector, $eSelNb: number of selector $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_setPos(int32_t i32SetPos)
{
	if (i32SetPos < 0 || i32SetPos > 3)
	{
		ESP_LOGE("selector_setPos: ", "Error position out of range");
		return ERR_SEL_POS_OUTOFRANGE;
	}

	// const uint32_t caui32AnglePosSel[4] = {250, 340, 160, 80};		//angle pos for selector - device 1
	// const uint32_t caui32AnglePosSel[4] = {40, 120, 300, 250};		//angle pos for selector - device 2

	return selector_setAngle(s_saSelProperties.ui32Angle[i32SetPos], false);	//set angle for selector
}


//calibrate slectors position $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_caliPos(void)
{
	ESP_LOGI("Selector: ", "selector_caliPos()");
	uint32_t ui32TimeCount = 0;

	//enable driver/magnet sensor
	gpio_set_level(PIN_SEL_EN, 1);        		//enable driver
	vTaskDelay(100);

	TLV_init();		//init TLV sensor
	int32_t i32LastAngle = 0;
	uint32_t ui32StuckCount = 0;

	//position points dir down
	for (int i = 0; i < 36; i++)
	{
		selector_setAngle(i * 10, true);	//set angle +10° for next position

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

		ESP_LOGI("Sel", "Pos: %ld Frequency: %ld", i32Angle*10, (uint32_t)ui32AdcTouch_readPwmMux(PWM_MUX_SEL, 100));
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

//
esp_err_pump_t pump_runTime(uint32_t ui32Time, motor_direction_t eDirection)
{
	const int32_t c_i32MotorSpeedSet = 2500;	//set motor speed in rpm

	esp_err_pump_t err = ERR_PUMP_OK;
	int32_t i32BattVoltage = (int32_t)ui32BattVolt_read();					//measrure battery voltage

	gpio_set_level(PIN_PUMP_EN, 1);        		//enable driver
	vTaskDelay(100);                            //wait for capacitors loaded
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

	int32_t i32AdcReadVoltageOffset = (int32_t)ui32AdcTouch_readAdcMux(ADC_MUX_CUR_PUMP, 2);
	ESP_LOGI("pump_runTime","CurrVoltAct: %d", (int)ui32AdcTouch_readAdcMux(ADC_MUX_CUR_PUMP, 2));

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
			int32_t i32SpeedKp = (int32_t)(0.1 * 1024);		//10Q10
			int32_t i32SpeedKi = (int32_t)(0.02 * 1024);		//10Q10;

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
				err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, i32DutyCyle);//i32DutyCyle
		    	if (err != ESP_OK) break;
				err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//				err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, fDutyCylce);
				break;

			case MOTOR_DIR_DOWN:
				//activate pwm
				err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, i32DutyCyle);//i32DutyCyle
		    	if (err != ESP_OK) break;
				err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
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
	gpio_set_level(PIN_PUMP_EN, 0);        		//enable driver

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


//initialise adc function
static pcnt_unit_handle_t s_pcnt_unit = NULL;				//pulsecount unit handle
static volatile adc_mux_t s_eAdcChannel = ADC_MUX_IDLE;		//current adc channel
static volatile bool s_bAdcMuxIdle = true;							//adc mux idle flag
static adc_oneshot_unit_handle_t s_eAdc1Handle = NULL;		//adc1 handle
adc_cali_handle_t s_eCaliHandle = NULL;

void adcTouch_init()
{
	//init sensing and supply enable pin
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT;                                                          //set as output mode
	io_conf.pin_bit_mask = ((1ULL << PIN_ADC_MUX_EN)|(1ULL << PIN_PWM_MUX_EN)|(1ULL << PIN_SENS_EN));     		  		 //bit mask of the pins
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

	//init PIN_PWM_MUX
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	io_conf.mode = GPIO_MODE_INPUT;                                                          //set as output mode
	io_conf.pin_bit_mask = ((1ULL << PIN_PWM_MUX));    		  									//bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE  ;                                           //disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                //disable pull-up mode
	gpio_config(&io_conf);

	// //init PIN_ADC_MUX
	// adc1_config_width(ADC_WIDTH_BIT_12);
	// adc1_config_channel_atten(PIN_ADC_MUX, ADC_ATTEN_DB_0);

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
	gpio_set_level(PIN_ADC_MUX_EN, 0);                                                            //disable adc mux 
	gpio_set_level(PIN_PWM_MUX_EN, 0);                                                            //disable pwm mux
	gpio_set_level(PIN_SENS_EN, 1);                                                           //enable supply sensing
	gpio_set_level(PIN_ADCMUX1, 0);                                                           //disable MUX 
	gpio_set_level(PIN_ADCMUX2, 0);                                                           //
	gpio_set_level(PIN_ADCMUX3, 0);                                                           //

	//init pulse counter for touch sensing
	pcnt_unit_config_t unit_config = {
		.high_limit = 30000,      
		.low_limit = -30000,
		// .flags.accum_count = true, // enable counter accumulation
	};
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &s_pcnt_unit));
	pcnt_chan_config_t chan_config = {
		.edge_gpio_num = PIN_PWM_MUX,
		.level_gpio_num = -1, // Not used
	};
	pcnt_channel_handle_t pcnt_chan = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &chan_config, &pcnt_chan));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
	ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit));
}


//read adc values from mux $eAdcChannel: enum of adc channel,  $ui32NbMean: quantities of adc measurements for mean value, $output: adc voltage 12Q0[mV]
uint32_t ui32AdcTouch_readAdcMux(adc_mux_t eAdcChannel, uint32_t ui32NbMean)
{
	//check if module is running
	if (!s_bAdcMuxIdle)
	{
		ESP_LOGE("ui32Adc_readMux", "ADC Mux busy");
		return 0;
	}
	s_bAdcMuxIdle = false;

	if (s_eAdcChannel != eAdcChannel)
	{
		if (eAdcChannel > 7) {
			ESP_LOGE("ui32AdcTouch_readAdcMux", "Invalid ADC channel %d", (int)eAdcChannel);
			return 0;
		}

		gpio_set_level(PIN_PWM_MUX_EN, 0);                                                            		//disable PWM sensing
		gpio_set_level(PIN_ADC_MUX_EN, 1);                                                            		//enable ADC sensing
		
		//wait for previous adc mux to disable
		vTaskDelay(10);	

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
	// ESP_LOGI("ADC: ", "ui32AdcReadSum: %d", (int)ui32AdcReadSum);
	s_bAdcMuxIdle = true;			//release idle flag	
	return ui32AdcReadSum;
}

//count touch pwm pulses $return: uint32_t - frequency of pulses [Hz]
uint32_t ui32AdcTouch_readPwmMux(adc_mux_t eAdcChannel, uint32_t ui32TimeMs)
{
	//check if module is running
	if (!s_bAdcMuxIdle)
	{
		ESP_LOGE("ui32AdcTouch_readPwmMux", "ADC Mux busy");
		return 0;
	}
	s_bAdcMuxIdle = false;

	if (eAdcChannel < 8) {
		ESP_LOGE("ui32AdcTouch_readPwmMux", "Invalid ADC channel %d", (int)eAdcChannel);
		return 0;
	}

	if (s_pcnt_unit == NULL) {
		ESP_LOGE("Humidity", "PCNT unit not initialized");
		return 0;
	}

	s_eAdcChannel = eAdcChannel;

	//set mux channel
	gpio_set_level(PIN_PWM_MUX_EN, 1);                                                            		//enable PWM sensing
	gpio_set_level(PIN_ADC_MUX_EN, 0);                                                            		//disable ADC sensing

	//set mux channel
	gpio_set_level(PIN_ADCMUX1, eAdcChannel&1);        	//select Mux
	gpio_set_level(PIN_ADCMUX2, eAdcChannel&2);
	gpio_set_level(PIN_ADCMUX3, eAdcChannel&4);	

	vTaskDelay(50);                                 //wait for capacitors loaded

	// Clear the count and start the unit
	ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit));
	ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_unit));
	int64_t i64StartTime = esp_timer_get_time(); 

	vTaskDelay(ui32TimeMs / portTICK_PERIOD_MS); // wait xxxms

	ESP_ERROR_CHECK(pcnt_unit_stop(s_pcnt_unit));
	int64_t i64EndTime = esp_timer_get_time();
	int32_t i32Duration = (int32_t)(i64EndTime - i64StartTime);		//[us]

	int iPulseCount = 0;
	ESP_ERROR_CHECK(pcnt_unit_get_count(s_pcnt_unit, &iPulseCount));

	int32_t i32Frequency = (int32_t)((int64_t)iPulseCount * 1000000 / (int64_t)i32Duration); 			// Calculate frequency in Hz
	// ESP_LOGI("Humidity", "Frequency %d: %d", (int)ui32Channel, (int)i32Frequency);

	// ESP_LOGI("Humidity", "Pulse Count: %d, Frequency: %d, Duration: %d", (int)iPulseCount, (int)i32Frequency, (int)i32Duration);

	s_eAdcChannel = ADC_MUX_IDLE;
	s_bAdcMuxIdle = true;			//release idle flag	
	return (uint32_t)i32Frequency; // Convert to Hz
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

//read water level in ml $output: water level [ml]
uint32_t ui32Level_readMl(void)
{
	int32_t i32LevelFreq = (int32_t)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100);				//read water level sensor
	i32LevelFreq += (int32_t)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100);				//read water level sensor
	i32LevelFreq += (int32_t)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100);				//read water level sensor
	i32LevelFreq += (int32_t)ui32AdcTouch_readPwmMux(PWM_MUX_TANKLVL, 100);				//read water level sensor
	i32LevelFreq = i32LevelFreq >> 2;		//calc mean value

	int32_t i32LevelMl = 0;		

	//0ml -> 6000Hz, 600ml -> 5740Hz, 1750ml -> 4820Hz
	const int32_t c_i32LevelCurveFreq[] = {5980, 5740, 4820};
	const int32_t c_i32LevelCurveMl[] = {0, 600, 1750};

	if (i32LevelFreq > c_i32LevelCurveFreq[1])
	{
		i32LevelMl = (c_i32LevelCurveFreq[0] - i32LevelFreq) * (c_i32LevelCurveMl[1] - c_i32LevelCurveMl[0]) / (c_i32LevelCurveFreq[0] - c_i32LevelCurveFreq[1]);
	}
	else
	{
		i32LevelMl = (c_i32LevelCurveFreq[1] - i32LevelFreq) * (c_i32LevelCurveMl[2] - c_i32LevelCurveMl[1]) / (c_i32LevelCurveFreq[1] - c_i32LevelCurveFreq[2]);
		i32LevelMl += c_i32LevelCurveMl[0];
	}
	ESP_LOGI("ui32Level_readMl","i32LevelFreq: %d, i32LevelMl: %d", (int)i32LevelFreq, (int)i32LevelMl);

	if (i32LevelMl < 0) i32LevelMl = 0;

	return (uint32_t)i32LevelMl;			
}

//read water level in ml $output: water level [%]
uint32_t ui32Level_readPerc(void)
{
	uint32_t ui32WaterLevelMl = ui32Level_readMl();
	uint32_t ui32LevelPerc = 0;

	//200ml -> 0%, 1600ml -> 100%

	const uint32_t c_ui32LevelMinMl = 200;
	const uint32_t c_ui32LevelMaxMl = 1600;

	if (ui32WaterLevelMl > 1600)
	{
		ui32LevelPerc = 100;
	}
	else if(ui32WaterLevelMl < 200)
	{
		ui32LevelPerc = 0;
	}
	else
	{
		ui32LevelPerc = (ui32WaterLevelMl - c_ui32LevelMinMl) *100 / (c_ui32LevelMaxMl - c_ui32LevelMinMl);
	}

	ESP_LOGI("ui32Level_readPerc","ui32LevelPerc: %d", (int)ui32LevelPerc);

	return ui32LevelPerc;
}

#ifdef PIN_HX710_OUT && PIN_HX710_SCK
void HX710_init()
{
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
    io_conf.pin_bit_mask = (1ULL << PIN_HX710_SCK);
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // SCK initial LOW
    gpio_set_level(PIN_HX710_SCK, 0);
}

esp_err_t ui32HX710_read(int32_t *value)
{
	// wait for DOUT = LOW  (Data ready), Timeout ~200 ms
    uint32_t timeout = 200000; // in microseconds
	// esp_timer_get_time();
    while (gpio_get_level(PIN_HX710_OUT) == 1) {
        ets_delay_us(1);
        if (--timeout == 0) {
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

    // // Zusätzliche Pulse für Gain/Kanal-Auswahl (25 oder 26)
    // for (int i = 24; i < pulses; i++) {
    //     gpio_set_level(PIN_HX710_SCK,1);
    //     ets_delay_us(1);
    //     gpio_set_level(PIN_HX710_SCK, 0);
    //     ets_delay_us(1);
    // }

    // 24-Bit Two's Complement → int32_t (Vorzeichenerweiterung)
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }

	ESP_LOGI("HX710", "Raw ADC Value: %d", (int)raw);
    *value = (int32_t)raw;
    return ESP_OK;
}
#endif