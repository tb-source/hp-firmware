/*
 * periphery.c
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 */

#include "periphery.h"
#include "rom/ets_sys.h"		//for us delay function - delete further


static adc_calibration_t s_sAdcCalibration = {0, 0, 5440, 15363};												//{12Q0, 12Q0, 14Q6, 15Q16}

static const uint32_t s_cui32LogicVoltageFact = (uint32_t)((100.0+33.0)/33.0 * 4096);          					//voltage divider (Rpu(100kR)+Rpd(33kR))/Rpd(33kR)) (15Q12)
static const uint32_t s_cui32BattVoltageFact = (uint32_t)((33.0+10.0)/10.0 * 4096);          					//voltage divider (Rpu(33kR)+Rpd(10kR))/Rpd(10kR)) (15Q12)
static const uint32_t s_cui32SolarVoltageFact = (uint32_t)((200.0+33.0)/33.0 * 4096);          					//voltage divider (Rpu(200kR)+Rpd(33kR))/Rpd(33kR)) (15Q12)


static uint32_t s_ui32SupplyVoltage = 0;																		//logic supply voltage 14Q0 [mV]
static uint32_t s_ui32BattVoltage = 0;																			//battery supply voltage 14Q0 [mV]

const float g_cafNTCTempValues[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};
const float g_cafNTCFactValues[] = {0.0264, 0.0346, 0.0449, 0.0575, 0.0727, 0.0909, 0.1123, 0.1371, 0.1652, 0.1968, 0.2315, 0.2691, 0.3090};

//selector variables
static volatile uint32_t s_ui32SelPos = 20;		//position 1 1-2 2 2-3

static uint32_t s_ui32LevelMin = 0;
static uint32_t s_ui32LevelMax = 0;

static motor_regulation_t s_motorRegulation = {.voltageRegKp = 2000,				//12Q12
											   .voltageRegKi = 500,				//12Q12
											   .motorResitance = 2118,			//14Q10 [R]
											   .motorSpeedFactor = 18392,		//16Q16 [1/min*mV]
											   .voltageEmfSet = 10000};			//16Q4

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

//switch led  $input: uiLEDNumber - number of LED (1...4), uiLevel - state of led $output: instruction successful
// bool led_switch(uint32_t uiLEDNumber, uint32_t uiLevel)
// {
// 	if ((uiLEDNumber > 0) && (uiLEDNumber < 3))
// 	{
// 		if (uiLevel < 2)
// 		{
// 			gpio_set_level(PIN_LEDS[uiLEDNumber - 1], uiLevel);
// 			return true;
// 		}
// 	}
// 	else{

// 	}

// 	return false;
// }

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
	const uint32_t aui32BlinkOnTime[] = {1000, 500};		//blink delay in ms	
	const uint32_t aui32BlinkOffTime[] = {1000, 500};		//blink delay in ms	
	const uint32_t ui32LEDNumber = 0;

	while(1)
	{
		switch(s_aeLedState[ui32LEDNumber])
		{
			case LED_OFF:
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			break;
			case LED_ON:
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			break;
			case LED_BLINK_SLOW:
			gpio_set_level(PIN_LEDS[ui32LEDNumber], 1);
			vTaskDelay(aui32BlinkOnTime[ui32LEDNumber] / portTICK_PERIOD_MS);
			gpio_set_level(PIN_LEDS[ui32LEDNumber], 0);
			vTaskDelay(aui32BlinkOffTime[ui32LEDNumber] / portTICK_PERIOD_MS);
			break;
			case LED_BLINK_FAST:
			gpio_set_level(PIN_LEDS[ui32LEDNumber], 1);
			vTaskDelay(aui32BlinkOnTime[ui32LEDNumber] / portTICK_PERIOD_MS);
			gpio_set_level(PIN_LEDS[ui32LEDNumber], 0);
			vTaskDelay(aui32BlinkOffTime[ui32LEDNumber] / portTICK_PERIOD_MS);
			break;
		}
	}
}




//initialise humidity sensor
static pcnt_unit_handle_t s_pcnt_unit[2] = {NULL, NULL};
void humidity_init(void)
{
	pcnt_unit_config_t unit_config = {
		.high_limit = 10000,      
		.low_limit = -10000,
	};

	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &s_pcnt_unit[0]));
	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &s_pcnt_unit[1]));

	pcnt_chan_config_t chan_config = {
		.edge_gpio_num = PIN_HUM_1,
		.level_gpio_num = -1, // Not used
	};
	pcnt_channel_handle_t pcnt_chan = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit[0], &chan_config, &pcnt_chan));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

	chan_config.edge_gpio_num = PIN_HUM_2; // Use the second GPIO for the second channel
	ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit[1], &chan_config, &pcnt_chan));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

	ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit[0]));
	ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit[1]));
}

//count humidity pulses $return: uint32_t - frequency of pulses [Hz]
uint32_t ui32Humidity_count(uint32_t ui32Channel)
{
	if (ui32Channel > 1) {
		return 0;
	}

	if (s_pcnt_unit[ui32Channel] == NULL) {
		ESP_LOGE("Humidity", "PCNT unit not initialized for channel %d", (int)ui32Channel);
		return 0;
	}

	// Clear the count and start the unit
	ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit[ui32Channel]));
	ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_unit[ui32Channel]));
	int64_t i64StartTime = esp_timer_get_time(); 

	vTaskDelay(100 / portTICK_PERIOD_MS); // wait 100ms

	ESP_ERROR_CHECK(pcnt_unit_stop(s_pcnt_unit[ui32Channel]));
	int64_t i64EndTime = esp_timer_get_time();
	int32_t i32Duration = (int32_t)(i64EndTime - i64StartTime);		//[us]

	int iPulseCount = 0;
	ESP_ERROR_CHECK(pcnt_unit_get_count(s_pcnt_unit[ui32Channel], &iPulseCount));

	int32_t i32Frequency = (int32_t)iPulseCount * 1000000 / i32Duration; // Calculate frequency in Hz
	// ESP_LOGI("Humidity", "Frequency %d: %d", (int)ui32Channel, (int)i32Frequency);
	return (uint32_t)i32Frequency; // Convert to Hz
}

//check humidity state $return: bool , true->water in pot, false -> no water in pot
bool bHumidity_check(uint32_t ui32Channel)
{
	const uint32_t ui32FrequencyLimit = 10500;

	if (ui32Channel > 1) {
		return false;
	}

	if (ui32Humidity_count(ui32Channel) > ui32FrequencyLimit)		//no/less water in pot 
	{
		return true;
	}
	else
	{
		return false;									//water in pot
	}
}

//initalise powerstage
bool bPowerstage_init(void)
{
	//init driver enable
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    			//disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT;                                                          			//set as output mode
	io_conf.pin_bit_mask = ((1ULL << PIN_SEL1_EN)|(1ULL << PIN_SEL2_EN)|(1ULL << PIN_PUMP_EN));   	  	//bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE  ;                                           			//disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                			//disable pull-up mode
	gpio_config(&io_conf);
	gpio_set_level(PIN_SEL1_EN, 0);
	gpio_set_level(PIN_SEL2_EN, 0);
	gpio_set_level(PIN_PUMP_EN, 0);


	//init speed measurement (Hall sensor)
	io_conf.intr_type = GPIO_INTR_POSEDGE;//GPIO_INTR_POSEDGE;                                                    //interrupt pos edge
	io_conf.mode = GPIO_MODE_INPUT;                                                          	//set as output mode
	io_conf.pin_bit_mask = (1ULL << PIN_PUMP_SPEED);     											//bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                           	//disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                                //disable pull-up mode
	gpio_config(&io_conf);
	
	speed_sens_evt_queue = xQueueCreate(1, sizeof(uint32_t));
	// xTaskCreate(speed_sens_task, "speed_sens_task", 2048, NULL, 10, NULL);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(PIN_PUMP_SPEED, speed_sens_isr_handler, (void*) PIN_PUMP_SPEED);

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
        .freq_hz          = 500,  					// Set output frequency at 1 kHz
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
				xQueueSendFromISR(speed_sens_evt_queue, &ui32SpeedPulse, NULL);		xQueueSendFromISR(speed_sens_evt_queue, &ui32SpeedPulse, NULL);
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

// //regulation of selector motor $return: float -> dutycycle for selector
// esp_err_sel_t selector_motor_regulation(motor_direction_t eDirection)
// {
// 	esp_err_sel_t err = ERR_SEL_OK;

// 	static int32_t i32OCcount = 0;
// 	int32_t i32AdcValue = adc1_get_raw(PIN_CURSEL_SENS);
// 	i32AdcValue += adc1_get_raw(PIN_CURSEL_SENS);
// 	i32AdcValue >>= 1;
// 	//16Q4 = ((12Q0 * 15Q16 -> 27Q16 >> 10 -> 17Q6) + 14Q6 -> 17Q6) >> 2 -> 15Q4[mV]
// 	int32_t i32AdcReadVoltage = (((i32AdcValue * s_sAdcCalibration.adcValueFactor) >> 10) + s_sAdcCalibration.adcValueOffset) >> 2;
// 	int32_t i32SelCurrent = i32AdcReadVoltage * 10;		// i32AdcReadVoltage / 0.1Ohm -> *10
// 	//18Q4*10Q6 -> 28Q10
// 	int32_t i32SelResVoltage = i32SelCurrent * (uint32_t)(1.75 * 64);	//[mV] = 15Q4[mA] * 10Q6[R] (Resistance motor 1,6R + driver ,15R)
// 	int32_t i32SelSetVoltage = i32SelResVoltage + (uint32_t)(700 * 1024);
// 	int32_t i32DutyCyle = i32SelSetVoltage / s_ui32BattVoltage;			//29Q10 / 14Q0 -> 15Q10
// 	// float fDutyCylce = (float)i32DutyCyle / 10.24;

// 	//check for stucked motor/ overcurrent
// 	if (i32DutyCyle > 1000)
// 	{
// 		i32DutyCyle = 1000;
// 		i32OCcount++;
// 		if (i32OCcount > 200)
// 		{
// 			selector_stop();
// 			return ERR_SEL_OC;
// 		}
// 	}
// 	else
// 	{
// 		i32OCcount = 0;
// 	}

// 	switch(eDirection)
// 	{
// 		case MOTOR_DIR_UP:
// 			//activate pwm
// 			err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, i32DutyCyle);
// 		    if (err != ESP_OK) break;
// 			err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
// 			// err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, fDutyCylce);
// 			break;

// 		case MOTOR_DIR_DOWN:
// 			//activate pwm
// 			err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, i32DutyCyle);
// 		    if (err != ESP_OK) break;
// 			err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
// 		    // err = mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, fDutyCylce);
// 			break;

// 		default:
// 			break;
// 	}
// 	return err;
// }

// //position of selector $eMotorDirection: motor direction,  $return: motor position 0...16, 0(sel1), 1(sel1-2), 2(sel2),...
// static uint32_t selector_pos(motor_direction_t eMotorDirection)
// {
// 	static uint32_t s_ui32HallIterator = 0;

// 	gpio_set_level(PIN_MUX1, s_ui32HallIterator&1);        	//select Mux
// 	gpio_set_level(PIN_MUX2, s_ui32HallIterator&2);
// 	gpio_set_level(PIN_MUX3, s_ui32HallIterator&4);
// 	vTaskDelay(10);										//wait 10ms
// 	//selector pos changed
// 	uint32_t ui32SelState = gpio_get_level(PIN_SEL_POS);

// 	//select pos 0/8
// 	if (s_ui32HallIterator == 0)
// 	{
// 		if (ui32SelState == 0)
// 		{
// 			//state changed from open - close
// 			if((s_ui32SelPos != 0) ||(s_ui32SelPos != 16))
// 			{
// 				//select pos 0/8
// 				if(s_ui32SelPos < 8)
// 				{
// 					s_ui32SelPos = 0;		//pos1
// 				}
// 				else
// 				{
// 					s_ui32SelPos = 16;		//pos9
// 				}
// 			}
// 		}
// 		else
// 		{
// 			//state changed from close - open
// 			if ((s_ui32SelPos == 0) ||(s_ui32SelPos == 16))
// 				{
// 					switch(eMotorDirection)
// 					{
// 					case MOTOR_DIR_UP:
// 						s_ui32SelPos++;
// 						break;
// 					case MOTOR_DIR_DOWN:
// 						s_ui32SelPos--;
// 						break;
// 					default:
// 						break;
// 					}
// 				}
// 			}
// 	}
// 	//for the other states
// 	else
// 	{
// 		if (ui32SelState == 0)
// 		{
// 			//state changed from open - close
// 			if((s_ui32HallIterator << 1) != s_ui32SelPos)
// 			{
// 					s_ui32SelPos = s_ui32HallIterator << 1;
// 			}
// 		}
// 		else
// 		{
// 			//state changed from close - open
// 			if ((s_ui32HallIterator << 1) == s_ui32SelPos)
// 			{
// 				switch(eMotorDirection)
// 				{
// 				case MOTOR_DIR_UP:
// 					s_ui32SelPos++;
// 					break;
// 				case MOTOR_DIR_DOWN:
// 					s_ui32SelPos--;
// 					break;
// 				default:
// 					break;
// 				}
// 			}
// 		}
// 	}

// 	s_ui32HallIterator++;
// 	if (s_ui32HallIterator > 7) s_ui32HallIterator = 0;
// 	return s_ui32SelPos;
// }

// //positionate slector $ui32position: set position of selector, $return: esp_err_t -> positioning successfull
// esp_err_sel_t selector_set(uint32_t ui32selPos)
// {
// 	if (ui32selPos > 16)
// 	{
// 		ESP_LOGE("Selector: ", "Error ui32selPos > 16");
// 		return ERR_SEL_POS_OUTOFRANGE;
// 	}
// 	if (s_ui32SelPos > 16)
// 	{
// 		ESP_LOGE("Selector: ", "Error s_ui32SelPos > 16");
// 		return ERR_SEL_POS_OUTOFRANGE;
// 	}
	
// 	esp_err_sel_t err = ERR_SEL_OK;
// 	motor_direction_t eMotorDir = MOTOR_DIR_IDLE;
// 	uint32_t s_ui32TimeWatch = 0;
// 	gpio_set_level(PIN_DRV_EN, 1);        		//enable driver
// 	vTaskDelay(100);                            //wait for capacitors loaded

// 	//positionate selector
// 	if(selector_pos(eMotorDir) != ui32selPos)
// 	{
// //		led_switch(1, 1);
// 		uint32_t ui32Position = selector_pos(eMotorDir);

// 		if(ui32selPos > ui32Position)
// 		{
// 			eMotorDir = MOTOR_DIR_UP;
// 		}
// 		else
// 		{
// 			eMotorDir = MOTOR_DIR_DOWN;
// 		}
// 		selector_run(700, eMotorDir);		//dutycylce 70% * 1023

// 		int32_t i32SelPosLast = (int32_t)s_ui32SelPos;
// 		while(selector_pos(eMotorDir) != ui32selPos)
// 		{
// 			s_ui32TimeWatch++;
// 			if (s_ui32TimeWatch > 1000)		//longer than 10s
// 			{
// 				selector_stop();
// 				s_ui32SelPos = 20;	//pos unknown
// 				err = ERR_SEL_OT;
// 				ESP_LOGE("Selector: ", "Error Overtime");
// 				break;
// 			}
// 			if (selector_motor_regulation(eMotorDir) == ERR_SEL_OC)
// 			{
// 				selector_stop();
// 				err = ERR_SEL_OC;
// 				ESP_LOGE("Selector: ", "Error Overcurrent");
// 				break;
// 			}
// 			//selector pos changed
// 			if (s_ui32SelPos != i32SelPosLast)
// 			{
// 				if ((eMotorDir == MOTOR_DIR_UP) && (s_ui32SelPos > ui32selPos))
// 				{
// 					selector_stop();
// 					err = ERR_SEL_POS_OUTOFRANGE;
// 					ESP_LOGE("Selector: ", "Error wrong direction");
// 					break;
// 				}
// 				if ((eMotorDir == MOTOR_DIR_DOWN) && (s_ui32SelPos <  ui32selPos))
// 				{
// 					selector_stop();
// 					err = ERR_SEL_POS_OUTOFRANGE;
// 					ESP_LOGE("Selector: ", "Error wrong direction");
// 					break;
// 				}
// 				//selector state next
// 				if (abs((int32_t)s_ui32SelPos - i32SelPosLast) > 2)
// 				{
// 					selector_stop();
// 					err = ERR_SEL_POS_OUTOFRANGE;
// 					ESP_LOGE("Selector: ", "Error overstepping");
// 					s_ui32SelPos = 20;
// 					break;
// 				}
// 				i32SelPosLast = (int32_t)s_ui32SelPos;
// 				ESP_LOGI("Selector: ", "ui32SelPosErr: %d", (int)s_ui32SelPos);
// 			}
// 		}

// 		//intersteps
// 		if (ui32selPos%2)
// 		{
// 			const uint32_t ui32Voltage = 500>>10;	//run with 500mV 18Q10
// 			selector_run(ui32Voltage/s_ui32BattVoltage, eMotorDir);		//dutycylce 30% * 1023 - 10Q10
// 			vTaskDelay(1000);
// 		}
// 		selector_stop();
// //		led_switch(1, 0);
// 	}

// 	gpio_set_level(PIN_DRV_EN, 0);        		//disable driver
// 	return err;
// }

//positionate slector $i32SetAngleRaw: set angle of selector, $eSelNb: number of selector, $bInitTLV: TLV initialized before? $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_setAngle(int32_t i32SetAngleRaw, selector_nb_t eSelNb, bool bInitTLV)
{
	ESP_LOGI("Selector: ", "setAngle(%d,%d)", (int)i32SetAngleRaw, (int)eSelNb);
	static gpio_num_t aePinSel[2] = {PIN_SEL1_EN, PIN_SEL2_EN};
	// gpio_num_t ePinNb = aePinSel[eSelNb];

	if (i32SetAngleRaw > 359)
	{
		ESP_LOGE("Selector: ", "Error angle > 360°");
		return ERR_SEL_POS_OUTOFRANGE;
	}

	//enable driver/magnet sensor
	if (!bInitTLV)
	{	
		i2c_deinit();
		ESP_ERROR_CHECK(gpio_set_level(aePinSel[eSelNb], 1));        		//enable driver
		vTaskDelay(100);		//wait for capacitors loaded
		i2c_init();
		TLV_init();				//init TLV sensor
	}

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
		gpio_set_level(aePinSel[eSelNb], 0);        		//disable driver
	}														//deinit TLV sensor

	return err;
}


//positionate slector $i32SetPos: set pos of selector, $eSelNb: number of selector $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_setPos(int32_t i32SetPos, selector_nb_t eSelNb)
{
	const uint32_t caui32AnglePosSel1[5] = {300, 100, 160, 270, 70};		//angle pos for selector 1
	const uint32_t caui32AnglePosSel2[5] = {320, 120, 240, 280, 70};		//angle pos for selector 2

	switch (eSelNb)
	{
		case SELECTOR_1:
			return selector_setAngle(caui32AnglePosSel1[i32SetPos], SELECTOR_1, false);	//set angle for selector 1
		case SELECTOR_2:
			return selector_setAngle(caui32AnglePosSel2[i32SetPos], SELECTOR_2, false);	//set angle for selector 2
		default:
			return ERR_SEL_POS_OUTOFRANGE;
	}
}


//calibrate slectors position $selector_nb_t: number of selector, $return: esp_err_t -> positioning successfull
esp_err_sel_t selector_caliPos(selector_nb_t eSelNb)
{
	ESP_LOGI("Selector: ", "selector_caliPos()");
	uint32_t ui32TimeCount = 0;
	gpio_num_t aePinSel[2] = {PIN_SEL1_EN, PIN_SEL2_EN};
	adc_mux_t aeAdcSel[2] = {ADC_MUX_CUR_SEL1, ADC_MUX_CUR_SEL2};

	//enable driver/magnet sensor
	gpio_set_level(aePinSel[eSelNb], 1);        		//enable driver
	vTaskDelay(100);
	adc_read_mux(aeAdcSel[eSelNb], 1);		//read adc mux for current sensor

	TLV_init();		//init TLV sensor
	int32_t i32LastAngle = 0;
	uint32_t ui32StuckCount = 0;

	//position points dir down
	for (int i = 0; i < 20; i++)
	{
		selector_setAngle(i * 10, eSelNb, true);	//set angle +10° for next position

		uint32_t ui32Duty = 0;
		int32_t i32Angle = i32TLV_getAngle();			//read first angle value
		i32LastAngle = i32Angle;		
		
		while(i32LastAngle == i32Angle)
		{
			ui32Duty+=10;		//increase dutycycle
			selector_run(ui32Duty, MOTOR_DIR_DOWN);		//dutycylce xx% * 1023
			vTaskDelay(20);			//wait 10ms
			i32Angle = i32TLV_getAngle();			//read new angle value
			if(ui32Duty > 1000)
			{
				selector_stop();
				ESP_LOGE("Selector: ", "Error timeout");
				break;
			}
		}

		ESP_LOGI("Sel", "Pos: %i Duty: %i", i*10, (int)ui32Duty);
		selector_stop();
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
	const int32_t c_i32MotorSpeedSet = 5000;	//set motor speed in rpm

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

	volatile int32_t i32SpeedVoltage = 1500 * 1024; 		//initial speed voltage (first 100ms) 20Q10 [mV]

	static volatile int32_t s_i32SpeedDevSum = 0; 			//i32SpeedVoltage/1024*1000;
	volatile int32_t s_i32SpeedDevSumLast[] = {0,0,0,0,0};

	if(s_i32SpeedDevSum == 0)
	{
		ESP_ERROR_CHECK(storage_read("SpeedDevSum", &s_i32SpeedDevSum));
	}
	
	s_i32SpeedDevSum = 20000;		//delete for later purpose

	// volatile int32_t i32CurrSet = 0;						//[mA]
	// volatile int32_t i32CurrDevSum = 0;
	// volatile int32_t i32CurrDevLast = 0;

	while(esp_timer_get_time() < i64EndTime)
	{		
		static int32_t i32OCcount = 0;

		//************************************************* speed adaption  ************************************************/
		static int32_t s_i32MotorSpeed = 60000000;
		const int32_t i32SpeedKp = (int32_t)(.005 * 1024);			//10Q10
		const int32_t i32SpeedKi = (int32_t)(.005 * 1024);			//10Q10

		//wait for 50ms
		if(counter > 5)
		{
			int32_t i32MotorSpeed = 1;
			s_i32MotorSpeed = s_i32MotorDuration;

			i32MotorSpeed = 60000000 / s_i32MotorSpeed;	
			int32_t i32RegDev = c_i32MotorSpeedSet - i32MotorSpeed;								//calc deviation to set speed
			s_i32SpeedDevSum+=i32RegDev;
			if (s_i32SpeedDevSum > 50000) s_i32SpeedDevSum = 500000;		//limit integral part to 2,5V
			if (s_i32SpeedDevSum < -50000) s_i32SpeedDevSum = -200000;		//limit integral part to -1V
			i32SpeedVoltage = (i32RegDev * i32SpeedKp) + (s_i32SpeedDevSum * i32SpeedKi);	//32Q10[mV]
			s_i32SpeedDevSumLast[counter % 5] = s_i32SpeedDevSum;
			// i32SpeedVoltage = 352000;
			i32SpeedVoltage = 700*1024;
		}
		

		//************************************************* current voltage regulation  ************************************************/
		int32_t i32AdcValue = adc_read_mux(ADC_MUX_CUR_PUMP, 2);		//read adc mux for current sensor
		// 16Q4 = ((12Q0 * 15Q16 -> 27Q16 >> 10 -> 17Q6) + 14Q6 -> 17Q6) >> 2 -> 15Q4[mV]
		int32_t i32AdcReadVoltage = (((i32AdcValue * s_sAdcCalibration.adcValueFactor) >> 10) + s_sAdcCalibration.adcValueOffset) >> 2;
		int32_t i32SelCurrent = i32AdcReadVoltage * 5;		// i32AdcReadVoltage / 0.2Ohm -> *5
		//18Q4*10Q6 -> 28Q10
		int32_t i32SelResVoltage = i32SelCurrent * (uint32_t)(1.05 * 64);	//[mV] = 15Q4[mA] * 10Q6[R] (Resistance motor 0,5R + driver ,15R)
		// ESP_LOGI("pump_runTime","i32SelResVoltage: %ld",i32SelResVoltage>>10);
		int32_t i32SelSetVoltage = i32SelResVoltage + i32SpeedVoltage;
		i32SelSetVoltage = 2500 * 1024;	//2V
		int32_t i32DutyCyle = i32SelSetVoltage / i32BattVoltage;			//29Q10 / 14Q0 -> 15Q10

		ESP_LOGI("pump_runTime","Counter: %d, Speed: %drpm, i32SpeedVoltage: %d, CurrAct: %d, s_i32SpeedDevSum: %d", (int)counter, (int)(60000000 / s_i32MotorDuration), (int)i32SpeedVoltage, (int)i32AdcValue, (int)s_i32SpeedDevSum);
		//************************************************* speed regulation  ************************************************/
		// static int32_t s_i32MotorSpeed = 60000000;
		
		// if ((counter%3)==0)
		// {
		// 	int32_t i32MotorSpeed = 1;
		// 	// xQueueReceive(speed_sens_evt_queue, &s_i32MotorSpeed, 0);
		// 	s_i32MotorSpeed = s_i32MotorDuration;
		// 	i32MotorSpeed = 60000000 / s_i32MotorSpeed;											//calc time to frequency [rpm]
		// 	int32_t i32SpeedKp = (int32_t)(0.0 * 1024);		//10Q10
		// 	int32_t i32SpeedKi = (int32_t)(0.1 * 1024);		//10Q10;

		// 	int32_t i32RegDev = c_i32MotorSpeedSet - i32MotorSpeed;								//calc deviation to set speed

		// 	s_i32SpeedDevSum+=i32RegDev;
		// 	i32CurrSet = (i32RegDev * i32SpeedKp) + (s_i32SpeedDevSum * i32SpeedKi);											//calc dutycycle 22Q10
		// 	i32CurrSet>>=10;																	//shift 20Q10 to 32Q0
		// 	i32CurrSet += 160;			//[160mA @ 40001/min] "vorsteuerung" speed control, current with no load
		// }

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
		vTaskDelay(10 / portTICK_PERIOD_MS);	
	}
	pump_stop();
	gpio_set_level(PIN_PUMP_EN, 0);        		//enable driver

	//save data permanent of speed controller
	int32_t i32SpeedDevSumMean = (s_i32SpeedDevSumLast[0] + s_i32SpeedDevSumLast[1] + s_i32SpeedDevSumLast[2] + s_i32SpeedDevSumLast[3] + s_i32SpeedDevSumLast[4]) / 5;
	uint32_t ui32SpeedDevSumMeanLast = 0;
	ESP_ERROR_CHECK(storage_read("SpeedDevSum", &ui32SpeedDevSumMeanLast));
	int32_t i32SpeedDevSumMeanLast = (int32_t)ui32SpeedDevSumMeanLast;
	int32_t i32SpeedDevSumMeanNext = ((i32SpeedDevSumMean - i32SpeedDevSumMeanLast)>>1) + i32SpeedDevSumMeanLast;
	//only save on deviation > 5000
	if ((abs(i32SpeedDevSumMeanLast - i32SpeedDevSumMeanNext) > 5000) && (i32SpeedDevSumMean > 0))
	{
		ESP_ERROR_CHECK(storage_write("SpeedDevSum", (uint32_t)i32SpeedDevSumMeanNext));
		ESP_LOGI("pump_runTime","saved: %d", (int)i32SpeedDevSumMean);
	}
	ESP_LOGI("pump_runTime","i32SpeedDevSumMeanLast: %d, i32SpeedDevSumMeanNext: %d", (int)i32SpeedDevSumMeanLast, (int)i32SpeedDevSumMeanNext);
	return err;
	
}

// //run motor $ui32RunTime: run time in ms $return: esp_err_t -> running motor sucessful
// esp_err_t motor_run(uint32_t ui32RunTime)
// {
// 	esp_err_t err = ESP_OK;

// //	gpio_set_level(PIN_CUR_SENS_EN, 1);        	//enable sensing
// //	vTaskDelay(100);                            //wait for capacitors loaded

// 	gpio_set_level(PIN_DRV_EN, 1);        		//enable driver

// 	//Proof if battery voltage is measured
// 	if (s_ui32SupplyVoltage == 0)
// 	{
// 		ui32EspVolt_read();
// 		if (err != ESP_OK) return err;
// 	}

// 	//calc average of 64 measurements
// 	int32_t i32AdcReadSum = 0;
// 	for (int iCount = 0; iCount < 64; iCount++)
// 	{
// 		i32AdcReadSum += adc1_get_raw(PIN_CURSEL_SENS);
// 	}

// 	//16Q4 = ((17Q6 * 15Q16 -> 32Q22 >> 16 -> 16Q6) + 14Q6 -> 17Q6) >> 2 -> 15Q4
// 	i32AdcReadSum = (((i32AdcReadSum * s_sAdcCalibration.adcValueFactor) >> 16) + s_sAdcCalibration.adcValueOffset) >> 2;

// 	//proof offset of current sensor
// 	if((i32AdcReadSum < (120*16)) || (i32AdcReadSum > (180*16)))				//nominal 150mV (16Q4), proof range 120...180mV
// 	{
// 		err = ESP_ERR_INVALID_ARG;
// 	}
// 	s_i32CurSensOffsetVoltage = i32AdcReadSum;

// 	//Proof if battery voltage is measured
// 	if (s_ui32BattVoltage == 0)
// 	{
// 		ui32BattVolt_read();
// 		if (err != ESP_OK) return err;
// 	}

// 	//activate pwm
// 	err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 300);
//    	// mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 30.0);	
//     if (err != ESP_OK) return err;
// 	err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);	
//     // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 30.0);
//     if (err != ESP_OK) return err;
// 	err = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 300);
// 	// mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);   	//nmos 1 switching
//     if (err != ESP_OK) return err;
// 	err = ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
// 	// mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);   	//pmos 1 switching
//     if (err != ESP_OK) return err;

//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = 10;		//frequency 100Hz
// 	xLastWakeTime = xTaskGetTickCount();

// //	led_switch(1, 1);
// 	uint32_t xWasDelayed = 0;
// 	uint32_t ui32AnalyserCount = 0;
// 	for( ;; )
// 	{
// 		if (ui32AnalyserCount < 511)
// 		{
// 			// g_ai32Analyser[ui32AnalyserCount] = s_motorRegulation.currentAct;
// 			ui32AnalyserCount++;
// 		}

// //		led_switch(1, 1);
// 		vTaskDelayUntil( &xLastWakeTime, xFrequency);
// 		motorRegulation();
// 	    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, s_motorRegulation.pwmSet);
// 	    // mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, s_motorRegulation.pwmSet);
// 		xWasDelayed += 10;
// 		if (xWasDelayed > ui32RunTime) break;
// 	}

// //	led_switch(1, 0);
// //	gpio_set_level(PIN_CUR_SENS_EN, 0);        	//disable sensing

//     // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);    //set to idle - nmos 1 low
//     if (err != ESP_OK) return err;
//     // mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);   //set to idle - pmos 1 high
//     if (err != ESP_OK) return err;

// 	gpio_set_level(PIN_DRV_EN, 0);        		//disable driver
// 	return err;
// }

// uint32_t ui32ReadVariable(){
// 	return s_motorRegulation.currentAct >> 4;
// }

//initialise adc function
void adc_init()
{
	//init driver/sensing and supply enable pin
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT;                                                          //set as output mode
	io_conf.pin_bit_mask = ((1ULL << PIN_DRV_EN)|(1ULL << PIN_SENS_EN));     		  //bit mask of the pins
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

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(PIN_ADC_MUX, ADC_ATTEN_DB_0);

	gpio_set_level(PIN_DRV_EN, 1);                                                            //enable drv sensing
	gpio_set_level(PIN_SENS_EN, 1);                                                           //enable supply sensing
	gpio_set_level(PIN_ADCMUX1, 0);                                                           //disable MUX 
	gpio_set_level(PIN_ADCMUX2, 0);                                                           //
	gpio_set_level(PIN_ADCMUX3, 0);                                                           //

	//set values from storage
	storage_read("adc_cali_factor", &s_sAdcCalibration.adcValueFactor);		//32Q32
	storage_read("adc_cali_offset", &s_sAdcCalibration.adcValueOffset);		//32Q16
}

//read adc values over mux $eAdcChannel: enum of adc channel,  $ui32NbMean: quantities of adc measurements for mean value, $output: adc value Q12
uint32_t adc_read_mux(adc_mux_t eAdcChannel, uint32_t ui32NbMean)
{
	static uint32_t s_ui32Mem = 0;		//static memory for adc channel

	if (s_ui32Mem != (eAdcChannel + 1))
	{	
		// gpio_set_level(PIN_DRV_EN, 1);                                                            		//enable drv sensing
		
		// //enable sensing supply 
		// if (eAdcChannel < 4)
		// {
		// 	gpio_set_level(PIN_SENS_EN, 1);                                                           //enable supply sensing
		// }

		//set mux channel
		gpio_set_level(PIN_ADCMUX1, eAdcChannel&1);        	//select Mux
		gpio_set_level(PIN_ADCMUX2, eAdcChannel&2);
		gpio_set_level(PIN_ADCMUX3, eAdcChannel&4);	

		vTaskDelay(50);                                 //wait for capacitors loaded

		s_ui32Mem = eAdcChannel + 1;		//save last channel		
	}
	
	uint32_t ui32AdcReadSum = adc1_get_raw(PIN_ADC_MUX);			//adc read sum
	// ESP_LOGI("ADC: ", "ui32AdcReadSum: %d", (int)ui32AdcReadSum);
	if (ui32NbMean > 1)
	{	
		for (uint32_t iCount = 1; iCount < ui32NbMean; iCount++)
		{
			ui32AdcReadSum += adc1_get_raw(PIN_ADC_MUX);
		}	
		ui32AdcReadSum /= ui32NbMean;		//calc mean value
	}
	// ESP_LOGI("ADC: ", "ui32AdcReadSum: %d", (int)ui32AdcReadSum);
	return ui32AdcReadSum;
}

//calibrate ADC peripherie with 2 point calibration - on Pin S7 of ADC Mux
esp_err_t adc_calibration_150mV()
{
	esp_err_t eEspError = ESP_OK;

	//calc average of 100 measurements
	s_sAdcCalibration.adcValue150mV = adc_read_mux(ADC_MUX_TP, 100);		//read adc mux for 150mV
	if((s_sAdcCalibration.adcValue150mV < 180) || (s_sAdcCalibration.adcValue150mV > 380))
	{
		eEspError = ESP_ERR_INVALID_ARG;
	}
	return eEspError;
}

esp_err_t adc_calibration_850mV()
{
	esp_err_t eEspError = ESP_OK;

	//calc average of 100 measurements
	s_sAdcCalibration.adcValue850mV = adc_read_mux(ADC_MUX_TP, 100);
	if((s_sAdcCalibration.adcValue850mV < 3060) || (s_sAdcCalibration.adcValue850mV > 3460))
	{
		return ESP_ERR_INVALID_ARG;
	}
	s_sAdcCalibration.adcValueFactor = (700U * 65536U) / (s_sAdcCalibration.adcValue850mV - s_sAdcCalibration.adcValue150mV); 			//deltaV(700mV->27Q16) / deltaADC(12Q0) = factor(15Q16)[digit/mV]
	s_sAdcCalibration.adcValueOffset = (9830400 - (s_sAdcCalibration.adcValueFactor * s_sAdcCalibration.adcValue150mV)) >> 10;					//offsetV(150mV->32Q16) - (factor(15Q16) * valueADC150mV(12Q0) -> offsetV(32Q16) >> 10 -> 16Q6[mV]
	eEspError = storage_write("adc_cali_factor", s_sAdcCalibration.adcValueFactor);
	if (eEspError != ESP_OK) return eEspError;
	eEspError = storage_write("adc_cali_offset", s_sAdcCalibration.adcValueOffset);
	if (eEspError != ESP_OK) return eEspError;
	return eEspError;
}

//read battery voltage $output: battery voltage [mV]
uint32_t ui32BattVolt_read(void)
{
	//16Q0 = ((14Q2 * 15Q16 -> 29Q18 >> 12 -> 17Q6) + 14Q6 -> 17Q6) * 15Q12 -> 32Q18 >> 18 -> 14Q0
	s_ui32BattVoltage =  ((((adc_read_mux(ADC_MUX_VOLT_BATT, 4) * s_sAdcCalibration.adcValueFactor) >> 10) + s_sAdcCalibration.adcValueOffset) * s_cui32BattVoltageFact) >> 18;
	ESP_LOGI("ADC","ui32BattVolt_read: %d", (int)s_ui32BattVoltage);
	return s_ui32BattVoltage;
}

//read solar voltage $output: solar voltage [mV]
uint32_t ui32SolarVolt_read(void)
{
	//16Q0 = ((14Q2 * 15Q16 -> 29Q18 >> 12 -> 17Q6) + 14Q6 -> 17Q6) * 15Q12 -> 32Q18 >> 18 -> 14Q0
	uint32_t ui32SolarVoltage =  ((((adc_read_mux(ADC_MUX_VOLT_BATT, 4) * s_sAdcCalibration.adcValueFactor) >> 10) + s_sAdcCalibration.adcValueOffset) * s_cui32SolarVoltageFact) >> 18;
	ESP_LOGI("ADC","ui32SolarVoltage: %d", (int)ui32SolarVoltage);
	return ui32SolarVoltage;
}

//read esp supply voltage $output: esp voltage [mV]
uint32_t ui32EspVolt_read(void)
{
	  //16Q0 = ((14Q2 * 15Q16 -> 29Q18 >> 12 -> 17Q6) + 14Q6 -> 17Q6) * 15Q12 -> 32Q18 >> 18 -> 14Q0
	  s_ui32SupplyVoltage = ((((adc_read_mux(ADC_MUX_VOLT_3V3, 4) * s_sAdcCalibration.adcValueFactor) >> 10) + s_sAdcCalibration.adcValueOffset) * s_cui32LogicVoltageFact) >> 18;
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
	float fTemperatureFact = (float)((((adc_read_mux(ADC_MUX_TEMP_PCB, 4) * s_sAdcCalibration.adcValueFactor) >> 10) + s_sAdcCalibration.adcValueOffset) >> 6) / s_ui32SupplyVoltage;
	for (int iTempCount = 0; iTempCount < 14; iTempCount ++)
	  {
		if (g_cafNTCFactValues[iTempCount] >  fTemperatureFact)
		{
		  return g_cafNTCTempValues[iTempCount - 1] + (5/(g_cafNTCFactValues[iTempCount] - g_cafNTCFactValues[iTempCount - 1])*(fTemperatureFact - g_cafNTCFactValues[iTempCount - 1]));
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

// //touch interrupt service routine
// static volatile uint32_t s_ui32TouchIsrButtonTriggered = 0;
// /*
// static void touch_isr(void *arg)
// {
// 	if ((touch_pad_get_status() >> PIN_BUTTON) & 0x01)
// 	{
// 		s_ui32TouchIsrButtonTriggered = 1;
// 	}
// 	touch_pad_clear_status();
// }
// */

//initialise touch
void touch_init1(void)
{
	uint16_t clock_cycle = 0;
	ESP_ERROR_CHECK(touch_pad_init());
	ESP_ERROR_CHECK(touch_pad_set_cnt_mode(PIN_TOUCH, TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_LOW));      		//slope of touch sensor - equivalent ot current from 1(weak) to 7(fast)
	ESP_ERROR_CHECK(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V));          	//voltages levels
	// ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));												//hardware mode
	// ESP_ERROR_CHECK(touch_pad_set_meas_time(0x7530, 0xFFFF));                                                	//sleept time 0x7530 / 150Khz = 200 ms; measure time 0xffff / 8Mhz = 8.19ms
	ESP_ERROR_CHECK(touch_pad_config(PIN_TOUCH, 0));                                                   		//touch treshold no use
	ESP_ERROR_CHECK(touch_pad_get_measurement_clock_cycles(&clock_cycle));
	ESP_ERROR_CHECK(touch_pad_set_measurement_clock_cycles(0xffff));
	ESP_LOGI("TOUCH", "Clock cycles: %d", clock_cycle);														//get clock cycles for touch pad measurement

	// touch_pad_config(PIN_WATER_BOTTOM, 0);                                                  					//touch treshold no use
	// ESP_ERROR_CHECK(touch_pad_config(PIN_BUTTON, 1500));                                                        //touch treshold no use
	// ESP_ERROR_CHECK(touch_pad_set_trigger_mode(TOUCH_TRIGGER_BELOW));
	// ESP_ERROR_CHECK(touch_pad_isr_register(touch_isr, NULL));
	// ESP_ERROR_CHECK(touch_pad_intr_enable());

	// read storage
	// storage_read("level_min", &s_ui32LevelMin);
	// storage_read("level_max", &s_ui32LevelMax);

	// xTaskCreate(Button_task, "button_event_task", 1024, NULL, 5, NULL);    										//1024 Create a task to handler TOUCH event from ISR
}

void touch_deinit1(void)
{
	touch_pad_deinit();
}

//read the value of water level sensor
uint32_t ui32Level_read(void)
{
	uint16_t i32TouchValue;
	uint32_t i32TouchValueSum = 0;

	//calculate mean value of x measurements
	static const uint32_t ui32MeanQty = 4;
	for (uint32_t count = 0; count < ui32MeanQty; count++)
	{
		touch_pad_read(PIN_TOUCH, &i32TouchValue);
		i32TouchValueSum += i32TouchValue;
	}

	i32TouchValueSum /= ui32MeanQty;			//round and calc mean value
	return i32TouchValueSum;
} 

// // //read the value of water level sensor
// // uint32_t ui32Position_read(void)
// // {
// // 	uint16_t iTouchValue;
// // 	uint32_t iTouchValueSum = 0;

// // 	//calculate mean value of x measurements
// // 	static const uint32_t uiMeanQty = 4;
// // 	for (uint32_t count = 0; count < uiMeanQty; count++)
// // 	{
// // 		touch_pad_read(PIN_SELECTOR, &iTouchValue);
// // 		iTouchValueSum += iTouchValue;
// // 	}

// // 	iTouchValueSum /= uiMeanQty;			//round and calc mean value
// // 	return iTouchValueSum;
// // } 

// /* //set min water level value
// bool bLevel_set_min(void)
// {
// 	uint16_t ui16TouchValue;
// 	uint32_t ui32TouchValueSum = 0;

// 	//calculate mean value of x measurements
// 	static const uint32_t uiMeanQty = 20;
// 	for (uint32_t count = 0; count < uiMeanQty; count++)
// 	{
// 		touch_pad_read(PIN_WATER_LEVEL, &ui16TouchValue);
// 		ui32TouchValueSum += ui16TouchValue;
// 	}

// 	ui32TouchValueSum = (ui32TouchValueSum + (uiMeanQty / 2)) / uiMeanQty ;			//round and calc mean value
// 	s_ui32LevelMin = ui32TouchValueSum;

// 	esp_err_t err;
// 	err = storage_write("level_min", ui32TouchValueSum);

// 	if (err != ESP_OK)
// 	{
// 		return false;
// 	}
// 	return true;
// } */

// /* //set max water level value
// bool bLevel_set_max(void)
// {
// 	uint16_t ui16TouchValue;
// 	uint32_t ui32TouchValueSum = 0;

// 	//calculate mean value of x measurements
// 	static const uint32_t uiMeanQty = 20;
// 	for (uint32_t count = 0; count < uiMeanQty; count++)
// 	{
// 		touch_pad_read(PIN_WATER_LEVEL, &ui16TouchValue);
// 		ui32TouchValueSum += ui16TouchValue;
// 	}

// 	ui32TouchValueSum = (ui32TouchValueSum + (uiMeanQty / 2)) / uiMeanQty ;			//round and calc mean value
// 	s_ui32LevelMax = ui32TouchValueSum;

// 	esp_err_t err;
// 	err = storage_write("level_max", ui32TouchValueSum);

// 	if (err != ESP_OK)
// 	{
// 		return false;
// 	}
// 	return true;
// } */

// /* //read the value of water level sensor
// uint32_t ui32Button_read(void)
// {
// 	uint16_t iTouchValue;
// 	uint32_t iTouchValueSum = 0;

// 	//calculate mean value of x measurements
// 	static const uint32_t uiMeanQty = 4;
// 	for (uint32_t count = 0; count < uiMeanQty; count++)
// 	{
// 		touch_pad_read(PIN_BUTTON, &iTouchValue);
// 		iTouchValueSum += iTouchValue;
// 	}

// 	iTouchValueSum /= uiMeanQty;			//round and calc mean value
// 	return iTouchValueSum;
// } */



// //static void timerx_init(timer_group_t group, timer_idx_t timer, bool auto_reload, float timer_interval_sec)
// //{
// //    /* Select and initialize basic parameters of the timer */
// //    timer_config_t timer_config;
// //      timer_config.alarm_en = TIMER_ALARM_EN;
// //      timer_config.counter_en = TIMER_PAUSE;
// //      timer_config.counter_dir = TIMER_COUNT_UP;
// //      timer_config.auto_reload = TIMER_AUTORELOAD_EN;
// //      timer_config.divider = 80;                //timer frequency 1Mhz
// //    timer_init(group, timer, &timer_config);
// //
// //    /* Timer's counter will initially start from value below.
// //       Also, if auto_reload is set, this value will be automatically reload on alarm */
// //    timer_set_counter_value(group, timer, 0);
// //
// //    /* Configure the alarm value and the interrupt on alarm. */
// //    timer_set_alarm_value(group, timer, (int)(timer_interval_sec * 1000000.0));
// //    timer_enable_intr(group, timer);
// //
// //    //timer_isr_callback_add(group, timer, &timer0_callback, NULL, ESP_INTR_FLAG_IRAM,NULL);
// //    timer_isr_register(group, timer, &timer0_callback, NULL, 0, NULL);
// //    timer_start(group, timer);
// //}

// //timer0 interrupt - 5kHZ cycle time for motor regulation
// //static void IRAM_ATTR timer0_callback(void *arg)
// //{
// //	TIMERG0.int_clr_timers.t0 = 1;
// //	TIMERG0.hw_timer[0].config.alarm_en = 1;
// //
// //	//digitalWrite(PIN_LED_RED, HIGH);
// //	int32_t i32Value = adc1_get_raw(PIN_CURSENS) - s_i32CurSensOffsetVoltage;									//raw adc value
// //	s_fCurSensValue = s_ci32CurSensFact * ((float)i32Value) / s_motorRegulation.pwmSet;				//calc current value in mA
// //
// //	if (s_fCurSensValue > s_motorRegulation.currentLimit)
// //	{
// //		//switch off device
// //		motor_stop();
// //		//set error
// //		s_eMotorRegulationSM = MOTOR_REG_IDL;
// //	}
// //
// //	//calc regulation
// //	float fBackEMF = (s_ui32SupplyVoltage * s_motorRegulation.pwmSet / 100) - (s_fCurSensValue * s_cfMotorResistance);		//calc back emf value in mV
// //
// //	s_motorRegulation.pwmSet = 500.0 * (s_motorRegulation.voltageSet - fBackEMF);
// //
// //    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, s_motorRegulation.pwmSet);
// //    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, s_motorRegulation.pwmSet);
// //    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, s_motorRegulation.pwmSet);
// //    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, s_motorRegulation.pwmSet);
// //
// //      //digitalWrite(PIN_LED_YEL, HIGH);
// //
// //  //digitalWrite(PIN_LED_RED, LOW);
// //}
