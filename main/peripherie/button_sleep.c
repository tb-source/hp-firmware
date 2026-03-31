/*
 * button_sleep.c
 *
 *  Created on: 02.02.2024
 *      Author: T.Bretzke
 */

#include "button_sleep.h"

void button_state(void);
void deepSleep_wakeupCause(void);

static volatile button_t s_eButtonState = BTN_IDLE;		
static volatile wakeup_t s_eWakeupState = WAKEUP_IDLE;

//initialise button
void button_sleep_init(void)
{
	deepSleep_wakeupCause();
	ESP_LOGI("button_sleep_init", "start");
	//init pins
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;                                                    //disable interrupt
	io_conf.mode = GPIO_MODE_INPUT;                                                          	//set as input mode
	io_conf.pin_bit_mask = ((1ULL << PIN_BUTTON));     											//bit mask of the pins
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                           	//disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE ;                                                //disable pull-up mode
	ESP_ERROR_CHECK(gpio_config(&io_conf));
	xTaskCreate(button_state, "button_event_task", 1024, NULL, 3, NULL);    										//1024 Create a task to handler TOUCH event from ISR
	ESP_LOGI("button_sleep_init", "end");
}

//get state of button
button_t eButton_state(void)
{
    button_t eStateBuffer = s_eButtonState;
    s_eButtonState = BTN_IDLE;
    return(eStateBuffer);
}

//get state of button
wakeup_t eWakeup_state(void)
{
    wakeup_t eStateBuffer = s_eWakeupState;
    return(eStateBuffer);
}

//button read task
void button_state(void)
{
	static int32_t s_i32TouchTimeCount = 0;
	while(1)
	{
		if(gpio_get_level(PIN_BUTTON) == 0)
		{

			if (s_i32TouchTimeCount == 0)		//neg edge - button pressed
			{
				//led_switch(1,1);
			}

			s_i32TouchTimeCount++;
//			s_ui32TouchIsrButtonTriggered = 0;
		}
		else //pos edge - button released
		{
			if (s_i32TouchTimeCount > 0)		
			{
				//press time < 5s - device startup
				if (s_i32TouchTimeCount < 16)		
				{
                    if((s_eButtonState == BTN_IDLE) || (s_eButtonState == BTN_WAKEUP))
                    {
						ESP_LOGI("BTN_TASK", "Short press detected: %d", (int)s_i32TouchTimeCount);
                        s_eButtonState = BTN_PRESSED_SHORT;
						s_eWakeupState = WAKEUP_BTN_PRESSED_SHORT;
                    }
				}

				//press time > 5s - device setup mode
				else							
				{	
                    if((s_eButtonState == BTN_IDLE) || (s_eButtonState == BTN_WAKEUP))
                    {
						ESP_LOGI("BTN_TASK", "Mid press detected: %d", (int)s_i32TouchTimeCount);
                        s_eButtonState = BTN_PRESSED_MID;
						s_eWakeupState = WAKEUP_BTN_PRESSED_MID;
                    }
				}
			}
			else
			{
				if (s_eButtonState == BTN_WAKEUP)	//very short press 
				{
					ESP_LOGI("BTN_TASK", "Very short press detected: %d", (int)s_i32TouchTimeCount);
					s_eButtonState = BTN_PRESSED_SHORT;
					s_eWakeupState = WAKEUP_BTN_PRESSED_SHORT;
				}
			}
//
			s_i32TouchTimeCount = 0;
		}
		vTaskDelay(250);
	}
}

void deepSleep_activate(uint64_t sleepTimeus)
{
	ESP_LOGI("DEEPSLEEP", "Entering deep sleep for %llu us", sleepTimeus);
	s_eButtonState = BTN_IDLE;
	s_eWakeupState = WAKEUP_IDLE;

	//deinit touch pad to save power
	// touch_deinit1();

	//set wakeup time to next watering sequence
	esp_sleep_enable_timer_wakeup(sleepTimeus);		//in �s - 100s
	//set wakeup for external interrupt for button
	esp_sleep_enable_ext0_wakeup(PIN_BUTTON, 0);

	esp_deep_sleep_start();
}

void deepSleep_wakeupCause(void)
{
	switch(esp_sleep_get_wakeup_cause())
	{
	case ESP_SLEEP_WAKEUP_TIMER:
		{
			s_eWakeupState = WAKEUP_TIMER;
			break;
		}

//	case ESP_SLEEP_WAKEUP_TOUCHPAD:
//		{
//			s_ui32TouchIsrButtonTriggered = 2;				//touchpad in sleep mode touched
//			led_switch(3, 1);
//			break;
//		}

	case ESP_SLEEP_WAKEUP_EXT0:
		{
			//short button press
			// if (gpio_get_level(PIN_BUTTON) == 0)
			// {
				s_eButtonState = BTN_WAKEUP;
				s_eWakeupState = WAKEUP_BTN_PRESSED_ACT;
				// led_switch(2, 1);
			// }
			break;
		}

	default:
		{
			s_eWakeupState = WAKEUP_RESET;
			break;
		}
	}
}