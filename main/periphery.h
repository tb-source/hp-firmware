/*
 * periphery.h
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 */

#ifndef MAIN_PERIPHERY_H_
#define MAIN_PERIPHERY_H_

#include <stdio.h>
#include "storage.h"
#include "hal/gpio_ll.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "driver/mcpwm.h"
#include "esp_timer.h"
// #include "driver/timer.h"
#include "driver/touch_pad.h"
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <esp_log.h>
#include "peripherie/iic.h"

#define SmaWaS_HPSWC1_V01			//pinout of board

enum {
  MOTOR_REG_IDL = 0,
  MOTOR_REG_FOR_READCUR = 1,
  MOTOR_REG_FOR_SWITCHOFF = 9,
  MOTOR_REG_FOR_READVOLT = 10,
  MOTOR_REG_REV_READCUR = 11,
  MOTOR_REG_REV_SWITCHOFF = 19,
  MOTOR_REG_REV_READVOLT = 20,
};

typedef enum {
  SELECTOR_1 = 0,
  SELECTOR_2 = 1,
}selector_nb_t;

typedef enum {
  MOTOR_DIR_IDLE,
  MOTOR_DIR_UP,
  MOTOR_DIR_DOWN
}motor_direction_t;

typedef enum{
	ERR_SEL_OK,					      //nor error
	ERR_SEL_POS_OUTOFRANGE,		//position out of range
	ERR_SEL_INIT,				      //initialisation error
	ERR_SEL_OC,					      //motor overcurrent error
	ERR_SEL_OT,					      //selector overtime error
}esp_err_sel_t;

typedef enum{
	ERR_PUMP_OK,
	ERR_PUMP_INIT,
}esp_err_pump_t;

typedef enum {
  LED_OFF = 0,
  LED_ON = 1,
  LED_BLINK_SLOW = 2,
  LED_BLINK_FAST = 3,
}led_status_t;

typedef struct {
  int32_t currentAct;          		//current value adc (16Q4) [mA]
  uint32_t speedAct;				//speed value act 18Q4 [1/min]
  uint32_t speedSet;				//speed value set 18Q4 [1/min]
  uint32_t currentLimit;       		//current max value in operation point (12Q0)
  int32_t voltageSet;				//set voltage eq to pwm 16Q4 [mV]
  int32_t voltageEmfSet;			//set voltage eq to speed 16Q4 [mV]
  int32_t voltageRegKp;          	//voltage regulation proportional factor (12Q12) -  define variable as constant
  int32_t voltageRegKi;             //voltage regulation integration factor (12Q12) -  define variable as constant
  int32_t voltageRegISum;			//voltage regulation integration sum (28Q16)
  int32_t motorResitance;			//motor resistance 14Q10
  uint32_t motorSpeedFactor;		//motor speed factor 16Q16 [1/min*mV]
  float pwmSet;         			//set value of pwm dc 20.0 means 20% dutycycle
}motor_regulation_t;

typedef struct {
	uint32_t adcValue150mV;			//12Q0
	uint32_t adcValue850mV;			//12Q0
	uint32_t adcValueOffset;		//16Q8
	uint32_t adcValueFactor;		//15Q16
}adc_calibration_t;



 


#ifdef SmaWaS_HPSWC1_V01
static const gpio_num_t PIN_LEDS[]= {GPIO_NUM_16, GPIO_NUM_23};
static const gpio_num_t PIN_BUTTON = GPIO_NUM_4;              					      //button
static const gpio_num_t PIN_CHRG = GPIO_NUM_22;                               //GPIO23

typedef enum {
  ADC_MUX_VOLT_3V3 = 0,        //S0
  ADC_MUX_TEMP_PCB = 1,        //S1
  ADC_MUX_VOLT_SOLAR = 2,      //S2
  ADC_MUX_VOLT_BATT = 3,       //S3
  ADC_MUX_CUR_SEL1 = 4,        //S4
  ADC_MUX_CUR_SEL2 = 5,       //S5
  ADC_MUX_CUR_PUMP = 6,       //S6
  ADC_MUX_TP = 7,             //S0
}adc_mux_t;

typedef enum {
  IO_EXP_PS_SEL1_EN = 0,        //GP0
  IO_EXP_PS_PUMP_EN = 1,        //GP1
  IO_EXP_PS_SEL2_EN = 2,        //GP2
  IO_EXP_PS_HUM1_EN = 3,        //GP3
  IO_EXP_PS_HUM2_EN = 4,        //GP4
  IO_EXP_PS_HUM3_EN = 5,        //GP5
  IO_EXP_PS_TP1_EN = 6,         //GP6 
  IO_EXP_PS_TP2_EN = 7,         //GP7
}io_exp_t;

// // static const gpio_num_t PIN_TEMP_VOLT_EN = GPIO_NUM_25;                       //GPIO2
// static const adc1_channel_t PIN_3V3_SENS = ADC1_CHANNEL_6;                         	//ADC1_CH6
// static const adc1_channel_t PIN_BATT_SENS = ADC1_CHANNEL_7;                         //ADC1_CH7
// static const adc1_channel_t PIN_TEMP_SENS = ADC1_CHANNEL_0;                         //ADC1_CH0
// static const adc1_channel_t PIN_SOLAR_SENS = ADC1_CHANNEL_3;                         //ADC1_CH3
static const gpio_num_t PIN_SENS_EN = GPIO_NUM_19;                        //GPIO2


// static const touch_pad_t PIN_WATER_LEVEL = TOUCH_PAD_NUM0;        					              //level sensor on Touch0
static const touch_pad_t PIN_TOUCH = TOUCH_PAD_NUM8;								      //Touch senor on Touch8 (GPIO4)

static const gpio_num_t PIN_DRV_EN = GPIO_NUM_17;
static const gpio_num_t PIN_SEL1_EN = GPIO_NUM_25;
static const gpio_num_t PIN_SEL2_EN = GPIO_NUM_21;
static const gpio_num_t PIN_PUMP_EN = GPIO_NUM_2;
static const gpio_num_t PIN_PUMP_SPEED = GPIO_NUM_35;
static const gpio_num_t PIN_LS = GPIO_NUM_5;
static const gpio_num_t PIN_HS = GPIO_NUM_18;
static const gpio_num_t PIN_ADCMUX1 = GPIO_NUM_15;              //MUX selection 1 ... 3
static const gpio_num_t PIN_ADCMUX2 = GPIO_NUM_13;
static const gpio_num_t PIN_ADCMUX3 = GPIO_NUM_14;

static const adc1_channel_t PIN_ADC_MUX = ADC1_CHANNEL_4;            //Analog mux signal
static const gpio_num_t PIN_HUM_1 = GPIO_NUM_34;                   //GPIO4
static const gpio_num_t PIN_HUM_2 = GPIO_NUM_39;                   //GPIO34

static const gpio_num_t PIN_I2C_SDA = GPIO_NUM_27;     //GPIO27
static const gpio_num_t PIN_I2C_SCL = GPIO_NUM_26;      //GPIO26
#endif //SmaWaS_HPSWC1_V01

//global variables
extern int32_t g_ai32Analyser[];

//LED functions
extern void led_init(void);
extern void led_set(uint32_t ui32LEDNumber, led_status_t eLedStatus);

//Buzzer functions
extern void buzzer_init(void);
extern bool bBuzzer_beep(uint32_t uiLevel);
extern bool bBuzzer_beep_times(uint32_t uiTimes);

//Sensing functions
extern void adc_init();
uint32_t adc_read_mux(adc_mux_t eAdcChannel, uint32_t ui32NbMean);
extern esp_err_t adc_calibration_150mV(void);
extern esp_err_t adc_calibration_850mV(void);
extern uint32_t ui32BattVolt_read(void);
extern uint32_t ui32EspVolt_read(void);
extern uint32_t ui32SolarVolt_read(void);
extern float fTemp_read(void);
extern uint32_t ui32Charge_read(void);

//Touch functions
extern void touch_init1(void);
extern void touch_deinit1(void);
extern uint32_t ui32Level_read(void);
extern void humidity_init(void);
extern uint32_t ui32Humidity_count(uint32_t ui32Channel);
bool bHumidity_check(uint32_t ui32Channel);

//Powerstage functions
extern bool bPowerstage_init(void);
extern void powerstage_deinit(void);
extern esp_err_t motor_run(uint32_t ui32RunTime);	//bool motor_run(uint32_t iRunTime, uint32_t iBackEMF, motor_direction_t eDirection);
extern esp_err_t selector_run( uint32_t ui32Speedbool, motor_direction_t eDirection);
esp_err_t selector_stop(void);
extern esp_err_sel_t selector_set(uint32_t ui32selPos);
extern uint32_t selector_get(void);
static void IRAM_ATTR timer0_callback(void *arg);
extern esp_err_t selector_runTime(uint32_t ui32Time, motor_direction_t eDirection);
extern esp_err_pump_t pump_runTime(uint32_t ui32Time, motor_direction_t eDirection);
extern esp_err_sel_t selector_setAngle(int32_t i32SetAngle, selector_nb_t eSelNb, bool bInitTLV);
extern esp_err_sel_t selector_setPos(int32_t i32SetPos, selector_nb_t eSelNb);
extern esp_err_sel_t selector_caliPos(selector_nb_t eSelNb);
uint32_t ui32ReadVariable();

#endif /* MAIN_PERIPHERY_H_ */
