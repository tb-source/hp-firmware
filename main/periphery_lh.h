/*
 * periphery_lh.h
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 */

#ifndef MAIN_PERIPHERY_LH_H_
#define MAIN_PERIPHERY_LH_H_

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
#include "esp_adc/adc_oneshot.h"
#include "rom/ets_sys.h"

#define SmaWaS_HPSWC1_V01 // pinout of board

enum
{
  MOTOR_REG_IDL = 0,
  MOTOR_REG_FOR_READCUR = 1,
  MOTOR_REG_FOR_SWITCHOFF = 9,
  MOTOR_REG_FOR_READVOLT = 10,
  MOTOR_REG_REV_READCUR = 11,
  MOTOR_REG_REV_SWITCHOFF = 19,
  MOTOR_REG_REV_READVOLT = 20,
};


typedef enum
{
  MOTOR_DIR_IDLE,
  MOTOR_DIR_UP,
  MOTOR_DIR_DOWN
} motor_direction_t;

typedef enum
{
  ERR_SEL_OK,             // nor error
  ERR_SEL_POS_OUTOFRANGE, // position out of range
  ERR_SEL_INIT,           // initialisation error
  ERR_SEL_OC,             // motor overcurrent error
  ERR_SEL_OT,             // selector overtime error
} esp_err_sel_t;

typedef enum
{
  ERR_PUMP_OK,
  ERR_PUMP_INIT,
} esp_err_pump_t;

typedef enum
{
  LED_OFF = 0,
  LED_ON = 1,
  LED_BLINK_SLOW = 2,
  LED_BLINK_FAST = 3,
} led_status_t;


typedef struct
{
  int32_t currentAct;        // current value adc (16Q4) [mA]
  uint32_t speedAct;         // speed value act 18Q4 [1/min]
  uint32_t speedSet;         // speed value set 18Q4 [1/min]
  uint32_t currentLimit;     // current max value in operation point (12Q0)
  int32_t voltageSet;        // set voltage eq to pwm 16Q4 [mV]
  int32_t voltageEmfSet;     // set voltage eq to speed 16Q4 [mV]
  int32_t voltageRegKp;      // voltage regulation proportional factor (12Q12) -  define variable as constant
  int32_t voltageRegKi;      // voltage regulation integration factor (12Q12) -  define variable as constant
  int32_t voltageRegISum;    // voltage regulation integration sum (28Q16)
  int32_t motorResitance;    // motor resistance 14Q10
  uint32_t motorSpeedFactor; // motor speed factor 16Q16 [1/min*mV]
  float pwmSet;              // set value of pwm dc 20.0 means 20% dutycycle
} motor_regulation_t;

typedef enum
{
  ADC_MUX_IDLE = 0,       // S0
  ADC_MUX_VOLT_3V3 = 1,   // S1
  ADC_MUX_TEMP_PCB = 2,   // S2
  ADC_MUX_VOLT_SOLAR = 3, // S3
  ADC_MUX_VOLT_BATT = 4,  // S4
  ADC_MUX_CUR_SEL = 5,    // S5
  ADC_MUX_CUR_PUMP = 6,   // S6
  ADC_MUX_TP = 7,         // S2
  PWM_MUX_IDLE = 8,       // S0
  PWM_MUX_HUM1 = 9,       // S1
  PWM_MUX_HUM2 = 10,      // S2
  PWM_MUX_HUM3 = 11,      // S3
  PWM_MUX_TANKETY = 12,   // S4
  PWM_MUX_SEL = 13,       // S5
  PWM_MUX_PUMP = 14,      // S6
  PWM_MUX_TANKLVL = 15,   // S7

} adc_mux_t;


#ifdef SmaWaS_HPSWC1_V01
static const gpio_num_t PIN_LEDS[] = {GPIO_NUM_25, GPIO_NUM_2};
static const gpio_num_t PIN_BUTTON = GPIO_NUM_39; // button
static const gpio_num_t PIN_CHRG = GPIO_NUM_36;   // GPIO23

// // static const gpio_num_t PIN_TEMP_VOLT_EN = GPIO_NUM_25;                       //GPIO2
// static const adc1_channel_t PIN_3V3_SENS = ADC1_CHANNEL_6;                         	//ADC1_CH6
// static const adc1_channel_t PIN_BATT_SENS = ADC1_CHANNEL_7;                         //ADC1_CH7
// static const adc1_channel_t PIN_TEMP_SENS = ADC1_CHANNEL_0;                         //ADC1_CH0
// static const adc1_channel_t PIN_SOLAR_SENS = ADC1_CHANNEL_3;                         //ADC1_CH3
static const gpio_num_t PIN_SENS_EN = GPIO_NUM_33; // GPIO2

// static const touch_pad_t PIN_WATER_LEVEL = TOUCH_PAD_NUM0;        					              //level sensor on Touch0
// static const touch_pad_t PIN_TOUCH = TOUCH_PAD_NUM8;								      //Touch senor on Touch8 (GPIO4)

// static const gpio_num_t PIN_DRV_EN = GPIO_NUM_17;

static const gpio_num_t PIN_SEL_EN = GPIO_NUM_17;
static const gpio_num_t PIN_PUMP_EN = GPIO_NUM_21;
static const gpio_num_t PIN_PUMP_SPEED = GPIO_NUM_19;
static const gpio_num_t PIN_LS = GPIO_NUM_5;
static const gpio_num_t PIN_HS = GPIO_NUM_18;
static const gpio_num_t PIN_ADCMUX1 = GPIO_NUM_15; // MUX selection 1 ... 3
static const gpio_num_t PIN_ADCMUX2 = GPIO_NUM_13;
static const gpio_num_t PIN_ADCMUX3 = GPIO_NUM_14;
// static const gpio_num_t PIN_HX710_OUT = GPIO_NUM_26;
// static const gpio_num_t PIN_HX710_SCK = GPIO_NUM_27;
static const gpio_num_t PIN_AHT20_EN = GPIO_NUM_27; // power pin for AHT20

static const adc_channel_t PIN_ADC_MUX = ADC_CHANNEL_6; // Analog mux signal
static const gpio_num_t PIN_PWM_MUX = GPIO_NUM_23;        // pwm mux signal
static const gpio_num_t PIN_ADC_MUX_EN = GPIO_NUM_32;
static const gpio_num_t PIN_PWM_MUX_EN = GPIO_NUM_22;

static const gpio_num_t PIN_I2C_SDA = GPIO_NUM_16; // GPIO16
static const gpio_num_t PIN_I2C_SCL = GPIO_NUM_4; // GPIO4

#endif   

//define hardware peripheries
#define CAPHUMSENSE_ENABLE
// #define FDC1004_ENABLE
// #define HX710_ENABLE
#define AHT20_ENABLE
/* #define MIFLORA_ENABLE */


// global variables
extern int32_t g_ai32Analyser[];

// LED functions
extern void led_init(void);
extern void led_set(uint32_t ui32LEDNumber, led_status_t eLedStatus);

// Buzzer functions
extern void buzzer_init(void);
extern bool bBuzzer_beep(uint32_t uiLevel);
extern bool bBuzzer_beep_times(uint32_t uiTimes);

// Sensing functions
extern void adcTouch_init();
uint32_t ui32AdcTouch_readAdcMux(adc_mux_t eAdcChannel, uint32_t ui32NbMean);
uint32_t ui32AdcTouch_readPwmMux(adc_mux_t eAdcChannel, uint32_t ui32TimeMs);
extern uint32_t ui32BattVolt_read(void);
extern uint32_t ui32BattLevel_read(void);
extern uint32_t ui32EspVolt_read(void);
extern uint32_t ui32SolarVolt_read(void);
extern float fTemp_read(void);
extern uint32_t ui32Charge_read(void);
extern esp_err_t erLevel_readPerc(int32_t *value);
extern uint32_t ui32Level_readMl(void);
extern esp_err_t ui32HX710_read(int32_t *value);
extern void HX710_init(void);
// Touch functions
// extern void touch_init1(void);
// extern void touch_deinit1(void);
// extern uint32_t ui32Level_read(void);
// extern void humidity_init(void);
// extern uint32_t ui32Humidity_count(uint32_t ui32Channel);
bool bHumidity_check(uint32_t ui32Channel);

// Powerstage functions
extern bool bPowerstage_init(void);
extern void powerstage_deinit(void);
extern esp_err_t motor_run(uint32_t ui32RunTime); // bool motor_run(uint32_t iRunTime, uint32_t iBackEMF, motor_direction_t eDirection);
extern esp_err_t selector_run(uint32_t ui32Speedbool, motor_direction_t eDirection);
esp_err_t selector_stop(void);
extern esp_err_sel_t selector_set(uint32_t ui32selPos);
extern uint32_t selector_get(void);
static void IRAM_ATTR timer0_callback(void *arg);
extern esp_err_t selector_runTime(uint32_t ui32Time, motor_direction_t eDirection);
extern esp_err_pump_t pump_runTime(uint32_t ui32Time, motor_direction_t eDirection, uint32_t ui32PumpNb);
extern esp_err_pump_t pump_runAmount(uint32_t ui32Amount, motor_direction_t eDirection, int32_t ui32PumpNb);
extern esp_err_sel_t selector_setAngle(int32_t i32SetAngle, bool bInitTLV, uint32_t ui32SelNb);
extern esp_err_sel_t selector_setPos(int32_t i32SetPos);
extern esp_err_sel_t selector_caliPos(void);
uint32_t ui32ReadVariable();

#endif /* MAIN_PERIPHERY_LH_H_ */
