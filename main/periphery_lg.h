/*
 * periphery_lg.h
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 */

#ifndef MAIN_PERIPHERY_LG_H_
#define MAIN_PERIPHERY_LG_H_

#include <stdio.h>
#include "sdkconfig.h"
#include "sdkconfig.h"
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

#define LG_SWC_V01 // Lasting garden board

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
  ERR_PUMP_CNT,
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
  ADC_MUX_VOLT_3V3 = 0,   // S0
  ADC_MUX_TEMP_PCB = 1,   // S1
  ADC_MUX_VOLT_SOLAR = 2, // S2
  ADC_MUX_VOLT_BATT = 3,  // S3
  ADC_MUX_CUR_SEL1 = 4,    // S4
  ADC_MUX_CUR_SEL2 = 5,   // S5
  ADC_MUX_CUR_PUMP1 = 6,  // S6
  ADC_MUX_CUR_PUMP2 = 7,  // S7
} adc_mux_t;

#ifdef LG_SWC_V01
static const gpio_num_t PIN_LEDS[] = {GPIO_NUM_25, GPIO_NUM_27};
static const gpio_num_t PIN_DEBUG = GPIO_NUM_2;
static const gpio_num_t PIN_BUTTON = GPIO_NUM_39; // button
static const gpio_num_t PIN_CHRG = GPIO_NUM_36;   // GPIO23

static const gpio_num_t PIN_PUMP_SPEED[] = {GPIO_NUM_5, GPIO_NUM_17};

static const gpio_num_t PIN_LS = GPIO_NUM_18;
static const gpio_num_t PIN_HS = GPIO_NUM_19;

static const gpio_num_t PIN_ADCMUX1 = GPIO_NUM_15; // MUX selection 1 ... 3
static const gpio_num_t PIN_ADCMUX2 = GPIO_NUM_13;
static const gpio_num_t PIN_ADCMUX3 = GPIO_NUM_14;

static const gpio_num_t PIN_HX710_EN = GPIO_NUM_33;
static const gpio_num_t PIN_HX710_OUT = GPIO_NUM_35;
static const gpio_num_t PIN_HX710_SCK = GPIO_NUM_32;

// static const gpio_num_t PIN_AHT20_EN = GPIO_NUM_27; // power pin for AHT20

static const gpio_num_t PIN_SENS_EN = GPIO_NUM_26; // GPIO2
static const adc_channel_t PIN_ADC_MUX = ADC_CHANNEL_6; // Analog mux signal

static const gpio_num_t PIN_I2C_SDA = GPIO_NUM_16; // GPIO16
static const gpio_num_t PIN_I2C_SCL = GPIO_NUM_4; // GPIO4


#endif                                             // LG_SWC_V01

//define hardware peripheries
// #define CAPHUMSENSE_ENABLE
// #define FDC1004_ENABLE
#define HX710_ENABLE
// #define AHT20_ENABLE
/* #define MIFLORA_ENABLE */
#define VERTILIZING_ENABLE


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
extern void debug_init(void);
extern void adcTouch_init();
uint32_t ui32AdcTouch_readAdcMux(adc_mux_t eAdcChannel, uint32_t ui32NbMean);
extern uint32_t ui32BattVolt_read(void);
extern uint32_t ui32EspVolt_read(void);
extern uint32_t ui32SolarVolt_read(void);
extern float fTemp_read(void);
extern uint32_t ui32Charge_read(void);
extern esp_err_t erLevel_readPerc(int32_t *value);
extern esp_err_t ui32HX710_read(int32_t *value);
extern void HX710_init(void);
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
extern esp_err_pump_t pump_runTime(uint32_t ui32Time, motor_direction_t eDirection, int32_t ui32PumpNb);
extern esp_err_pump_t pump_runTimeOpenLoop(uint32_t ui32Time, motor_direction_t eDirection, int32_t ui32PumpNb, int32_t i32EmfVoltageMv);
extern esp_err_sel_t selector_setAngle(int32_t i32SetAngle, bool bInitTLV, int32_t i32SelNb);
extern esp_err_sel_t selector_setPos(int32_t i32SetPos);
extern esp_err_sel_t selector_caliPos(void);
uint32_t ui32ReadVariable();

#endif /* MAIN_PERIPHERY_LG_H_ */
