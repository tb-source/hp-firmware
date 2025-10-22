/*
 * tca6408.h
 *
 *  Created on: 23.08.2023
 *      Author: T.Bretzke
 */

#ifndef MAIN_TCA6408_H_
#define MAIN_TCA6408_H_


#include <stdint.h>
#include <stddef.h>
#include "esp_system.h"
#include "driver/i2c_master.h"
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "periphery.h"
#include "esp_log.h"
#include <math.h>


#define TCA6408_ADDR               0x20


extern void tca_init(void);
// extern int32_t i32TLV_getAngle(void);

#endif /* MAIN_TCA6408_H_ */
