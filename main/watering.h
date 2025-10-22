/*
 * watering.h
 *
 *  Created on: 23.08.2023
 *      Author: tobby
 */

#ifndef MAIN_WATERING_H_
#define MAIN_WATERING_H_

#include "peripherie/button_sleep.h"
#include "provisioning.h"
#include "types.h"
#include <esp_log.h>
#include "time.h"
#include <sys/time.h>
#include "periphery.h"
#include "peripherie/wifi_time.h"
#include "log.h"

extern void watering_init(void);
extern deviceData_t eDeviceData_get(void);


#endif /* MAIN_WATERING_H_ */
