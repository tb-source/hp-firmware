/*
 * communication.h
 *
 *  Created on: 04.12.2021
 *      Author: tobby
 */

#ifndef MAIN_COMMUNICATION_H_
#define MAIN_COMMUNICATION_H_

#include <stdio.h>
#include "watering.h"
#include <string.h>
#include "storage.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "periphery.h"
#include "watering.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "log.h"

extern void uart_init(void);

#endif /* MAIN_COMMUNICATION_H_ */
