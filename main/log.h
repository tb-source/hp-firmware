/*
 * log.h
 *
 *  Created on: 07.10.2023
 *      Author: tobby
 */

#ifndef MAIN_LOG_H_
#define MAIN_LOG_H_

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "periphery.h"

typedef enum{
    LOG_TYPE_PERIPHERY,				    
	LOG_TYPE_WATERING,	                
	LOG_TYPE_ERROR,	
    LOG_TYPE_UNKNOWN,		            
}log_type_t;

typedef enum{
    LOG_ERR_TYPE_PERIPHERY, 
    LOG_ERR_TYPE_WATERING, 
    LOG_ERR_TYPE_ERROR,
} log_error_type_t;


extern void log_readData(log_type_t eType);
extern void log_clearData(log_type_t eType);
extern void log_peripherieData(void);
extern void log_wateringData(uint32_t wateringChannel, uint32_t wateringEvent, uint32_t wateringAmount);
extern void log_errorData(log_error_type_t eErrorType, char* pacErrorMessage);

#endif /* MAIN_LOG_H_ */