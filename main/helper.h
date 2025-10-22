/*
 * helper.h
 *
 *  Created on: 07.10.2023
 *      Author: tobby
 */

#ifndef MAIN_HELPER_H_
#define MAIN_HELPER_H_

#include "types.h"
#include "watering.h"
#include <esp_err.h>


typedef enum {
    DATA_APPTOPOT = '1', 
    DATA_POTTOAPP = '2',
    DATA_ERROR = '3',
    DATA_ANALYSER = '4',
}data_type_t;

typedef enum {
    DATAFLOW_IDLE = '0',
    DATAFLOW_SEND = '1',
    DATAFLOW_RECEIVE = '3',
    DATAFLOW_END = '4',
    DATAFLOW_ERROR = '5',
}data_state_t;

extern esp_err_t data_convert_read(deviceData_t* device_data, char* json_data);
extern char* data_convert_write(deviceData_t device_data);
extern const char* pacData_send_receive(char* pGetData, deviceData_t* peDevice_data);

#endif /* MAIN_HELPER_H_ */
