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

extern esp_err_t deviceDataJson_parse(deviceData_t *psDeviceData, const char *pacJson, bool bSyncTime);
extern char *deviceDataJson_serialize(const deviceData_t *psDeviceData);
extern const char* pacData_send_receive(char* pGetData, deviceData_t* peDevice_data);
extern void data_logDeviceData(const char *pacTag, const deviceData_t *psData);
extern void data_logWateringData(const char *pacTag, const wateringData_t *psData);

#endif /* MAIN_HELPER_H_ */
