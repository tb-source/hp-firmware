#ifndef SUPABASE_H
#define SUPABASE_H

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "types.h"
#include "storage.h"

/* Supabase Edge Function endpoint (GET config + POST status). */
#define SUPABASE_EDGE_URL "https://mfzpmltlrivvoztuhjkc.supabase.co/functions/v1/esp-device"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t supabase_get_config(const credentials_t *psCredentials,
							  deviceData_t *psDeviceData,
							  char *pacResponseBuf,
							  size_t uiResponseBufSize,
							  int *piHttpStatus);

esp_err_t supabase_wifiConnect(const char *pacSsid, const char *pacPassword);
esp_err_t supabase_wifiDisconnect(void);

esp_err_t supabase_post_status(const credentials_t *psCredentials,
							   const deviceData_t *psDeviceData,
							   const char *pacFirmwareVersion,
							   bool *pbUpdated,
							   char *pacResponseBuf,
							   size_t uiResponseBufSize,
							   int *piHttpStatus);

#ifdef __cplusplus
}
#endif

#endif