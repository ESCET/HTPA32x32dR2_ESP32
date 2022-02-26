#ifndef _CONNECT_H_
#define _CONNECT_H_
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

extern xSemaphoreHandle initSemaphore;

void wifiInit(void *params);
void resetwifi( );
 
#endif