#ifndef BT_task
#define BT_task
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
extern void initBT(char* setName);
extern uint8_t * getBTbuffer();
extern SemaphoreHandle_t dataAvailable;
#endif