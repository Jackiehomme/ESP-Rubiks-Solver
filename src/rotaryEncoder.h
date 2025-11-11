#ifndef ROTARY_ENCODER_H_
#define ROTARY_ENCODER_H_

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdint.h>


#define P_SW 25
#define P_RA 26
#define P_RB 27

#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL  ((1ULL<<P_SW))
void initRotaryEncoder();

extern uint8_t switch_state;
extern uint16_t rotary_pos;
extern SemaphoreHandle_t updateIHM;
#endif

