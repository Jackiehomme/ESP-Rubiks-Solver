#include "rotaryEncoder.h"
#include <rom/ets_sys.h>//temp pour délais
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
uint8_t switch_state;
uint16_t rotary_pos;
uint8_t rotFlag_A;
uint8_t rotFlag_B;

SemaphoreHandle_t updateIHM;


//ISR SW
static void IRAM_ATTR ISR_SWITCH(void* arg){
    switch_state = 1;
    xSemaphoreGive(updateIHM);
}

//ISR SW A
static void IRAM_ATTR ISR_ROTARY(void* arg){
    switch_state = 0;
    xSemaphoreGive(updateIHM);
    //rotary_pos++;
}

void initRotaryEncoder(){
    updateIHM = xSemaphoreCreateBinary();
    switch_state = 0;
    rotary_pos = 0;
    //switch
    gpio_reset_pin(P_SW);
    gpio_set_direction(P_SW, GPIO_MODE_INPUT);
    gpio_pulldown_en(P_SW);
    gpio_pullup_dis(P_SW);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_set_intr_type(P_SW, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(P_SW, ISR_SWITCH, (void*) P_SW);

    //Rotary
    //Pin A 
    gpio_reset_pin(P_RA);
    gpio_set_direction(P_RA, GPIO_MODE_INPUT);
    gpio_pulldown_en(P_RA);
    gpio_pullup_dis(P_RA);
    
    //Pin B 
    gpio_reset_pin(P_RB);
    gpio_set_direction(P_RB, GPIO_MODE_INPUT);
    gpio_pulldown_en(P_RB);
    gpio_pullup_dis(P_RB);

    //ISR ROTARY
    gpio_set_intr_type(P_RB, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(P_RB, ISR_ROTARY, (void*) P_RB);
    
}


