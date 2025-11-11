
#include "Servo.h"
// #define SERVOA             13  // GPIO pour le premier servo
// #define SERVOB             14  // GPIO pour le deuxième servo

uint8_t pinServoA;
uint8_t pinServoB;

mcpwm_cmpr_handle_t comparatorA = NULL;
mcpwm_cmpr_handle_t comparatorB = NULL;

static inline uint32_t angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void initServo(uint8_t _pinServoA,uint8_t _pinServoB){
    pinServoA = _pinServoA;
    pinServoB = _pinServoB;
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    // mcpwm_cmpr_handle_t comparatorA = NULL;
    mcpwm_comparator_config_t comparator_configA = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_configA, &comparatorA));

    mcpwm_gen_handle_t generatorA = NULL;
    mcpwm_generator_config_t generator_configA = {
        .gen_gpio_num = pinServoA,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_configA, &generatorA));

    // mcpwm_cmpr_handle_t comparatorB = NULL;
    mcpwm_comparator_config_t comparator_configB = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_configB, &comparatorB));

    mcpwm_gen_handle_t generatorB = NULL;
    mcpwm_generator_config_t generator_configB = {
        .gen_gpio_num = pinServoB,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_configB, &generatorB));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorA, angle_to_compare(0)));//Angle de base
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorB, angle_to_compare(0)));//Angle de base

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generatorA,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generatorA,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorA, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generatorB,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generatorB,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparatorB, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void moveServo(uint8_t Servo, int16_t degree){
    if(Servo == pinServoA){         
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorA, angle_to_compare(degree)));
    }
    else if(Servo == pinServoB){
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparatorB, angle_to_compare(degree)));
    }
}