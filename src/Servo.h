// #ifndef Servo
// #define Servo
#include <stdint.h>
#include "driver/mcpwm_prelude.h"

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

void moveServo(uint8_t Servo, int16_t degree);
void initServo(uint8_t _pinServoA,uint8_t _pinServoB);

// #endif