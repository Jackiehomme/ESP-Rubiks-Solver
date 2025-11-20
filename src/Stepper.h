#ifndef Stepper
#define Stepper
#include <stdint.h>
#include "driver/gpio.h"
#include <stdio.h>
#include <rom/ets_sys.h>//temp pour délais
#include "driver/uart.h"

//HELPFUL
#define CW      0
#define CCW     1
#define HIGH    1
#define LOW     0

#define MOTOR_STEPS     200
#define MICROSTEPING    1
#define STEP_PER_T      (MOTOR_STEPS*MICROSTEPING)
#define STEP_ANGLE_Q15  1029u
#define STEP_ANGLE_Q20  32941u
#define DEG_TO_STEP(d)  ((uint16_t)((((d)*STEP_PER_T)/360)))

#define C0_COMPENSATION_FACTOR 173u//0.676 en Q8
#define Q8_RATIO_RPM_to_RADpS (((2*3.14159265359*MICROSTEPING)/60)*256)
#define RPM_TO_RADpS(x)	(((x)*Q8_RATIO_RPM_to_RADpS)/256)

#define TIMER 1000000u
#define Q20 1048576UL
#define Q15 32768u

#define CNST_RATIO1 (uint32_t) ((((uint32_t) TIMER * C0_COMPENSATION_FACTOR) / 256u))
#define CNST_RATIO2 (uint32_t) (STEP_ANGLE_Q15* ((uint32_t) TIMER))

/*
vitesse définie en tour/s
tour/s en step/ms : (60*1000/(STEP_PER_T*vitesse)
minutes(en ms)/ 
*/
#define TO_STEP_MS(a) (120000/(STEP_PER_T*(a)))

// #define DEBUG_STEPPER


typedef enum {ACCEL, STABLE, DECEL} stateMotor;

typedef struct{
  uint8_t HOME_pin;
  uint8_t STEP_pin;
  uint8_t DIR_pin;
  uint16_t accel;
  uint16_t speed;
}Motor;

extern void initStepper(Motor* M1, Motor* M2, Motor* M3, Motor* M4);

void turn(uint16_t degree, uint8_t dir, Motor *m);
void turnMultiple(uint16_t degree, uint8_t dir, Motor *m1, Motor *m2);
void home(Motor *m);
#endif