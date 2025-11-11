#include "Stepper.h"

Motor * motors[4];

void initStepper(Motor* M1, Motor* M2, Motor* M3, Motor* M4){
    motors[0] = M1;
    motors[1] = M2;
    motors[2] = M3;
    motors[3] = M4;
   
    for(uint8_t i=0;i<4;i++){
      //PIN STEP
      gpio_reset_pin(motors[i]->STEP_pin);
      gpio_set_direction(motors[i]->STEP_pin,GPIO_MODE_OUTPUT);
      //PIN DIR
      gpio_reset_pin(motors[i]->DIR_pin);
      gpio_set_direction(motors[i]->DIR_pin,GPIO_MODE_OUTPUT);
      //PIN HOME POS
      gpio_reset_pin(motors[i]->HOME_pin);
      gpio_set_direction(motors[i]->HOME_pin,GPIO_MODE_INPUT);
      //gpio_pulldown_en(motors[i]->HOME_pin);
  }
}

uint32_t sqrt32 (uint32_t n){
   uint32_t c = (1<<15);//15 iteration
  uint32_t g = 0x8000;
  while(1)
  {
    if (g*g > n)
        g ^= c;
    c >>= 1;
    if (!c)
        return g;
    g |= c;
  }
}
void OneStep(uint8_t pin){
  gpio_set_level(pin,HIGH);
  ets_delay_us(5000);
  gpio_set_level(pin,LOW);
  ets_delay_us(5000); 
}
void home(Motor *m){
  OneStep(m->STEP_pin);
  OneStep(m->STEP_pin);
  do{
    OneStep(m->STEP_pin);
  }while(gpio_get_level(m->HOME_pin));
  OneStep(m->STEP_pin);
  // OneStep(m->STEP_pin);
}

void turn(uint16_t degree, uint8_t dir, Motor *m){
  uint16_t step = DEG_TO_STEP(degree);//nombre de steps à tourner
  uint32_t temp, temp2;
	//Calcul du délais pour vitesse cible
	temp = (uint32_t) m->speed * Q15;
	uint16_t FinalDelay = (uint16_t) (CNST_RATIO2 / temp); 
  //calcul du nombre de steps pour l'accélération
  temp = (uint32_t) m->speed * m->speed * Q20;
	temp2 = (uint32_t) ((uint32_t) ((uint32_t) 2u * (STEP_ANGLE_Q20)) * m->accel);
	uint16_t StepsForAccel = (uint16_t) (temp / temp2);
  //calcul du premier terme de la suite
  uint32_t C0 = ((sqrt32((2u * (uint32_t) (STEP_ANGLE_Q20) * 16384u) / m->accel) / 2u) * CNST_RATIO1 ) / 256u;

  uint32_t Delay = C0/256u;
  uint32_t Cn = C0;
  stateMotor PhaseMVT = ACCEL;
  //direction
  gpio_set_level(m->DIR_pin,dir);
  
  uint16_t stepCounter = 1;
  for(uint16_t i = 0;i<step;i++){
    //bouge de 1 step
    gpio_set_level(m->STEP_pin,HIGH);
    ets_delay_us(1);//1us de delais
    gpio_set_level(m->STEP_pin,LOW);
    ets_delay_us(Delay);
    switch(PhaseMVT){
      case ACCEL:
        if (i < StepsForAccel) {//si en phase d'acceleration
          temp = ((uint32_t) (4u) * stepCounter) + 1u;
          temp2 = (2u * Cn + (temp / 2u)) / temp;
          Cn = (uint32_t) Cn - temp2;
          stepCounter++;
          if(Cn/256u>FinalDelay){
            Delay = Cn/256u;
          }else{
            PhaseMVT = STABLE;
            Delay = FinalDelay;
            StepsForAccel = stepCounter;
          }
        }else {//si fini
          Delay = FinalDelay;
			    PhaseMVT = STABLE;
		    }

        break;
      case STABLE:
        if(i+1 >= step-StepsForAccel){
          PhaseMVT=DECEL;
          }
        break;
        
      case DECEL:
          temp = ((uint32_t) (4u) * stepCounter) + 1u;
          temp2 = (2u * Cn + (temp / 2u)) / temp;
          Cn = (uint32_t) Cn + temp2;
          stepCounter--;
          Delay = Cn/256u;
        break;
    }   
  }
}

void turnMultiple(uint16_t degree, uint8_t dir, Motor *m1, Motor *m2){
  uint16_t step = DEG_TO_STEP(degree);//nombre de steps à tourner
  uint32_t temp, temp2;
	//Calcul du délais pour vitesse cible
	temp = (uint32_t) m1->speed * Q15;
	uint16_t FinalDelay = (uint16_t) (CNST_RATIO2 / temp); 
  //calcul du nombre de steps pour l'accélération
  temp = (uint32_t) m1->speed * m1->speed * Q20;
	temp2 = (uint32_t) ((uint32_t) ((uint32_t) 2u * (STEP_ANGLE_Q20)) * m1->accel);
	uint16_t StepsForAccel = (uint16_t) (temp / temp2);
  //calcul du premier terme de la suite
  uint32_t C0 = ((sqrt32((2u * (uint32_t) (STEP_ANGLE_Q20) * 16384u) / m1->accel) / 2u) * CNST_RATIO1 ) / 256u;
  uint32_t Delay = C0/256u;
  uint32_t Cn = C0;
  stateMotor PhaseMVT = ACCEL;

  gpio_set_level(m1->DIR_pin,dir);
  gpio_set_level(m2->DIR_pin,!dir);
  uint16_t stepCounter = 1;
  //turn
  for(uint16_t i = 0;i<step;i++){
    //bouge de 1 step
    gpio_set_level(m1->STEP_pin,HIGH);
    gpio_set_level(m2->STEP_pin,HIGH);
    ets_delay_us(1);//1us de delais
    // sendByte(data);
    gpio_set_level(m1->STEP_pin,LOW);
    gpio_set_level(m2->STEP_pin,LOW);
    ets_delay_us(Delay);

    switch(PhaseMVT){
      case ACCEL:
        if (i < StepsForAccel) {//si en phase d'acceleration
          temp = ((uint32_t) (4u) * stepCounter) + 1u;
          temp2 = (2u * Cn + (temp / 2u)) / temp;
          Cn = (uint32_t) Cn - temp2;
          stepCounter++;
          if(Cn/256u>FinalDelay){
            Delay = Cn/256u;
          }else{
            PhaseMVT = STABLE;
            Delay = FinalDelay;
            StepsForAccel = stepCounter;
          }
        }else {//si fini
          Delay = FinalDelay;
			    PhaseMVT = STABLE;
		    }

        break;
      case STABLE:
        if(i+1 >= step-StepsForAccel){
          PhaseMVT=DECEL;        
          }
        break;
        
      case DECEL:
          temp = ((uint32_t) (4u) * stepCounter) + 1u;
          temp2 = (2u * Cn + (temp / 2u)) / temp;
          Cn = (uint32_t) Cn + temp2;
          stepCounter--;
          Delay = Cn/256u;
        break;
    }   
  }
}