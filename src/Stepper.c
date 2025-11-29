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
    
    // 1. Conversion de l'angle en nombre de pas
    uint16_t step = DEG_TO_STEP(degree); // nombre de steps à tourner
    
    uint16_t effective_speed = (m->speed) / SLOWNESS_FACTOR;
    // 2. Calcul du délais unique pour la vitesse cible
    // La formule utilisée précédemment pour le délai final est conservée :
    // FinalDelay = CNST_RATIO2 / (m->speed * Q15)
    uint32_t temp = (uint32_t) effective_speed * Q15;
    uint16_t FinalDelay = (uint16_t) (CNST_RATIO2 / temp); 
    
    // 3. Définir la direction
    gpio_set_level(m->DIR_pin, dir);
    
    // 4. Délai de stabilisation après changement de direction
    ets_delay_us(10000); // Un petit délai, souvent nécessaire
    
    // 5. Boucle de mouvement à vitesse constante
    for(uint16_t i = 0; i < step; i++){
        
        gpio_set_level(m->STEP_pin, HIGH);
        ets_delay_us(10000);
        gpio_set_level(m->STEP_pin, LOW);
        ets_delay_us(10000);
        ets_delay_us(FinalDelay);
    }
}

void turnMultiple(uint16_t degree, uint8_t dir, Motor *m1, Motor *m2){
    
    // 1. Conversion de l'angle en nombre de pas (utilise les paramètres de m1 ou m2, car ils doivent être les mêmes)
    uint16_t step = DEG_TO_STEP(degree); // nombre de steps à tourner
    
    uint16_t effective_speed = (m1->speed) / SLOWNESS_FACTOR;
    // 2. Calcul du délais unique pour la vitesse cible (basé sur la vitesse de m1)
    // FinalDelay est le délai constant entre les pas.
    uint32_t temp = (uint32_t) effective_speed * Q15;
    uint16_t FinalDelay = (uint16_t) (CNST_RATIO2 / temp); 
    
    // 3. Définir les directions
    // Le code original utilise DIR et !DIR, suggérant une rotation opposée,
    // ce qui est typique pour un virage ou un mouvement de tourelle/châssis.
    gpio_set_level(m1->DIR_pin, dir);
    gpio_set_level(m2->DIR_pin, !dir);
    
    // 4. Délai de stabilisation après changement de direction
    ets_delay_us(10000); 
    
    // 5. Boucle de mouvement à vitesse constante
    for(uint16_t i = 0; i < step; i++){
        
        // Les deux moteurs bougent de 1 step SIMULTANÉMENT
        gpio_set_level(m1->STEP_pin, HIGH);
        ets_delay_us(10000);
        gpio_set_level(m2->STEP_pin, HIGH);
        
        ets_delay_us(10000); // 1us de delais pour la largeur de l'impulsion STEP
        // sendByte(data); // Laissez cette ligne en commentaire ou supprimez-la si elle est inutile
        
        gpio_set_level(m1->STEP_pin, LOW);
        ets_delay_us(10000);
        gpio_set_level(m2->STEP_pin, LOW);

        ets_delay_us(10000);
        // Délai constant appliqué aux deux moteurs
        ets_delay_us(FinalDelay);
    }
}