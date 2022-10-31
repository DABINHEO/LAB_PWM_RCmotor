/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
* 
******************************************************************************
*/


#include "ecPWM.h"

/* PWM Configuration */

void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin){
// 0. Match Output Port and Pin for TIMx 	
		pwm->port = port;
		pwm->pin  = pin;
		PWM_pinmap(pwm);
		TIM_TypeDef *TIMx = pwm->timer;
		int CHn = pwm->ch;	

// 1. Initialize GPIO port and pin as AF
		GPIO_init(port, pin, AF);  		 // Alternate Fuction = 2
		GPIO_ospeed(port, pin, High);  // high speed = 3
		GPIO_pupd(GPIOA, pin, 0);			 // No pull-up, pull-down
	
// 2. Configure GPIO AFR by Pin num.				
	int val;
	if (TIMx == TIM1 || TIMx == TIM2) val = 1;												//AFR=1 for TIM1,TIM2
	else if (TIMx == TIM3 || TIMx == TIM4 || TIMx == TIM5) val = 2;		//AFR=2 for TIM3,TIM4 ,TIM5
	else if (TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11) val = 3;	//AFR=3 for TIM9,TIM10,TIM11
	
	//  AFR[0] for pin: 0~7,     AFR[1] for pin 8~15
	if(pin < 8)					port->AFR[0] |= val <<(4*pin);
	else if(pin < 16)		port->AFR[1] |= val <<(4*(pin-8));


// 3. Initialize Timer 
		TIM_init(TIMx, 1);						// with default msec=1 value.
		TIMx->CR1 &= ~TIM_CR1_CEN;		// disable counter
// 3-2. Direction of Counter
		TIMx->CR1 &= ~TIM_CR1_DIR;    // Counting direction: 0 = up-counting, 1 = down-counting
	
// 4. Configure Timer Output mode as PWM
	uint32_t ccVal=TIMx->ARR/2;  // default value  CC=ARR/2
	if(CHn == 1){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear ouput compare mode bits for channel 1
		TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // OC1M = 110 for PWM Mode 1 output on ch1. #define TIM_CCMR1_OC1M_1 (0x2UL << TIM_CCMR1_OC1M_Pos)
		TIMx->CCMR1	|= TIM_CCMR1_OC1PE;                     // Output 1 preload enable (make CCR1 value changable)
		TIMx->CCR1  = ccVal; 																// Output Compare Register for channel 1 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC1P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC1E;												// Enable output for ch1
	}
	else if(CHn == 2){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;                     // Clear ouput compare mode bits for channel 2
		TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // OC1M = 110 for PWM Mode 1 output on ch2
		TIMx->CCMR1	|= TIM_CCMR1_OC2PE;                     // Output 1 preload enable (make CCR2 value changable)	
		TIMx->CCR1  = ccVal; 																// Output Compare Register for channel 2 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC2P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC2E;												// Enable output for ch2
	}
	else if(CHn == 3){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;                     // Clear ouput compare mode bits for channel 3
		TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // OC1M = 110 for PWM Mode 1 output on ch3
		TIMx->CCMR2	|= TIM_CCMR2_OC3PE;                     // Output 1 preload enable (make CCR3 value changable)	
		TIMx->CCR1  = ccVal; 																// Output Compare Register for channel 3 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC3P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC3E;												// Enable output for ch3
	}
	else if(CHn == 4){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;                     // Clear ouput compare mode bits for channel 4
		TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // OC1M = 110 for PWM Mode 1 output on ch4
		TIMx->CCMR2	|= TIM_CCMR2_OC4PE;                     // Output 1 preload enable (make CCR3 value changable)	
		TIMx->CCR1  = ccVal; 																// Output Compare Register for channel 4 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC4P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC4E;												// Enable output for ch4
	}	

// 5. Enable Timer Counter
	if(TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;					// Main output enable (MOE): 0 = Disable, 1 = Enable	
	TIMx->CR1  |= TIM_CR1_CEN;  													// Enable counter
}


void PWM_period_ms(PWM_t *pwm, uint32_t msec){
	TIM_TypeDef *TIMx = pwm->timer;
	TIM_period_ms(TIMx, msec);
}

void PWM_period_us(PWM_t *pwm, uint32_t usec){
	TIM_TypeDef *TIMx = pwm->timer;
	TIM_period_us(TIMx, usec);
}


void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms){ 
	int CHn = pwm->ch;
	uint32_t fsys = 0;
	uint32_t psc = pwm->timer->PSC;
	
	// Check System CLK: PLL or HSI
	if((RCC->CFGR & (3<<0)) == 2)      { fsys = 84000; }  // for msec 84MHz/1000
	else if((RCC->CFGR & (3<<0)) == 0) { fsys = 16000; }
	
	float fclk = fsys/(psc+1);													// fclk=fsys/(psc+1);
	uint32_t ccval = pulse_width_ms *fclk - 1;					// pulse_width_ms *fclk - 1;
	
	switch(CHn){
		case 1: pwm->timer->CCR1 = ccval; break;
		case 2: pwm->timer->CCR1 = ccval; break;
		case 3: pwm->timer->CCR1 = ccval; break;
		case 4: pwm->timer->CCR1 = ccval; break;
		default: break;
	}
}


void PWM_duty(PWM_t *pwm, float duty) {                 //  duty=0 to 1	
	float ccval = (pwm->timer->ARR+1)*duty - 1;    				// (ARR+1)*dutyRatio - 1          
	int CHn = pwm->ch;
  	
	switch(CHn){
		case 1: pwm->timer->CCR1 = ccval; break;
		case 2: pwm->timer->CCR1 = ccval; break;
		case 3: pwm->timer->CCR1 = ccval; break;
		case 4: pwm->timer->CCR1 = ccval; break;
		default: break;
	}
}


// DO NOT MODIFY HERE
void PWM_pinmap(PWM_t *pwm){
   GPIO_TypeDef *port = pwm->port;
   int pin = pwm->pin;
   
   if(port == GPIOA) {
      switch(pin){
         case 0 : pwm->timer = TIM2; pwm->ch = 1; break;
         case 1 : pwm->timer = TIM2; pwm->ch = 2; break;
         case 5 : pwm->timer = TIM2; pwm->ch = 1; break;
         case 6 : pwm->timer = TIM3; pwm->ch = 1; break;
         //case 7: PWM_pin->timer = TIM1; PWM_pin->ch = 1N; break;
         case 8 : pwm->timer = TIM1; pwm->ch = 1; break;
         case 9 : pwm->timer = TIM1; pwm->ch = 2; break;
         case 10: pwm->timer = TIM1; pwm->ch = 3; break;
         case 15: pwm->timer = TIM2; pwm->ch = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: PWM_pin->timer = TIM1; PWM_pin->ch = 2N; break;
         //case 1: PWM_pin->timer = TIM1; PWM_pin->ch = 3N; break;
         case 3 : pwm->timer = TIM2; pwm->ch = 2; break;
         case 4 : pwm->timer = TIM3; pwm->ch = 1; break;
         case 5 : pwm->timer = TIM3; pwm->ch = 2; break;
         case 6 : pwm->timer = TIM4; pwm->ch = 1; break;
         case 7 : pwm->timer = TIM4; pwm->ch = 2; break;
         case 8 : pwm->timer = TIM4; pwm->ch = 3; break;
         case 9 : pwm->timer = TIM4; pwm->ch = 4; break;
         case 10: pwm->timer = TIM2; pwm->ch = 3; break;
         
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : pwm->timer = TIM3; pwm->ch = 1; break;
         case 7 : pwm->timer = TIM3; pwm->ch = 2; break;
         case 8 : pwm->timer = TIM3; pwm->ch = 3; break;
         case 9 : pwm->timer = TIM3; pwm->ch = 4; break;
         
         default: break;
      }
   }
	 // TIM5 needs to be added, if used.
}
