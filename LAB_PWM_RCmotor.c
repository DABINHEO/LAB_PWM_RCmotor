/**
******************************************************************************
* @author  HEODABIN
* @Date		 2022-10-31
* @brief   Embedded Controller:  LAB: PWM_RCmotor
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecEXTI.h"

#define PWM_PIN 	6
#define BUTTON_PIN 	13

PWM_t pwm;
int flag = 0;
int duty_count = 0;

void setup(void){

	// Initialiization --------------------------------------------------------
	RCC_PLL_init();                         		// System Clock = 84MHz
	
	PWM_init(&pwm, GPIOA, PWM_PIN);							// PWM Initialiization
	PWM_period_ms(&pwm, 20);										// set PWM period

	TIM_INT_init(TIM2,1);												// TIMER2 Initialiization
	TIM_period_us(TIM2, 100);										// Timer period 100usec
	TIM_INT_enable(TIM2);												
	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  			// calls RCC_GPIOC_enable()
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 2);			// EXTI(button) Initialiization
}


int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}


void TIM2_IRQHandler(void){
	int count;
	if(is_UIF(TIM2)){ // update interrupt flag
		//Create the code to rotate Servo motor by 500ms
		count++;
		if(count > 5000){
			count = 0;
			if(flag == 0) duty_count++;
			if(flag == 1) duty_count--;
			if(duty_count > 17 & flag == 0)	 flag = 1;		//0 to 180 degree
			if(duty_count < 1 & flag == 1)   flag = 0;		//180 to 0 degree
			PWM_duty(&pwm, 0.00556*duty_count + 0.025);		//0.00556 <- 1/180
		}
		clear_UIF(TIM2);// clear by writing 0	
	}
}

void EXTI15_10_IRQHandler(void) { 
	if (is_pending_EXTI(BUTTON_PIN)) {		// reset rotating
		duty_count = 0;
		flag = 0;
		clear_pending_EXTI(BUTTON_PIN); 		// cleared by writing '1'
		for(int i = 0; i < 500000; i++){}		// button push delay
	}
}