/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
* 
******************************************************************************
*/


#include "ecTIM.h"
#include "ecGPIO.h"

/* Timer Configuration */

void TIM_init(TIM_TypeDef* timerx, uint32_t msec){ 
	
// 1. Enable Timer CLOCK
	if(timerx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(timerx ==TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(timerx ==TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(timerx ==TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(timerx ==TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(timerx ==TIM9) RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	else if(timerx ==TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
	
	
// 2. Set CNT period
	TIM_period_ms(timerx,msec); 
	
	
// 3. CNT Direction
	timerx->CR1 &= ~TIM_CR1_DIR;					// Upcounter	
	
// 4. Enable Timer Counter
	timerx->CR1 |= 1<<0;		
}

void TIM_init2(TIM_TypeDef *timerx){
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//1MHz. (1kHz -> 84-1)
	timerx->PSC = 840-1; 									// Timer counter clock: 1MHz(1us)
	//1MHz. (1kHz -> 1000-1)
	timerx->ARR = 100-1; // 1000-1				// Set auto reload register to maximum (count up to 65535)
	timerx->DIER |= TIM_DIER_UIE;         // Enable Interrupt
	timerx->CR1 |=     1<<0;              // Enable counter
	
	NVIC_EnableIRQ(TIM2_IRQn);				    // TIM2_IRQHandler Enable
	NVIC_SetPriority(TIM2_IRQn,2);        // TIM2_IRQHandler Set priority as 2
	
}

//	Q. Which combination of PSC and ARR for msec unit?
// 	Q. What are the possible range (in sec ?)
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec){   
	// Period usec = 1 to 1000

	// 1us(1MHz, ARR=1 . sec = ARR / Hz) to 65msec (ARR=0xFFFF)
	//uint16_t prescaler = 84;
	//uint16_t ARRval= (84/(prescaler)*usec);  // 84MHz/1000000 us
	
	//TIMx->PSC = prescaler-1;					
	//TIMx->ARR = ARRval-1;
	
	uint16_t PSCval;
	uint32_t Sys_CLK;
	
	if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL){
		Sys_CLK = 84000000;
	}
	else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI){
		Sys_CLK = 16000000;
	}
	
	if(TIMx == TIM2 || TIMx == TIM5){
		uint32_t ARRval;
		
		PSCval = Sys_CLK / 1000000;
		ARRval = Sys_CLK / PSCval / 1000000 * usec;
		TIMx->PSC = PSCval - 1;
		TIMx->ARR = ARRval - 1;
	}
	else{
		uint16_t ARRval;
		
		PSCval = Sys_CLK / 1000000;
		ARRval = Sys_CLK / PSCval / 1000000 * usec;
		TIMx->PSC = PSCval - 1;
		TIMx->ARR = ARRval - 1;
	}
}




void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec) {
	// Period msec = 1 to 6000
	
	
	// 0.1ms(10kHz, ARR=1) to 6.5sec (ARR=0xFFFF)
	//uint16_t prescaler = 840;		// 84000KhZ / 840 = 100KhZ
	//uint16_t ARRval=100*msec;  			// 0.1ms * how count

	//TIMx->PSC = 840-1;					
	//TIMx->ARR = 100-1;							
	
	uint16_t PSCval;
	uint32_t Sys_CLK;
	
	if((RCC->CFGR & RCC_CFGR_SW_PLL) == RCC_CFGR_SW_PLL){
		Sys_CLK = 84000000;
	}
	else if((RCC->CFGR & RCC_CFGR_SW_HSI) == RCC_CFGR_SW_HSI){
		Sys_CLK = 16000000;
	}
	
	if(TIMx == TIM2 || TIMx == TIM5){
		uint32_t ARRval;
		
		PSCval = Sys_CLK / 100000;
		ARRval = Sys_CLK / PSCval / 1000 * msec;
		TIMx->PSC = PSCval - 1;
		TIMx->ARR = ARRval - 1;
	}
	else{
		uint16_t ARRval;
		
		PSCval = Sys_CLK / 100000;
		ARRval = Sys_CLK / PSCval / 1000 * msec;
		TIMx->PSC = PSCval - 1;
		TIMx->ARR = ARRval - 1;
	}
}


// Update Event Interrupt
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec){
// 1. Initialize Timer	
	TIM_init(timerx, msec);
	
// 2. Enable Update Interrupt
	TIM_INT_enable(timerx);
	
// 3. NVIC Setting
	int IRQn_reg =0;
	if(timerx ==TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(timerx ==TIM2)  IRQn_reg = TIM2_IRQn;
	else if(timerx ==TIM3)  IRQn_reg = TIM3_IRQn;
	else if(timerx ==TIM4)  IRQn_reg = TIM4_IRQn;
	else if(timerx ==TIM4)  IRQn_reg = TIM4_IRQn;
	else if(timerx ==TIM5)  IRQn_reg = TIM5_IRQn;
	else if(timerx ==TIM9)  IRQn_reg = TIM1_BRK_TIM9_IRQn;
	else if(timerx ==TIM10)  IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(timerx ==TIM11)  IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;
	// ......TIM5 ~ 9, 10, 11
	  	
	NVIC_EnableIRQ(IRQn_reg);		
	NVIC_SetPriority(IRQn_reg, 2);
	
}



void TIM_INT_enable(TIM_TypeDef* timerx){
	timerx->DIER |= TIM_DIER_UIE;			// Enable Timer Update Interrupt		
}

void TIM_INT_disable(TIM_TypeDef* timerx){
	timerx->DIER &= ~(TIM_DIER_UIE);				// Disable Timer Update Interrupt		
}

uint32_t is_UIF(TIM_TypeDef *TIMx){
	return (TIMx->SR & TIM_SR_UIF) == TIM_SR_UIF;
}

void clear_UIF(TIM_TypeDef *TIMx){
	TIMx->SR &= ~(TIM_SR_UIF);
}
