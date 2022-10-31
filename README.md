# LAB_PWM_RCmotor
### LAB : Timer & PWM

**Date:** 2022.10.31

**Author/Partner:** Heo Dabin/Ga Seungho

**Github:** [Github code](https://github.com/DABINHEO/LAB_PWM_RCmotor.git)

**Demo Video:** [Youtube Link](https://youtube.com/shorts/ByzMxC2KUdk)

##            



### Introduction

In this LAB, we will first create a header code for PWM output, and based on this, we will output PWM signals from NUCLEO-F401RE and control the servo motor. The servomotor will move 10 degrees at intervals of 0.5 msec from 0 to 180 degrees, and will repeatedly return to 0 degrees in the same way after reaching 180 degrees. The interval of 0.5 msec will be made through Timer and Timer interupt. Button works as an EXTI, and when pressed, the motor will return to zero degrees and return to its original operation.



### Requirement

#### Hardware

* MCU

  * NUCLEO-F401RE

* Actuator/Display

  * RC Servo Motor (SG90)

  

#### Software

* Keil uVision, CMSIS, EC_HAL library

##          



### Problem1: Create HAL library



#### Procedure

In this problem, we will create functions for outputting PWM. The procedure for generating the PWM output is as follows. We first set the GPIO to send PWM output to Alternate function mode, and set the AFR for high speed, No pull-up, pull-down, and timer.
And I set the timer. After enabling the timer peripheral clock, setting the counting direction, setting the psc and arr of the timer, setting the output mode to PWM and CCR to default, selecting the output polarity, enabling Compare Capture Output, and enabling the counter. In addition, functions for setting the period and duty of PWM were also conducted.

* ecPWM.c  description

```c++
//PWM Initialization
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
		TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // OC1M = 110 for PWM Mode 1 output on ch1
		TIMx->CCMR1	|= TIM_CCMR1_OC1PE;          // Output 1 preload enable (make CCR1 value changable)
		TIMx->CCR1  = ccVal; 		// Output Compare Register for channel 1 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC1P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC1E;						// Enable output for ch1
	}
	else if(CHn == 2){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;                     // Clear ouput compare mode bits for channel 2
		TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // OC1M = 110 for PWM Mode 1 output on ch2
		TIMx->CCMR1	|= TIM_CCMR1_OC2PE;          // Output 1 preload enable (make CCR2 value changable)	
		TIMx->CCR1  = ccVal; 		// Output Compare Register for channel 2 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC2P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC2E;						// Enable output for ch2
	}
	else if(CHn == 3){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;                     // Clear ouput compare mode bits for channel 3
		TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // OC1M = 110 for PWM Mode 1 output on ch3
		TIMx->CCMR2	|= TIM_CCMR2_OC3PE;           // Output 1 preload enable (make CCR3 value changable)	
		TIMx->CCR1  = ccVal; 		// Output Compare Register for channel 3 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC3P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC3E;						// Enable output for ch3
	}
	else if(CHn == 4){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;                     // Clear ouput compare mode bits for channel 4
		TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // OC1M = 110 for PWM Mode 1 output on ch4
		TIMx->CCMR2	|= TIM_CCMR2_OC4PE;           // Output 1 preload enable (make CCR3 value changable)	
		TIMx->CCR1  = ccVal; 		// Output Compare Register for channel 4 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC4P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC4E;						// Enable output for ch4
	}	

// 5. Enable Timer Counter
	if(TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;					// Main output enable (MOE): 0 = Disable, 1 = Enable	
	TIMx->CR1  |= TIM_CR1_CEN;  													// Enable counter
}

//Set PWM period(msec, usec)
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
	
    float fclk = fsys/(psc+1);							  // fclk=fsys/(psc+1);
	uint32_t ccval = pulse_width_ms *fclk - 1;	   		  // pulse_width_ms *fclk - 1;
	
	switch(CHn){
		case 1: pwm->timer->CCR1 = ccval; break;
		case 2: pwm->timer->CCR1 = ccval; break;
		case 3: pwm->timer->CCR1 = ccval; break;
		case 4: pwm->timer->CCR1 = ccval; break;
		default: break;
	}
}

//Set PWM duty ratio
void PWM_duty(PWM_t *pwm, float duty) {                   //  duty=0 to 1	
	float ccval = (pwm->timer->ARR+1)*duty - 1;    	   	  // (ARR+1)*dutyRatio - 1          
	int CHn = pwm->ch;
  	
	switch(CHn){
		case 1: pwm->timer->CCR1 = ccval; break;
		case 2: pwm->timer->CCR1 = ccval; break;
		case 3: pwm->timer->CCR1 = ccval; break;
		case 4: pwm->timer->CCR1 = ccval; break;
		default: break;
	}
}

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
}
```

##          



### Problem2: RC Servo motor



#### Procedure

In this problem, we will control the servomotor using the PWM header code created earlier. A servo motor is a motor operated by a PWM signal. The motor used in this LAB operates on a PWM signal of 20ms, and the angle toward it varies depending on the duty. 0 degree when duty is 0.025 and 180 degree when duty is 0.125. We made the motor move as follows. The servomotor will move 10 degres at 0.5sec from 0 to 180 degres, and will repeatly return to 0 degres in the same way after receiving 180 degres. The interval of 0.5sec will be made through Timer and Timer interrupt. The timer period was set to 100 usec, and 5000 counters were added to interrupt to make 500 msec. Button works as an EXTI, and when pressed, the motor will return to zero degrees and return to its original operation.

#### Configuration

![image](https://user-images.githubusercontent.com/113574284/198910936-12c20b37-985a-4456-9678-f525ad46cffa.png)



#### Circuit Diagram

![image](https://user-images.githubusercontent.com/113574284/198910110-50ccb0ee-3ee5-4d61-9554-2d66a1c1299e.png)



#### Discussion

1. Derive a simple logic to calculate for CRR and ARR values to generate xHz and y% duty ratio of PWM. How can you read the values of input clock frequency and PSC?

   Frequency can be obtained by dividing (PSC+1) and (ARR+1) at system clock (PLL: 84 MHz, HSI: 16 MHz). Access to the       TIMx->PSC allows you to check or change the PSC, and access to the TIMx->ARR allows you to check or change the ARR. The period of PWM is the same as that of Timer, and duty can be simply obtained by multiplying ARR by y%.

   

2. What is the smallest and highest PWM frequency that can be generated for Q1?

   In general, PSC and ARR are determined by 16-bit, and their values are 0 to 65535. (ARR is available up to 32-bit for 32-bit timers.) Therefore, the minimum value is 16*10^6/(65535+1)^2 in HSI. The maximum value is 84*10^6/1^2 in the PLL.



#### Description with Code

* LAB_PMW_RCmotor.c  description

```c++
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
```



#### Results

![KakaoTalk_20221031_095438768](https://user-images.githubusercontent.com/113574284/198911202-a7a8a30e-d03b-4c0c-89c3-ed2a1cc3928e.jpg)

![KakaoTalk_20221031_093554097](https://user-images.githubusercontent.com/113574284/198911255-81c9bb82-7e06-4416-845e-4d92db202a62.jpg)

[Youtube Link](https://youtube.com/shorts/ByzMxC2KUdk)



##          



### Reference

[LAB: Timer & PWM](https://ykkim.gitbook.io/ec/course/lab/lab-timer-and-pwm)

Class materials in Embedded Controller by Prof. Kim

RM0383 Reference manual

##          



### Troubleshooting

The PWM_init function did not work, so it was debugged by checking each part of the problem. I found out that the problem was that the timer did not start properly, but it was not solved, and with the help of TA, I was able to confirm that & | was written incorrectly.  '|, &'Make sure you check.
PWM operation in GPIOA and pin5 worked well, but GPIOA and pin1 did not work. As a result of checking several pins by turns, it was found that PWM did not operate except for channel 1 (GPIOA0/5/6/15, B4, C6, etc.). Accordingly, GPIOA's pin5 was used to solve PWM output, and the timer for interrupt was tried to use number 3, but it did not work properly. I'm not sure but i think it is  because there was a conflict in the setting.
Accordingly, pin 6 of GPIOA, timer 3 of channel 1, was used, and timer for interrupt was set to use 2, so it worked as intended.
