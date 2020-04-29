 #include "stm32f10x_lib.h"
  TIM_TimeBaseInitTypeDef TIM8_TimeBaseStructure;
  TIM_OCInitTypeDef TIM8_OCInitStructure;
  TIM_BDTRInitTypeDef TIM8_BDTRInitStructure;
  /////////////////////// PWM Peripheral Input clock ////////////////////////////
#define CKTIM	((u32)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */

 ////////////////////// PWM Frequency ///////////////////////////////////

 /****  Pattern type is center aligned  ****/

	#define PWM_PRSC ((u8)0)
 /**** Power devices switching frequency  ****/
#define PWM_FREQ ((u16) 15000) // in Hz  (N.b.: pattern type is center aligned)
         /* Resolution: 1Hz */
	#define PWM_PERIOD ((u16) (CKTIM / (u32)(1 * PWM_FREQ *(PWM_PRSC+1))))

   /**** ADC IRQ-HANDLER frequency, related to PWM  ****/
#define REP_RATE (0)  	// (N.b): Internal current loop is performed every
#define CKTIM	((u32)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */
 /****    Deadtime Value   ****/
#define DEADTIME_NS	((u16) 800)  //in nsec; range is [0...3500]
 ////////////////////////////// Deadtime Value /////////////////////////////////
	#define DEADTIME  (u16)((unsigned long long)CKTIM/2 \
           *(unsigned long long)DEADTIME_NS/1000000000uL)
 void TIM8_Configuration1(void)
 {
     /* TIM1 Registers reset */
     /* Enable TIM1 clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
   TIM_DeInit(TIM8);
   TIM_TimeBaseStructInit(&TIM8_TimeBaseStructure);
   /* Time Base configuration */
   TIM8_TimeBaseStructure.TIM_Prescaler = 0x0;
   TIM8_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM8_TimeBaseStructure.TIM_Period = PWM_PERIOD;
   TIM8_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

   // Initial condition is REP=0 to set the UPDATE only on the underflow
   TIM8_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;
   TIM_TimeBaseInit(TIM8, &TIM8_TimeBaseStructure);

   TIM_OCStructInit(&TIM8_OCInitStructure);
   /* Channel 1, 2,3 in PWM mode */
   TIM8_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
   TIM8_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 //  TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
   TIM8_OCInitStructure.TIM_Pulse = 0x2505; //dummy value
   TIM8_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 //  TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
 //  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
 //  TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

   TIM_OC1Init(TIM8, &TIM8_OCInitStructure);
   TIM_OC2Init(TIM8, &TIM8_OCInitStructure);
   TIM_OC3Init(TIM8, &TIM8_OCInitStructure);

 //  TIM_OCStructInit(&TIM1_OCInitStructure);
 //  /* Channel 4 Configuration in OC */
 //  TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
 //  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 //  TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
 //  TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD - 1;
 //
 //  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 //  TIM1_OCInitStructure.TIM_OCNPolarity =TIM_OCNPolarity_Low;
 //  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
 //  TIM1_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

   TIM_OC4Init(TIM8, &TIM8_OCInitStructure);

   /* Enables the TIM1 Preload on CC1 Register */
   TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
   /* Enables the TIM1 Preload on CC2 Register */
   TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
   /* Enables the TIM1 Preload on CC3 Register */
   TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
   /* Enables the TIM1 Preload on CC4 Register */
   TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

 //  /* Automatic Output enable, Break, dead time and lock configuration*/
 //  TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
 //  TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
 //  TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
 //  TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME;
 //  TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
 //  TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
 //  TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

 //  TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);

 //  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
 //
 //  TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
 //  TIM_ITConfig(TIM1, TIM_IT_Break,ENABLE);

   /* TIM1 counter enable */
   TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIM3在ARR上的预装载寄存器
   TIM_CtrlPWMOutputs(TIM8, DISABLE);
   TIM_Cmd(TIM8, DISABLE);


 //  // Resynch to have the Update evend during Undeflow
 //  TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
 //
 //  // Clear Update Flag
 //  TIM_ClearFlag(TIM1, TIM_FLAG_Update);
 //
 //  TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
 //
 //  TIM_ITConfig(TIM1, TIM_IT_CC4,DISABLE);
 }
