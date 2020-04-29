 #include "stm32f10x_lib.h"
  extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  extern TIM_OCInitTypeDef  TIM_OCInitStructure;
 void TIM2_Configuration1(void)
 {
     TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
     //  TIM_OCInitTypeDef  TIM_OCInitStructure ;
     TIM_DeInit( TIM2);                              //复位TIM2定时器

     /* TIM2 clock enable [TIM2定时器允许]*/
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

     /* TIM2 configuration */
     TIM_TimeBaseStructure.TIM_Period = 10;          //
     TIM_TimeBaseStructure.TIM_Prescaler = 719;    // 72M/(35999+1)/10 = 200Hz
     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分割
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //计数方向向上计数
     TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

     /* Clear TIM2 update pending flag[清除TIM2溢出中断标志] */
     TIM_ClearFlag(TIM2, TIM_FLAG_Update);

     /* Enable TIM2 Update interrupt [TIM2溢出中断允许]*/
     TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

     /* TIM2 enable counter [允许tim2计数]*/
     TIM_Cmd(TIM2, ENABLE);
 }
