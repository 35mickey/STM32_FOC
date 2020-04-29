/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "main.h"
#include "CAN\can.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

/////////////////////////////////////////////////
////下桥臂控制
#define PWM_UL_ON()  GPIO_WriteBit(GPIOA,GPIO_Pin_7,1)
#define PWM_VL_ON()  GPIO_WriteBit(GPIOB,GPIO_Pin_0,1)
#define PWM_WL_ON()  GPIO_WriteBit(GPIOB,GPIO_Pin_1,1)

#define PWM_UL_OFF()  GPIO_WriteBit(GPIOA,GPIO_Pin_7,0)
#define PWM_VL_OFF()  GPIO_WriteBit(GPIOB,GPIO_Pin_0,0)
#define PWM_WL_OFF()  GPIO_WriteBit(GPIOB,GPIO_Pin_1,0)
/////无感启动换向次数
#define COM_CNT 200
///// 无感启动反电动势过零点次数
#define ST_CNT 3
/////用户定义变量
#define BIT0 1
#define BIT1 2
#define BIT2 4
/////////////////////

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static vu32 TimingDelay = 0;

volatile struct {
		unsigned Key 	  :		1;
		unsigned CalSpeed : 	1;
		unsigned Sec      :		1;
		unsigned Fault 	  :		1;
		}Flags;

enum MOSTAUS
{
	CHONGD,ALIGNED,STARRT,RUN,FAULT,IDLE
};
enum MOSTAUS motorstaus=IDLE;
unsigned int DesiredSpeed=500;
unsigned int ActualSpeed;
unsigned int pwm=500;
unsigned int T3Count;
unsigned int ActualSpeed5[3];
vu16 ADC_DMABUF;
unsigned int AveActualSpeed;
unsigned char AveNum;
unsigned char j;


float kp=0.1,ki=0.08,kd=0.0;
int ek=0,ek1=0,ek2=0;
float duk;
int du;
int ekSpeed=0;
u16 motor_statue=0;
u16  startcnt=0;
u8   StartOk;//启动成功标志
u16  Comtime;//启动时换相次数
u16  stComcount;//启动时反电动势
u16  bhtime=0;//霍尔切换滤波时间
u8   bhallstep;//霍尔换相顺序变量

u8   RxBuffer[8];//串口接收缓存

u8   bHallStemps[8]={0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};//霍尔换相顺序
u8   CheckBemf[6]={0x05,0x01,0x03,0x02,0x06,0x04};//反电动势过零点顺序
//u8   CheckBemf[6]={0x04,0x06,0x02,0x03,0x01,0x05};//反电动势过零点顺序
extern u16 My_PWM;
extern u16 Hall,time;
extern bool Direction;
extern void TIM8_Configuration1(void);
int state,state1,state2,state3,counter1,counter2,counter3,speed_1,aim_speed,check_run,speed_code;
short ADC_ConvertedValue[5]={0,0,0,0,0};
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
EXTI_InitTypeDef EXTI_InitStructure;
ErrorStatus HSEStartUpStatus;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void CalculateDC(int u,int y);
void TIM3_Configuration1(void);
void TIM2_Configuration1(void);
void TIM4_Configuration1(void);
void SysTick_Configuration(void);
void DMA_Configuration1(void);
void ADC_Configuration1(void);
int pid(int nonce,int aim);
u8 key_con(void);

////////////////////////////////
void PrechargeD(u16 ctime);  //预充电
void Positioning(u8 bhallstep ,u16 bpwm,u16 ctime);  //电机定位
u8 MotorStart( u16 bpwm,u16 ctime);  //电机启动
void MotorStop(void);  //电机停止
/*******************************************************************************
* Function Name  : PrechargeD
* Description    : chargeD program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PrechargeD(u16 ctime)  //预充电
{
	   TIM_Cmd(TIM8, ENABLE);//启动
		  TIM_CtrlPWMOutputs(TIM8, ENABLE);
	  time=0;

	  PWM_UL_ON();  //U下管打开
    PWM_VL_ON();  //V下管打开
    PWM_WL_ON();  //W下管打开

   while(time<ctime)
	 {
		  time=time;
      time=time;
	 }
    PWM_UL_OFF();  //U下管关闭
    PWM_VL_OFF();  //V下管关闭
    PWM_WL_OFF();	 //W下管关闭
}


/*******************************************************************************
* Function Name  : Positioning
* Description    : Positioning program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Positioning(u8 bhallstep ,u16 bpwm,u16 ctime)  //电机定位
{

	 My_PWM = bpwm;  //定位pwm值
	 Hall_SW(bhallstep);//定位电机
	 time=0;
	 while(time<ctime)  //定位时间
	 {
		  time=time;
      time=time;
	 }

}
/*******************************************************************************
* Function Name  : MotorStart
* Description    : MotorStart program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 MotorStart( u16 bpwm,u16 ctime)  //电机启动
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	 My_PWM = bpwm;  //启动pwm值

	do{

     bhtime=0;
		 bhallstep++;
	 if(bhallstep>5) bhallstep=0;
		 Hall_SW(CheckBemf[bhallstep]);//启动电机
	 while(bhtime<ctime)  //启动时间
	  {
		  bhtime=bhtime;
	  }
		Comtime++;
 }while(StartOk==0&&Comtime<COM_CNT);	 //启动成功标志判断

 if(Comtime>=COM_CNT)
 {
	     TIM_Cmd(TIM8, DISABLE);//停止
		   TIM_CtrlPWMOutputs(TIM8, DISABLE);
	     PWM_UL_OFF();  //U下管关闭
       PWM_VL_OFF();  //V下管关闭
       PWM_WL_OFF();	 //W下管关闭
	     return 0;
 }

	return 1;   //小于COM_CNT次认为启动成功

}
/*******************************************************************************
* Function Name  : MotorStop
* Description    : MotorStop program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MotorStop(void)  //电机停止
{
	     TIM_Cmd(TIM8, DISABLE);//停止
		   TIM_CtrlPWMOutputs(TIM8, DISABLE);
	     PWM_UL_OFF();  //U下管关闭
       PWM_VL_OFF();  //V下管关闭
       PWM_WL_OFF();	 //W下管关闭
}
/*******************************************************************************
* Function Name  : MotorStatus
* Description    : MotorStatus program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MotorStatus(void)  //电机状态
{

	   switch(motorstaus)
		 {
			 case CHONGD:
				   PrechargeD(50);
			     motorstaus= ALIGNED;
				 break;
			 case ALIGNED:
				   Positioning(5,1000,50);
			     motorstaus= STARRT;
				 break;
			 case STARRT:
				   if(MotorStart(500,28)==1)
					 {
						    motorstaus= RUN;
					 }
					 else
					 {
						    motorstaus= FAULT;
					 }
				 break;
			 case RUN:
				 break;
			 case FAULT:
			    MotorStop();
				 break;
			 default:
				 break;
		 }
}
 /*******************************************************************************
* Function Name  : UART3_Init
* Description    : Configure the UART3
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART3_Init(void)
{
  USART_InitTypeDef USART_InitStructure;
 //******************************************************************************
    //串口2参数初始化定义部分,串口2参数为00 ， 8 ，1 ，N  接收中断方式
//*****************************************************************************
  USART_DeInit(USART3);
  USART_InitStructure.USART_BaudRate = 19200; //设定传输速率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //设定传输数据位数
  USART_InitStructure.USART_StopBits = USART_StopBits_1;    //设定停止位个数
  USART_InitStructure.USART_Parity = USART_Parity_No ;      //不用校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//不用流量控制
  USART_InitStructure.USART_Mode =   USART_Mode_Tx | USART_Mode_Rx ;   //使用接收和发送功能
  USART_Init(USART3, &USART_InitStructure);  //初始化串口1

  USART_ITConfig(USART3, USART_IT_TXE , ENABLE);  //发送
  USART_ITConfig(USART3, USART_IT_RXNE , ENABLE); //接收
  USART_Cmd(USART3, ENABLE);  //使能串口1


}
/*******************************************************************************
* Function Name  : USART_Send
* Description    : USART Send Buffer Content.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Send(u8 *Send, u8 Count)
{
  int i = 0;
  for(i = 0; i < Count; i++)
  {
  	USART_SendData(USART3, (u8)*(Send+i));
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
    }
  }

}

 u8 keytemp=0,bkey;
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
 {
	#ifdef DEBUG
	debug();
	#endif
  	int i;

	u8 flagccw=0;
	 	NVIC_InitTypeDef NVIC_InitStructure;
	/* System Clocks Configuration */
	RCC_Configuration();//时钟初始化

	/* NVIC configuration */
	NVIC_Configuration();//中断配置
  DMA_Configuration1();
	ADC_Configuration1();

	TIM8_Configuration1();	 //定时器初始化
	TIM2_Configuration1();
	TIM4_Configuration1();



	GPIO_Configuration();  //GPIO
	UART3_Init();
	SysTick_Configuration();
	SysTick_CounterCmd(SysTick_Counter_Enable);

   StartOk=0;//启动成功标志
   Comtime=0;//启动时换相次数
   aim_speed =500;
	while (1)
	{

	 MotorStatus();

   keytemp= key_con(); //按键值

		if(keytemp==0)
		{
			bkey=1;
		}


  if(keytemp==1)
	{
    if(bkey==1)
		{
		motorstaus=CHONGD;
		startcnt=0;
		StartOk=0;
	  bkey=0;
		Comtime=0;
		}


	}

	if(keytemp==2)
	{
		if(bkey==1)
		{
			 bkey=0;
			 TIM_Cmd(TIM8, DISABLE);//停止
		   TIM_CtrlPWMOutputs(TIM8, DISABLE);
		   aim_speed =200;
		}


	}
  if(keytemp==3)
	{
		if(time >1000)
		{
			  if(aim_speed <2500)
				 aim_speed+=10;   //速度加
				time =0;
		}
		 My_PWM = aim_speed;  //设定PWM值

	}
else if(keytemp==4)
{
	 	if(time >1000)
		{
			  if(aim_speed >200 )
				 aim_speed-=10;//速度减
				time =0;
		}
		 My_PWM = aim_speed;  //设定PWM值
}
//if(keytemp==5)
//{
//		if(time >1800)
//		{
//			time=0;
//			if(flagccw==0)
//		{
//			 Direction=1; //正传
//
//		}
//		else
//		{
//			  Direction=0;//反转
//
//		}
//		flagccw=~flagccw;
//		}
//
//
//
//}





	}
}

int pid(int nonce,int aim)
{
	static int ek_2=0;
	static int ek_1=0;
	static int ek=0;
//	int ec;
	int uk;

	ek=aim-nonce;
//	ec=ek/T;
	uk=kp*(ek-ek_1+ki*ek+kd*(ek-2*ek_1+ek_2));
	ek_2=ek_1;
	ek_1=ek;
	return (uk);
}

void delays(u16 cnt)
{
	u16 bctime;
	for(bctime=0;bctime>cnt;bctime++)
	{

	}

}
void EXTI9_5_IRQHandler(void)   //霍尔线中断判断
{

  u8 bHall=0,bHallstemp;
	if(EXTI_GetITStatus(EXTI_Line6)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
		if(EXTI_GetITStatus(EXTI_Line7)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line7);
	}
	if(EXTI_GetITStatus(EXTI_Line8)!= RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line8);
	}

	bHall=GPIO_ReadInputData(GPIOB);
 delays(2);
	bHallstemp=GPIO_ReadInputData(GPIOB);


	if(bHall!=bHallstemp) return;
	bHall=0;

	//A相反电动势过零点检测
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)==1)
	{
		bHall|= BIT0;
	}
	//B相反电动势过零点检测
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)==1)
	{
		bHall|= BIT1;
	}
	//C相反电动势过零点检测
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)==1)
	{
		bHall|= BIT2;
	}




	if(StartOk==0) //启动
	{
		 if(bHall == CheckBemf[bhallstep])
		 {
			 stComcount++;

		 }
     else
   	 {
			 stComcount=0;
		 }
    if(stComcount>=ST_CNT)//连续检测到一定数量过零点切换到过零换相
 	   {
		   StartOk=1;

	   }
	}
  if(StartOk==1)
	{
  Hall_SW(bHall);
	}

}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

    // RCC system reset(for debug purpose)
    RCC_DeInit();

    // Enable HSE
    RCC_HSEConfig(RCC_HSE_ON);

    // Wait till HSE is ready
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if(HSEStartUpStatus == SUCCESS)
    {
        // Enable Prefetch Buffer
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        // Flash 2 wait state
        FLASH_SetLatency(FLASH_Latency_2);

        // HCLK = SYSCLK
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        // PCLK2 = HCLK
        RCC_PCLK2Config(RCC_HCLK_Div1);

        // PCLK1 = HCLK/2
        RCC_PCLK1Config(RCC_HCLK_Div2);

        // PLLCLK = 8MHz * 9 = 72 MHz
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

        // Enable PLL
        RCC_PLLCmd(ENABLE);

        // Wait till PLL is ready
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        // Select PLL as system clock source
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        // Wait till PLL is used as system clock source
        while(RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
    else
    {
        // If HSE fails to start-up, the application will have wrong clock configuration.
        // User can add here some code to deal with this error

        // Go to infinite loop
        while (1)
        {
        }
    }


}
/*******************************************************************************
* Function Name  : key_con
* Description    : Configures the Motor operation mode
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u8 key_con(void)
{
	 static u8 key;
	key=0;
	 if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5))
	 {
		 key=5;

	 }
	  if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))
	 {
		 key=1;

	 }
	  if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14))
	 {
		 key=2;

	 }
	  if(!GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15))
	 {
		 key=3;

	 }
	  if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
	 {
		 key=4;

	 }
	 return key;
}
/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD ,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//按键输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//IO口上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);
		//按键输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//IO口上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2  ;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//IO口
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//led输出
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	 GPIO_Init(GPIOD, &GPIO_InitStructure);

  	/* 配置Hall接口IO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*霍尔信号线中断配置*/
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);

	EXTI_InitStructure.EXTI_Line = EXTI_Line6|EXTI_Line7|EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	 /*PC6,PC7,PC8 为上半桥臂*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7 | GPIO_Pin_8 ;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /*PB0,PB1,PA7 为下半桥臂*/
	  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_0 | GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

		 //******************************************************************************
//串口3所使用管脚输出输入定义
//******************************************************************************
//定义UART3 TX (PB.10)脚为复用推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;         //IO口的第10脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO口速度
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //IO口复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);            //初始化串口3输出IO口

  // 定义 USART3 Rx (PB.11)为悬空输入
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           //IO口的第11脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//IO口悬空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);


}

//闭环计算子程序
void CalculateDC(int u,int y)
{
	ek=u-y;
	if(ek>1||ek<-1)
	{
		duk=kp*(ek-ek1)+ki*ek+kd*(ek+ek2-ek1*2);
		du=(int)duk;
		if(duk>1)duk=1;
		if(duk<-1)duk=-1;
		if(du>10)du=10;
		if(du<-5)du=-5;
		pwm+=du;
		if(pwm<60)
		{
			pwm=60;
		}
		if(pwm>0x7FE)
		{

			pwm=0x7FE;
		}

		ek2=ek1;
		ek1=ek;
	}
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configure the nested vectored interrupt controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
		/* Set the Vector Table base location at 0x20000000 */
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  /* VECT_TAB_FLASH  */
		/* Set the Vector Table base location at 0x08000000 */
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);



	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//ENABLE;
	NVIC_Init(&NVIC_InitStructure);





	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*******************************************************************************
* Function Name  : SysTick_Config
* Description    : Configure a SysTick Base time to 10 ms.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Configuration(void)
{
    /* Select AHB clock(HCLK) as SysTick clock source */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

    /* Set SysTick Priority to 3 */
    NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 5, 0);

    /* SysTick interrupt each 1ms with HCLK equal to 72MHz */
    SysTick_SetReload(900000);

    /* Enable the SysTick Interrupt */
    SysTick_ITConfig(ENABLE);
}

/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length (time base 10 ms).
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(u32 nCount)
{
  TimingDelay = nCount;

  /* Enable the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Enable);

  while(TimingDelay != 0)
  {
  }

  /* Disable the SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Disable);

  /* Clear the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Clear);
}

/*******************************************************************************
* Function Name  : Decrement_TimingDelay
* Description    : Decrements the TimingDelay variable.
* Input          : None
* Output         : TimingDelay
* Return         : None
*******************************************************************************/
void Decrement_TimingDelay(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
