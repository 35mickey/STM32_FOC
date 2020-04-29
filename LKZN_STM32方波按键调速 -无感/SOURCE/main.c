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
////���űۿ���  
#define PWM_UL_ON()  GPIO_WriteBit(GPIOA,GPIO_Pin_7,1) 
#define PWM_VL_ON()  GPIO_WriteBit(GPIOB,GPIO_Pin_0,1)
#define PWM_WL_ON()  GPIO_WriteBit(GPIOB,GPIO_Pin_1,1)

#define PWM_UL_OFF()  GPIO_WriteBit(GPIOA,GPIO_Pin_7,0)
#define PWM_VL_OFF()  GPIO_WriteBit(GPIOB,GPIO_Pin_0,0)
#define PWM_WL_OFF()  GPIO_WriteBit(GPIOB,GPIO_Pin_1,0)
/////�޸������������
#define COM_CNT 200
///// �޸��������綯�ƹ�������
#define ST_CNT 3
/////�û��������
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
u8   StartOk;//�����ɹ���־
u16  Comtime;//����ʱ�������
u16  stComcount;//����ʱ���綯�� 
u16  bhtime=0;//�����л��˲�ʱ��
u8   bhallstep;//��������˳�����

u8 RxBuffer[8];//���ڽ��ջ���

u8   bHallStemps[8]={0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};//��������˳��
u8   CheckBemf[6]={0x05,0x01,0x03,0x02,0x06,0x04};//���綯�ƹ����˳��
//u8   CheckBemf[6]={0x04,0x06,0x02,0x03,0x01,0x05};//���綯�ƹ����˳��
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
void PrechargeD(u16 ctime);  //Ԥ���
void Positioning(u8 bhallstep ,u16 bpwm,u16 ctime);  //�����λ
u8 MotorStart( u16 bpwm,u16 ctime);  //�������
void MotorStop(void);  //���ֹͣ
/*******************************************************************************
* Function Name  : PrechargeD
* Description    : chargeD program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PrechargeD(u16 ctime)  //Ԥ���
{
	   TIM_Cmd(TIM8, ENABLE);//����
		  TIM_CtrlPWMOutputs(TIM8, ENABLE);
	  time=0;
	 
	  PWM_UL_ON();  //U�¹ܴ�
    PWM_VL_ON();  //V�¹ܴ�
    PWM_WL_ON();  //W�¹ܴ�

   while(time<ctime)
	 {
		  time=time;
      time=time; 		 
	 }		 
    PWM_UL_OFF();  //U�¹ܹر�
    PWM_VL_OFF();  //V�¹ܹر�
    PWM_WL_OFF();	 //W�¹ܹر�
}


/*******************************************************************************
* Function Name  : Positioning
* Description    : Positioning program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Positioning(u8 bhallstep ,u16 bpwm,u16 ctime)  //�����λ
{
	 
	 My_PWM = bpwm;  //��λpwmֵ
	 Hall_SW(bhallstep);//��λ���
	 time=0;
	 while(time<ctime)  //��λʱ��
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
u8 MotorStart( u16 bpwm,u16 ctime)  //�������
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	 My_PWM = bpwm;  //����pwmֵ
	
	do{
	
     bhtime=0;
		 bhallstep++;
	 if(bhallstep>5) bhallstep=0;
		 Hall_SW(CheckBemf[bhallstep]);//�������
	 while(bhtime<ctime)  //����ʱ��
	  {
		  bhtime=bhtime;		 
	  }
		Comtime++; 
 }while(StartOk==0&&Comtime<COM_CNT);	 //�����ɹ���־�ж�

 if(Comtime>=COM_CNT)
 {
	     TIM_Cmd(TIM8, DISABLE);//ֹͣ
		   TIM_CtrlPWMOutputs(TIM8, DISABLE); 
	     PWM_UL_OFF();  //U�¹ܹر�
       PWM_VL_OFF();  //V�¹ܹر�
       PWM_WL_OFF();	 //W�¹ܹر�
	     return 0; 
 }
 
	return 1;   //С��COM_CNT����Ϊ�����ɹ�
	 
}
/*******************************************************************************
* Function Name  : MotorStop
* Description    : MotorStop program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MotorStop(void)  //���ֹͣ
{
	     TIM_Cmd(TIM8, DISABLE);//ֹͣ
		   TIM_CtrlPWMOutputs(TIM8, DISABLE); 
	     PWM_UL_OFF();  //U�¹ܹر�
       PWM_VL_OFF();  //V�¹ܹر�
       PWM_WL_OFF();	 //W�¹ܹر�
}
/*******************************************************************************
* Function Name  : MotorStatus
* Description    : MotorStatus program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MotorStatus(void)  //���״̬
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
    //����2������ʼ�����岿��,����2����Ϊ00 �� 8 ��1 ��N  �����жϷ�ʽ
//*****************************************************************************  
  USART_DeInit(USART3);
  USART_InitStructure.USART_BaudRate = 19200; //�趨��������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�趨��������λ��
  USART_InitStructure.USART_StopBits = USART_StopBits_1;    //�趨ֹͣλ����
  USART_InitStructure.USART_Parity = USART_Parity_No ;      //����У��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//������������
  USART_InitStructure.USART_Mode =   USART_Mode_Tx | USART_Mode_Rx ;   //ʹ�ý��պͷ��͹��� 
  USART_Init(USART3, &USART_InitStructure);  //��ʼ������1
  
  USART_ITConfig(USART3, USART_IT_TXE , ENABLE);  //����
  USART_ITConfig(USART3, USART_IT_RXNE , ENABLE); //����
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1
   

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
	RCC_Configuration();//ʱ�ӳ�ʼ��
     
	/* NVIC configuration */
	NVIC_Configuration();//�ж�����
  DMA_Configuration1();
	ADC_Configuration1();
	
	TIM8_Configuration1();	 //��ʱ����ʼ�� 
	TIM2_Configuration1();
	TIM4_Configuration1();

	
	
	GPIO_Configuration();  //GPIO
	UART3_Init();
	SysTick_Configuration();
	SysTick_CounterCmd(SysTick_Counter_Enable);
	
   StartOk=0;//�����ɹ���־
   Comtime=0;//����ʱ�������
   aim_speed =500;
	while (1)
	{
		
	 MotorStatus();	
		
   keytemp= key_con(); //����ֵ
		
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
			 TIM_Cmd(TIM8, DISABLE);//ֹͣ
		   TIM_CtrlPWMOutputs(TIM8, DISABLE); 
		   aim_speed =200;
		}
		   

	}
  if(keytemp==3)
	{
		if(time >1000)
		{
			  if(aim_speed <2500)
				 aim_speed+=10;   //�ٶȼ�
				time =0;
		}
		 My_PWM = aim_speed;  //�趨PWMֵ
			
	}
else if(keytemp==4)
{
	 	if(time >1000)
		{
			  if(aim_speed >200 )
				 aim_speed-=10;//�ٶȼ�
				time =0;
		}
		 My_PWM = aim_speed;  //�趨PWMֵ
}
//if(keytemp==5)
//{
//		if(time >1800)
//		{
//			time=0;
//			if(flagccw==0)
//		{
//			 Direction=1; //����
//		
//		}
//		else
//		{
//			  Direction=0;//��ת
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
void EXTI9_5_IRQHandler(void)   //�������ж��ж�
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
	
	//A�෴�綯�ƹ������
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6)==1)
	{
		bHall|= BIT0; 
	}
	//B�෴�綯�ƹ������
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)==1)
	{
		bHall|= BIT1; 
	}
	//C�෴�綯�ƹ������
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)==1)
	{
		bHall|= BIT2; 
	}

	

	 
	if(StartOk==0) //����
	{
		 if(bHall == CheckBemf[bhallstep])
		 {
			 stComcount++; 
 		 
		 }
     else
   	 {
			 stComcount=0;
		 }	
    if(stComcount>=ST_CNT)//������⵽һ������������л������㻻��
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
	//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ;          
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//IO������
  GPIO_Init(GPIOC, &GPIO_InitStructure);   
		//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;          
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//IO������
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2  ;           
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//IO��
//  GPIO_Init(GPIOB, &GPIO_InitStructure);   
	//led���
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	 GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	 GPIO_Init(GPIOD, &GPIO_InitStructure);  
	 
  	/* ����Hall�ӿ�IO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*�����ź����ж�����*/
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);
 
	EXTI_InitStructure.EXTI_Line = EXTI_Line6|EXTI_Line7|EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	

	 /*PC6,PC7,PC8 Ϊ�ϰ��ű�*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7 | GPIO_Pin_8 ;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /*PB0,PB1,PA7 Ϊ�°��ű�*/
	  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_0 | GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
		
		 //******************************************************************************
//����3��ʹ�ùܽ�������붨��
//******************************************************************************
//����UART3 TX (PB.10)��Ϊ�����������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;         //IO�ڵĵ�10��
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO���ٶ�
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //IO�ڸ����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);            //��ʼ������3���IO��

  // ���� USART3 Rx (PB.11)Ϊ�������� 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           //IO�ڵĵ�11��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//IO����������
  GPIO_Init(GPIOB, &GPIO_InitStructure);   
	

}
 
//�ջ������ӳ���
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
