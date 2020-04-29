/*******************************************************************************
* 文件功能 : 实现CAN通讯的配置和使用 
* 配置方法 : 在主函数中调用	CAN_Config(); 配置，本配置为PA11‘12为CAN接口
* 使用方法 ：在要用的文件中加入 can.h ，在 stm32f10x_it.c 中加入对应
*			 的中断服务函数 CAN_Receive(CAN_FIFO0, &tmp_CanRxMessage);
*			 接收的数据被放在 tmp_CanRxMessage 中，在 NVIC_Configuration(); 中
*			 配置好终端优先级直接调用 CAN_Send_Message(&tmp_TxMessage); 
*			 把 tmp_TxMessage 中的数据发出去。
*   注意   ：extern TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
*			 extern TIM_OCInitTypeDef  TIM_OCInitStructure;
*            以上两个结构体，如果单独使用TIM4，不使用TIM3时，要把 extern 去掉
*            同时定义 int Change_PWM_Pulse(TIM_TypeDef* TIMx,int chanel,int x)；
*            这个函数。
*   作者   ： taoyeah 
*   时间   ：2011年7月13日
*******************************************************************************/
#include "stm32f10x_lib.h"
CanRxMsg tmp_CanRxMessage;
CanTxMsg tmp_TxMessage;
/*-----------------------------------------------------------------------------
** 函数原型：void CAN_Configuration(void)
** 函数功能：Configures the CAN.
** 入口参数：无
** 出口参数：无
** 备    注：无
-----------------------------------------------------------------------------*/
void CAN_Config(void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

    // Configure CAN pin: RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Configure CAN pin: TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // CAN register init
    CAN_DeInit();
    CAN_StructInit(&CAN_InitStructure);

    // CAN cell init
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    //CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;

    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
    CAN_InitStructure.CAN_Prescaler = 4;          //波特率 72MHz/2=36MHz=PCLK1 / 4 => 9000KHz / (1+10+7) => 500KHz

    CAN_Init(&CAN_InitStructure);

    // CAN filter init
	//标准帧在 31~21 位  共11位 STID[10:0] 
    //扩展帧在 20~3 位   共29位 EXID[17:0]+STID[10:0] 

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    //CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x00<<5;        // STD_ID<<5	各位要一一对应才能过
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x00;			   //
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x00<<5;    // STD_ID<<5	对应位为1的不能过
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x00;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // CAN FIFO0 message pending interrupt enable
    CAN_ITConfig(CAN_IT_FMP0, ENABLE);
}

void CAN_Send_Message(CanTxMsg *temp_TXMsg)
{
    u32 i = 0;
    u8 TransmitMailbox = 0;
	//while
    TransmitMailbox = CAN_Transmit(temp_TXMsg);
    i = 0;
    while((CAN_TransmitStatus(TransmitMailbox) != CANTXOK)&&i<=0xff )
    {
      i++;
    }
}
