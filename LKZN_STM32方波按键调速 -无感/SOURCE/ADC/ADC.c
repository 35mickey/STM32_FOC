/*******************************************************************************
* 文件功能 : 实现ADC配置和使用 
* 配置方法 : 在主函数中调用	DMA_Config();  ADC_Config(); 配置。
* 使用方法 ：在要用的文件中加入 ADC.h ，直接读取 ADC_ConvertedValue 即为转换结果。
*   注意   ：DMA_Config(); 要放在 ADC_Config(); 前面，否则转换数据位置会错乱。
*   作者   ： taoyeah 
*   时间   ：2011年7月14日
*******************************************************************************/
#include "stm32f10x_lib.h"
#define ADC1_DR_Address    ((u32)0x4001244C)
vu16 ADC_ConvertedValue[8];
/*******************************************************************************
* Function Name  : ADC_Configuration1
* Description    : Configures the ADC1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;	
    GPIO_InitTypeDef GPIO_InitStructure;

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	/* Configure PA (ADC Channel11) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | \
								  GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   // 独立工作模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;         // 扫描方式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   // 连续转换
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 外部触发禁止
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              // 数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 8;              // 用于转换的通道数
    ADC_Init(ADC1, &ADC_InitStructure);
    
    /* ADC1 regular channel14 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_71Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_71Cycles5);
    
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
    
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));
    
    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
     
    /* Start ADC1 Software Conversion */ 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
}
/***********************************************************************************/
/*******************************************************************************
* Function Name  : DMA_Configuration1
* Description    : Configures the DMA1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	  /* DMA channel1 configuration ----------------------------------------------*/
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;      // 外设地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue; // 内存地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;               // DMA 传输方向单向
    DMA_InitStructure.DMA_BufferSize = 8;                            // 设置DMA在传输时缓冲区的长度 word
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 设置DMA的外设递增模式，一个外设
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;         // 设置DMA的内存递增模式
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // 外设数据字长
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          // 内存数据字长
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;     // 设置DMA的传输模式：连续不断的循环模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; // 设置DMA的优先级别
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;        // 设置DMA的2个memory中的变量互相访问
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    
    /* Enable DMA channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
}
