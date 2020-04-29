/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : main.h
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 10/08/2007
* Description        : Header for main.c module
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
//#ifndef __MAIN_H
#define __MAIN_H
#define _TIM
#define _EXTI
/* Includes ------------------------------------------------------------------*/
//#include "lcd.h"
//#include "spi_flash.h"
//#include "LCD1602.h"

#define CLOCKWISE      1
#define ANTICLOCKWISE  0

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void GPIO_Config(void);
void SysTick_Config(void);
void Delay(u32 nCount);
void Decrement_TimingDelay(void);
void TIM8_Configuration(void);
void TIM3CH1_Configuration(void);
void CalculateDC(int u,int y);
void ADC1_Configuration(void);
//#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
