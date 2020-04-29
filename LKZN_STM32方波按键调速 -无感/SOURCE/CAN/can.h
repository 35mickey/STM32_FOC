#include "stm32f10x_lib.h"
extern CanTxMsg tmp_TxMessage;
extern CanRxMsg tmp_CanRxMessage;
void CAN_Configuration(void);
void CAN_Send_Message(CanTxMsg *temp_TXMsg);

