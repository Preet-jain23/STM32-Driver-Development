/*
 * stm32f411xx_usart_driver.h
 *
 *  Created on: Dec 24, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F411XX_USART_DRIVER_H_
#define INC_STM32F411XX_USART_DRIVER_H_




#include "stm32f411xx.h"

//USART CONFIGURATION
typedef struct{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint32_t USART_WordLength;
	uint8_t USART_NoOfStopbits;
	uint8_t  USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

//USART HANDLE
typedef struct{
	USART_RegDef_t  *pUSARTx;//USART_RegDef_t
	USART_Config_t USART_Config;
	uint8_t Rxstate;
	uint8_t Txstate;
	uint32_t Rxlen;
	uint32_t Txlen;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
}USART_Handle_t;





//EVENT

#define USART_EV_TX_CMPLT   0
#define USART_EV_RX_CMPLT   1
#define USART_EV_CTS_CMPLT   2
#define USART_EV_ORE_CMPLT   3
#define USART_EV_IDLE_CMPLT  4
#define USART_EV_LBD_CMPLT   5
#define USART_EV_FE_CMPLT   6
#define USART_EV_SR_CMPLT   7

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX    2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3




///

#define USART_READY              0
#define USART_BUSY_IN_TX         1
#define USART_BUSY_IN_RX         2


/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);



void USART_PeriClockControl(USART_RegDef_t *pUSART,uint8_t EnorDi);
void USART_PeripheralControl(USART_RegDef_t *pUSART,uint8_t EnorDi);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSART,uint8_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSART,uint8_t FlagName);

void USART_IRQInterruptConfig(uint8_t IRQNumber ,uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber ,uint8_t Priority);


void USART_ApplicationEventCallBack(USART_Handle_t*pUSARTHandle , uint8_t AppEv);




#endif /* INC_STM32F411XX_USART_DRIVER_H_ */
