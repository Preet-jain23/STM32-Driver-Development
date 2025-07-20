
#include "stm32f411xx.h"
#include "string.h"
#include "stdint.h"






void Set_Baud_Rate(USART_RegDef_t *pUSARTx,uint32_t baudrate){
	uint32_t pclk1;
	uint32_t I_p;
	uint32_t F_p;
	uint32_t temp;


	if(pUSARTx == USART2){
		pclk1=get_clock_value_APB1();

	}

	else if(pUSARTx == USART1  || pUSARTx == USART6){
		pclk1=get_clock_value_APB2();
	}
	if(pUSARTx->USART_CR1  & (1<<USART_CR1_OVER8)){
		temp=((pclk1)/(8*baudrate))*100;
		 I_p=temp/100;
		 F_p =((((temp % 100)*8)+50)/100) & 0x07;

	}

	else{
		temp=((pclk1)/(16*baudrate))*100;
		 I_p=temp/100;
		 F_p =((((temp % 100)*16)+50)/100) & 0x0F;
	}

	pUSARTx->USART_BRR  |=(F_p)&(0xF);
	pUSARTx->USART_BRR  |=((I_p  << 4)&(0x3FF0));






}


void USART_IRQHandling(USART_Handle_t *pUSARTHandle_t){

	uint32_t temp1,temp2,temp3;

	temp1=pUSARTHandle_t->pUSARTx->USART_CR1   & (1<<USART_CR1_TCIE);
	temp2=pUSARTHandle_t->pUSARTx->USART_SR  &  (1<<USART_SR_TC);

	if(temp1 & temp2){

		if(pUSARTHandle_t->Txstate == USART_BUSY_IN_TX){
			if(pUSARTHandle_t->Txlen == 0){

				pUSARTHandle_t->pUSARTx->USART_CR1  &=~(1<<USART_CR1_TCIE);
				pUSARTHandle_t->pUSARTx->USART_SR  &=~(1<<USART_SR_TC);
				pUSARTHandle_t->Txlen=0;
				pUSARTHandle_t->Txstate=USART_READY;
				pUSARTHandle_t->pTxBuffer=NULL;
				USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_TX_CMPLT);

			}
		}

	}

	temp1=pUSARTHandle_t->pUSARTx->USART_CR1   & (1<<USART_CR1_TXEIE);
	temp2=pUSARTHandle_t->pUSARTx->USART_SR  &  (1<<USART_SR_TXE);


	if(temp1 & temp2){
		if(pUSARTHandle_t->Txstate == USART_BUSY_IN_TX){
			uint16_t *pDATA;
			if(pUSARTHandle_t->Txlen >0){
			if(pUSARTHandle_t->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
				pDATA=(uint16_t*)pUSARTHandle_t->pTxBuffer;
				pUSARTHandle_t->pUSARTx->USART_DR=(*pDATA)&(0x1FF);

				if(pUSARTHandle_t->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
                   pUSARTHandle_t->pTxBuffer++;
                   pUSARTHandle_t->pTxBuffer++;
                   pUSARTHandle_t->Txlen--;
                   pUSARTHandle_t->Txlen--;
				}
				else{
					pUSARTHandle_t->pTxBuffer++;
					pUSARTHandle_t->Txlen--;

				}
			}

			else{
				pUSARTHandle_t->pUSARTx->USART_DR=*(pUSARTHandle_t->pTxBuffer);
				pUSARTHandle_t->pTxBuffer++;
				pUSARTHandle_t->Txlen--;
			}
		}


		if(pUSARTHandle_t->Txlen ==0){
			pUSARTHandle_t->pUSARTx->USART_CR1 &=~(1<<USART_CR1_TXEIE);
		}
	}
	}

	temp1=pUSARTHandle_t->pUSARTx->USART_CR1  & (1<<USART_CR1_RXNEIE);
	temp2=pUSARTHandle_t->pUSARTx->USART_SR  &  (1<<USART_SR_RXNE);

	if(temp1 & temp2){

		if(pUSARTHandle_t->Rxlen > 0){
		if(pUSARTHandle_t->Rxstate == USART_BUSY_IN_RX){

			if(pUSARTHandle_t->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){

				 if(pUSARTHandle_t->USART_Config.USART_ParityControl  == USART_PARITY_DISABLE){

					 *((uint16_t*)pUSARTHandle_t->pRxBuffer)=pUSARTHandle_t->pUSARTx->USART_DR;
					 pUSARTHandle_t->Rxlen--;
					 pUSARTHandle_t->Rxlen--;
					 pUSARTHandle_t->pRxBuffer++;
					 pUSARTHandle_t->pRxBuffer++;
				 }

				 else{
					 *pUSARTHandle_t->pRxBuffer = pUSARTHandle_t->pUSARTx->USART_DR;
					 pUSARTHandle_t->Rxlen--;
					 pUSARTHandle_t->pRxBuffer++;
				 }
			}

			else {
				if(pUSARTHandle_t->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
					*pUSARTHandle_t->pRxBuffer = pUSARTHandle_t->pUSARTx->USART_DR & 0xFF;
					pUSARTHandle_t->pRxBuffer++;
					pUSARTHandle_t->Rxlen--;
				}

				else{
					*pUSARTHandle_t->pRxBuffer = pUSARTHandle_t->pUSARTx->USART_DR & 0x7F;
					pUSARTHandle_t->pRxBuffer++;
					pUSARTHandle_t->Rxlen--;
				}
			}

		}

		if(pUSARTHandle_t->Rxlen == 0){
			if(pUSARTHandle_t->Txstate == USART_BUSY_IN_RX){
				pUSARTHandle_t->pUSARTx->USART_CR1  &=~(1<<USART_CR1_RXNEIE);
				pUSARTHandle_t->pRxBuffer=NULL;
				pUSARTHandle_t->Rxlen=0;
				pUSARTHandle_t->Rxstate=USART_READY;
			    USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_RX_CMPLT);
			}

		}



	}
	}




		temp1=pUSARTHandle_t->pUSARTx->USART_CR3  & (1<<USART_CR3_CTSE);
		temp2=pUSARTHandle_t->pUSARTx->USART_CR3  &  (1<<USART_CR3_CTSIE);
        temp3=pUSARTHandle_t->pUSARTx->USART_SR   & (1<<USART_SR_CTS);

        if(temp1 & temp2 & temp3){

        	pUSARTHandle_t->pUSARTx->USART_SR &=~(1<<USART_SR_CTS);
        	USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_CTS_CMPLT);
        }

        temp1=pUSARTHandle_t->pUSARTx->USART_CR1  & (1<<USART_CR1_RXNEIE);
        temp2=pUSARTHandle_t->pUSARTx->USART_SR & (1<<USART_SR_ORE);

        if(temp1 & temp2){
        	pUSARTHandle_t->pUSARTx->USART_SR &=~(1<<USART_SR_ORE);
        	USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_ORE_CMPLT);
        }

        temp1=pUSARTHandle_t->pUSARTx->USART_CR1  & (1<<USART_CR1_IDLEIE);
        temp2=pUSARTHandle_t->pUSARTx->USART_SR & (1<<USART_SR_IDLE);

        if(temp1 & temp2){
                	pUSARTHandle_t->pUSARTx->USART_SR &=~(1<<USART_SR_IDLE);
                	USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_IDLE_CMPLT);

                }

        temp1=pUSARTHandle_t->pUSARTx->USART_CR2  & (1<<USART_CR2_LBDIE);
               temp2=pUSARTHandle_t->pUSARTx->USART_SR & (1<<USART_SR_LBD);

               if(temp1 & temp2){
                       	pUSARTHandle_t->pUSARTx->USART_SR &=~(1<<USART_SR_LBD);
                       	USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_LBD_CMPLT);

             }


           	temp2 =  pUSARTHandle_t->pUSARTx->USART_CR3 & ( 1 << USART_CR3_EIE) ;

           	if(temp2 )
           	{
           		temp1 = pUSARTHandle_t->pUSARTx->USART_SR;
           		if(temp1 & ( 1 << USART_SR_FE))
           		{
           			/*
           				This bit is set by hardware when a de-synchronization, excessive noise or a break character
           				is detected. It is cleared by a software sequence (an read to the USART_SR register
           				followed by a read to the USART_DR register).
           			*/
           			USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_FE_CMPLT);
           		}

           		if(temp1 & ( 1 << USART_SR_FE) )
           		{
           			/*
           				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
           				software sequence (an read to the USART_SR register followed by a read to the
           				USART_DR register).
           			*/
           			USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_SR_CMPLT);
           		}

           		if(temp1 & ( 1 << USART_SR_ORE) )
           		{
           			USART_ApplicationEventCallBack(pUSARTHandle_t, USART_EV_ORE_CMPLT);
           		}
           	}




















}

uint8_t get_flag_status(USART_RegDef_t *pUSART,uint8_t FlagName){
	if(pUSART->USART_SR   &  (1<< FlagName)  ){
		return 1;
	}

	return 0;
}

void USART_PeriClockControl(USART_RegDef_t *pUSART,uint8_t EnorDi){
	if(EnorDi == ENABLE){
	if(pUSART == USART1){
		USART1_PCLK_EN();
	}
	else if(pUSART == USART2){
		USART2_PCLK_EN();
	}

	else if(pUSART == USART6){
			USART6_PCLK_EN();
	}

	}

	else if(pUSART == DISABLE){

			if(pUSART == USART1){
				USART1_PCLK_DI();
			}
			else if(pUSART == USART2){
				USART2_PCLK_DI();
			}

			else if(pUSART == USART6){
					USART6_PCLK_DI();
			}


	}

}
void USART_PeripheralControl(USART_RegDef_t *pUSART,uint8_t EnorDi){
     if(EnorDi == ENABLE){
    	 pUSART ->USART_CR1 |= (1<<USART_CR1_UE);
     }
     else{
    	 pUSART ->USART_CR1 &=~ (1<<USART_CR1_UE);
    	 }
}


void USART_Init(USART_Handle_t*pUSARTHandle){

	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<USART_CR1_RE);
	}

	else if(pUSARTHandle->USART_Config.USART_Mode  == USART_MODE_ONLY_TX){
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<USART_CR1_TE);
	}

	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		pUSARTHandle->pUSARTx->USART_CR1 |=(3<<USART_CR1_RE);
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<USART_CR1_TE);
	}

	pUSARTHandle->pUSARTx->USART_CR2 |=((pUSARTHandle->USART_Config.USART_NoOfStopbits)<<USART_CR2_STOP);

	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<USART_CR1_PCE);
		pUSARTHandle->pUSARTx->USART_CR1 &=~(1<<USART_CR1_PS);
	}

	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD){
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<USART_CR1_PCE);
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<USART_CR1_PS);
	}


	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		pUSARTHandle->pUSARTx->USART_CR3 |=(1<<USART_CR3_CTSE);
	}

	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS){
			pUSARTHandle->pUSARTx->USART_CR3 |=(1<<USART_CR3_RTSE);
		}

	else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
			pUSARTHandle->pUSARTx->USART_CR3 |=(1<<USART_CR3_CTSE);
			pUSARTHandle->pUSARTx->USART_CR3 |=(1<<USART_CR3_RTSE);
		}

}

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){

	uint16_t *pdata;

	for(uint32_t i=Len ;i>0;i--){

		while(!get_flag_status(pUSARTHandle->pUSARTx,USART_SR_TXE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){

			pdata=(uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pdata & (0x1FF));

			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				pTxBuffer++;
				pTxBuffer++;
			}


			else{
				pTxBuffer++;
			}

		}

		else{
			pUSARTHandle->pUSARTx->USART_DR = *pTxBuffer;
			pTxBuffer++;
		}




	}

	while(!get_flag_status(pUSARTHandle->pUSARTx,USART_SR_TC));
}

void USART_ReceiveData(USART_Handle_t*pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint16_t *pData;

	for(uint32_t i=Len;i>0;i--){

		while(!get_flag_status(pUSARTHandle->pUSARTx,USART_SR_RXNE));

		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			pData=(uint16_t*)pRxBuffer;

			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				*pRxBuffer = pUSARTHandle->pUSARTx->USART_DR & 0xFF;
				pRxBuffer++;
				*pRxBuffer=((pUSARTHandle->pUSARTx->USART_DR >> 8 ) & 0x01);
				pRxBuffer++;
		}
			else{
				*pRxBuffer = pUSARTHandle->pUSARTx->USART_DR & 0xFF;
				pRxBuffer++;
			}

		}

		else{

			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){

			*pRxBuffer = pUSARTHandle->pUSARTx->USART_DR & 0xFF;

			pRxBuffer++;
			}

			else{
				*pRxBuffer = pUSARTHandle->pUSARTx->USART_DR & 0x07F;

				pRxBuffer++;

			}
		}
	}


}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){
    uint8_t state = pUSARTHandle->Txstate;

    if(state != USART_BUSY_IN_TX){
    	pUSARTHandle->Txstate=USART_BUSY_IN_TX;
    	pUSARTHandle->Txlen=Len;
    	pUSARTHandle->pTxBuffer =(uint8_t*)pTxBuffer;
    	pUSARTHandle->pUSARTx->USART_CR1  |=(1<<USART_CR1_TXEIE);
        pUSARTHandle->pUSARTx->USART_CR1  |=(1<<USART_CR1_TCIE);

    }

    return state;

}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
	  uint8_t state = pUSARTHandle->Rxstate;

	    if(state != USART_BUSY_IN_RX){
	    	pUSARTHandle->Txstate=USART_BUSY_IN_RX;
	    	pUSARTHandle->Rxlen=Len;
	    	pUSARTHandle->pRxBuffer =(uint8_t*)pRxBuffer;
	    	pUSARTHandle->pUSARTx->USART_CR1  |=(1<<USART_CR1_RXNEIE);


	    }

	    return state;

}


 void USART_ApplicationEventCallBack(USART_Handle_t*pUSARTHandle , uint8_t AppEv){
	;
}




