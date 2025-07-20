
#include<stm32f411xx_spi_driver.h>
//perihpereal clcok setup
static void spi_tx_handler(SPI_Handle_t *pHandle);
static void spi_rx_handler(SPI_Handle_t *pHandle);
static void spi_ovr_handler(SPI_Handle_t *pHandle);




void SPI_Peripheral_last(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
	pSPIx->SPI_CR1  |= (1<<6);
	}

	else{
		pSPIx->SPI_CR1 &=~(1<<6);
	}
	return;
}
void SPI_perclockcontrol(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
  if(EnorDi == ENABLE){
	  if(pSPIx==SPI1){
		   SPI1_PCLK_EN();
	  }
	  else if(pSPIx==SPI2){
	 		  SPI2_PCLK_EN();
	 	}
	  else if(pSPIx==SPI3){
	 		  SPI3_PCLK_EN();
	 	 }
	  else if(pSPIx==SPI4){
	 		  SPI4_PCLK_EN();
	 }

  }

  else{
	     if(pSPIx==SPI1){
	 		    SPI1_PCLK_DI();
	 	  }
	 	  else if(pSPIx==SPI2){
	 	 		  SPI2_PCLK_DI();
	 	 	}
	 	  else if(pSPIx==SPI3){
	 	 		  SPI3_PCLK_DI();
	 	 	 }
	 	  else if(pSPIx==SPI4){
	 	 		  SPI4_PCLK_DI();
	 	 }


  }
}



void SPI_Init(SPI_Handle_t *pSPIHandle){


	SPI_perclockcontrol(pSPIHandle->pSPIx, ENABLE);
	uint32_t temp=0;


	temp|=(pSPIHandle->SPIConfig.SPI_DeviceMode  << 2);


	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FD){
	temp &=~(SPI_BUS_FD << 15);
	}

	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HD){
	temp |= (1<<15);
	}

	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RX_ONLY){
		temp &=~(SPI_BUS_FD << 15);
        temp |=(1<<10);
	}


	temp |=(pSPIHandle->SPIConfig.SPI_SclkSpeed << 3);

	temp |=(pSPIHandle->SPIConfig.SPI_DFF << 11);

	temp |=(pSPIHandle->SPIConfig.SPI_CPOL << 1);

	temp |=(pSPIHandle->SPIConfig.SPI_CPHA << 0);

   temp |= (pSPIHandle->SPIConfig.SPI_SSM << 9);


   pSPIHandle->pSPIx->SPI_CR1 |= temp;

}

void SPI_DeInit(SPI_Handle_t *pSPIHandle){


}


uint8_t SPI_getflagstatus(SPI_RegDef_t *pSPIx,uint32_t FlagName){

	 if(pSPIx->SPI_SR & (1<<FlagName)){
		 return 1;
	 }

	 return 0;
}
//len=nof of bytes
//*buffere=data to send
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len)
{
	//this is blocking type or polling based api(permannetly block due to error or sommething else)

	while(len > 0){

		while(!SPI_getflagstatus(pSPIx, 1));

		//check the dff

		if(pSPIx->SPI_CR1 & (1<<11)){
			//16 bit

			           pSPIx->SPI_DR =*((uint16_t*) *pTxBuffer);

						(uint16_t*)pTxBuffer++;

						len--;

						len--;

		}

		else{
			//8 but
			pSPIx->SPI_DR = *pTxBuffer;

			pTxBuffer++;

			len--;
		}

	}

}

void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len){


	while(len > 0){
		while(!SPI_getflagstatus(pSPIx,0));

		if(pSPIx->SPI_CR1 & (1<<11)){
					//16 bit

			*((uint16_t*) *pRxBuffer)=pSPIx->SPI_DR ;

								(uint16_t*)pRxBuffer++;

								len--;

								len--;

				}

				else{
					//8 but
					*pRxBuffer=pSPIx->SPI_DR ;

					pRxBuffer++;

					len--;
				}


	}
}




uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len){
	uint8_t state =pSPIHandle->TxState;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		pSPIHandle->pTxbuffer=*pTxBuffer;
		pSPIHandle->Txlen=len;

		pSPIHandle->TxState=SPI_BUSY_IN_TX;

		pSPIHandle->pSPIx->SPI_CR2 |=(1<<7);
	}

	return state;

}

uint8_t SPI_RecieveData_IT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len){
	uint8_t state =pSPIHandle->RxState;

	if(pSPIHandle->RxState != SPI_BUSY_IN_RX){
		pSPIHandle->pRxbuffer=(uint8_t)*pRxBuffer;
		pSPIHandle->Rxlen=len;
		pSPIHandle->RxState=SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->SPI_CR2 |=(1<<6);
	}
	return state;
}

void SSI_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
	pSPIx->SPI_CR1  |= (1<<8);
	}

	else{
		pSPIx->SPI_CR1 &=~(1<<8);
	}
	return;
}


void SSIOE_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	if(EnorDi == ENABLE){
	pSPIx->SPI_CR2  |= (1<<2);
	}

	else{
		pSPIx->SPI_CR2 &=~(1<<2);
	}
	return;
}



void SPI_IRQHandling(SPI_Handle_t *pHandle){

	uint32_t temp1,temp2;

	temp1=pHandle->pSPIx->SPI_CR2 & (1<<7);
	temp2=pHandle->pSPIx->SPI_SR & (1<<1);

	if(temp1 & temp2){
		spi_tx_handler(pHandle);
	}

	temp1=pHandle->pSPIx->SPI_CR2 & (1<<6);
	temp2=pHandle->pSPIx->SPI_SR & (1<<0);

	if(temp1 & temp2){
		spi_rx_handler(pHandle);
	}


	temp1=pHandle->pSPIx->SPI_CR2 & (1<<5);
	temp2=pHandle->pSPIx->SPI_SR & (1<<6);

	if(temp1 & temp2){
		spi_ovr_handler(pHandle);
	}




}

static void spi_tx_handler(SPI_Handle_t *pHandle){


	while(pHandle->Txlen > 0){

		while(!SPI_getflagstatus(pHandle->pSPIx, 1));

		//check the dff

		if(pHandle->pSPIx->SPI_CR1 & (1<<11)){
			//16 bit

			           pHandle->pSPIx->SPI_DR =*((uint16_t*)pHandle->pTxbuffer);

						(uint16_t*)pHandle->pTxbuffer++;

						pHandle->Txlen--;

						pHandle->Txlen--;

		}

		else{
			//8 but
			pHandle->pSPIx->SPI_DR = *(pHandle->pTxbuffer);

			pHandle->pTxbuffer++;

			pHandle->Txlen--;
		}



	}


	if(pHandle->Txlen == 0){

		SPI_Close_transmisson(pHandle);
	    SPI_ApplicationEventCallBack(pHandle,SPI_EVENT_TX_CMPLT);


	}



}

static void spi_rx_handler(SPI_Handle_t *pHandle){


	while(pHandle->Rxlen > 0){
		while(!SPI_getflagstatus(pHandle->pSPIx,0));

		if(pHandle->pSPIx->SPI_CR1 & (1<<11)){
					//16 bit

			*((uint16_t*) pHandle->pRxbuffer)=pHandle->pSPIx->SPI_DR ;

								(uint16_t*)pHandle->pRxbuffer++;

								pHandle->Rxlen--;

								pHandle->Rxlen--;

				}

				else{
					//8 but
					*(pHandle->pRxbuffer)=pHandle->pSPIx->SPI_DR ;

					pHandle->pRxbuffer++;;

					pHandle->Rxlen--;
				}


	}

	if(pHandle->Rxlen == 0){
		SPI_Close_Reception(pHandle);
		SPI_ApplicationEventCallBack(pHandle,SPI_EVENT_RX_CMPLT);

	}

}


static void spi_ovr_handler(SPI_Handle_t *pHandle){
	uint32_t temp;
	if(pHandle->TxState  != SPI_BUSY_IN_TX){
	temp=pHandle->pSPIx->SPI_DR;
	temp=pHandle->pSPIx->SPI_SR;
	}

	SPI_ApplicationEventCallBack(pHandle,SPI_EVENT_OVR_ERR);
}

void SPI_Close_transmisson(SPI_Handle_t *pHandle){
	       pHandle->pSPIx->SPI_CR2  &=~(1<<7);
			pHandle->pTxbuffer=NULL;
			pHandle->Txlen=0;
			pHandle->TxState=SPI_READY;
}
void SPI_Close_Reception(SPI_Handle_t *pHandle){
	pHandle->pSPIx->SPI_CR2 &=~ (1<<6);
				pHandle->Rxlen=0;
				pHandle->pRxbuffer=NULL;
				pHandle->RxState=SPI_READY;
}

void SPI_Clear_OVR_Flag(SPI_Handle_t *pHandle){
	uint32_t temp;
	temp=pHandle->pSPIx->SPI_DR;
		temp=pHandle->pSPIx->SPI_SR;
}


__attribute__((weak)) void  SPI_ApplicationEventCallBack(SPI_Handle_t *pHandle,uint8_t APP){

}









