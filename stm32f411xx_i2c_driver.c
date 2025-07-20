/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: Jul 9, 2024
 *      Author: ASUS
 */

#include "stm32f411xx.h"
#include "string.h"
#include "stdint.h"

static void I2C_Generate_Start_Conditon(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateAddress(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress,uint8_t Read_Write);
static void Clear_AddrFlag(I2C_Handle_t *pI2Chandle);
static void I2C_Generate_Stop_Condition(I2C_RegDef_t *pI2Cx);

static void I2C_MASTER_HANDLE_TXE_INTERRUPT(I2C_Handle_t *pI2CHANDLE);

static void I2C_MASTER_HANDLE_RXNE_INTERRUPT(I2C_Handle_t *pI2CHANDLE);


static void I2C_Generate_Start_Conditon(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |= (1<<I2C_CR1_START);
}



static void I2C_GenerateAddress(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress,uint8_t Read_Write){
	if(Read_Write == READ){
	SlaveAddress = SlaveAddress<<1;
	SlaveAddress &=~(1);
	pI2Cx->I2C_DR = SlaveAddress;
	}

	else if(Read_Write == WRITE){
		    SlaveAddress = SlaveAddress<<1;
			SlaveAddress |=1;
			pI2Cx->I2C_DR = SlaveAddress;
	}

}

static void Clear_AddrFlag(I2C_Handle_t *pI2Chandle){
	uint32_t temp;



	if(pI2Chandle->pI2Cx->I2C_SR2 & (1<<I2C_SR2_MSL)){
		if(pI2Chandle->TxorRxState == I2C_BUSY_IN_RX){
			if(pI2Chandle->Rxsize == 1){
				pI2Chandle->pI2Cx->I2C_CR1 &=~(1<<10);
				   temp=pI2Chandle->pI2Cx->I2C_SR1;
					temp=pI2Chandle->pI2Cx->I2C_SR2;

				}
		}
		else{
				temp=pI2Chandle->pI2Cx->I2C_SR1;
				temp=pI2Chandle->pI2Cx->I2C_SR2;


		}
		}
	else{
		temp=pI2Chandle->pI2Cx->I2C_SR1;
		temp=pI2Chandle->pI2Cx->I2C_SR2;
		(void)temp;

	}

}



static void I2C_Generate_Stop_Condition(I2C_RegDef_t *pI2Cx){
	pI2Cx->I2C_CR1 |=(1<<I2C_CR1_STOP);
}


static void I2C_MASTER_HANDLE_TXE_INTERRUPT(I2C_Handle_t *pI2CHANDLE)
{
	if(pI2CHANDLE->TxorRxState == I2C_BUSY_IN_TX){

				if(pI2CHANDLE->pTxlen > 0){
					//1 load th data in DR
					pI2CHANDLE->pI2Cx->I2C_DR = *(pI2CHANDLE->pTxbuffer);

					//2 decrement the Tx len
					pI2CHANDLE->pTxlen--;


					//3 increment the buffer addrss
					pI2CHANDLE->pTxbuffer++;
				}
			}
}

static void I2C_MASTER_HANDLE_RXNE_INTERRUPT(I2C_Handle_t *pI2CHANDLE){

	if(pI2CHANDLE->TxorRxState == I2C_BUSY_IN_RX){
		if(pI2CHANDLE->Rxsize == 1){
            *pI2CHANDLE->pRxbuffer = pI2CHANDLE->pI2Cx->I2C_DR;
             pI2CHANDLE->pRxlen--;

		}
		if(pI2CHANDLE->Rxsize > 1){

			if(pI2CHANDLE->pRxlen == 2){

				pI2CHANDLE->pI2Cx->I2C_CR1 &=~(1<<10);

			}

			*pI2CHANDLE->pRxbuffer=pI2CHANDLE->pI2Cx->I2C_DR;
			 pI2CHANDLE->pRxlen--;
			pI2CHANDLE->pRxbuffer++;
		}
}}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){
	 if(EnorDi == ENABLE){
		 if(pI2Cx == I2C1){
			 I2C1_PCLK_EN();
		 }
		 if(pI2Cx == I2C2){
		  I2C2_PCLK_EN();
		 }
		 if(pI2Cx == I2C3){
		  I2C3_PCLK_EN();
		 }

	 }

	 else if(EnorDi == DISABLE){
			 if(pI2Cx == I2C1){
				 I2C1_PCLK_DI();
			 }
			 if(pI2Cx == I2C2){
			  I2C2_PCLK_DI();
			 }
			 if(pI2Cx == I2C3){
			  I2C3_PCLK_DI();
			 }

		 }

}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName){
	if((pI2Cx->I2C_SR1 & (1<<FlagName)) == 1){
		return 1;
	}
	return 0;
}


void I2C_Init(I2C_Handle_t *pI2CHANDLE){
  uint32_t temp=0;

  I2C_PeriClockControl(pI2CHANDLE->pI2Cx,ENABLE);

  temp = (pI2CHANDLE->I2C_Config.I2C_ACKControl<<10);

  pI2CHANDLE->pI2Cx->I2C_CR1 = temp;


  temp=0;

  temp=get_clock_value_APB1()/1000000U;

  pI2CHANDLE->pI2Cx->I2C_CR2 = temp & 0x3F;

  temp=0;

 temp = pI2CHANDLE->I2C_Config.I2C_DeviceAddress << 1;

 temp |=(1<<14);

 pI2CHANDLE->pI2Cx->I2C_OAR1 = temp;

 temp=0;

if(pI2CHANDLE->I2C_Config.I2C_SCLSpeed  <= I2C_SCL_Speed_SM){

	temp=(get_clock_value_APB1())/(2*pI2CHANDLE->I2C_Config.I2C_SCLSpeed);

	pI2CHANDLE->pI2Cx->I2C_CCR = temp & 0xFFF;

}


else{

	if(pI2CHANDLE->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY2){
		temp=(get_clock_value_APB1())/(3*pI2CHANDLE->I2C_Config.I2C_SCLSpeed) & 0xFFF;
		temp |= (1<<15);
	}

	else if(pI2CHANDLE->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY16_9){
		temp=(get_clock_value_APB1())/(25*pI2CHANDLE->I2C_Config.I2C_SCLSpeed) & 0xFFF;
				temp |= (1<<15);
				temp |=(1<<14);
	}

	pI2CHANDLE->pI2Cx->I2C_CCR = temp;

}

temp=0;

if(pI2CHANDLE->I2C_Config.I2C_SCLSpeed <= I2C_SCL_Speed_SM){
  temp=(get_clock_value_APB1()/1000000U)+1;
}
else{
	temp=((get_clock_value_APB1()*300)/1000000000)+1;

}

pI2CHANDLE->pI2Cx->I2C_TRISE =temp & 0x3F;








}
uint8_t I2C_Master_SenddataIT(I2C_Handle_t *pI2Chandle,uint8_t *pTxbuffer,uint32_t len,uint8_t SlaveAddress,uint8_t SR){
uint8_t Busy_state = pI2Chandle->TxorRxState;

if((Busy_state != I2C_BUSY_IN_TX) && (Busy_state != I2C_BUSY_IN_RX)){
	pI2Chandle->pTxbuffer =pTxbuffer;
	pI2Chandle->pTxlen=len;
	pI2Chandle->TxorRxState=I2C_BUSY_IN_TX;
	pI2Chandle->Deviceaddr=SlaveAddress;
	pI2Chandle->Sr=SR;

	I2C_Generate_Start_Conditon(pI2Chandle->pI2Cx);

	pI2Chandle->pI2Cx->I2C_CR2 |=(1<<8);

	pI2Chandle->pI2Cx->I2C_CR2 |=(1<<9);

	pI2Chandle->pI2Cx->I2C_CR2 |=(1<<10);



}

return Busy_state;

}

uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHANDLE,uint8_t *pRxBuffer,uint8_t len,uint8_t SlaveAddr,uint8_t SR){
	uint8_t Busy_state = pI2CHANDLE->TxorRxState;
	if((Busy_state != I2C_BUSY_IN_TX) && (Busy_state != I2C_BUSY_IN_RX)){
		pI2CHANDLE->pRxbuffer =*pRxBuffer;
		pI2CHANDLE->pRxlen=len;
		pI2CHANDLE->TxorRxState=I2C_BUSY_IN_RX;
		pI2CHANDLE->Rxsize=len;
		pI2CHANDLE->Deviceaddr=SlaveAddr;
		pI2CHANDLE->Sr=SR;

		I2C_Generate_Start_Conditon(pI2CHANDLE->pI2Cx);

		pI2CHANDLE->pI2Cx->I2C_CR2 |=(7<<8);



	}


return Busy_state;


}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		pI2Cx->I2C_CR1  |= 1;
	}
	else{
		pI2Cx->I2C_CR1 &=~(1);
	}
}


void I2C_Master_Senddata(I2C_Handle_t *pI2Chandle,uint8_t *pTxbuffer,uint32_t len,uint8_t SlaveAddress,uint8_t SR){

	I2C_Generate_Start_Conditon(pI2Chandle->pI2Cx);

	while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SR1_SB));

	I2C_GenerateAddress(pI2Chandle->pI2Cx,SlaveAddress,READ);

	while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SR1_ADDR));

	Clear_AddrFlag(pI2Chandle->pI2Cx);

	while(len > 0){

		while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx,I2C_SR1_TxE));

		pI2Chandle->pI2Cx->I2C_DR = *pTxbuffer;

		pTxbuffer++;

		len--;

	}


		while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx,I2C_SR1_TxE));

		while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx,I2C_SR1_BTF));


		if(SR=I2C_DISABLE_SR){
		I2C_Generate_Stop_Condition(pI2Chandle->pI2Cx);
		}




}

void I2C_Master_Recivedata(I2C_Handle_t *pI2Chandle,uint8_t *pRxbuffer,uint32_t len,uint8_t SlaveAddress,uint8_t SR){

	I2C_Generate_Start_Conditon(pI2Chandle->pI2Cx);

	while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SR1_SB));

	I2C_GenerateAddress(pI2Chandle->pI2Cx, SlaveAddress,WRITE);

	while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SR1_ADDR));

	if(len == 1){


		pI2Chandle->pI2Cx->I2C_CR1 &=~(1<<10);//disable ackinh

		Clear_AddrFlag(pI2Chandle->pI2Cx);

		while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SR1_RxNE));

		if(SR=I2C_DISABLE_SR){
		 I2C_Generate_Stop_Condition(pI2Chandle->pI2Cx);
		}
		 *pRxbuffer=pI2Chandle->pI2Cx->I2C_DR;

		 len--;


		 return;

	}

	else{

		Clear_AddrFlag(pI2Chandle->pI2Cx);

		while(len>2){

			while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SR1_RxNE));

			 *pRxbuffer=pI2Chandle->pI2Cx->I2C_DR;

			 pRxbuffer++;

			 len--;


		}


		while(!I2C_GetFlagStatus(pI2Chandle->pI2Cx, I2C_SR1_RxNE));


		pI2Chandle->pI2Cx->I2C_CR1 &=~(1<<10);//disable ackinh


		if(SR=I2C_DISABLE_SR){
		 I2C_Generate_Stop_Condition(pI2Chandle->pI2Cx);
		}
		 *pRxbuffer=pI2Chandle->pI2Cx->I2C_DR;

		 *pRxbuffer++;

		 len--;


	}

	pI2Chandle->pI2Cx->I2C_CR1 |=(1<<10);

}

void I2C_EV_IRQhandling(I2C_Handle_t *pI2CHANDLE){
	uint32_t temp1,temp2,temp3;

	temp1 = pI2CHANDLE->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	temp1 = pI2CHANDLE->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 =pI2CHANDLE->pI2Cx->I2C_SR1 & (1<<I2C_SR1_SB);

	if(temp1 && temp3){
		//INTERRUPT IS GENERATED BY SB EVENT
		//THE BLOCK S ALWAYS EXECUTED IN SLAVE MODE BECAUSE IN SLAVE SB IS ALWAYS ZERO
		//in thsi block execute address ohase
		if(pI2CHANDLE->TxorRxState == I2C_BUSY_IN_TX){
			I2C_GenerateAddress(pI2CHANDLE->pI2Cx,pI2CHANDLE->Deviceaddr,WRITE);
		}

		if(pI2CHANDLE->TxorRxState == I2C_BUSY_IN_RX){
			I2C_GenerateAddress(pI2CHANDLE->pI2Cx,pI2CHANDLE->Deviceaddr,READ);
		}

	}

	temp3=pI2CHANDLE->pI2Cx->I2C_SR1 & (1<<I2C_SR1_ADDR);

	//HANLDE FOR INTERRUPT GENERATED BY ADDR EVENT
	//WHEN MASTER MODE ADDRESS IS SENT
	//WHEN SLAVE MODE ADDRESS MATCHED WITH OWN ADDREESS

	if(temp1 && temp3){
        //ADDR FLAG IS SET
      Clear_AddrFlag(pI2CHANDLE->pI2Cx);
	}

	temp3=pI2CHANDLE->pI2Cx->I2C_SR1 & (1<<I2C_SR1_BTF);

		if(temp1 && temp3){

           if(pI2CHANDLE->TxorRxState == I2C_BUSY_IN_TX){
        	   if(pI2CHANDLE->pI2Cx->I2C_SR1 & (1<<I2C_SR1_TxE)){
        		   if(pI2CHANDLE->pTxlen == 0){
        		   //GENERATE THE STOP CONDITION

        		   if(pI2CHANDLE->Sr == I2C_DISABLE_SR){
        		   I2C_Generate_Stop_Condition(pI2CHANDLE->pI2Cx);
        		   }
        		   //REST ALL THE MEMBER ELEMENT
                   I2C_CloseSendData(pI2CHANDLE);
        		   //NOTIFY THE APPLICATIOON ABOUT TRANSMISSON COMPLETE
        		   I2C_ApplicationeventCallBack(pI2CHANDLE,I2C_EV_TX_CMPLT);
        	   }
           }
           }

           else if(pI2CHANDLE->TxorRxState == I2C_BUSY_IN_RX){

           }
		}


		//for stopf flag
	temp3=pI2CHANDLE->pI2Cx->I2C_SR1 & (1<<I2C_SR1_STOPF);

	if(temp1 && temp3){
       ///
	  pI2CHANDLE->pI2Cx->I2C_CR1 |=0x0000;

	  //notify the application that stop is completed

	  I2C_ApplicationeventCallBack(pI2CHANDLE,I2C_EV_STOP);

	}



	temp3=pI2CHANDLE->pI2Cx->I2C_SR1 & (1<<I2C_SR1_RxNE);

	if(temp1 && temp3 && temp2){
       //rxne flag is set
		if(pI2CHANDLE->pI2Cx->I2C_SR2 & (1<<I2C_SR2_MSL)){

			I2C_MASTER_HANDLE_RXNE_INTERRUPT(pI2CHANDLE);

		}

		else{

			if(!(pI2CHANDLE->pI2Cx->I2C_SR2 & (1<<I2C_SR2_TRA))){
				    	  I2C_ApplicationEventCallBack(pI2CHANDLE,I2C_EV_DATA_REQ);
				   }
			//
		}
		}




	temp3=pI2CHANDLE->pI2Cx->I2C_SR1 & (1<<I2C_SR1_TxE);

	if(temp1 && temp3 && temp2){
           //we have to do dats transmisson
		if(pI2CHANDLE->pI2Cx->I2C_SR2 & (1<<I2C_SR2_MSL)){
			I2C_MASTER_HANDLE_TXE_INTERRUPT(pI2CHANDLE);

		}

		else{
	      if((pI2CHANDLE->pI2Cx->I2C_SR2 & (1<<I2C_SR2_TRA))){
	    	  I2C_ApplicationEventCallBack(pI2CHANDLE,I2C_EV_DATA_TX);
	      }
		}

		}



}

void I2C_CloseSendData(I2C_Handle_t *pI2CHANDLE){

	pI2CHANDLE->pI2Cx->I2C_CR2 &=~(1<<I2C_CR2_ITBUFEN);
	pI2CHANDLE->pI2Cx->I2C_CR2 &=~(1<<I2C_CR2_ITEVTEN);

	pI2CHANDLE->pTxlen=0;
	pI2CHANDLE->pTxbuffer=NULL;
	pI2CHANDLE->TxorRxState=I2C_READY;

  if(pI2CHANDLE->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
	  pI2CHANDLE->pI2Cx->I2C_CR1 |=(1<<10);
  }
}
void I2C_CloseRecieveData(I2C_Handle_t *pI2CHANDLE){
	pI2CHANDLE->pI2Cx->I2C_CR2 &=~(1<<I2C_CR2_ITBUFEN);
	pI2CHANDLE->pI2Cx->I2C_CR2 &=~(1<<I2C_CR2_ITEVTEN);

	pI2CHANDLE->pRxlen=0;
	pI2CHANDLE->Rxsize=0;
	pI2CHANDLE->TxorRxState=I2C_READY;
	pI2CHANDLE->pRxbuffer=NULL;
	if(pI2CHANDLE->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		 pI2CHANDLE->pI2Cx->I2C_CR1 |=(1<<10);
	}
}

void I2C_ER_IRQhandling(I2C_Handle_t *pI2CHANDLE){
uint32_t  temp1,temp2;

temp2=(pI2CHANDLE->pI2Cx->I2C_CR2)&(1<<I2C_CR2_ITERREN);


temp1=(pI2CHANDLE->pI2Cx->I2C_SR2)&(1<<I2C_SR1_BERR);

if(temp1 & temp2){

	pI2CHANDLE->pI2Cx->I2C_SR2 &=~(1<<I2C_SR1_BERR);

	I2C_ApplicationEventCallBack(pI2CHANDLE, I2C_ERROR_BERR);
}



temp1=(pI2CHANDLE->pI2Cx->I2C_SR2)&(1<<I2C_SR1_ARLO);

if(temp1 & temp2){

	pI2CHANDLE->pI2Cx->I2C_SR2 &=~(1<<I2C_SR1_ARLO);

	I2C_ApplicationEventCallBack(pI2CHANDLE, I2C_ERROR_ARLO);
}


temp1=(pI2CHANDLE->pI2Cx->I2C_SR2)&(1<<I2C_SR1_AF);

if(temp1 & temp2){

	pI2CHANDLE->pI2Cx->I2C_SR2 &=~(1<<I2C_SR1_AF);

	I2C_ApplicationEventCallBack(pI2CHANDLE, I2C_ERROR_AF);
}


temp1=(pI2CHANDLE->pI2Cx->I2C_SR2)&(1<<I2C_SR1_OVR);

if(temp1 & temp2){

	pI2CHANDLE->pI2Cx->I2C_SR2 &=~(1<<I2C_SR1_OVR);

	I2C_ApplicationEventCallBack(pI2CHANDLE, I2C_ERROR_OVR);
}

temp1=(pI2CHANDLE->pI2Cx->I2C_SR2)&(1<<I2C_SR1_TIMEOUT);

if(temp1 & temp2){

	pI2CHANDLE->pI2Cx->I2C_SR2 &=~(1<<I2C_SR1_TIMEOUT);

	I2C_ApplicationEventCallBack(pI2CHANDLE, I2C_ERROR_TIMEOUT);
}




}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data){
	pI2Cx->I2C_DR=data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx){
	return (uint8_t)pI2Cx->I2C_DR;
}











