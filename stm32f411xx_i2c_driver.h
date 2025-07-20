/*
 * stm32f411xx_i2c_driver.h
 *
 *  Created on: Jul 9, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F411XX_I2C_DRIVER_H_
#define INC_STM32F411XX_I2C_DRIVER_H_


#include "stm32f411xx.h"

typedef struct{

	uint32_t  I2C_SCLSpeed;
	uint8_t   I2C_DeviceAddress;//slave devie address is of 7 bit
	uint8_t   I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t  I2C_Config;

	uint8_t *pRxbuffer;//TO STORE THE RX BUFFER ADDRESS
	uint8_t *pTxbuffer;//TO STORE THE TX BUFFER ADDRESS
	uint32_t pRxlen;//TO STORE THE RX LEN
	uint32_t pTxlen;//TO STORE THE TX LEN
	uint8_t   TxorRxState;//TO STORE THE STATE OF COMMUNICATION WHETHER IT IS HALF DUPLEX OR FULL DUPLEX
	//because in i2c if you are sending you are not receving
	uint8_t   Deviceaddr;//TO STORE THE DEVICE/SLAVE ADDRESS
	uint8_t   Rxsize;//TO STORERXX SIZE
	uint8_t Sr; //TO STORE REPERATED START VALUE*/

}I2C_Handle_t;

#define I2C_READY                       0
#define I2C_BUSY_IN_RX                  1
#define I2C_BUSY_IN_TX                  2


#define I2C_SCL_Speed_SM      100000
#define I2C_SCL_Speed_FM4K    400000
#define I2C_SCL_SPEED_FM2K    200000



#define I2C_ACK_ENABLE       1
#define I2C_ACK_DISABLE      0


#define I2C_FM_DUTY2         0
#define I2C_FM_DUTY16_9      1


#define I2C_FLAG_TxE      (1<< I2C_SR1_TxE)
#define I2C_FLAG_RxNE        (1<<I2C_SR1_RxNE)
#define I2C_FLAG_SB       (1<<I2C_SR1_SB)
#define I2C_FLAG_ADDR     (1<<I2C_SR1_ADDR)
#define I2C_FLAG_BTF     (1<<I2C_SR1_BTF)



#define I2C_EV_STOP        0
#define I2C_EV_TX_CMPLT     1
#define I2C_EV_RX_CMPLT    2


#define I2C_ERROR_BERR     3
#define I2C_ERROR_ARLO     4
#define I2C_ERROR_AF       5
#define I2C_ERROR_OVR      6
#define I2C_ERROR_TIMEOUT  7



//PERIOHERAL CLOCK SETUP
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

//init and de_init

void I2C_Init(I2C_Handle_t *pI2CHANDLE);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

//master send and recieve

void I2C_Master_Senddata(I2C_Handle_t *pI2Chandle,uint8_t *pTxbuffer,uint32_t len,uint8_t SlaveAddress,uint8_t SR);
void I2C_Master_Recivedata(I2C_Handle_t *pI2Chandle,uint8_t *pRxbuffer,uint32_t len,uint8_t SlaveAddress,uint8_t SR);
//IN THIS SR DENOTE FOR REPEATED START

uint8_t I2C_Master_SenddataIT(I2C_Handle_t *pI2Chandle,uint8_t *pTxbuffer,uint32_t len,uint8_t SlaveAddress,uint8_t SR);
uint8_t I2C_Master_RecivedataIT(I2C_Handle_t *pI2Chandle,uint8_t *pRxbuffer,uint32_t len,uint8_t SlaveAddress,uint8_t SR);
//MASTER SEND DATA AND RECIVE DATA USING INTERRUPT

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHANDLE,uint8_t *pTxBuffer,uint8_t len,uint8_t SlaveAddr,uint8_t SR);
uint8_t I2C_MasterRecieveDataIT(I2C_Handle_t *pI2CHANDLE,uint8_t *pRxBuffer,uint8_t len,uint8_t SlaveAddr,uint8_t SR);


//IRQ CONFIG AND I2C PRIORITY

void I2C_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);


//IRQ HANDLING AND IRQ INTERRUPT HANDLING

void I2C_EV_IRQhandling(I2C_Handle_t *pI2CHANDLE);
void I2C_ER_IRQhandling(I2C_Handle_t *pI2CHANDLE);


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);



///OTHERT CONTROL

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);








//OTHER PERIPHERAK CONTROL APIS

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName);
uint32_t get_clock_value_apb1(void);



void I2C_ApplicationEventCallBack(I2C_Handle_t*pI2CHandle , uint8_t AppEv);












#endif /* INC_STM32F411XX_I2C_DRIVER_H_ */



























//////////


