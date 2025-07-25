/*
 * stm32f411xx_spi_driver.h
 *
 *
 *  Created on: Jul 2, 2024
 *      Author: ASUS
 */





#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include<stm32f411xx.h>

typedef struct
{
 uint8_t SPI_DeviceMode;
 uint8_t SPI_BusConfig;
 uint8_t SPI_SclkSpeed;
 uint8_t SPI_DFF;
 uint8_t SPI_CPOL;
 uint8_t SPI_CPHA;
 uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t  *pTxbuffer;
	uint8_t *pRxbuffer;
	uint32_t Rxlen;
	uint32_t Txlen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;


#define SPI_READY 0
#define SPI_BUSY_IN_TX 1
#define SPI_BUSY_IN_RX 2


//PSSSOIBLE SPI APPLICATION
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4


#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

#define SPI_BUS_FD               1
#define SPI_BUS_HD               2
#define SPI_BUS_SIMPLEX_RX_ONLY  3

//SPI_SCL_SPEED
#define SPI_SCLSPEED_DIV2    0
#define SPI_SCLSPEED_DIV4    1
#define SPI_SCLSPEED_DIV8    2
#define SPI_SCLSPEED_DIV16   3
#define SPI_SCLSPEED_DIV32   4
#define SPI_SCLSPEED_DIV64   5
#define SPI_SCLSPEED_DIV128  6
#define SPI_SCLSPEED_DIV256  7

//DFF
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BIT 1

//CPOL
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

//CPHA

#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

//SSM

#define SPI_SSM_EN 1
#define SPI_SSM_DI 0




void SPI_perclockcontrol(SPI_RegDef_t *pSPIHandle,uint8_t EnorDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);

void SPI_DeInit(SPI_Handle_t *pSPIHandle);

//date send and recieve

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);

void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len);

uint8_t SPI_RecieveData_IT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len);

void SPI_IRQInterrupConfig(uint8_t IRQNumber,uint8_t EnorDi);

void SPI_IRQpriority(uint8_t IRQNumber,uint32_t IRQPriority);

void SPI_IRQHandling(SPI_Handle_t *pHandle);

//other peripheraal control

void SPI_Peripheral_last(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

void SSI_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

__attribute__((weak)) void  SPI_ApplicationEventCallBack(SPI_Handle_t *pHandle,uint8_t APP);

void SSIOE_Config(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

void SPI_Clear_OVR_Flag(SPI_Handle_t *pHandle);

void SPI_Close_Reception(SPI_Handle_t *pHandle);

void SPI_Close_transmisson(SPI_Handle_t *pHandle);

#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
