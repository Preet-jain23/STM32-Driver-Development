/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Jun 22, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"



typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;             //  @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;            //  @GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;      //  @GPIO_PIN_MODES
	uint8_t GPIO_Pin0PType;           //  @GPIO_PIN_MODES
	uint8_t GPIO_PinAltFunMode;       //  @GPIO_PIN_MODES

}GPIO_PinConfig_t;



typedef struct{

	GPIO_RegDef_t *pGPIOx;//this hold the base addres of the gpio port to which the pin belong
	GPIO_PinConfig_t GPIO_PinConfig;//this hold the gpio configuration setting

}GPIO_Handle_t;


/*********************************
 * GPIO_PIN_MODES
 * GPIO PIN POSSIBLE MODE
 */


#define GPIO_MODE_IN         0
#define GPIO_MODE_OUT        1
#define GPIO_MODE_ALTFN      2
#define GPIO_MODE_ANALOG     3
#define GPIO_MODE_IT_FT      4
#define GPIO_MODE_IT_RT      5
#define GPIO_MODE_IT_RFT     6


/*
 * GPIO POSSIBLE OUTPUT TYPE
 */

#define GPIO_OP_TYPE_PP    0
#define GPIO_OP_TYPE_OD    1


/*
 * GPIO port output speed register
 */

#define GPIO_SPEED_LOW           0
#define GPIO_SPEED_MEDIUM        1
#define GPIO_SPEED_FAST          2
#define GPIO_SPEED_HIGH          3

/*
 * GPIO port pull-up/pull-down register
 */

#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO_PIN_PD         2


//GPIO PIN NUMBER

#define GPIO_PIN_NO_0          0
#define GPIO_PIN_NO_1          1
#define GPIO_PIN_NO_2          2
#define GPIO_PIN_NO_3          3
#define GPIO_PIN_NO_4          4
#define GPIO_PIN_NO_5          5
#define GPIO_PIN_NO_6          6
#define GPIO_PIN_NO_7          7
#define GPIO_PIN_NO_8          8
#define GPIO_PIN_NO_9          9
#define GPIO_PIN_NO_10         10
#define GPIO_PIN_NO_11         11
#define GPIO_PIN_NO_12         12
#define GPIO_PIN_NO_13         13
#define GPIO_PIN_NO_14         14
#define GPIO_PIN_NO_15         15


//peripheral clock

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

//init and de_init

void GPIO_Init(GPIO_Handle_t *pGPIOHANDLE);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//data read and write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputport(GPIO_RegDef_t *pGPIOx);
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

//IRQ configuartion

void GPIO_IRQConfig(uint32_t IRQNumber ,uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint32_t IRQNumber ,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);//we use only pin number becaouse we have to know fron which pin interupt is comming








#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
