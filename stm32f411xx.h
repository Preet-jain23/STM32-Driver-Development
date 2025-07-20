/*
 * stm32f411xx.h
 *
 *  Created on: Jun 22, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include<stdint.h>
#include<stddef.h>
#define _vo volatile

///ARM CORTEX M4 PROCESSOR
//ISER REGISTOR ADDRESS

#define NVIC_ISER0               ((_vo uint32_t*)0xE000E100)
#define NVIC_ISER1               ((_vo uint32_t*)0xE000E104)
#define NVIC_ISER2               ((_vo uint32_t*)0xE000E108)
#define NVIC_ISER3               ((_vo uint32_t*)0xE000E10C)

//ICER REGISTOR ADDRESS
#define NVIC_ICER0               ((_vo uint32_t*)0xE000E180)
#define NVIC_ICER1               ((_vo uint32_t*)0xE000E184)
#define NVIC_ICER2               ((_vo uint32_t*)0xE000E188)
#define NVIC_ICER3               ((_vo uint32_t*)0xE000E19C)

#define NVIC_PR_BASE_ADDR        ((_vo uint32_t*)0xE000E400)


#define no_prio_bit_imp                  4





















#define FLASH_BASEADDR              0x08000000U
#define SRAM1_BASEADDR              0x20000000U

#define ROM_BASEADDR                0x1FFF0000U
#define SRAM                        SRAM1_ADDRESS


#define PERIPH_BASE                     0x40000000U
#define APB1_PERIPHBASEADDR             PERIPH_BASE
#define APB2_PERIPHBASEADDR             0x40010000U
#define AHB1_PERIPHBASEADDR             0x40020000U
#define AHB2_PERIPHBASEADDR             0x50000000U

#define GPIOA_BASEADDR              (AHB1_PERIPHBASEADDR + 0x0000)
#define GPIOB_BASEADDR              (AHB1_PERIPHBASEADDR + 0x0400)
#define GPIOC_BASEADDR              (AHB1_PERIPHBASEADDR + 0x0800)
#define GPIOD_BASEADDR              (AHB1_PERIPHBASEADDR + 0x0C00)
#define GPIOE_BASEADDR              (AHB1_PERIPHBASEADDR + 0x1000)
#define GPIOH_BASEADDR              (AHB1_PERIPHBASEADDR + 0x1C00)
#define RCC_BASEADDR                (AHB1_PERIPHBASEADDR + 0x3800)


#define I2C1_BASEADDR               (APB1_PERIPHBASEADDR + 0x5400)
#define I2C2_BASEADDR               (APB1_PERIPHBASEADDR + 0x5800)
#define I2C3_BASEADDR               (APB1_PERIPHBASEADDR + 0x5C00)

#define SPI2_BASEADDR               (APB1_PERIPHBASEADDR + 0x3800)
#define SPI3_BASEADDR               (APB1_PERIPHBASEADDR + 0x3C00)
#define USART2_BASEADDR             (APB1_PERIPHBASEADDR + 0x4400)


#define USART1_BASEADDR             (APB2_PERIPHBASEADDR +  0x1000)
#define USART6_BASEADDR             (APB2_PERIPHBASEADDR +  0x1400)
#define SPI1_BASEADDR               (APB2_PERIPHBASEADDR +  0x3000)
#define SPI4_BASEADDR               (APB2_PERIPHBASEADDR +  0x3400)
#define EXTI_BASEADDR               (APB2_PERIPHBASEADDR +  0x3C00)  // 0x4001 3C00
#define SYSCFG_BASEADDR             (APB2_PERIPHBASEADDR +  0x3800)                      //3800


//PERIPHERAL DEFINATION OF GPIO REGISTOR
typedef struct{

	_vo uint32_t MODER;                 //gpio mode registor
	_vo uint32_t OTYPER;
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFR[2];


}GPIO_RegDef_t;

//PERIPHERAL DEFINATION OF RCC REGISTOR

typedef struct{
	_vo uint32_t CR;
	_vo uint32_t PLLCFGR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t AHB1RSTR;
	_vo uint32_t AHB2RSTR;
	    uint32_t RESERVED1[2];
    _vo uint32_t APB1RSTR;
	_vo uint32_t APB2RSTR;
	    uint32_t RESERVED2[2];
	_vo uint32_t AHB1ENR;
	_vo uint32_t AHB2ENR;
	    uint32_t RESERVED3[2];
	_vo uint32_t APB1ENR;
	_vo uint32_t APB2ENR;
	    uint32_t RESERVED4[2];
	_vo uint32_t AHB1LPENR;
	_vo uint32_t AHB2LPENR;
	    uint32_t RESERVED5[2];
	_vo uint32_t APB1LPENR;
    _vo uint32_t APB2LPENR;
        uint32_t RESERVED6[2];
    _vo uint32_t BDCR;
	_vo uint32_t CSR;
	    uint32_t RESERVED7[2];
    _vo uint32_t SSCGR;
	_vo uint32_t PLLI2SCFGR;
	_vo uint32_t DCKCFGR;

}RCC_RegDef_t;




//peripheral register structure for exti
typedef struct{
	_vo  uint32_t   IMR;
	_vo  uint32_t   EMR;
	_vo  uint32_t   RTSR;
	_vo  uint32_t   FTSR;
	_vo  uint32_t   SWIER;
	_vo uint32_t    PR;

}EXTI_RegDef_t;//EXTI_RegDef_t

//PERIPHERAL REGISTER STRUCTURE FOR SYSCFG;

typedef struct{
	_vo  uint32_t   MEMRMP;
	_vo  uint32_t   PMC;
	_vo  uint32_t   EXTICR[4];
	_vo  uint32_t   RESERVED[2];
	_vo  uint32_t   CMPCR;

}SYSCFG_RegDef_t;


//PERIPHERAL REGISTER DEFINATION FOR SPI REGISTER

typedef struct{
	_vo  uint32_t  SPI_CR1;
	_vo  uint32_t  SPI_CR2;
	_vo  uint32_t  SPI_SR;
	_vo  uint32_t  SPI_DR;
	_vo  uint32_t  SPI_CRCPR;
	_vo  uint32_t  SPI_RXCRCR;
	_vo  uint32_t  SPI_TXCRCR;
	_vo  uint32_t  SPI_I2SCFGR;
	_vo  uint32_t  SPI_I2SPR;

}SPI_RegDef_t;


typedef struct{
	_vo  uint32_t  I2C_CR1;
	_vo  uint32_t  I2C_CR2;
	_vo  uint32_t  I2C_OAR1;
	_vo  uint32_t  I2C_OAR2;
	_vo  uint32_t  I2C_DR;
	_vo  uint32_t  I2C_SR1;
	_vo  uint32_t  I2C_SR2;
	_vo  uint32_t  I2C_CCR;
	_vo  uint32_t  I2C_TRISE;
	_vo  uint32_t  I2C_FLTR;
}I2C_RegDef_t;


typedef struct{
	_vo uint32_t USART_SR;
	_vo uint32_t USART_DR;
	_vo uint32_t USART_BRR;
	_vo uint32_t USART_CR1;
	_vo uint32_t USART_CR2;
	_vo uint32_t USART_CR3;
	_vo uint32_t USART_GTPR;

}USART_RegDef_t;


//some generic macros

#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET
#define FLAG_RESET      RESET
#define FLAG_SET        SET


 //CLOCK ENABLE MACROS FOR GPIOX PERIPHERALS;
#define GPIOA_PCLK_EN()      (RCC->AHB1ENR|=(1<<0))
#define GPIOB_PCLK_EN()      (RCC->AHB1ENR|=(1<<1))
#define GPIOC_PCLK_EN()      (RCC->AHB1ENR|=(1<<2))
#define GPIOD_PCLK_EN()      (RCC->AHB1ENR|=(1<<3))
#define GPIOE_PCLK_EN()      (RCC->AHB1ENR|=(1<<4))
#define GPIOH_PCLK_EN()      (RCC->AHB1ENR|=(1<<7))

//CLOCK ENABLE MACROS FOR I2C PERIHERALS;
#define I2C1_PCLK_EN()      (RCC->APB1ENR|=(1<<21))
#define I2C2_PCLK_EN()      (RCC->APB1ENR|=(1<<22))
#define I2C3_PCLK_EN()      (RCC->APB1ENR|=(1<<23))

//CLOCK ENABLE MACROS FOR USARTX PERIPHERALS
#define USART1_PCLK_EN()      (RCC->APB2ENR|=(1<<4))
#define USART6_PCLK_EN()      (RCC->APB2ENR|=(1<<5))
#define USART2_PCLK_EN()      (RCC->APB1ENR|=(1<<17))

//CLOCK ENABLE MACROS FOR SPIX PERIPHERALS
#define SPI1_PCLK_EN()        (RCC->APB2ENR|=(1<<12))
#define SPI2_PCLK_EN()        (RCC->APB1ENR|=(1<<14))
#define SPI3_PCLK_EN()        (RCC->APB1ENR|=(1<<15))
#define SPI4_PCLK_EN()        (RCC->APB2ENR|=(1<<13))


//CLOCK ENABLE MACROS FOR SYSCFG PERIPHERALS

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR|=(1<<14))



//CLOCK DISABLE
//CLOCK DISABLE MACROS FOR GPIOX PERIPHERALS;
#define GPIOA_PCLK_DI()      (RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI()      (RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI()      (RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI()      (RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI()      (RCC->AHB1ENR &=~(1<<4))
#define GPIOH_PCLK_DI()      (RCC->AHB1ENR &=~(1<<7))

//CLOCK DISABLE MACROS FOR I2C PERIHERALS;
#define I2C1_PCLK_DI()      (RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DI()      (RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DI()      (RCC->APB1ENR &=~(1<<23))

//CLOCK DISABLE MACROS FOR USARTX PERIPHERALS
#define USART1_PCLK_DI()      (RCC->APB2ENR &=~(1<<4))
#define USART6_PCLK_DI()      (RCC->APB2ENR &=~(1<<5))
#define USART2_PCLK_DI()      (RCC->APB1ENR &=~(1<<17))

//CLOCK DISABLE MACROS FOR SPIX PERIPHERALS
#define SPI1_PCLK_DI()        (RCC->APB2ENR &=~(1<<12))
#define SPI2_PCLK_DI()        (RCC->APB1ENR &=~(1<<14))
#define SPI3_PCLK_DI()        (RCC->APB1ENR &=~(1<<15))
#define SPI4_PCLK_DI()        (RCC->APB2ENR &=~(1<<13))

//CLOCK DISABLE MACROS FOR SYSCFG PERIPHERALS

#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &=~(1<<14))



//PERI[HERAL DEFINATION OF SPI PEROPHERAL

#define SPI1            ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2            ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3            ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4            ((SPI_RegDef_t*)SPI4_BASEADDR)


#define I2C1             ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2             ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3             ((I2C_RegDef_t*)I2C3_BASEADDR)


#define USART1      ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2      ((USART_RegDef_t*)USART2_BASEADDR)
#define USART6      ((USART_RegDef_t*)USART6_BASEADDR)



//peripeheral defination base addres tupecasted

#define GPIOA           ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB           ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC           ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD           ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE           ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH           ((GPIO_RegDef_t*)GPIOH_BASEADDR)


#define RCC              ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI             ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG           ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define GPIOA_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<0));     (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()      do{(RCC->AHB1RSTR |= (1<<1));     (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<2));    (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<3));    (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<4));    (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()       do{(RCC->AHB1RSTR |= (1<<7));    (RCC->AHB1RSTR &= ~(1<<7));}while(0)



//IRQ(INTERRUPT REQUEST NUMBER)
#define IRQ_NO_EXTI0         6
#define IRQ_NO_EXTI1         7
#define IRQ_NO_EXTI2         8
#define IRQ_NO_EXTI3         9
#define IRQ_NO_EXTI4         10
#define IRQ_NO_EXTI9_5       23
#define IRQ_NO_EXTI15_10     40
#define IRQ_NO_SPI1			 35
#define IRQ_NO_SPI2			 36
#define IRQ_NO_SPI3			 51

#define IRQ_NO_I2C1_EV       31
#define IRQ_NO_I2C1_ER       32
#define IRQ_NO_I2C2_EV       33
#define IRQ_NO_I2C2_ER       34
#define IRQ_NO_I2C3_EV       79
#define IRQ_NO_I2C3_ER       80



//IRQ PRIORITY

#define NVIC_IRQ_PRI_15    15
#define NVIC_IRQ_PRI_0     0


//bit position of spi peropheral

#define SPI_CR1_CPHA        0
#define SPI_CR1_CPOL        1
#define SPI_CR1_MSTR       2
#define SPI_CR1_BR          3
#define SPI_CR1_SPE         6
#define SPI_CR1_SSI         8
#define SPI_CR1_SSM         9
#define SPI_CR1_RXONLY      10
#define SPI_CR1_DFF         11
#define SPI_CR1_CRCNEXT     12
#define SPI_CR1_CRCEN       13
#define SPI_CR1_BIDIOE      14
#define SPI_CR1_BIDIMODE    15


//BIT DEFINATION MACROS FOR CR2 REGISTER

#define SPI_CR2_TXEIE        7
#define SPI_CR2_RXNEIE       6
#define SPI_CR2_ERRIE        5
#define SPI_CR2_FRF          4
#define SPI_CR2_SSOE         2
#define SPI_CR2_TXDMAEN      1
#define SPI_CR2_RXDMAEN      0


//BIT DEFINATION FOR STATUS REGISTER
#define SPI_SR_FRE           8
#define SPI_SR_BSY           7
#define SPI_SR_OVR           6
#define SPI_SR_MODF          5
#define SPI_SR_CRCERR        4
#define SPI_SR_UDR           3
#define SPI_SR_CHSIDE        2
#define SPI_SR_TXE           1
#define SPI_SR_RXNE          0




//bit defination for i2c registor


///bit defination for cr1 resisitor
//#define I2C_CR1_

#define I2C_CR1_SWRST     15
#define I2C_CR1_ALERT     13
#define I2C_CR1_PEC       12
#define I2C_CR1_POS       11
#define I2C_CR1_ACK       10
#define I2C_CR1_STOP       9
#define I2C_CR1_START      8
#define I2C_CR1_NOSTRETCH  7
#define I2C_CR1_ENGC       6
#define I2C_CR1_ENPEC      5
#define I2C_CR1_ENARP      4
#define I2C_CR1_SMBTYPE    3
#define I2C_CR1_SMBUS      1
#define I2C_CR1_PE         0


//bit definaton for cr2 resistor

#define I2C_CR2_LAST       12
#define I2C_CR2_DMAEN      11
#define I2C_CR2_ITBUFEN    10
#define I2C_CR2_ITEVTEN     9
#define I2C_CR2_ITERREN     8
#define I2C_CR2_FREQ5_0   1

//BIT DEFINATION OF OAR REGISTOR
#define I2C_OAR1_ADD0       0
#define I2C_OAR1_ADD71      1
#define I2C_OAR1_ADD98      8
#define I2C_OAR1_ADDMODE    15

///BIT DEFINATION OF DATA REGISTER

#define I2C_DR_DR70         0

//BIT DEFINATION OF SR1 REGISTER

#define I2C_SR1_SMBALERT    15
#define I2C_SR1_TIMEOUT     14
#define I2C_SR1_PECERR      12
#define I2C_SR1_OVR         11
#define I2C_SR1_AF          10
#define I2C_SR1_ARLO         9
#define I2C_SR1_BERR         8
#define I2C_SR1_TxE          7
#define I2C_SR1_RxNE         6
#define I2C_SR1_STOPF        4
#define I2C_SR1_ADD10        3
#define I2C_SR1_BTF          2
#define I2C_SR1_ADDR         1
#define I2C_SR1_SB           0

//BIT DEFINATION FOR SR2 REGISTER
#define I2C_SR2_PEC70          8
#define I2C_SR2_DUALF          7
#define I2C_SR2_SMBHOST        6
#define I2C_SR2_SMBDEFAULT     5
#define I2C_SR2_GENCALL        4
#define I2C_SR2_TRA            2
#define I2C_SR2_BUSY           1
#define I2C_SR2_MSL            0

//BIT DEFINATION FOR CCR REGISTER

#define I2C_CCR_FS            15
#define I2C_CCR_DUTY           14
#define I2C_CCR_CCR110          0

//BIT DEFINATOION FOR TRISE REGISTER

#define I2C_ERROR_BERR 3
#define I2C_


//enable or disable sr

#define I2C_DISABLE_SR 0
#define I2C_ENABLE_SR  1

#define I2C_ERROR_BERR  3
#define I2C_ERROR_ARLO  4
#define I2C_ERROR_AF    5
#define I2C_ERROR_OVR   6
#define I2C_ERROR_TIMEOUT 7


#define I2C_EV_DATA_TX 8
#define I2C_EV_DATA_REQ 9

//USART REGISTER DEFINATION

#define USART_CR1_SBK   0
#define USART_CR1_RWU   1
#define USART_CR1_RE    2
#define USART_CR1_TE    3
#define USART_CR1_IDLEIE   4
#define USART_CR1_RXNEIE 5
#define USART_CR1_TCIE  6
#define USART_CR1_TXEIE 7
#define USART_CR1_PEIE  8
#define USART_CR1_PS    9
#define USART_CR1_PCE   10
#define USART_CR1_WAKE  11
#define USART_CR1_M     12
#define USART_CR1_UE    13
#define USART_CR1_OVER8    15



#define USART_CR2_LBDL  5
#define USART_CR2_LBDIE 6
#define USART_CR2_LBCL  8
#define USART_CR2_CPHA  9
#define USART_CR2_CPOL  10
#define USART_CR2_CLKEN 11
#define USART_CR2_STOP  13
#define USART_CR2_LINMEN 14


#define USART_CR3_EIE    0
#define USART_CR3_IREN   1
#define USART_CR3_IRLP   2
#define USART_CR3_HDSEL  3
#define USART_CR3_NACK   4
#define USART_CR3_SCEN   5
#define USART_CR3_DMAR   6
#define USART_CR3_DMAT   7
#define USART_CR3_RTSE   8
#define USART_CR3_CTSE   9
#define USART_CR3_CTSIE  10
#define USART_CR3_ONEBIT 11



#define USART_SR_PE    0
#define USART_SR_FE    1
#define USART_SR_NF    2
#define USART_SR_ORE    3
#define USART_SR_IDLE    4
#define USART_SR_RXNE    5
#define USART_SR_TC    6
#define USART_SR_TXE    7
#define USART_SR_LBD    8
#define USART_SR_CTS    9


#define READ 0
#define WRITE 1






#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_i2c_driver.h"
#include "stm32f411xx_usart_driver.h"

#endif /* INC_STM32F411XX_H_ */
