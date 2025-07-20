/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: Jun 22, 2024
 *      Author: ASUS
 */

/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: Jun 22, 2024
 *      Author: ASUS
 */

#include "stm32f411xx_gpio_driver.h"


#include "stm32f411xx.h"

//peripheral clock
/************************************************************
 * @fn   -GPIO_PeriClockControl
 *
 * @brief  -this function enable or disables peripheral clock for the give gpio port
 *
 * @param[in]    -base address of the gpio port
 * @param[in]    -enable or diable macros
 *
 * @return       -none
 * @note         -none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
    if(EnorDi == ENABLE){
              if(pGPIOx == GPIOA){
        	  GPIOA_PCLK_EN();
          }
          else if(pGPIOx == GPIOB){
        	  GPIOB_PCLK_EN();
          }

          else if(pGPIOx == GPIOC){
                GPIOC_PCLK_EN();
           }
          else if(pGPIOx == GPIOD){
                GPIOD_PCLK_EN();
           }
          else if(pGPIOx == GPIOE){
                GPIOE_PCLK_EN();
           }
          else if(pGPIOx == GPIOH){
                  	  GPIOH_PCLK_EN();
           }
    }
    else{
    	         if(pGPIOx == GPIOA){
    	        	  GPIOA_PCLK_DI();
    	          }
    	          else if(pGPIOx == GPIOB){
    	        	  GPIOB_PCLK_DI();
    	          }

    	          else if(pGPIOx == GPIOC){
    	                  	  GPIOC_PCLK_DI();
    	           }
    	          else if(pGPIOx == GPIOD){
    	                  	  GPIOD_PCLK_DI();
    	           }
    	          else if(pGPIOx == GPIOE){
    	                  	  GPIOE_PCLK_DI();
    	           }
    	          else if(pGPIOx == GPIOH){
    	                  	  GPIOH_PCLK_DI();
    	           }
    }
}

//init and de_init

void GPIO_Init(GPIO_Handle_t *pGPIOHANDLE){
  uint32_t  temp=0;


  //ENABLE THE PERIPHRL CLOCK

  GPIO_PeriClockControl(pGPIOHANDLE->pGPIOx, ENABLE);
	//1.CONFIGURE THE MODE OF GPIO PIN
    if(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
	temp=pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
    	pGPIOHANDLE->pGPIOx->MODER &=~(0x3 << 2*pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
    	pGPIOHANDLE->pGPIOx->MODER|=temp;
    }
    else{
        if(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
        	//configure the FTSR
         EXTI->FTSR|=(1<<(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
         //clear the RTSR
         EXTI->RTSR &=~(1<<(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));


        }
        else if(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
        	     //CLEAR the FTSR
        	         EXTI->FTSR &=~(1<<pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
        	         //CONFIGURE the RTSR
        	         EXTI->RTSR |=(1<<pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
        }
        else if(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
              //CONFIGURE THE FTSR AND STSR
        	//CONFIGURE THE FTSR
        	  EXTI->FTSR |=(1<<pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
        	   //CONFIGURE the RTSR
        	   EXTI->RTSR |=(1<<pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
        }



        //2 CONFIGURE THE GPIO PORT SECTION IN SYSC_EXTICR

        uint8_t temp1=pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber/4;
        uint8_t temp2=pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber%4;

               SYSCFG_PCLK_EN();


                   if(pGPIOHANDLE->pGPIOx== GPIOA){
                	   SYSCFG->EXTICR[temp1] |=(0 << 4*temp2);
            	          }
            	          else if(pGPIOHANDLE->pGPIOx == GPIOB){
            	        	  SYSCFG->EXTICR[temp1] |=(1 << 4*temp2);
            	          }

            	          else if(pGPIOHANDLE->pGPIOx== GPIOC){
            	        	  SYSCFG->EXTICR[temp1] |=(2 << 4*temp2);
            	           }
            	          else if(pGPIOHANDLE->pGPIOx== GPIOD){
            	        	  SYSCFG->EXTICR[temp1] |=(3 << 4*temp2);
            	           }
            	          else if(pGPIOHANDLE->pGPIOx== GPIOE){
            	        	  SYSCFG->EXTICR[temp1] |=(4 << 4*temp2);
            	           }
            	          else if(pGPIOHANDLE->pGPIOx == GPIOH){
            	        	  SYSCFG->EXTICR[temp1] |=(7 << 4*temp2);
            	           }

    }

	temp=0;

  //2 CONFIGURE THE SPEED
    temp=(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHANDLE->pGPIOx->OSPEEDR &=~(0x3 << (2 * pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHANDLE->pGPIOx->OSPEEDR|=temp;

    temp=0;


  //3 CONFIGURE THE PUPD SETTINGS
     temp=pGPIOHANDLE->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
     pGPIOHANDLE->pGPIOx->PUPDR &=~(0x3 <<(2*pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber));
     pGPIOHANDLE->pGPIOx->PUPDR|=temp;

     temp=0;
  //4 CONFIGURE THE OPTTYPE

     temp=pGPIOHANDLE->GPIO_PinConfig.GPIO_Pin0PType<<(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
     pGPIOHANDLE->pGPIOx->OTYPER &=~(0x1 << pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber);
     pGPIOHANDLE->pGPIOx->OTYPER|=temp;



  //5 CONFIGURE THE ALT FUNCTIONALITY
     temp=0;

     if(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
    	 uint8_t temp1=0,temp2=0;
    	 temp1=(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber)/8;
    	 temp2=(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber)%8;
    	pGPIOHANDLE->pGPIOx->AFR[temp1] &=~(0xF << 4*temp2);
         pGPIOHANDLE->pGPIOx->AFR[temp1]|=(pGPIOHANDLE->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));

     /*
    	 if ( pGPIOHANDLE->GPIO_PinConfig.GPIO_PinNumber  <= 7 )
    		{
    			pGPIOHANDLE->pGPIOx->AFR[0] |= 128;
    		}
    		else
    		{
    			pGPIOHANDLE->pGPIOx->AFR[1] |= 128;
    			//GPIOx->AFR[1] |= (alt_fun_value << ( ( pin_no % 8) * 4 ));
    		}
    		*/}



}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	          if(pGPIOx == GPIOA){
	        	  GPIOA_REG_RESET();

	          }
	          else if(pGPIOx == GPIOB){
	        	  GPIOB_REG_RESET();
	          }

	          else if(pGPIOx == GPIOC){
	        	  GPIOC_REG_RESET();
	           }
	          else if(pGPIOx == GPIOD){
	        	  GPIOD_REG_RESET();
	           }
	          else if(pGPIOx == GPIOE){
	        	  GPIOE_REG_RESET();
	           }
	          else if(pGPIOx == GPIOH){
	        	  GPIOH_REG_RESET();
	           }

}

//data read and write

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){


	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

//gpio read fron input pint
uint16_t GPIO_ReadFromInputport(GPIO_RegDef_t *pGPIOx){
	    uint16_t value;
		value = (uint16_t)pGPIOx->IDR;
		return value;

}

//
void GPIO_WritetoOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
	if(Value == 1){
		pGPIOx->ODR |=(1<<PinNumber);
	}
	else{
		pGPIOx->ODR &=~(1<<PinNumber);
	}

}
void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx , uint16_t Value){
	pGPIOx->ODR =Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
     uint32_t temp=0;
	temp=(pGPIOx->ODR ^ (1<<PinNumber));
	pGPIOx->ODR=temp;
       return;
}

//IRQ configuartion

void GPIO_IRQConfig(uint32_t IRQNumber,uint8_t EnorDi)
{
	if(EnorDi == ENABLE){

		if(IRQNumber <=31){
			*NVIC_ISER0 |=(1<<IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber <64){
					*NVIC_ISER1 |=(1<<(IRQNumber%32));
		}
		else if(IRQNumber >=64 && IRQNumber < 96){
					*NVIC_ISER2 |=(1<<(IRQNumber%64));
	  }

	}

	else{

			if(IRQNumber <=31){
				*NVIC_ICER0 |=(1<<IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber <64){
						*NVIC_ICER1 |=(1<<(IRQNumber%32));
			}
			else if(IRQNumber >=64 && IRQNumber < 96){
						*NVIC_ICER2 |=(1<<(IRQNumber%64));
		  }

		}


}
void GPIO_IRQPriorityConfig(uint32_t IRQNumber,uint32_t IRQPriority){
    //FIRST LESTS FIND OUT IPR REGISTOR
	uint32_t temp1=IRQNumber/4;
     uint32_t temp2=IRQNumber%4;
     uint8_t shift_amount=(8*temp2)+(8-no_prio_bit_imp);
     *(NVIC_PR_BASE_ADDR + (temp1))|=(IRQPriority<<shift_amount);


}
void GPIO_IRQHandling(uint8_t PinNumber)//we use only pin number becaouse we have to know fron which pin interupt is comming
{
 if(EXTI->PR & (1<<PinNumber)){

	 //CLEAR
	 EXTI->PR |=(1<<PinNumber);
 }

}
