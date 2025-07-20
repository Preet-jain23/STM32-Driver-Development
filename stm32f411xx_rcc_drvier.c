/*
 * stm32f411xx_rcc_drvier.c
 *
 *  Created on: Dec 25, 2024
 *      Author: ASUS
 */



#include "stm32f411xx.h"
#include "string.h"
#include "stdint.h"




uint8_t get_pll_clk(void){
	return 0;
}

uint16_t ahp[8]={2,4,8,16,64,128,256,512};
uint16_t apb[4]={2,4,8,16};


uint32_t get_clock_value_APB1(void){
	uint8_t clk1;

	uint32_t clksrc;

	uint8_t temp1,temp2;


	clk1=(RCC->CFGR >> 2) & 0x03;

	if(clk1 == 0){
		clksrc=16000000U;
	}

	else if(clk1 == 1){
		clksrc=8000000U;
	}

	else if(clk1 == 2){
		clksrc=get_pll_clk();
	}


	clk1=0;

	clk1=(RCC->CFGR  >> 4) & 0x0F;

	if(clk1 < 8){

		temp1=1;
	}

	else if(clk1 >= 8){
		temp1=ahp[clk1-8];
	}


	clk1=0;

	clk1=(RCC->CFGR >> 10) & 0x07;


	if(clk1 < 4){

		temp2=1;
	}

	else if(clk1 >= 4){

		temp2=apb[clk1-4];
	}

	uint32_t ans=(clksrc/temp1)/temp2;

	return ans;



}




uint32_t get_clock_value_APB2(void){
	uint8_t clk1;

	uint32_t clksrc;

	uint8_t temp1,temp2;


	clk1=(RCC->CFGR >> 2) & 0x03;

	if(clk1 == 0){
		clksrc=16000000U;
	}

	else if(clk1 == 1){
		clksrc=8000000U;
	}

	else if(clk1 == 2){
		clksrc=get_pll_clk();
	}


	clk1=0;

	clk1=(RCC->CFGR  >> 4) & 0x0F;

	if(clk1 < 8){

		temp1=1;
	}

	else if(clk1 >= 8){
		temp1=ahp[clk1-8];
	}


	clk1=0;

	clk1=(RCC->CFGR >> 13) & 0x07;


	if(clk1 < 4){

		temp2=1;
	}

	else if(clk1 >= 4){

		temp2=apb[clk1-4];
	}

	uint32_t ans=(clksrc/temp1)/temp2;

	return ans;
}
