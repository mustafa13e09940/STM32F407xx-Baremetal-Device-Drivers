/*					MC Specific Header file for STM32F407G 									*/
#include <STM32F407xx.h>
#include "SYSCTL.h"
/* 								Select PLL Clock											*/
#define P		2U
#define M		16U
/**
	*@Brief 	Adjust desired clock source.
	*@Param1 	Desired Clock in MHz
	*@RetVal	None
*/
void PLL_CLOCK(uint8_t CLK){

	//PLL
	__MCAL_SETBIT(RCC->CR,19);
	for(int i=0;i<6;i++)__MCAL_CLRBIT(RCC->PLLCFGR,i);
	for(int i=6;i<15;i++)__MCAL_CLRBIT(RCC->PLLCFGR,i);
	for(int i=16;i<18;i++)__MCAL_CLRBIT(RCC->PLLCFGR,i);
	RCC->PLLCFGR|=(4U<<0);			//M=4
	RCC->PLLCFGR|=(2*(CLK)<<6);		//N=clk
	RCC->PLLCFGR|=(1U<<16);			//P=4
	RCC->PLLCFGR|=(1U<<22);
	RCC->CR|=(1U<<16);				//Turn on HSE
	RCC->CR|=(1U<<24);				//Turn on PLL
	RCC->CFGR|=(1U<<1);				//System Clock is PLL
	SystemCoreClockUpdate();
}
