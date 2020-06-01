/*******************************************************************************
 * @file    RCC.c
 * @author  Moustafa Noufale
 * @email   mustafa13e09940@alexu.edu.eg
 * @date    01.06.2020
 *
 * @brief   RCC Intialization functions
 * @note
 *

Copyright (C) Moustafa Noufale, 2020

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
@endverbatim
/*========================RCC DEVICE DRIVER STM32F407xx=====================================*/
/*					MC Specific Header file for STM32F407G 									*/
#include <STM32F407xx.h>
#include "RCC.h"
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
