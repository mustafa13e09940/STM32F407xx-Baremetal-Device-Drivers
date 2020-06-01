/*******************************************************************************
 * @file    RCC.h
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
#ifndef __SYSCTL_H
#define __SYSCTL_H
/*					MC Specific Header file for STM32F407G 									*/
#include <STM32F407xx.h>
/*==========================================================================================*/
/*																							*/
/*							Macros for SYSCFG Initialization								*/
/*																							*/
/*==========================================================================================*/
/*								General Bit Access Macros									*/
#define __MCAL_SETBIT(REG,BIT) 	 			(REG |= 1 << BIT)
#define __MCAL_CLRBIT(REG,BIT)  			(REG &= ~(1 << BIT))
#define __MCAL_TOGBIT(REG,BIT)  			(REG ^= 1 << BIT)
#define __MCAL_READBIT(REG,BIT)  			((REG >> BIT) & 1)
/*							Enable System Configuration Controller							*/
#define __MCAL_SYSCFG_EN 					(RCC->APB2ENR |= 1 << 14)
/*							Enable Compensation Cell Configuration 							*/
#define __MCAL_COMCR_EN 					(SYSCFG->CMPCR |= 1)

/* 								Select PLL Clock											*/

/**
	*@Brief 	Adjust desired clock source.
	*@Param1 	Desired Clock in MHz
	*@RetVal	None
*/
void PLL_CLOCK(uint8_t CLK);
#endif
