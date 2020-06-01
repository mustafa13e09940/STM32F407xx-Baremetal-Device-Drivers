/*========================GPIO DEVICE DRIVER TIVAC 123GH6PM=================================*/
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
