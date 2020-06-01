/*========================GPIO DEVICE DRIVER STM32F407 		=================================*/
#ifndef __GPIO_H
#define __GPIO_H

/*					MC Specific Header file for STM32F407G 									*/
#include <STM32F407xx.h>
/*	Standard Integer Header File															*/
#include <stdint.h>
/*==========================================================================================*/
/*																							*/
/*							Macros for GPIO Initialization									*/
/*																							*/
/*==========================================================================================*/
/*								General Bit Access Macros									*/
#define __MCAL_SETBIT(REG,BIT) 	 			(REG |= 1 << BIT)
#define __MCAL_CLRBIT(REG,BIT)  			(REG &= ~(1 << BIT))
#define __MCAL_TOGBIT(REG,BIT)  			(REG ^= 1 << BIT)
#define __MCAL_READBIT(REG,BIT)  			((REG >> BIT) & 1)	
/*
LED 3 			PD12
LED 4			PD13
LED 5			PD14
LED 6			PD15
USER BUTTON 	PA0
*/
/*								RCC->AHB1ENR PORT CLOCK ENABLE								*/
#define __MCAL_PORTA_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,0) 		//Enable Clock for Port A
#define __MCAL_PORTB_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,1)		//Enable Clock for Port B
#define __MCAL_PORTC_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,2)		//Enable Clock for Port C
#define __MCAL_PORTD_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,3)		//Enable Clock for Port D
#define __MCAL_PORTE_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,4)		//Enable Clock for Port E
#define __MCAL_PORTF_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,5)		//Enable Clock for Port F
#define __MCAL_PORTG_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,6)		//Enable Clock for Port G
#define __MCAL_PORTH_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,7)		//Enable Clock for Port H
#define __MCAL_PORTI_CLK_EN					__MCAL_SETBIT(RCC->AHB1ENR,8)		//Enable Clock for Port I

/*								GPIOx_MODER Mode Register options							*/
#define GPIO_IN					(unsigned char)(0x0U<<0)			//Input
#define GPIO_OUT	 			(unsigned char)(0x1U<<0)			//Output
#define GPIO_AF					(unsigned char)(0x2U<<0)			//Alternate Function		
#define GPIO_AM					(unsigned char)(0x3U<<0)			//Analog Function		

/*								GPIOx_OTYPER Output Type Register options					*/
#define GPIO_PU					(unsigned char)(0x0U<<0)			//Push-Up Enable							
#define GPIO_OD		 			(unsigned char)(0x1U<<0)			//Open Drain Enable

/*								GPIOx_OSPEEDR Output Speed Register options					*/
#define GPIO_LS					(unsigned char)(0x0U<<0)			//Low Speed
#define GPIO_MS		 			(unsigned char)(0x1U<<0)			//Medium Speed
#define GPIO_HS					(unsigned char)(0x2U<<0)			//High Speed		
#define GPIO_VHS				(unsigned char)(0x3U<<0)			//Very High Speed		
	
/*								GPIOx_PUPDR   Register options	(Pull up/ Pull down)		*/
#define GPIOPUD_DEN				(unsigned char)(0x0U<<0)			//Pull-Up/Down Disable
#define GPIOPU_EN				(unsigned char)(0x1U<<0)			//Pull-Up Enable
#define GPIOPD_EN	 			(unsigned char)(0x2U<<0)			//Pull-Down Enable
	
/*								GPIOx_AFR  Alternate Function Register options				*/
#define AF0						(unsigned char)(0x0U<<0)			//SYS
#define AF1						(unsigned char)(0x1U<<0)			//TIM1/2
#define AF2						(unsigned char)(0x2U<<0)			//TIM3/4/5
#define AF3						(unsigned char)(0x3U<<0)			//TIM8/9/10/11
#define AF4						(unsigned char)(0x4U<<0)			//I2C1/2/3
#define AF5						(unsigned char)(0x5U<<0)			//SPI1/2-I2S2-I2S2ext
#define AF6						(unsigned char)(0x6U<<0)			//SPI3-I2Sext-I2S3
#define AF7						(unsigned char)(0x7U<<0)			//USART1/2/3-I2S3ext
#define AF8						(unsigned char)(0x8U<<0)			//UART4/5-USART6
#define AF9						(unsigned char)(0x9U<<0)			//CAN1/2-TIM12/13/14
#define AF10					(unsigned char)(0xAU<<0)			//OTG_FS/OTG_HS
#define AF11					(unsigned char)(0xBU<<0)			//ETH
#define AF12					(unsigned char)(0xCU<<0)			//FSMC-SDIO-OTG_FS
#define AF13					(unsigned char)(0xDU<<0)			//DCMI
#define AF14					(unsigned char)(0xEU<<0)			//-
#define AF15					(unsigned char)(0xFU<<0)			//EVENTOUT


/*==============================PIN CONFIGURATION MACROS============================*/
/**
	*@Brief 	GPIO Initialization Structure Definition
*/
typedef struct{
	uint32_t PIN;			//pins to be configured 			0
	uint32_t MODE;			//MODE REGISTER						1
	uint32_t OP_TYPE;		//Output Type						2
	uint32_t OP_SPEED;		//Output Speed						3
	uint32_t PUPD;			//Pull Up/Down						4
	uint32_t AF;			//Alternate Function Register 		5
}GPIO_PIN_CONFIG;

/* Initializing Function(Initializes all registers except for LOCK, CR and PCTL)	*/
/**
	*@Brief 	Initialize GPIO
	*@Param1 	Pointer to GPIO Base Address
	*@Param2	Pointer to GPIO PIN CONFIG structure
	*@RetVal	None
*/
 void GPIO_INIT(GPIO_TypeDef *port, GPIO_PIN_CONFIG *gpio_pin_config);

/* 								Read from Pin Function								*/

/**
	*@Brief 	Read from GPIO Pin
	*@Param1 	Pointer to GPIO Base Address
	*@Param2	Pin number
	*@RetVal	unsigned 8-bit integer
*/
uint8_t GPIO_READ(GPIO_TypeDef *port, uint8_t pin);

/* 								Write to Pin Function								*/
/**
	*@Brief 	Write to GPIO Pin
	*@Param1 	Pointer to GPIO Base Address
	*@Param2	Pin number
	*@Param3	Value to be written
	*@RetVal	None
*/
void GPIO_WRITE(GPIO_TypeDef *port, uint8_t pin, uint8_t val);

/* 								Toggle to Pin Function								*/
/**
	*@Brief 	Toggle to GPIO Pin
	*@Param1 	Pointer to GPIO Base Address
	*@Param2	Pin number
	*@RetVal	None
*/
void GPIO_Toggle(GPIO_TypeDef *port, uint8_t pin);

/*
	*@Brief		Interrupt Edge Selection enum
*/
typedef enum{
	RISING_EDGE,
	FALLING_EDGE,
	FALLING_RISING
}EDGE_SELECT;

/* 								GPIO Interrupt Configuration Function 			 		*/
/**
	*@Brief 	Configure GPIO interrupt 
	*@Param1 	uint8_t Pin Number
	*@Param2	Edge_Select enum
	*@RetVal	None
*/
void GPIO_INTERRUPT_CONFIG(uint8_t pin, EDGE_SELECT edge);

/* 								GPIO Interrupt Enable Function 			 				*/
/**
	*@Brief 	Configure GPIO interrupt 
	*@Param1 	uint8_t Pin Number
	*@Param2	IRQn_Type enum
	*@RetVal	None
*/
void GPIO_INTERRUPT_ENABLE(uint8_t pin, IRQn_Type irq);

/* 								GPIO Interrupt Clear Function 			 				*/
/**
	*@Brief 	Clear interrupt flag
	*@Param1	Pin number
	*@RetVal	None
*/
void GPIO_INTERRUPT_CLEAR(uint8_t pin);
#endif

/*==========================================================================================*/
/*																							*/
/*							End of Macros for GPIO Initialization							*/
/*																							*/
/*==========================================================================================*/
