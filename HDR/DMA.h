/*******************************************************************************
 * @file    DMA.h
 * @author  Moustafa Noufale
 * @email   mustafa13e09940@alexu.edu.eg
 * @date    01.06.2020
 *
 * @brief   DMA Initialization and process functions
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
/*========================DMA DEVICE DRIVER STM32F407=======================================*/

#ifndef __DMA_H
#define __DMA_H
/*						Standard Integer Header File										*/
#include <stdint.h>

/*					MC Specific Header file for STM32F407G 									*/
#include <STM32F407xx.h>
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

/*								RCC->AHB1ENR DMA CLOCK ENABLE								*/
#define DMA1_EN								__MCAL_SETBIT(RCC->AHB1ENR,21)
#define DMA2_EN								__MCAL_SETBIT(RCC->AHB1ENR,22)
/*								DMA_Sx->CR DMA Current Target								*/
#define CT_DE	(REG)							__MCAL_CLRBIT(REG,19)
#define CT_EN(REG)							__MCAL_SETBIT(REG,19)

/*==============================DMA CONFIGURATION MACROS====================================*/
/*								DMA Streamx Configuration 									*/
/*								Data Direction												*/
#define	P2M			(0)					//Peripheral to Memory
#define M2P			(1U)				//Memory to Peripheral
#define M2M			(2U)				//Memory to Memory
/*								Data Size	 												*/
#define BYTE		(0)					//1- Byte
#define HWORD		(1U)				//Half Word
#define FWORD		(2U)				//Full Word
/*								Priority Level 												*/
#define	PL_LOW		(0)					//Low Priority
#define	PL_MED		(1U)				//Medium Priority
#define	PL_HIGH		(2U)				//High Priority
#define	PL_VHIGH	(3U)				//Very High Priority
/*								Memory Burst												*/
#define STransfer	(0)					//Single Transfer
#define INCR4		(1U)				//Incremental Burst of 4 beats
#define INCR8		(2U)				//Incremental Burst of 8 beats
#define INCR16		(3U)				//Incremental Burst of 16 beats
/*								Channel Selection											*/
#define CH0			(0)					//Channel 0
#define CH1			(1U)				//Channel 1
#define CH2			(2U)				//Channel 2
#define CH3			(3U)				//Channel 3
#define CH4			(4U)				//Channel 4
#define CH5			(5U)				//Channel 5
#define CH6			(6U)				//Channel 6
#define CH7			(7U)				//Channel 7
/**
	*@Brief 	DMA Initialization Structure Definition
*/
typedef struct{
	uint8_t		DMEIE;				//Direct Mode Interrupt Enable	1
	uint32_t 	CR;					//Configuration Register		1
	uint32_t 	FCR;				//FIFO Control Register			2
}DMA_CONFIG;
/* 								DMA Initialization  Function 			 					*/
/**
	*@Brief 	Initializes DMA
	*@Param1	DMA_Stream_TypeDef pointer
	*@RetVal	None
*/
void intialize_DMA(DMA_Stream_TypeDef *dma_stream, DMA_CONFIG *dma);

/* 								DMA Stream Disable  Function 			 					*/
/**
	*@Brief 	Disables Stream
	*@Param1	DMA_Stream_TypeDef pointer
	*@RetVal	None
*/
void DMA_STREAM_DEN(DMA_Stream_TypeDef *dma_stream);
/* 								DMA Stream Enable  Function 			 					*/
/**
	*@Brief 	Enables Stream
	*@Param1	DMA_TypeDef pointer
	*@Param2	DMA_Stream_TypeDef pointer
	*@RetVal	None
*/
void DMA_STREAM_EN(DMA_TypeDef *dma, DMA_Stream_TypeDef *dma_stream);

/* 								DMA Setting Address Function 			 					*/
/**
	*@Brief 	Setting Prephieral and Memory addresses
	*@Param1	DMA_Stream_TypeDef pointer
	*@Param2	Pointer to constant unsigned 32-bit integer(Source)
	*@Param3	Pointer to constant unsigned 32-bit integer(Destination)
	*@Param4	Constant unsigned 32-bit integer(Size)
	*@RetVal	None
*/
void DMA_SET_ADDRESS(DMA_Stream_TypeDef *dma_stream, const uint32_t *src, const uint32_t *dst, const uint32_t size);

/* 								DMA Interrupt Clear Function 			 					*/
/**
	*@Brief 	Clear interrupts flag
	*@Param1	DMA_TypeDef pointer
	*@RetVal	None
*/
void INTERRUPT_CLEAR(DMA_TypeDef *dma);
#endif
