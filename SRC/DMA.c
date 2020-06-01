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

#include "DMA.h"

/* 								DMA Initialization  Function 			 					*/
/**
	*@Brief 	Initializes DMA
	*@Param1	DMA_Stream_TypeDef pointer
	*@RetVal	None
*/
void intialize_DMA(DMA_Stream_TypeDef *dma_stream, DMA_CONFIG *dma){
	//Disable Stream
	DMA_STREAM_DEN(dma_stream);
	dma_stream->CR=dma->CR;
	dma_stream->CR=dma->FCR;
	//Priority Level
	dma_stream->CR|=3<<16;
	//Direction Memory to memory
	dma_stream->CR|=2<<6;
	//Preipheral Data Size
	dma_stream->CR|=2<<11;
	//Memory Data Size
	dma_stream->CR|=2<<13;
	//Memory Incremented Mode
	dma_stream->CR|=1<<10;
	//Prephiral Incremented Mode
	dma_stream->CR|=1<<9;
	//Disable Direct Mode
	dma_stream->FCR&=~(1<<2);
	//FIFO Threshold
	dma_stream->FCR|=(1<<0);
	//Enable Transfer Complete Interrupt
	dma_stream->CR|=1<<4;
}
/* 								DMA Setting Address Function 			 					*/
/**
	*@Brief 	Setting Prephieral and Memory addresses
	*@Param1	DMA_Stream_TypeDef pointer
	*@Param2	Pointer to constant unsigned 32-bit integer(Source)
	*@Param3	Pointer to constant unsigned 32-bit integer(Destination)
	*@Param4	Constant unsigned 32-bit integer(Size)
	*@RetVal	None
*/
void DMA_SET_ADDRESS(DMA_Stream_TypeDef *dma_stream, const uint32_t *src, const uint32_t *dst, const uint32_t size){
	//Source Address to Prephiral
	dma_stream->PAR=(uint32_t)src;
	//Destination Address to Memory
	dma_stream->M0AR=(uint32_t)dst;
	//Number of Data
	dma_stream->NDTR=size;
}

/* 								DMA Stream Disable  Function 			 					*/
/**
	*@Brief 	Disables Stream
	*@Param1	DMA_Stream_TypeDef pointer
	*@RetVal	None
*/
void DMA_STREAM_DEN(DMA_Stream_TypeDef *dma_stream){
	__MCAL_CLRBIT(dma_stream->CR,0);
}

/* 								DMA Stream Enable  Function 			 					*/
/**
	*@Brief 	Enables Stream
	*@Param1	DMA_TypeDef pointer
	*@Param2	DMA_Stream_TypeDef pointer
	*@RetVal	None
*/
void DMA_STREAM_EN(DMA_TypeDef *dma, DMA_Stream_TypeDef *dma_stream){
	INTERRUPT_CLEAR(dma);
	__MCAL_SETBIT(dma_stream->CR,0);
}

/* 								DMA Interrupt Clear Function 			 				*/
/**
	*@Brief 	Clear interrupts flag
	*@Param		None
	*@RetVal	None
*/
void INTERRUPT_CLEAR(DMA_TypeDef *dma){
	dma->HIFCR|=(0xFFFFFFFF);
	dma->LIFCR|=(0xFFFFFFFF);
}





/*
static uint32_t SRC_Buffer[Buffer_Size];
static uint32_t DST_Buffer[Buffer_Size];


int main(){
	intialize_DMA();
	DMA_SET_ADDRESS(SRC_Buffer,DST_Buffer,Buffer_Size);
	for(int i=0; i< Buffer_Size;i++){
		SRC_Buffer[i]=i*i;
	}
	SysTick->VAL=0;
	SysTick->LOAD=18000000;
	LED_INIT();
	SysTick->CTRL|=(7U<<0);
	__enable_irq();
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	DMA_STREAM_EN(DMA2,DMA2_Stream0); 	
*/
