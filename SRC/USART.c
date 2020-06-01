/*******************************************************************************
 * @file    USART.h
 * @author  Moustafa Noufale
 * @email   mustafa13e09940@alexu.edu.eg
 * @date    01.06.2020
 *
 * @brief   USART Intialization and process functions
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
#include "USART.h"
/* 								UART Initialization		 			 								*/
/**
	*@Brief 	Initialize UART 
	*@Param1 	Pointer to USART Handle Structure
	*@Param2 	Pointer to UART Configuration Structure
	*@RetVal	None
*/
void USART_INIT(USART_Handle *usart){
		//CR1
		//Oversampling
		if(usart->Init.OVERS==OVER16)
			__MCAL_CLRBIT(usart->Instance->CR1,15);	//Break Enable
		else
			__MCAL_SETBIT(usart->Instance->CR1,15);	//Break Disable
		//DATA Length
		if(usart->Init.M==M8)
			__MCAL_CLRBIT(usart->Instance->CR1,12);	//8-bit DATA
		else
			__MCAL_SETBIT(usart->Instance->CR1,12);	//9-bit DATA
		//Parity Control Enable
		if(usart->Init.PCE==PCDE)
			__MCAL_CLRBIT(usart->Instance->CR1,10);	//Parity Control Disable
		else
			__MCAL_SETBIT(usart->Instance->CR1,10);	//Parity Control Enable
		
		//Parity Selection
		if(usart->Init.PS==PE)
			__MCAL_CLRBIT(usart->Instance->CR1,9);	//Parity Even
		else
			__MCAL_SETBIT(usart->Instance->CR1,9);	//Parity Odd
		
		//CR2	
		//Stop Bits
		if(usart->Init.STOP==STOP1){
			__MCAL_CLRBIT(usart->Instance->CR2,12);	//1-Stop Bit
			__MCAL_CLRBIT(usart->Instance->CR2,13);	//1-Stop Bit
		}
		else
			usart->Instance->CR2|=(usart->Init.STOP<<12);
		//CR3
		//Sample Bit Mode
		if(usart->Init.ONE_BIT==SAMPLE_THREEB)
			__MCAL_CLRBIT(usart->Instance->CR3,11);	//Three Bit Mode
		else
			__MCAL_SETBIT(usart->Instance->CR3,11);	//One Bit Mode
		
		//Enable USART Peripheral
		USART_ENABLE_DISABLE(usart->Instance,ENABLE);
		//Change USART State to Ready
		usart->RxState=MCAL_UART_STATE_READY;
		usart->TxState=MCAL_UART_STATE_READY;
		//Reset Error State
		usart->ErrorCode=MCAL_UART_ERROR_NONE;
}	

/* 								USART Enable		 			 								*/
/**
	*@Brief 	Enable or Disable USART 
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None 
*/
void USART_ENABLE_DISABLE(USART_TypeDef *usart, uint8_t T){
	//Enable
	if(T==ENABLE)__MCAL_SETBIT(usart->CR1,13);
	//Disable
	else __MCAL_CLRBIT(usart->CR1,13);
}
/* 								USART Tx Enable/Disable	 			 							*/
/**
	*@Brief 	Enable or Disable USART Tx
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_Tx(USART_TypeDef *usart, uint8_t T){
	//Enable
	if(T==ENABLE)__MCAL_SETBIT(usart->CR1,3);
	//Disable
	else __MCAL_CLRBIT(usart->CR1,3);
}

/* 								USART TxE Enable/Disable	 			 							*/
/**
	*@Brief 	Enable or Disable USART TxE Interrupt
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_TxE(USART_TypeDef *usart, uint8_t T){
	//Enable
	if(T==ENABLE)__MCAL_SETBIT(usart->CR1,7);
	//Disable
	else __MCAL_CLRBIT(usart->CR1,7);
}

/* 								USART Rx Enable/Disable	 			 							*/
/**
	*@Brief 	Enable or Disable USART Rx
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_Rx(USART_TypeDef *usart, uint8_t T){
	//Enable
	if(T==ENABLE)__MCAL_SETBIT(usart->CR1,2);
	//Disable
	else __MCAL_CLRBIT(usart->CR1,2);
}

/* 								USART RxNE Interrupt Enable/Disable	 			 					*/
/**
	*@Brief 	Enable or Disable USART RxNE Interrupt
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_RxNE(USART_TypeDef *usart, uint8_t T){
	//Enable
	if(T==ENABLE)__MCAL_SETBIT(usart->CR1,5);
	//Disable
	else __MCAL_CLRBIT(usart->CR1,5);
}

/* 								USART Error Interrupt Enable/Disable	 			 				*/
/**
	*@Brief 	Enable or Disable USART Error Interrupt
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_EI(USART_TypeDef *usart, uint8_t T){
	//Enable
	if(T==ENABLE)__MCAL_SETBIT(usart->CR3,0);
	//Disable
	else __MCAL_CLRBIT(usart->CR3,0);
}

/* 								USART Pairty Error Interrupt Enable/Disable	 			 			*/
/**
	*@Brief 	Enable or Disable USART Pairty Error Interrupt
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_PEI(USART_TypeDef *usart, uint8_t T){
	//Enable
	if(T==ENABLE)__MCAL_SETBIT(usart->CR1,8);
	//Disable
	else __MCAL_CLRBIT(usart->CR1,8);
}
/* 								UART Low Power Configuration				 						*/
/**
	*@Brief 	Sets up Low-Power Register UARTILPR
	*@Param1 	Pointer to USART Handle Structure
	*@Param2	ILLPDVSR=SYSCLK/F, 1.42MHz<F<2.12MHz
				low power 1.42 us<pulse duration< 2.12 us
	*@RetVal	None
*/
void UART_ILLP(USART_Handle *usart, uint8_t ilpdvsr){
}
/* 								UART Baud rate setup						 						*/
/**
	*@Brief 	Sets-up Baud Rate
	*@Param1 	Pointer to USART Handle Structure
	*@Param2	Desired Baud Rate Value
	*@RetVal	None
*/
	/*BAUD RATE SETTING OF 115200*/
	/*BRD = BRDI + BRDF = UARTSysClk / (ClkDiv * Baud Rate)*/
	/*UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5)*/
void USART_BRD(USART_Handle *usart, uint32_t BRD){
	if(BRD==9600)
		usart->Instance->BRR=0x683;
	else if(BRD==115200)
		usart->Instance->BRR=0x8A;
	else
		usart->Instance->BRR=0x8A;
}



/* 								UART Interrupt Clear Function 			 						*/
/**
	*@Brief 	Clear interrupt from ICR register by setting it by 1
	*@Param1 	Pointer to USART Handle Structure
	*@Param2	Pin number, See page. 930
	*@RetVal	None
*/
void USART_INTERRUPT_CLEAR(USART_Handle *usart, uint8_t pin){
	
}

/* 								USART Send Data Function 			 							*/
/**
	*@Brief 	Send Data using UART Communication Protocol
	*@Param1 	Pointer to USART Handle Structure
	*@Param2	Pointer to 8-bit Data
	*@Param3	Size of data
	*@RetVal	None
*/
void USART_SEND(USART_Handle *usart, uint8_t *data, uint8_t size){
	//Populate to-be-sent data, length and count
	usart->Txptr=data;
	usart->TxSize=size;
	usart->TxCounter=size;
	
	//Change state to busy transmitting
	usart->TxState=MCAL_UART_STATE_BUSY_Tx;
	
	//Enable USART
	USART_ENABLE_DISABLE(usart->Instance,ENABLE);
	
	//Enable TXE Interrupt
	USART_ENABLE_DISABLE_TxE(usart->Instance, ENABLE);
}
/* 								UART Receive Data Function 			 							*/
/**
	*@Brief 	Receive Data using UART Communication Protocol
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	8-bit Data
*/
void USART_RECEIVE(USART_Handle *usart, uint8_t *data, uint32_t size){
	uint32_t VAL;
	//Populate to-be-Received data, length and count
	usart->Txptr=data;
	usart->TxSize=size;
	usart->TxCounter=size;
	
	//Change state to busy receving
	usart->RxState=MCAL_UART_STATE_BUSY_Rx;
	
	//Enable Parity Error Interrupt
	__MCAL_SETBIT(usart->Instance->CR1, CR1_PEI_EN);
	
	//Enable Error Interrupt
	__MCAL_SETBIT(usart->Instance->CR3, CR3_EI_EN);
	
	//Read Data Register
	VAL=usart->Instance->DR;
	//Enable RXNE Interrupt
	__MCAL_SETBIT(usart->Instance->CR1,CR1_RXNEI_EN);
}

/* 								USART Interrupt Handle Function 			 					*/
/**
	*@Brief 	Handles USART Interrupt IRQ
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void USART_HANDLE_IRQ(USART_Handle *usart){
	//Variables to store Status Register & Interrupt Flag
	//and CR1 &  Error Interrupt Enable 
	uint32_t tmp1,tmp2;
	//Pairty Error
	tmp1=usart->Instance->SR&USART_PE_FLAG;
	//Parity Interrupt Enable
	tmp2=usart->Instance->CR1&(1U<<8);
	//If parity Error enabled and happened
	if((tmp1)&&(tmp2)){
		//Clear Error
		USART_ERROR_CLEAR(usart);
		//Store error code
		usart->ErrorCode=MCAL_UART_ERROR_PE;
	}
	
	//Frame Error
	tmp1=usart->Instance->SR&USART_FE_FLAG;
	//Error Interrupt Enable
	tmp2=usart->Instance->CR3&(1U<<0);
	//If parity Error enabled and happened
	if((tmp1)&&(tmp2)){
		//Clear Error
		USART_ERROR_CLEAR(usart);
		//Store error code
		usart->ErrorCode=MCAL_UART_ERROR_FE;
	}
	
	//Noise Error
	tmp1=usart->Instance->SR&USART_NE_FLAG;
	//Error Interrupt Enable
	tmp2=usart->Instance->CR3&(1U<<0);
	//If parity Error enabled and happened
	if((tmp1)&&(tmp2)){
		//Clear Error
		USART_ERROR_CLEAR(usart);
		//Store error code
		usart->ErrorCode=MCAL_UART_ERROR_NE;
	}
	
	//Frame Error
	tmp1=usart->Instance->SR&USART_ORE_FLAG;
	//Error Interrupt Enable
	tmp2=usart->Instance->CR3&(1U<<0);
	//If parity Error enabled and happened
	if((tmp1)&&(tmp2)){
		//Clear Error
		USART_ERROR_CLEAR(usart);
		//Store error code
		usart->ErrorCode=MCAL_UART_ERROR_ORE;
	}	
	
	//RXNE USART In Receiver Mode
	//if RXNE Flag
	tmp1=usart->Instance->SR&USART_RXNE_FLAG;
	//if RXNE Interrupt is enabled
	tmp2=usart->Instance->CR1&(1U<<5);
	if((tmp1)&&(tmp2)){
		RxE_INTERRUPT_HANDLE(usart);
	}	
	
	//TXE USART In Transmitter Mode
	//if TXE Flag
	tmp1=usart->Instance->SR&USART_TXE_FLAG;
	//if TXE Interrupt is enabled
	tmp2=usart->Instance->CR1&(1U<<7);
	if((tmp1)&&(tmp2)){
		TxE_INTERRUPT_HANDLE(usart);
	}
	
	//TXC USART In Transmitter Mode
	//if TXE Flag
	tmp1=usart->Instance->SR&USART_TC_FLAG;
	//if RXNE Interrupt is enabled
	tmp2=usart->Instance->CR1&(1U<<6);
	if((tmp1)&&(tmp2)){
		TC_INTERRUPT_HANDLE(usart);
	}		
	//Error is not None
	if(usart->ErrorCode!=MCAL_UART_ERROR_NONE){
		//Set error code to Ready
		usart->TxState=MCAL_UART_STATE_READY;
		usart->RxState=MCAL_UART_STATE_READY;
		//Indicate Error using Red LED
		__MCAL_SETBIT(GPIOD->ODR,12);
	}
}

/* 								USART Clear Error Register		 			 				*/
/**
	*@Brief 	Write any data to USARTECR Register to clear: 
				Framing, Parity,Break or Overrun errors.
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void USART_ERROR_CLEAR(USART_Handle *usart){
	//Temporary variable to store status and data registers
	uint32_t tmp=0x00;
	//Software sequence to clear the error flag
	//A read of status register and data register 
	tmp= usart->Instance->SR;
	tmp= usart->Instance->DR;
}

/* 								USART TxE Interrupt Handle Function 			 				*/
/**
	*@Brief 	Handles TxE USART Interrupt IRQ
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void TxE_INTERRUPT_HANDLE(USART_Handle *usart){
	uint32_t tmp=0;
	uint8_t val=0;
	//Reading current state
	tmp=usart->TxState;
	if(tmp==MCAL_UART_STATE_BUSY_Tx){
		//Reading Data
		val=(uint8_t)((*usart->Txptr++)&(uint8_t)0x00FF);
		USART_ENABLE_DISABLE_Tx(usart->Instance,ENABLE);
		usart->Instance->DR=val;
		if(--usart->TxCounter==0)
		{
			//Disable TxE Interrupt
			USART_ENABLE_DISABLE_TxE(usart->Instance,DISABLE);
			//Disable Tx			
			USART_ENABLE_DISABLE_Tx(usart->Instance,DISABLE);
			//Enable Transmit Complete Interrupt
			__MCAL_SETBIT(usart->Instance->CR1,CR1_TXCI_EN);
		}
	}
}

/* 								USART Transfer Complete Interrupt Handle Function 			 	*/
/**
	*@Brief 	Handles TC USART Interrupt IRQ
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void TC_INTERRUPT_HANDLE(USART_Handle *usart){
	//Disable Transmit Complete Interrupt
	__MCAL_CLRBIT(usart->Instance->CR1,CR1_TXCI_EN);
	//Change USART State
	usart->TxState=MCAL_UART_STATE_READY;
	//Call Application callback 
	if(usart->tx_comp){
		usart->tx_comp(&usart->TxSize);
	}
}

/* 								USART RxE Interrupt Handle Function 			 				*/
/**
	*@Brief 	Handles RxE USART Interrupt IRQ
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void RxE_INTERRUPT_HANDLE(USART_Handle *usart){

	//Check the state of the Rx buffer
	if(usart->RxState==MCAL_UART_STATE_BUSY_Rx){
		//check if USART using parity
		if(usart->Init.PCE==PCDE){
			//No Parity
			*usart->Rxptr++=((uint8_t)(usart->Instance->DR)&(uint8_t)0x00FF);
		}
		else{
			//Parity, Ignore MSB
			*usart->Rxptr++=((uint8_t)(usart->Instance->DR)&(uint8_t)0x007F);
		}
		//Is reception over?
		if(usart->RxCounter==0){
			//Disable RXNE
			__MCAL_CLRBIT(usart->Instance->CR1, 5);
			//Disable parity error interrupt
			__MCAL_CLRBIT(usart->Instance->CR1, 8);
			//Disable Error interrupt
			__MCAL_CLRBIT(usart->Instance->CR3, 0);
			//Change state to ready
			usart->RxState=MCAL_UART_STATE_READY;
			//Call the call back function
			if(usart->rx_comp){
				usart->rx_comp(&usart->RxSize);
			}
		}
	}
}
