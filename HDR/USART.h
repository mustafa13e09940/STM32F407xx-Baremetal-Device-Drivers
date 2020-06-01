/*========================USART DEVICE DRIVER TIVAC 123GH6PM================================*/
#ifndef __USART_H
#define __USART_H

/*					MC Specific Header file for STM32F407G 									*/
#include <STM32F407xx.h>       							       // Device header
#include "GPIO.h"
#include <stdint.h>
/*==========================================================================================*/
/*																							*/
/*							Macros for USART Initialization									*/
/*																							*/
/*==========================================================================================*/
/*==========================================================================================*/
/*==========================MCAL USART State Structure definition===========================*/
typedef enum{
	MCAL_UART_STATE_RESET					=0x00,		//Peripheral not initalized
	MCAL_UART_STATE_READY					=0x01,		//Peripheral Initialized and Ready
	MCAL_UART_STATE_BUSY					=0x02,		//Peripheral is processing
	MCAL_UART_STATE_BUSY_Tx					=0x12,		//Peripheral is processing Transmitting
	MCAL_UART_STATE_BUSY_Rx					=0x22,		//Peripheral is processing Receving 
	MCAL_UART_STATE_BUSY_Tx_Rx				=0x32,		//Peripheral is processing Receving and Transmitting
}MCAL_USART_STATE;

/*								Definitions for Enable or Disable							*/
#define	ENABLE					1
#define DISABLE					0
/*								UART Possible Errors										*/
#define MCAL_UART_ERROR_NONE			((uint32_t)0x00)	//No Error
#define MCAL_UART_ERROR_PE				((uint32_t)0x01)	//Parity Error
#define MCAL_UART_ERROR_NE				((uint32_t)0x02)	//Noise Error
#define MCAL_UART_ERROR_FE				((uint32_t)0x04)	//Frame Error
#define MCAL_UART_ERROR_ORE				((uint32_t)0x08)	//Overrun Error
#define MCAL_UART_ERROR_DMA				((uint32_t)0x10)	//Direct Memory access

/*								General Bit Access Macros									*/
#define __MCAL_SETBIT(REG,BIT) 	 			(REG |= 1 << BIT)
#define __MCAL_CLRBIT(REG,BIT)  			(REG &= ~(1 << BIT))
#define __MCAL_TOGBIT(REG,BIT)  			(REG ^= 1 << BIT)
#define __MCAL_READBIT(REG,BIT)  			((REG >> BIT) & 1)	

/*								RCGCUSART-> USART CLOCK ENABLE								*/
#define __MCAL_USART1_CLK_EN			__MCAL_SETBIT(RCC->APB2ENR,4)		//Enable Clock for USART1
#define __MCAL_USART2_CLK_EN			__MCAL_SETBIT(RCC->APB1ENR,17)		//Enable Clock for USART2
#define __MCAL_USART3_CLK_EN			__MCAL_SETBIT(RCC->APB1ENR,18)		//Enable Clock for USART3
#define __MCAL_USART4_CLK_EN			__MCAL_SETBIT(RCC->APB1ENR,19)		//Enable Clock for USART4
#define __MCAL_USART5_CLK_EN			__MCAL_SETBIT(RCC->APB1ENR,20)		//Enable Clock for USART5
#define __MCAL_USART6_CLK_EN			__MCAL_SETBIT(RCC->APB2ENR,5)		//Enable Clock for USART6
/*==========================================================================================*/
/*																							*/
/*							Macros for USART Initialization									*/
/*																							*/
/*==========================================================================================*/
/*									Status Register 1										*/
#define			USART_TXE_FLAG	((uint32_t)(1<<7))	//Transmit Buffer Empty Flag
#define			USART_TC_FLAG	((uint32_t)(1<<6))	//Transmit Complete  Flag
#define			USART_RXNE_FLAG	((uint32_t)(1<<5))	//Receive Buffer Not Empty Flag
#define			USART_IDLE_FLAG	((uint32_t)(1<<4))	//Idle Flage
#define			USART_ORE_FLAG	((uint32_t)(1<<3))	//Overrun error flage
#define			USART_NE_FLAG	((uint32_t)(1<<2))	//Noise error flag
#define			USART_FE_FLAG	((uint32_t)(1<<1))	//Frame error flag
#define			USART_PE_FLAG	((uint32_t)(1<<0))	//Parity error flag

/*									Control Register 1										*/
#define			OVER_EN		((uint32_t)(1<<15))		//Oversampling Enable
#define			OVER16		0						//Oversampling by 16
#define			OVER8		1						//Oversampling by 8

#define			M8			0						//1 Stop-bit, 8 Data-bits
#define			M9			1						//1 Stop-bit, 9 Data-bits

#define			PCDE		0						//Parity control enable
#define			PCEN		1						//Parity control enable

#define			PE			0						//Even Parity
#define			PO			1						//Odd Parity

#define			CR1_PEI_EN		8					//Parity error interrupt enable
#define			CR1_TXEI_EN		7					//Transmision buffer empty interrupt enable
#define			CR1_TXCI_EN		6					//Transmision Complete interrupt enable
#define			CR1_RXNEI_EN	5					//Receive buffer not empty interrupt enable
#define			CR1_Tx_EN		3					//Enable Transmission
#define			CR1_Rx_EN		2					//Enable Reception

/*									Control Register 2										*/
#define 		STOP1		0	//1-Stop Bit
#define 		STOP5		1	//0.5-Stop Bit
#define 		STOP2		2	//1.5-Stop Bit
#define 		STOP15		3	//2-Stop Bits

/*									Control Register 3										*/
#define			CR3_EI_EN		0	//Error interrupt enable
#define			SAMPLE_THREEB	0	//Three Sample Bit Method
#define			SAMPLE_ONEB		1	//One Sample Bit Method

/*								Application Callbacks function definitions					*/
typedef void (TX_COMP)(void *ptr);	//Callback when transmission is completed
typedef void (RX_COMP)(void *ptr);	//Callback when reception is completed

/*
	*@Brief 	USART Initialization Structure Definition
*/
typedef struct{
	//Control Register 1
	uint32_t OVERS;				//Oversampling Mode				15
	uint32_t M;					//Word Length					12
	uint32_t WAKE;				//Wake up Method				11
	uint32_t PCE;				//Parity Control Enable			10
	uint32_t PS;				//Parity Selection				9
	//Control Register 2
	uint32_t	STOP;			//Stop Bits
	//Control Register 3
	uint32_t ONE_BIT;			//One Sample Bit Mode
}USART_CONFIG;

/*
	*@Brief 	USART Handle Structure Definition
*/
typedef struct{
	USART_TypeDef		*Instance;	//pointer to USART Base address
	USART_CONFIG		Init;		//Pointer to configruation structure
	uint8_t				*Txptr;		//Pointer to Transmission buffer
	uint16_t			TxSize;		//Size of Transfer 
	uint16_t			TxCounter;	//Transfer counter
	uint8_t				*Rxptr;		//Pointer to Receive buffer
	uint16_t			RxSize;		//Size of Receive 
	uint16_t			RxCounter;	//reception counter
	MCAL_USART_STATE	TxState;	//Transmission State
	MCAL_USART_STATE	RxState;	//reception State
	uint32_t			ErrorCode;	//Error code
	TX_COMP				*tx_comp;	//Function Pointer to transmission complete callback function
	RX_COMP				*rx_comp;	//Function Pointer to reception complete callback function
}USART_Handle;

/* 								USART Initialization		 			 						*/
/*
	*@Brief 	Initialize USART 
	*@Param1 	Pointer to USART Handle Structure
	*@Param2 	Pointer to USART Configuration Structure
	*@RetVal	None
*/
void USART_INIT(USART_Handle *usart_handle);

/* 								USART Enable/Disable		 			 						*/
/**
	*@Brief 	Enable or Disable USART 
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE(USART_TypeDef *usart, uint8_t T);

/* 								USART Tx Enable/Disable	 			 							*/
/**
	*@Brief 	Enable or Disable USART Tx
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_Tx(USART_TypeDef *usart, uint8_t T);

/* 								USART TxE Enable/Disable	 			 							*/
/**
	*@Brief 	Enable or Disable USART TxE Interrupt
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_TxE(USART_TypeDef *usart, uint8_t T);

/* 								USART Rx Enable/Disable	 			 							*/
/**
	*@Brief 	Enable or Disable USART Rx
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/

void USART_ENABLE_DISABLE_Rx(USART_TypeDef *usart, uint8_t T);

/* 								USART RxNE Interrupt Enable/Disable	 			 					*/
/**
	*@Brief 	Enable or Disable USART RxNE Interrupt
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_RxNE(USART_TypeDef *usart, uint8_t T);

/* 								USART Error Interrupt Enable/Disable	 			 				*/
/**
	*@Brief 	Enable or Disable USART Error Interrupt
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_EI(USART_TypeDef *usart, uint8_t T);

/* 								USART Pairty Error Interrupt Enable/Disable	 			 			*/
/**
	*@Brief 	Enable or Disable USART Pairty Error Interrupt
	*@Param1 	Pointer to USART Base address
	*@Param2 	Parameter to specify Enable or Disable
	*@RetVal	None
*/
void USART_ENABLE_DISABLE_PEI(USART_TypeDef *usart, uint8_t T);

/* 								USART Clear Error Register		 			 					*/
/**
	*@Brief 	Write any data to USARTECR Register to clear: 
				Framing, Parity,Break or Overrun errors.
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void USART_ERROR_CLEAR(USART_Handle *usart);
/* 								USART Low Power Configuration				 				*/
/**
	*@Brief 	Sets up Low-Power Register USARTILPR
	*@Param1 	Pointer to USART Handle Structure
	*@Param2	ILLPDVSR=SYSCLK/F, 1.42MHz<F<2.12MHz
				low power 1.42 us<pulse duration< 2.12 us
	*@RetVal	None
*/
void USART_ILLP(USART_Handle *usart, uint8_t ilpdvsr);
/* 								USART Baud rate setup						 				*/
/**	
	*@Brief 	Sets-up Baud Rate
	*@Param1 	Pointer to USART Handle Structure
	*@Param2	Desired Baud Rate Value
	*@RetVal	None
*/
/*
BR=F/[8x(2-OVER8)xUSARTDIV]
OVER8 IS 0 OR 1
DIV_Fraction BR =0.XX x 16
DIV_Integer BR=Y<<4
*/
void USART_BRD(USART_Handle *usart, uint32_t BRD);


/* 								USART Interrupt Clear Function 			 					*/
/**
	*@Brief 	Clear interrupt from ICR register by setting it by 1
	*@Param1 	Pointer to USART Handle Structure
	*@Param2	Pin number, See page. 930
	*@RetVal	None
*/
void USART_INTERRUPT_CLEAR(USART_Handle *usart, uint8_t pin);
/* 								USART Send Data Function 			 						*/
/**
	*@Brief 	Send Data using USART Communication Protocol
	*@Param1 	Pointer to USART Handle Structure
	*@Param2	8-bit Data
	*@RetVal	None
*/
void USART_SEND(USART_Handle *usart, uint8_t *data, uint8_t size);
/* 								USART Receive Data Function 			 						*/
/**
	*@Brief 	Receive Data using USART Communication Protocol
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	8-bit Data
*/
void USART_RECEIVE(USART_Handle *usart, uint8_t *data, uint32_t size);
/* 								USART Interrupt Handle Function 			 					*/
/**
	*@Brief 	Handles USART Interrupt IRQ
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void USART_HANDLE_IRQ(USART_Handle *usart);
/* 								USART TxE Interrupt Handle Function 			 				*/
/**
	*@Brief 	Handles TxE USART Interrupt IRQ
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void TxE_INTERRUPT_HANDLE(USART_Handle *usart);

/* 								USART Transfer Complete Interrupt Handle Function 			 	*/
/**
	*@Brief 	Handles TC USART Interrupt IRQ
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void TC_INTERRUPT_HANDLE(USART_Handle *usart);

/* 								USART RxE Interrupt Handle Function 			 				*/
/**
	*@Brief 	Handles RxE USART Interrupt IRQ
	*@Param1 	Pointer to USART Handle Structure
	*@RetVal	None
*/
void RxE_INTERRUPT_HANDLE(USART_Handle *usart);
#endif
