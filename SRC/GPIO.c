#include "GPIO.h"
#include "SYSCTL.h"
/* 			Initializing Function(Initializes all registers 							*/
/**
	*@Brief 	Initialize GPIO
	*@Param1 	Pointer to GPIO Base Address
	*@Param2	Pointer to GPIO PIN CONFIG structure
	*@RetVal	None
*/
 void GPIO_INIT(GPIO_TypeDef *port, GPIO_PIN_CONFIG *gpio_pin_config){
	 //Setting Pin Mode
	 if(!gpio_pin_config->MODE){
		 __MCAL_CLRBIT(port->MODER,gpio_pin_config->PIN);
		 __MCAL_CLRBIT(port->MODER,((gpio_pin_config->PIN)+1));
	 }
	 else{
		 port->MODER|=(gpio_pin_config->MODE<<(2*gpio_pin_config->PIN));
	 }
	 
	 //Setting Output Type
	 if(!gpio_pin_config->OP_TYPE){
		 __MCAL_CLRBIT(port->OTYPER,gpio_pin_config->PIN);
	 }
	 else{
		 __MCAL_SETBIT(port->OTYPER,gpio_pin_config->PIN);
	 }

	 //Setting Output Speed
	 if(gpio_pin_config->OP_SPEED==GPIO_VHS){
		 __MCAL_SYSCFG_EN;
		 __MCAL_COMCR_EN;
		 while(!SYSCFG->CMPCR&(1<<8));
	 }
	 if(!gpio_pin_config->OP_SPEED){
		 __MCAL_CLRBIT(port->OSPEEDR,gpio_pin_config->PIN);
		 __MCAL_CLRBIT(port->OSPEEDR,(gpio_pin_config->PIN+1));
	 }
	 else{
		port->OSPEEDR|=(gpio_pin_config->OP_SPEED<<(2*gpio_pin_config->PIN));
	 }
	//Setting Pull Up/Down Register
	 if(!gpio_pin_config->PUPD){
		 __MCAL_CLRBIT(port->PUPDR,gpio_pin_config->PIN);
		 __MCAL_CLRBIT(port->PUPDR,(gpio_pin_config->PIN+1));
	 }
	 else{
		port->PUPDR|=(gpio_pin_config->PUPD<<(2*gpio_pin_config->PIN));
	 }
	
	 //Setting Alternate Function
	 //Low Register 
	if(gpio_pin_config->PIN<=7){
		if(!gpio_pin_config->AF){
			for(uint8_t i=0;i<4;i++){
			__MCAL_CLRBIT(port->AFR[0], (gpio_pin_config->PIN+i));
		}
	}
		else{
			port->AFR[0]|=(gpio_pin_config->AF<<(4*gpio_pin_config->PIN));
		}
	}
	//High Register
	else{
		if(!gpio_pin_config->AF){
			for(uint8_t i=0;i<4;i++){
				__MCAL_CLRBIT(port->AFR[1], (gpio_pin_config->PIN+i));
		}
	}
	else{
			port->AFR[1]|=(gpio_pin_config->AF<<((gpio_pin_config->PIN%8)*4));
		}
	}
 }
	
 
/* 								Read from Pin Function									*/

/**
	*@Brief 	Read from GPIO Pin
	*@Param1 	Pointer to GPIO Base Address
	*@Param2	Pin number
	*@RetVal	unsigned 8-bit integer
*/ 
uint8_t GPIO_READ(GPIO_TypeDef *port, uint8_t pin){
	 uint8_t val;
	 val=__MCAL_READBIT(port->IDR,pin);
	 return val;
 }	
/* 								Write to Pin Function									*/
/**
	*@Brief 	Write to GPIO Pin
	*@Param1 	Pointer to GPIO Base Address
	*@Param2	Pin number
	*@Param3	Value to be written
	*@RetVal	None
*/
void GPIO_WRITE(GPIO_TypeDef *port, uint8_t pin, uint8_t val){
	if(val)
		__MCAL_SETBIT(port->ODR,pin);
	else
		__MCAL_CLRBIT(port->ODR,pin);
}
/* 								Toggle to Pin Function									*/
/**
	*@Brief 	Toggle to GPIO Pin
	*@Param1 	Pointer to GPIO Base Address
	*@Param2	Pin number
	*@RetVal	None
*/
void GPIO_Toggle(GPIO_TypeDef *port, uint8_t pin){
	__MCAL_TOGBIT(port->ODR,pin);
}

/* 								GPIO Interrupt Configuration Function 			 		*/
/**
	*@Brief 	Configure GPIO interrupt 
	*@Param1 	uint8_t Pin Number
	*@Param2	Edge_Select enum
	*@RetVal	None
*/
void GPIO_INTERRUPT_CONFIG(uint8_t pin, EDGE_SELECT edge){
	//Setting Rising Edge Trigger Register
	if(edge==RISING_EDGE){
		EXTI->RTSR|=(1U<<pin);
	}
	//Setting Falling Edge Trigger Register
	else if(edge==FALLING_EDGE){
		EXTI->FTSR|=(1U<<pin);
	}
	//Setting Falling and Rising Edge Trigger Registers
	else if(edge==FALLING_RISING){
		EXTI->RTSR|=(1U<<pin);
		EXTI->FTSR|=(1U<<pin);
	}
	else{
	}
}

/* 								GPIO Interrupt Enable Function 			 				*/
/**
	*@Brief 	Enable GPIO interrupt 
	*@Param1 	uint8_t Pin Number
	*@Param2	IRQn_Type enum
	*@RetVal	None
*/
 void GPIO_INTERRUPT_ENABLE(uint8_t pin, IRQn_Type irq){
	//Unmask Interrupt
	EXTI->IMR|=(1U<<pin);
	//Enable Interrupt in NVIC
	NVIC_EnableIRQ(irq);
}

/* 								GPIO Interrupt Clear Function 			 				*/
/**
	*@Brief 	Clear interrupt flag by setting Pending Register
	*@Param1	Pin number
	*@RetVal	None
*/
void GPIO_INTERRUPT_CLEAR(uint8_t pin){
	//Check for Interrupt is Pended
	if(EXTI->PR&(1U<<pin)){
 		__MCAL_SETBIT(EXTI->PR,pin); 
	}
}
