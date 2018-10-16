/*
 * GPIO.c
 *
 *  Created on: Sep 11, 2018
 *      Author: LuisFernando
 */

#include "MK64F12.h"
#include "GPIO.h"

static GPIO_interruptFlags_t GPIO_intrStatusFlag;

void SW2_INT()
{
	GPIO_intrStatusFlag.flagPortC = TRUE;
	GPIO_clear_interrupt(GPIO_C);
}
void SW3_INT()
{
	GPIO_intrStatusFlag.flagPortA  = TRUE;
	GPIO_clear_interrupt(GPIO_A);
}
void SW_Bn_INT()
{
	/*Leer todo el puerto, y dependiendo de que se llego a leer, es lo que se retornara
	 * Dependiendo del bit que se lea en (0 o 1, checar en la mañana)*/
	uint32 portRead;
	portRead /*negado?*/= GPIO_read_port(GPIO_B);
	if(portRead == B1)
	{
		GPIO_intrStatusFlag.flagB1 = TRUE;
		GPIO_clear_interrupt(GPIO_B);
	}
	if (portRead == B2)
	{
		GPIO_intrStatusFlag.flagB2 = TRUE;
		GPIO_clear_interrupt(GPIO_B);
	}
	if (portRead == B3)
	{
		GPIO_intrStatusFlag.flagB3 = TRUE;
		GPIO_clear_interrupt(GPIO_B);
	}
	if (portRead == B4)
	{
		GPIO_intrStatusFlag.flagB4 = TRUE;
		GPIO_clear_interrupt(GPIO_B);
	}
	if (portRead == B5)
	{
		GPIO_intrStatusFlag.flagB5 = TRUE;
		GPIO_clear_interrupt(GPIO_B);
	}
	if (portRead == B6)
	{
		GPIO_intrStatusFlag.flagB6 = TRUE;
		GPIO_clear_interrupt(GPIO_B);
	}
}


uint8 GPIO_get_IRQ_status(gpio_port_name_t gpio)
{
	switch (gpio) {
		case GPIO_A:
			return(GPIO_intrStatusFlag.flagPortA);
			break;
		case GPIO_B:
			return(GPIO_intrStatusFlag.flagPortB | GPIO_intrStatusFlag.flagB1 | GPIO_intrStatusFlag.flagB2 | GPIO_intrStatusFlag.flagB3 | GPIO_intrStatusFlag.flagB4 | GPIO_intrStatusFlag.flagB5 | GPIO_intrStatusFlag.flagB6);
			break;
		case GPIO_C:
			return(GPIO_intrStatusFlag.flagPortC);
			break;
		case GPIO_D:
			return(GPIO_intrStatusFlag.flagPortD);
			break;
		case GPIO_E:
			return(GPIO_intrStatusFlag.flagPortE);
			break;
		default:
			return(ERROR);
			break;
	}

}
uint8 GPIO_clear_IRQ_status(gpio_port_name_t gpio)
{
	switch (gpio) {
		case GPIO_A:
			GPIO_intrStatusFlag.flagPortA = FALSE;
			break;
		case GPIO_B:
			GPIO_intrStatusFlag.flagPortB = FALSE;
			/**/
			GPIO_intrStatusFlag.flagB1 = FALSE;
			GPIO_intrStatusFlag.flagB2 = FALSE;
			GPIO_intrStatusFlag.flagB3 = FALSE;
			GPIO_intrStatusFlag.flagB4 = FALSE;
			GPIO_intrStatusFlag.flagB5 = FALSE;
			GPIO_intrStatusFlag.flagB6 = FALSE;
			break;
		case GPIO_C:
			GPIO_intrStatusFlag.flagPortC = FALSE;
			break;
		case GPIO_D:
			GPIO_intrStatusFlag.flagPortD = FALSE;
			break;
		case GPIO_E:
			GPIO_intrStatusFlag.flagPortE = FALSE;
			break;
		default:
			return(ERROR);
			break;
	}

	return(TRUE);

}

void GPIO_clear_interrupt(gpio_port_name_t portName)
{
	switch(portName)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			PORTA->ISFR=CLEAR_INT;
			break;
		case GPIO_B: /** GPIO B is selected*/
			PORTB->ISFR=CLEAR_INT;
			break;
		case GPIO_C: /** GPIO C is selected*/
			PORTC->ISFR = CLEAR_INT;
			break;
		case GPIO_D: /** GPIO D is selected*/
			PORTD->ISFR=CLEAR_INT;
			break;
		default: /** GPIO E is selected*/
			PORTE->ISFR=CLEAR_INT;
			break;

	}// end switch
}

uint8_t GPIO_clock_gating(gpio_port_name_t portName)
{
	switch(portName)/** Selecting the GPIO for clock enabling*/
			{
				case GPIO_A: /** GPIO A is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
					break;
				case GPIO_B: /** GPIO B is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
					break;
				case GPIO_C: /** GPIO C is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
					break;
				case GPIO_D: /** GPIO D is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
					break;
				default: /**If doesn't exist the option*/
					return(FALSE);
			}// end switch
	/**Successful configuration*/
	return(TRUE);
}// end function

uint8_t GPIO_pin_control_register(gpio_port_name_t portName, uint8_t pin, GPIO_pinControlRegisterType* pinControlRegister)
{

	switch(portName)
		{
		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] = *pinControlRegister;
			break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin]= *pinControlRegister;
		default:/**If doesn't exist the option*/
			return(FALSE);
		break;
		}
	/**Successful configuration*/
	return(TRUE);
}

void GPIO_write_port(gpio_port_name_t portName, uint32 Data )
{
	switch(portName)
		{
			case GPIO_A:/** GPIOA is selected*/
				GPIOA->PDOR = Data;
				break;
			case GPIO_B:/** GPIOB is selected*/
				GPIOB->PDOR = Data;
				break;
			case GPIO_C:/** GPIOC is selected*/
				GPIOC->PDOR = Data;
				break;
			case GPIO_D:/** GPIOD is selected*/
				GPIOD->PDOR = Data;
				break;
			case GPIO_E: /** GPIOE is selected*/
				GPIOE->PDOR = Data;
			default:/**If doesn't exist the option*/
				break;
		}
}
uint32 GPIO_read_port(gpio_port_name_t portName)
{
	switch(portName)
	{
		case GPIO_A:/** GPIOA is selected*/
			return(GPIOA->PDIR);
			break;
		case GPIO_B:/** GPIOB is selected*/
			return(GPIOB->PDIR);
			break;
		case GPIO_C:/** GPIOC is selected*/
			return(GPIOC->PDIR);
			break;
		case GPIO_D:/** GPIOD is selected*/
			return(GPIOD->PDIR);
			break;
		case GPIO_E: /** GPIOE is selected*/
			return(GPIOE->PDIR);
		default:/**If doesn't exist the option*/
			return(FALSE);
			break;
	}
}
uint8_t GPIO_read_pin(gpio_port_name_t portName, uint8 pin)
{

	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			return((GPIOA->PDIR >> pin) & 0x01);
			break;
		case GPIO_B:/** GPIO B is selected*/
			return((GPIOB->PDIR >> pin) & 0x01);
			break;
		case GPIO_C:/** GPIO C is selected*/
			return((GPIOC->PDIR >> pin) & 0x01);
			break;
		case GPIO_D:/** GPIO D is selected*/
			return((GPIOD->PDIR >> pin) & 0x01);
			break;
		case GPIO_E: /** GPIO E is selected*/
			return((GPIOE->PDIR >> pin) & 0x01);

		default:/**If doesn't exist the option*/
			return(FALSE);
			break;
	}
}
void GPIO_set_pin(gpio_port_name_t portName, uint8 pin)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PSOR |= TRUE << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PSOR |= TRUE << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PSOR |= TRUE << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PSOR |= TRUE << pin;
			break;
		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PSOR |= TRUE << pin;
			break;
		default:
			break;
	}
}
void GPIO_clear_pin(gpio_port_name_t portName, uint8 pin)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PCOR |= TRUE << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PCOR |= TRUE << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PCOR |= TRUE  << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PCOR |= TRUE  << pin;
			break;
		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PCOR |= TRUE  << pin;
			break;
		default:
			break;
	}
}
void GPIO_toogle_pin(gpio_port_name_t portName, uint8 pin)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PTOR |= TRUE << pin;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PTOR |= TRUE << pin;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PTOR |= TRUE << pin;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PTOR |= TRUE << pin;
			break;
		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PTOR |= TRUE << pin;
			break;
		default:
			break;
	}
}
void GPIO_dataDirectionPORT(gpio_port_name_t portName ,uint32 direction)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR = direction;
			break;
		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR = direction;
			break;
		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR = direction;
			break;
		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR = direction;
			break;
		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PDDR = direction;
			break;
		default:
			break;
	}

}
void GPIO_data_direction_pin(gpio_port_name_t portName, uint8 State, uint8 pin)
{
	switch(portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR &= ~(TRUE << pin);
			GPIOA->PDDR |= State << pin;
			break;

		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR &= ~(TRUE << pin);
			GPIOB->PDDR |= State << pin;
			break;

		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR &= ~(TRUE << pin);
			GPIOC->PDDR |= State << pin;
			break;

		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR &= ~(TRUE << pin);
			GPIOD->PDDR |= State << pin;
			break;

		case GPIO_E:/** GPIO E is selected*/
			GPIOE->PDDR &= ~(TRUE << pin);
			GPIOE->PDDR |= State << pin;
			break;

		default:
			break;
	}
}
