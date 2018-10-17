/*
 * GPIO.c
 *
 *  Created on: Sep 11, 2018
 *      Author: LuisFernando
 */

#include "MK64F12.h"
#include "GPIO.h"
#include "Buttons.h"

static GPIO_interruptFlags_t GPIO_intrStatusFlag;
static GPIO_flag_buttons_t GPIO_flag_buttons;

void SW2_INT()
{
	GPIO_intrStatusFlag.flagPortC = TRUE;
	GPIO_clear_interrupt(GPIO_C);
	printf("Entro intr SW2\n");
}
void SW3_INT()
{
	GPIO_intrStatusFlag.flagPortA  = TRUE;
	GPIO_clear_interrupt(GPIO_A);
	printf("Entro intr SW3\n");
}
void SW_Bn_INT()
{
	/*Leer todo el puerto, y dependiendo de que se llego a leer, es lo que se retornara
	 * Dependiendo del bit que se lea en (0 o 1, checar en la mañana)*/
	/*Modificar a dejar de leer el puerto a solo leer el pin*/
	uint32 intrB0,intrB1,intrB2,intrB3,intrB4,intrB5,intrB6;
	intrB0 = GPIO_read_pin(GPIO_B,SW_B0);
	intrB1 = GPIO_read_pin(GPIO_B,SW_B1);
	intrB2 = GPIO_read_pin(GPIO_B,SW_B2);
	intrB3 = GPIO_read_pin(GPIO_B,SW_B3);
	intrB4 = GPIO_read_pin(GPIO_B,SW_B4);
	intrB5 = GPIO_read_pin(GPIO_B,SW_B5);
	intrB6 = GPIO_read_pin(GPIO_B,SW_B6);
	if (intrB0 == TRUE)
	{
		GPIO_flag_buttons.flagPortB0 = TRUE;
		GPIO_clear_IRQ_statusB0();
		printf("Entro intr B0\n");
	}
	if (intrB1 == TRUE)
	{
		GPIO_flag_buttons.flagPortB1 = TRUE;
		GPIO_clear_IRQ_statusB1();
		printf("Entro intr B1\n");
	}
	if (intrB2 == TRUE)
	{
		GPIO_flag_buttons.flagPortB2 = TRUE;
		GPIO_clear_IRQ_statusB2();
		printf("Entro intr B2\n");
	}
	if (intrB3 == TRUE)
	{
		GPIO_flag_buttons.flagPortB3 = TRUE;
		GPIO_clear_IRQ_statusB3();
		printf("Entro intr B3\n");
	}
	if (intrB4 == TRUE)
	{
		GPIO_flag_buttons.flagPortB4 = TRUE;
		GPIO_clear_IRQ_statusB4();
		printf("Entro intr B4\n");
	}
	if (intrB5 == TRUE)
	{
		GPIO_flag_buttons.flagPortB5 = TRUE;
		GPIO_clear_IRQ_statusB5();
		printf("Entro intr B5\n");
	}
	if (intrB6 == TRUE)
	{
		GPIO_flag_buttons.flagPortB6 = TRUE;
		GPIO_clear_IRQ_statusB6();
		printf("Entro intr B6\n");
	}
	else
	{
		GPIO_intrStatusFlag.flagPortB = TRUE;
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
			return(GPIO_intrStatusFlag.flagPortB);
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
uint8 GPIO_get_IRQ_statusB0(void)
{
	return (GPIO_flag_buttons.flagPortB0);
}
void GPIO_clear_IRQ_statusB0()
{
	GPIO_flag_buttons.flagPortB0 = FALSE;
}
uint8 GPIO_get_IRQ_statusB1(void)
{
	return (GPIO_flag_buttons.flagPortB1);
}
void GPIO_clear_IRQ_statusB1()
{
	GPIO_flag_buttons.flagPortB1 = FALSE;
}
uint8 GPIO_get_IRQ_statusB2(void)
{
	return (GPIO_flag_buttons.flagPortB2);
}
void GPIO_clear_IRQ_statusB2()
{
	GPIO_flag_buttons.flagPortB2 = FALSE;
}
uint8 GPIO_get_IRQ_statusB3(void)
{
	return (GPIO_flag_buttons.flagPortB3);
}
void GPIO_clear_IRQ_statusB3()
{
	GPIO_flag_buttons.flagPortB3 = FALSE;
}
uint8 GPIO_get_IRQ_statusB4(void)
{
	return (GPIO_flag_buttons.flagPortB4);
}
void GPIO_clear_IRQ_statusB4()
{
	GPIO_flag_buttons.flagPortB4 = FALSE;
}
uint8 GPIO_get_IRQ_statusB5(void)
{
	return (GPIO_flag_buttons.flagPortB5);
}
void GPIO_clear_IRQ_statusB5()
{
	GPIO_flag_buttons.flagPortB5 = FALSE;
}
uint8 GPIO_get_IRQ_statusB6(void)
{
	return (GPIO_flag_buttons.flagPortB6);
}
void GPIO_clear_IRQ_statusB6()
{
	GPIO_flag_buttons.flagPortB6 = FALSE;
}
uint8 GPIO_clear_IRQ_status(gpio_port_name_t gpio)
{
	switch (gpio) {
		case GPIO_A:
			GPIO_intrStatusFlag.flagPortA = FALSE;
			break;
		case GPIO_B:
			GPIO_intrStatusFlag.flagPortB = FALSE;
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
