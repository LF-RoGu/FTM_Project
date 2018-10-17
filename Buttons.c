/*
 * Buttons.c
 *
 *  Created on: Oct 16, 2018
 *      Author: LuisFernando
 */

#include "Buttons.h"

void buttons_config(void)
{
	GPIO_pinControlRegisterType input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_FALLING_EDGE;

	/*Configuramos el PCR de los pines que seran push button*/
	GPIO_pin_control_register(GPIO_B, BIT2, &input_intr_config);
	GPIO_pin_control_register(GPIO_B, BIT3, &input_intr_config);
	GPIO_pin_control_register(GPIO_B, BIT10, &input_intr_config);
	GPIO_pin_control_register(GPIO_B, BIT11, &input_intr_config);
	GPIO_pin_control_register(GPIO_B, BIT19, &input_intr_config);
	GPIO_pin_control_register(GPIO_B, BIT18, &input_intr_config);
	GPIO_pin_control_register(GPIO_B, BIT9, &input_intr_config);
	GPIO_pin_control_register(GPIO_C, BIT6, &input_intr_config);

	/*Configuramos como entraba los pines del puerto B, que usaremos*/
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, BIT2);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, BIT3);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, BIT10);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, BIT11);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, BIT19);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, BIT18);
	GPIO_data_direction_pin(GPIO_B, GPIO_INPUT, BIT9);
}

void intr_init(void)
{
	/**Sets the threshold for interrupts, if the interrupt has higher priority constant that the BASEPRI, the interrupt will not be attended*/
	NVIC_setBASEPRI_threshold(PRIORITY_11);

	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enableInterruptAndPriotity(PORTA_IRQ, PRIORITY_4);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enableInterruptAndPriotity(PORTB_IRQ, PRIORITY_4);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enableInterruptAndPriotity(PORTC_IRQ, PRIORITY_4);

	EnableInterrupts;
}
