/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    PushButton_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "NVIC.h"
#include "GPIO.h"
#include "Bits.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
int main(void)
{
	uint8 statePortA = 0;
	uint8 statePortB0,statePortB1,statePortB2,statePortB3,statePortB4,statePortB5,statePortB6;
	uint8 statePortC = 0;

	GPIO_clock_gating(GPIO_A);
	GPIO_clock_gating(GPIO_B);
	GPIO_clock_gating(GPIO_C);

	/**Pin control configuration of GPIOB pin21 and pin21 as GPIO by using a special macro contained in Kinetis studio in the MK64F12. h file*/
	PORTB->PCR[21] = PORT_PCR_MUX(1);
	PORTB->PCR[22] = PORT_PCR_MUX(1);

	GPIO_pinControlRegisterType input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS|INTR_FALLING_EDGE;

	GPIO_pin_control_register(GPIO_A, BIT4, &input_intr_config);
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
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT,BIT2);
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT,BIT3);
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT,BIT10);
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT,BIT11);
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT,BIT19);
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT,BIT18);
	GPIO_data_direction_pin(GPIO_B,GPIO_INPUT,BIT9);

	GPIO_data_direction_pin(GPIO_B,GPIO_OUTPUT,BIT21);
	GPIO_data_direction_pin(GPIO_B,GPIO_OUTPUT,BIT22);

	/**Sets the threshold for interrupts, if the interrupt has higher priority constant that the BASEPRI, the interrupt will not be attended*/
	NVIC_setBASEPRI_threshold(PRIORITY_11);

	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enableInterruptAndPriotity(PORTA_IRQ, PRIORITY_4);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enableInterruptAndPriotity(PORTB_IRQ, PRIORITY_4);
	/**Enables and sets a particular interrupt and its priority*/
	NVIC_enableInterruptAndPriotity(PORTC_IRQ, PRIORITY_4);

	EnableInterrupts;


	while (1) {
		if (TRUE == GPIO_get_IRQ_status(GPIO_A)) {
			if (statePortA) {
				GPIOB->PDOR |= 0x00400000;/**Read led off*/
				GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			} else {
				GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
			}
			statePortA = !statePortA;
			GPIO_clear_IRQ_status(GPIO_A);
		}
		if (TRUE == GPIO_get_IRQ_status(GPIO_C)) {
			if (statePortC) {
				GPIOB->PDOR |= 0x00400000;/**Read led off*/
				GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			} else {
				GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
			}
			statePortC = !statePortC;
			GPIO_clear_IRQ_status(GPIO_C);
		}
		if (TRUE == GPIO_get_IRQ_statusB0()) {
			if (statePortB0) {
				GPIOB->PDOR |= 0x00400000;/**Read led off*/
				GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			} else {
				GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
			}
			statePortB0 = !statePortB0;
			GPIO_clear_IRQ_status(GPIO_B);
		}
		if(TRUE == GPIO_get_IRQ_statusB1()){
			if(statePortB1){
				GPIOB->PDOR |= 0x00400000;/**Read led off*/
				GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			}
			else{
				GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
			}
			statePortB1 =!statePortB1;
			GPIO_clear_IRQ_status(GPIO_B);
		}
		if(TRUE == GPIO_get_IRQ_statusB2())
		{
			if (statePortB2) {
				GPIOB->PDOR |= 0x00400000;/**Read led off*/
				GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			} else {
				GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
			}
			statePortB2 = !statePortB2;
			GPIO_clear_IRQ_status(GPIO_B);
		}
		if (TRUE == GPIO_get_IRQ_statusB3()) {
			if (statePortB3) {
				GPIOB->PDOR |= 0x00400000;/**Read led off*/
				GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			} else {
				GPIOB->PDOR &= ~(0x00400000);/**Red led on*/
			}
			statePortB3 = !statePortB3;
			GPIO_clear_IRQ_status(GPIO_B);
		}

	}
    return 0 ;
}
