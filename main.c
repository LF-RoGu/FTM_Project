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
 * @file    TareaFTM_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "GPIO.h"
#include "FlexTimer.h"
#include "NVIC.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

/*Output compare frequency = (bus clock)/(Prescaler(mod+1)).
 * Freq = 1M (1e6)
 * bus clock = 10.5M
 * Prescaler = 128
 * Mod = 0x02*/

uint8 FlagPortB = FALSE;
int main(void) {

	uint8 statePortA = 0;
	uint8 statePortB = 0;
	uint8 statePortC = 0;
	/*PushButton*/
	GPIO_pinControlRegisterType input_intr_config = GPIO_MUX1|GPIO_PE|GPIO_PS | INTR_FALLING_EDGE;

	GPIO_clock_gating(GPIO_A);
	GPIO_clock_gating(GPIO_B);
	GPIO_clock_gating(GPIO_C);
	/*clockGating*/
	FlexTimer_clockGating(FTM_0);
	/*output toogle on match*/
	FlexTimer_WPDIS_enable(FTM_0);
	FlexTimer_FTMEN_enable(FTM_0);

	/**Selects the FTM behavior in BDM mode.In this case in functional mode*/
	FTM0->CONF |= FTM_CONF_BDMMODE(3);
	/**Assign module register with a predefined value*/
	FlexTimer_MOD(FTM_0, 0x02);
	/*Selects the Edge-Aligned PWM mode mode*/
	FlexTimer_EdgeAligned_PWM(FTM_0);
	/*Assign channel value register with a predefined value
	 * CnV es el contador, que cada vez que se llega a este valor, se reinicia el cont*/
	FlexTimer0_updateCHValue(CH_0, 0x00);
	/*Select clock source and prescaler*/
	FlexTimer_CLKS_PS_PWM(FTM_0);

	while(1)
	{
		if (TRUE == GPIO_get_IRQ_status(GPIO_C)) {
			if (statePortC) {
				GPIOB->PDOR |= 0x00200000;/**Blue led off*/
			} else {
				GPIOB->PDOR &= ~(0x00200000);/**Blue led on*/
			}
			statePortC = !statePortC;
			GPIO_clear_IRQ_status(GPIO_C);
		}
		if(TRUE == GPIO_get_IRQ_status(GPIO_B))
		{
			if (statePortB) {
				GPIOB->PDOR |= 0x00400000;/**Read led off*/
			} else {
				GPIOB->PDOR &= ~(0x00400000);/**Read led on*/
			}
			statePortB = !statePortB;
			GPIO_clear_IRQ_status(GPIO_B);
		}

		if (TRUE == GPIO_get_IRQ_status(GPIO_A)) {
			if (statePortA) {
				GPIOB->PDOR |= 0x00400000;/**Read led off*/
			} else {
				GPIOB->PDOR &= ~(0x00400000);/**Read led on*/
			}
			statePortA = !statePortA;
			GPIO_clear_IRQ_status(GPIO_A);
		}
	}
    return 0 ;
}
