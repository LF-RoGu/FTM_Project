/*
 * FlexTimer.c
 *
 *  Created on: Oct 12, 2018
 *      Author: LuisFernando
 */

#include "FlexTimer.h"

/*Pulse Width Modulation*/


/**
 * Output compare frequency = (bus clock)/(Prescaler(mod+1)).
 * Note that is the same frequency of the overflow flow.
 */


void FTM0_ISR() {
	/*Limpia la bandera del flextimer*/
	FTM0->SC &= ~FLEX_TIMER_TOF;
	GPIOD->PDOR ^= FLEX_TIMER_0_OVERFLOW_LIMIT;
}
/*Intento V2*/
void FlexTimer0_updateCHValue(CH_type channel, sint16 channelValue) {
	switch (channel)
	{
	case CH_0:
		FTM0->CONTROLS[0].CnV = channelValue;
		break;
	case CH_1:
		FTM0->CONTROLS[1].CnV = channelValue;
		break;
	case CH_2:
		FTM0->CONTROLS[2].CnV = channelValue;
		break;
	case CH_3:
		FTM0->CONTROLS[3].CnV = channelValue;
		break;
	case CH_4:
		FTM0->CONTROLS[4].CnV = channelValue;
		break;
	case CH_5:
		FTM0->CONTROLS[5].CnV = channelValue;
		break;
	case CH_6:
		FTM0->CONTROLS[6].CnV = channelValue;
		break;
	default:
		break;
	}
}
void FlexTimer_clockGating(FTM_type FlexTimer) {
	switch (FlexTimer) {
	case FTM_0:
		SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;/** Habilita el clock  del FTM0*/
		break;
	case FTM_1:
		SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;/** Habilita el clock  del FTM1*/
		break;

	case FTM_2:
		SIM->SCGC3 |= SIM_SCGC3_FTM2_MASK;/** Habilita el clock  del FTM2*/
		break;

	case FTM_3:
		SIM->SCGC3 |= SIM_SCGC3_FTM3_MASK;/** Habilita el clock  del FTM3*/
		break;

	default:
		SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;/** Habilita el clock  del FTM0*/
		break;
	}
}
void FlexTimer_WPDIS_enable(FTM_type FlexTimer) {

	switch (FlexTimer) {
	case FTM_0:
		FTM0->MODE |= FLEX_TIMER_WPDIS;/** Habilita el WPDIS del FTM0*/
		break;
	case FTM_1:
		FTM1->MODE |= FLEX_TIMER_WPDIS;/** Habilita el WPDIS del FTM1*/
		break;

	case FTM_2:
		FTM2->MODE |= FLEX_TIMER_WPDIS;/** Habilita el WPDIS del FTM2*/
		break;

	case FTM_3:
		FTM3->MODE |= FLEX_TIMER_WPDIS;/** Habilita el WPDIS del FTM3*/
		break;

	default:
		FTM0->MODE |= FLEX_TIMER_WPDIS;/** Habilita el WPDIS del FTM0*/
		break;
	}
}
void FlexTimer_FTMEN_enable(FTM_type FlexTimer) {
	switch (FlexTimer) {
	case FTM_0:
		FTM0->MODE &= ~FLEX_TIMER_FTMEN;/** Habilita el FTMEN del FTM0*/
		break;
	case FTM_1:
		FTM1->MODE &= ~FLEX_TIMER_FTMEN;/** Habilita el FTMEN del FTM1*/
		break;

	case FTM_2:
		FTM2->MODE &= ~FLEX_TIMER_FTMEN;/** Habilita el FTMEN del FTM2*/
		break;

	case FTM_3:
		FTM3->MODE &= ~FLEX_TIMER_FTMEN;/** Habilita el FTMEN del FTM3*/
		break;

	default:
		FTM0->MODE &= ~FLEX_TIMER_FTMEN;/** Habilita el FTMEN del FTM0*/
		break;
	}
}
void FlexTimer_MOD(FTM_type FlexTimer, uint8 mod) {

	switch (FlexTimer) {
	case FTM_0:
		FTM0->MOD = mod;/** Carga el valor de mod al registro interno de configuración MOD del FTM0 */
		break;
	case FTM_1:
		FTM1->MOD = mod;/** Carga el valor de mod al registro interno de configuración MOD del FTM1 */
		break;

	case FTM_2:
		FTM2->MOD = mod;/** Carga el valor de mod al registro interno de configuración MOD del FTM2 */
		break;

	case FTM_3:
		FTM3->MOD = mod;/** Carga el valor de mod al registro interno de configuración MOD del FTM3 */
		break;

	default:
		FTM0->MOD = mod;/** Carga el valor de mod al registro interno de configuración MOD del FTM0 */
		break;
	}
}
void FlexTimer_EdgeAligned_PWM(FTM_type FlexTimer) {
	/**Selects the Edge-Aligned PWM mode mode*/
	switch (FlexTimer) {
	case FTM_0:
		FTM0->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;/** Se selecciona el modo de Edge Aligned del FTM0 */
		break;
	case FTM_1:
		FTM1->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;/** Se selecciona el modo de Edge Aligned del FTM1 */
		break;

	case FTM_2:
		FTM2->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;/** Se selecciona el modo de Edge Aligned del FTM2 */
		break;

	case FTM_3:
		FTM3->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;/** Se selecciona el modo de Edge Aligned del FTM3 */
		break;

	default:
		FTM0->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;/** Se selecciona el modo de Edge Aligned del FTM0 */
		break;
	}
}
void FlexTimer_CLKS_PS_PWM(FTM_type FlexTimer) {
	switch (FlexTimer) {
	case FTM_0:
		FTM0->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;/** Se selecciona el prescaler de 128 para el FTM0 */
		break;
	case FTM_1:
		FTM1->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;/** Se selecciona el prescaler de 128 para el FTM1 */
		break;

	case FTM_2:
		FTM2->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;/** Se selecciona el prescaler de 128 para el FTM2 */
		break;

	case FTM_3:
		FTM3->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;/** Se selecciona el prescaler de 128 para el FTM3 */
		break;

	default:
		FTM0->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;/** Se selecciona el prescaler de 128 para el FTM0 */
		break;
	}
}
