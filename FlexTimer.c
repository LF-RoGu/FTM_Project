/*
 * FlexTimer.c
 *
 *  Created on: Oct 12, 2018
 *      Author: LuisFernando
 */

#include "FlexTimer.h"

/*Pulse Width Modulation*/

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
	default:
		break;
	}
}
void FlexTimer1_updateCHValue(CH_type channel, sint16 channelValue) {
	switch (channel)
	{
	case CH_0:
		FTM1->CONTROLS[0].CnV = channelValue;
		break;
	case CH_1:
		FTM1->CONTROLS[1].CnV = channelValue;
		break;
	case CH_2:
		FTM1->CONTROLS[2].CnV = channelValue;
		break;
	case CH_3:
		FTM1->CONTROLS[3].CnV = channelValue;
		break;
	case CH_4:
		FTM1->CONTROLS[4].CnV = channelValue;
		break;
	case CH_5:
		FTM1->CONTROLS[5].CnV = channelValue;
		break;
	default:
		break;
	}
}
void FlexTimer_clockGating(FTM_type FlexTimer) {
	switch (FlexTimer) {
	case FTM_0:
		SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
		break;
	case FTM_1:
		SIM->SCGC6 |= SIM_SCGC6_FTM1_MASK;
		break;

	case FTM_2:
		SIM->SCGC3 |= SIM_SCGC3_FTM2_MASK;
		break;

	case FTM_3:
		SIM->SCGC3 |= SIM_SCGC3_FTM3_MASK;
		break;

	default:
		SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;
		break;
	}
}
void FlexTimer_WPDIS_enable(FTM_type FlexTimer) {

	switch (FlexTimer) {
	case FTM_0:
		FTM0->MODE |= FLEX_TIMER_WPDIS;
		break;
	case FTM_1:
		FTM1->MODE |= FLEX_TIMER_WPDIS;
		break;

	case FTM_2:
		FTM2->MODE |= FLEX_TIMER_WPDIS;
		break;

	case FTM_3:
		FTM3->MODE |= FLEX_TIMER_WPDIS;
		break;

	default:
		FTM0->MODE |= FLEX_TIMER_WPDIS;
		break;
	}
}
void FlexTimer_FTMEN_enable(FTM_type FlexTimer) {
	switch (FlexTimer) {
	case FTM_0:
		FTM0->MODE &= ~FLEX_TIMER_FTMEN;
		break;
	case FTM_1:
		FTM1->MODE &= ~FLEX_TIMER_FTMEN;
		break;

	case FTM_2:
		FTM2->MODE &= ~FLEX_TIMER_FTMEN;
		break;

	case FTM_3:
		FTM3->MODE &= ~FLEX_TIMER_FTMEN;
		break;

	default:
		FTM0->MODE &= ~FLEX_TIMER_FTMEN;
		break;
	}
}
void FlexTimer_MOD(FTM_type FlexTimer, uint8 mod) {

	switch (FlexTimer) {
	case FTM_0:
		FTM0->MOD = mod;
		break;
	case FTM_1:
		FTM1->MOD = mod;
		break;
	case FTM_2:
		FTM2->MOD = mod;
		break;
	case FTM_3:
		FTM3->MOD = mod;
		break;
	default:
		FTM0->MOD = mod;
		break;
	}
}
void FTM0_EdgeAligned_PWM(CH_type channel) {
	switch (channel) {
	case CH_0:
		FTM0->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_1:
		FTM0->CONTROLS[1].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_2:
		FTM0->CONTROLS[2].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_3:
		FTM0->CONTROLS[3].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_4:
		FTM0->CONTROLS[4].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_5:
		FTM0->CONTROLS[5].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_6:
		FTM0->CONTROLS[6].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	default:
		FTM0->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	}
}
void FTM1_EdgeAligned_PWM(CH_type channel) {
	switch (channel) {
	case CH_0:
		FTM1->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_1:
		FTM1->CONTROLS[1].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_2:
		FTM1->CONTROLS[2].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_3:
		FTM1->CONTROLS[3].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_4:
		FTM1->CONTROLS[4].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_5:
		FTM1->CONTROLS[5].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_6:
		FTM1->CONTROLS[6].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	default:
		FTM1->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	}
}
void FTM2_EdgeAligned_PWM(CH_type channel) {
	switch (channel) {
	case CH_0:
		FTM2->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_1:
		FTM2->CONTROLS[1].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_2:
		FTM2->CONTROLS[2].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_3:
		FTM2->CONTROLS[3].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_4:
		FTM2->CONTROLS[4].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_5:
		FTM2->CONTROLS[5].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_6:
		FTM2->CONTROLS[6].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	default:
		FTM2->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	}
}
void FTM3_EdgeAligned_PWM(CH_type channel) {
	switch (channel) {
	case CH_0:
		FTM3->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_1:
		FTM3->CONTROLS[1].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_2:
		FTM3->CONTROLS[2].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_3:
		FTM3->CONTROLS[3].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_4:
		FTM3->CONTROLS[4].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_5:
		FTM3->CONTROLS[5].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	case CH_6:
		FTM3->CONTROLS[6].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	default:
		FTM3->CONTROLS[0].CnSC = FLEX_TIMER_MSB | FLEX_TIMER_ELSB;
		break;
	}
}
void FlexTimer_CLKS_PS_PWM(FTM_type FlexTimer) {
	switch (FlexTimer) {
	case FTM_0:
		FTM0->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;
		break;
	case FTM_1:
		FTM1->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;
		break;
	case FTM_2:
		FTM2->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;
		break;
	case FTM_3:
		FTM3->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;
		break;
	default:
		FTM0->SC = FLEX_TIMER_CLKS_1 | FLEX_TIMER_PS_128;
		break;
	}
}
