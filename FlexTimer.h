/*
 * FlexTimer.h
 *
 *  Created on: Oct 12, 2018
 *      Author: LuisFernando
 */

#ifndef FLEXTIMER_H_
#define FLEXTIMER_H_

#include "MK64F12.h"
#include "Bits.h"
#include "GPIO.h"

/*Valor maximo de 255*/
#define FLEX_TIMER_0_OVERFLOW_LIMIT 0x00FF
/*Tolerancia para valor minimo de 0*/
#define FLEX_TIMER_0_UNDERFLOW_LIMIT 0x0001

#define FLEX_TIMER_0_CLOCK_GATING 0x01000000

#define FLEX_TIMER_FAULTIE  0x80
#define FLEX_TIMER_FAULTM_0   0x00
#define FLEX_TIMER_FAULTM_1   0x20
#define FLEX_TIMER_FAULTM_2   0x40
#define FLEX_TIMER_FAULTM_3   0x60
#define FLEX_TIMER_CAPTEST  0x10
#define FLEX_TIMER_PWMSYNC  0x08
#define FLEX_TIMER_WPDIS    0x04
#define FLEX_TIMER_INIT     0x02
#define FLEX_TIMER_FTMEN    0x01

#define FLEX_TIMER_TOF     0x80
#define FLEX_TIMER_TOIE    0x40
#define FLEX_TIMER_CPWMS   0x20
#define FLEX_TIMER_CLKS_0  0x00
#define FLEX_TIMER_CLKS_1  0x08
#define FLEX_TIMER_CLKS_2  0x10
#define FLEX_TIMER_CLKS_3  0x18
#define FLEX_TIMER_PS_1    0x00
#define FLEX_TIMER_PS_2    0x01
#define FLEX_TIMER_PS_4    0x02
#define FLEX_TIMER_PS_8    0x03
#define FLEX_TIMER_PS_16    0x04
#define FLEX_TIMER_PS_32    0x05
#define FLEX_TIMER_PS_64    0x06
#define FLEX_TIMER_PS_128    0x07

#define FLEX_TIMER_PWMLOAD_CH0 0x01
#define FLEX_TIMER_PWMLOAD_CH1 0x02
#define FLEX_TIMER_PWMLOAD_CH2 0x04
#define FLEX_TIMER_PWMLOAD_CH3 0x08
#define FLEX_TIMER_PWMLOAD_CH4 0x10
#define FLEX_TIMER_PWMLOAD_CH5 0x20
#define FLEX_TIMER_PWMLOAD_CH6 0x40
#define FLEX_TIMER_PWMLOAD_CH7 0x80
#define FLEX_TIMER_LDOK        0x200


#define  FLEX_TIMER_DMA   0x01
#define  FLEX_TIMER_ELSA  0x04
#define  FLEX_TIMER_ELSB  0x08
#define  FLEX_TIMER_MSA   0x10
#define  FLEX_TIMER_MSB   0x20
#define  FLEX_TIMER_CHIE  0x40
#define  FLEX_TIMER_CHF   0x80

typedef enum{
	FTM_0,
	FTM_1,
	FTM_2,
	FTM_3
}FTM_type;
typedef enum{
	CH_0,
	CH_1,
	CH_2,
	CH_3,
	CH_4,
	CH_5,
	CH_6
}CH_type;

/**/
void FlexTimer0_updateCHValue(CH_type channel, sint16 channelValue);

/**/
void FlexTimer1_updateCHValue(CH_type channel, sint16 channelValue);

/**/
void FlexTimer_clockGating(FTM_type FlexTimer);

/**/
void FlexTimer_WPDIS_enable(FTM_type FlexTimer);

/**/
void FlexTimer_FTMEN_enable(FTM_type FlexTimer);

/*
 * Sets the overflow fot the FMT_MOD*/
void FlexTimer_MOD(FTM_type FlexTimer, uint8 mod);

/**/
void FTM0_EdgeAligned_PWM(CH_type channel);

/**/
void FTM1_EdgeAligned_PWM(CH_type channel);

/**/
void FTM2_EdgeAligned_PWM(CH_type channel);

/**/
void FTM3_EdgeAligned_PWM(CH_type channel);

/**/
void FlexTimer_CLKS_PS_PWM(FTM_type FlexTimer);

#endif /* FLEXTIMER_H_ */
