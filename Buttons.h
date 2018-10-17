/*
 * Buttons.h
 *
 *  Created on: Oct 16, 2018
 *      Author: LuisFernando
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "GPIO.h"
#include "Bits.h"
#include "NVIC.h"

typedef struct{
	uint8 flagPortB0 :1;
	uint8 flagPortB1 :1;
	uint8 flagPortB2 :1;
	uint8 flagPortB3 :1;
	uint8 flagPortB4 :1;
	uint8 flagPortB5 :1;
	uint8 flagPortB6 :1;
}GPIO_flag_buttons_t;

/*Constant that represents the switch_B*/
#define SW_B0 BIT2
#define SW_B1 BIT3
#define SW_B2 BIT10
#define SW_B3 BIT11
#define SW_B4 BIT19
#define SW_B5 BIT18
#define SW_B6 BIT9

/**/
#define B0	0x00000002
#define B1	0x00000004
#define B2	0x00000400
#define B3	0x00000800
#define B4	0x00080000
#define B5	0x00040000
#define B6  0x00000200

void buttons_init(void);

void intr_init(void);

#endif /* BUTTONS_H_ */
