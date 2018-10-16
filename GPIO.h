/*
 * GPIO.h
 *
 *  Created on: Sep 11, 2018
 *      Author: LuisFernando
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "Bits.h"

#define SW3_INT PORTA_IRQHandler
#define SW2_INT PORTC_IRQHandler

#define SW_Bn_INT PORTB_IRQHandler




#define ERROR 0x02

typedef struct
{
	uint8 flagPortA : 1;
	uint8 flagPortB : 1;
	uint8 flagPortC : 1;
	uint8 flagPortD : 1;
	uint8 flagPortE : 1;
	/*Banderas de los botones, estos son del puerto C*/
	uint8 flagB1	: 1;
	uint8 flagB2	: 1;
	uint8 flagB3	: 1;
	uint8 flagB4	: 1;
	uint8 flagB5	: 1;
	uint8 flagB6	: 1;
} GPIO_interruptFlags_t;

/*Constant that represents the switch_B*/
/*Corregir a los bits del nuevo puerto*/
#define SW_B1 BIT5
#define SW_B2 BIT7
#define SW_B3 BIT0
#define SW_B4 BIT9
#define SW_B5 BIT8
#define SW_B6 BIT1

/**/
/*Negar estas madres para que se compare con logica negativa?*/
/*6to bit 0xBF*/
#define B1	0x00000020
#define B2	0x00000080
#define B3	0x00000001
#define B4	0x00000200
#define B5	0x00000100
#define B6  0x00000002
#define SW2	0x00000040

/**/
#define CLEAR_INT 0xFFFFFFFF

/** Constant that represent the clock enable for GPIO A */
#define GPIO_CLOCK_GATING_PORTA 0x00000200
/** Constant that represent the clock enable for GPIO B */
#define GPIO_CLOCK_GATING_PORTB 0x00000400
/** Constant that represent the clock enable for GPIO C */
#define GPIO_CLOCK_GATING_PORTC 0x00000800
/** Constant that represent the clock enable for GPIO D  */
#define GPIO_CLOCK_GATING_PORTD 0x00001000
/** Constant that represent the clock enable for GPIO E */
#define GPIO_CLOCK_GATING_PORTE 0x00002000


/** Selects a pullup resistor */
#define GPIO_PS    0x00000001
/** Enables the pulldown or pullup resistors*/
#define GPIO_PE    0x00000002
/** Selects slow or fast slew rate */
#define GPIO_SRE   0x00000004
/** Enables the passive filter */
#define GPIO_PFE   0x00000010
/** Enables the open drain  */
#define GPIO_ODE   0x00000020
/** Selects between low drive strength and high drive strength */
#define GPIO_DSE   0x00000040
/** Selects alternative function 1 (GPIO) */
#define GPIO_MUX1  0x00000100
/** Selects alternative function 2 */
#define GPIO_MUX2  0x00000200
/** Selects alternative function 3 */
#define GPIO_MUX3  0x00000300
/** Selects alternative function 4 */
#define GPIO_MUX4  0x00000400
/** Selects alternative function 5 */
#define GPIO_MUX5  0x00000500
/** Selects alternative function 6 */
#define GPIO_MUX6  0x00000600
/** Selects alternative function 7 */
#define GPIO_MUX7  0x00000700
/** Sets DMA request on rising edge.*/
#define DMA_RISING_EDGE    0x00010000
/** Sets DMA request on falling edge.*/
#define DMA_FALLING_EDGE   0x00020000
/** Sets DMA request on either edge.*/
#define DMA_EITHER_EDGE    0x00030000
/** Sets Interrupt when logic 0.*/
#define INTR_LOGIC0        0x00080000
/** Sets Interrupt on rising-edge.*/
#define INTR_RISING_EDGE   0x00090000
/** Sets Interrupt on falling-edge.*/
#define INTR_FALLING_EDGE  0x000A0000
/** Sets Interrupt on either edge.*/
#define INTR_EITHER_EDGE   0x000B0000
/** Sets Interrupt when logic 1.*/
#define INTR_LOGIC1        0x000C0000

/*! This definition is used to configure whether a pin is an input or an output*/
typedef enum {GPIO_INPUT,/*!< Definition to configure a pin as input */
			  GPIO_OUTPUT /*!< Definition to configure a pin as output */
			 }GPIO_PIN_CONFIG;
/*! These constants are used to select an specific port in the different API functions*/
typedef enum{GPIO_A, /*!< Definition to select GPIO A */
			 GPIO_B, /*!< Definition to select GPIO B */
			 GPIO_C, /*!< Definition to select GPIO C */
			 GPIO_D, /*!< Definition to select GPIO D */
			 GPIO_E, /*!< Definition to select GPIO E */
			 GPIO_F  /*!< Definition to select GPIO F */
			} gpio_port_name_t;

/*! This data type is used to configure the pin control register*/
typedef const uint32 GPIO_pinControlRegisterType;
typedef const uint32 GPIO_pin_control_register_t;


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function clears all interrupts that were sensed by the GPIO.

 	 \param[in]  portName Port to clear interrupts.
 	 \return void
 	 \todo Implement a mechanism to clear interrupts by a specific pin.
 */
void GPIO_clear_interrupt(gpio_port_name_t portName);

/*
 * */
uint8 GPIO_clear_IRQ_status(gpio_port_name_t gpio);
uint8 GPIO_get_IRQ_status(gpio_port_name_t gpio);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enables the GPIO clock by configuring the corresponding bit
 	 	 and register in the System Clock Gating Control Register.

 	 \param[in]  portName Port to be configured.
 	 \return 1 if the portName is valid else return 0
 */
uint8 GPIO_clock_gating(gpio_port_name_t portName);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 This function configure different characteristics in an specific GPIO:
 	 	 pullup or pulldown resistor,slew rate, drive strength, passive filter,open drain pin,alternative functions in the GPIO
 	 \param[in] portName Port to be configured.
 	 \param[in]  pin Specific pin to be configured.
 	 \param[in]  pinControlRegister Pointer to a constant configuration value that configures the pin characteristics. In particular this function
 	 uses the definitions GPIO_PS, GPIO_PE, GPIO_MUX1 etc. For example, in order to configure the pullup resistor ans the pin as GPIO it is need to
 	 declare the type in following way:
 	 gpio_pin_control_register_t PinControlRegister = GPIO_MUX1|GPIO_PS|GPIO_PE;
 	 \return 1 if the portName is valid else return 0
 */
uint8 GPIO_pin_control_register(gpio_port_name_t portName, uint8 pin, GPIO_pinControlRegisterType* pinControlRegister);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 This function configure all the GPIO port as input when 1 logic is written or output when 0 logic is written.
 	 \param[in] portName Port to configure
 	 \param[in] direction Input value to specify the port as input or output.
 	 \return void

 */
void GPIO_dataDirectionPORT(gpio_port_name_t portName ,uint32 direction);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief  This function configure specific pins of a GPIO port as input when 1 logic is written or output when 0 logic is written.
 	 \param[in] portName Port to configure.
 	 \param[in] state Value to specify if the pin behaves as input or output.
 	 \param[in] pin Input value to specify the pin number.
 	 \return void
 */
void GPIO_data_direction_pin(gpio_port_name_t portName, uint8 state, uint8 pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function reads all the GPIO port.
 	 \param[in] portName Port to be read.
 	 \return  It is the value read from a GPIO. It is a 32-bit value.

 */
uint32 GPIO_read_port(gpio_port_name_t portName);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function reads a specific GPIO pin.
	 \param[in] portName Port to be read.
 	 \param[in] pin Pin to be read.
 	 \return This function return 0 if the value of the pin is 0 logic or 1 is the value the pin is 1 logic.
 */
uint8 GPIO_read_pin(gpio_port_name_t portName, uint8 pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This function writes all the GPIO port.
 	 \param[in] portName Port to be written.
 	 \param[in] data Value to be written.
 	 \return void
 */
void GPIO_write_port(gpio_port_name_t portName, uint32 Data);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief This set an specific pin in a GPIO port, it uses GPIO_PSOR register.
 	\param[in] portName Port to be selected.
 	\param[in] pin Pin to be set.
 	\return void
 */
void GPIO_set_pin(gpio_port_name_t portName, uint8 pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This clear an specific pin in a GPIO port, it uses GPIO_PCOR register.
 	 \param[in] portName Selected Port.
 	 \param[in] pin Pin to be clear.
 	 \return void
 */
void GPIO_clear_pin(gpio_port_name_t portName, uint8 pin);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This toggle the value of a specific pin in a GPIO port, it uses GPIO_PTOR register.
 	 \param[in] portName Selected Port.
 	 \param[in] pin Pin to be toggled.
 	 \return void
 */
void GPIO_toogle_pin(gpio_port_name_t portName, uint8 pin);

#endif /* GPIO_H_ */