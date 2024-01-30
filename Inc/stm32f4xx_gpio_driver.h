/*
 * stm32f4xx_gpio_driver.h
 *
 *  Created on: Jan 26, 2024
 *      Author: yadav
 */

#ifndef STM32F4XX_GPIO_DRIVER_H_
#define STM32F4XX_GPIO_DRIVER_H_

#include <stdint.h>
#include "stm32f4xx_drivers.h"


/*
 * This is a configuration structure for GPIO pin
 */

typedef struct {
	uint8_t GPIO_PinNumber;			/* possible pins available @Gpio_pinS*/
	uint8_t	GPIO_PinMode;			/* possible modes available @Gpio_pin_Modes*/
	uint8_t	GPIO_PinSpeed;			/* possible output speeds available @Gpio_output_Speed_types*/
	uint8_t	GPIO_PinPupdControl;	/* possible PUPD types available @Gpio_pull_up_pull_down_Types*/
	uint8_t	GPIO_PinOPType;			/* possible output types available @Gpio_Output_Types*/
	uint8_t	GPIO_PinAltFuncMode;	/* possible modes available @Gpio_pin_Modes*/
}Gpio_Pinconfig_t;

typedef struct {
	GPIO_RegDef_t *pGpiox; //This holds the baseaddress of gpio port which pin belongs
	Gpio_Pinconfig_t GPIO_Pinconfig; // This holds GPIO pin config settings.
}Gpio_Handle_t;



/*
 * @Gpio_pins
 * Gpio possible pins
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @Gpio_pin_Modes
 * GPIO Pin possible Modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALT		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RF		6

/*
 * @Gpio_Output_Types
 * Gpio pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @Gpio_output_Speed_types
 * Gpio pin possible output speed types
 */
#define GPIO_OP_SP_LOW			0
#define GPIO_OP_SP_MED			1
#define GPIO_OP_SP_HIGH			2
#define GPIO_OP_SP_VHIGH		3

/* @Gpio_pull_up_pull_down_Types
 * Gpio pin pull_up and pull_down types
 */
#define GPIO_NO_PU				0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2




/****************************************************************************************************************************
 *												API Prototypes of this driver
 * 								for more information about this APIs check the function definitions
 ****************************************************************************************************************************/

/*
 * Peripheral clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGpiox, uint8_t EnorDi);

/*
 * Gpio initialize and deinitialize
 */
void GPIO_Init(Gpio_Handle_t *pGpioHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGpiox);
/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGpiox, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGpiox);
void GPIO_WriteTOOutputPin(GPIO_RegDef_t *pGpiox,uint8_t PinNumber, uint8_t Value );
void GPIO_WriteToOutpitPort(GPIO_RegDef_t *pGpiox,uint16_t Value );
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGpiox, uint8_t PinNumber);
/*
 * Irq configuration and ISR handling
 */

void IRQ_Config(uint8_t IRQNumber,uint8_t IRQPriority ,uint8_t EnorDi);
void IRQ_Handling(uint8_t PinNumber);













#endif /* STM32F4XX_GPIO_DRIVER_H_ */
