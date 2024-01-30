/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Jan 26, 2024
 *      Author: yadav
 */


#include "stm32f4xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */

/**
 * 	@fn				- GPIO_PeriClockControl
 *
 * 	@brief			- Enables the peripheral clock of GPIO
 *
 *  @param[in]		- parameter holds the base address of the GPIO port
 *
 *  @param[in]		- Enable or diable macros
 *
 *  @return 		- none.
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGpiox, uint8_t EnorDi)
{
	if(ENABLE == EnorDi){

		if(pGpiox == GPIOA){

			GPIOA_PCLK_EN();
		}
		else if(pGpiox == GPIOB){

			GPIOB_PCLK_EN();
		}
		else if(pGpiox == GPIOC){

			GPIOC_PCLK_EN();
		}
		else if(pGpiox == GPIOD){

			GPIOD_PCLK_EN();
		}
		else if(pGpiox == GPIOE){

			GPIOE_PCLK_EN();
		}
		else if(pGpiox == GPIOF){

			GPIOF_PCLK_EN();
		}
		else if(pGpiox == GPIOG){

			GPIOG_PCLK_EN();
		}
		else if(pGpiox == GPIOH){

			GPIOH_PCLK_EN();
		}
		else if(pGpiox == GPIOI){

			GPIOI_PCLK_EN();
		}

	}
	else{
		if(pGpiox == GPIOA){

			GPIOA_PCLK_DI();
		}
		else if(pGpiox == GPIOB){

			GPIOB_PCLK_DI();
		}
		else if(pGpiox == GPIOC){

			GPIOC_PCLK_DI();
		}
		else if(pGpiox == GPIOD){

			GPIOD_PCLK_DI();
		}
		else if(pGpiox == GPIOE){

			GPIOE_PCLK_DI();
		}
		else if(pGpiox == GPIOF){

			GPIOF_PCLK_DI();
		}
		else if(pGpiox == GPIOG){

			GPIOG_PCLK_DI();
		}
		else if(pGpiox == GPIOH){

			GPIOH_PCLK_DI();
		}
		else if(pGpiox == GPIOI){

			GPIOI_PCLK_DI();
		}
	}


}


/**
 * 	@fn			- GPIO_Init
 *
 * 	@brief		- Initialiases the gpio pin mode
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */


/*
 * Gpio initialize and deinitialize
 */
void GPIO_Init(Gpio_Handle_t *pGpioHandle){

	uint32_t temp;// temp register

	//1. configure the gpio mode
	if(pGpioHandle->GPIO_Pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG){

		//Non-Interrupt Mode
		/*
		 * 	why 2 multiplies with pin number?
		 *  the mode register have 2 bits for each pin. so multiplies into 2 achieves, whatever you pin choosenS
		 */

		temp = (pGpioHandle->GPIO_Pinconfig.GPIO_PinMode << (2 * pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber));/* set the mode of the pin selected,*/
		pGpioHandle->pGpiox->MODER &= ~(0x3  << pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber );
		pGpioHandle->pGpiox->MODER |= temp;


	}
	else{
		//interrupt mode
		if(pGpioHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			//1. configure the FTSR
			EXTI->EXTI_FTSR |= (1 << pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber );
			//clear the RTSR
			EXTI->EXTI_RTSR &= ~(1 << pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber );
		}
		else if (pGpioHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RT){

		}
		else if(pGpioHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_IT_RF){

		}

	}

	temp=0;
	//2, cofigure the gpio speed
	temp=(pGpioHandle->GPIO_Pinconfig.GPIO_PinSpeed << (2 * pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGpioHandle->pGpiox->MODER &= ~(0x3  << pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber);
	pGpioHandle->pGpiox->OSPEEDR |= temp;

	temp=0;
	//3. configure the pupd settings
	temp = (pGpioHandle->GPIO_Pinconfig.GPIO_PinPupdControl << (2 * pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGpioHandle->pGpiox->MODER &= ~(0x3  << pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber);
	pGpioHandle->pGpiox->PUPDR |= temp;

	temp=0;
	//3. configure the output type
	temp = (pGpioHandle->GPIO_Pinconfig.GPIO_PinOPType << (pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGpioHandle->pGpiox->MODER &= ~(0x1  << pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber);
	pGpioHandle->pGpiox->OTYPER |= temp;

	temp=0;
	//4, configure the alternate function.
	if(pGpioHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_ALT){
		//configure the alt function

		uint8_t temp1, temp2;
		temp1 = (pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber / 8);
		temp2 = (pGpioHandle->GPIO_Pinconfig.GPIO_PinNumber % 8);
		pGpioHandle->pGpiox->AFR[temp1] &= ~((0xF) << (4 * temp2));
		pGpioHandle->pGpiox->AFR[temp1] |=(pGpioHandle->GPIO_Pinconfig.GPIO_PinAltFuncMode << (4 * temp2));


	}

}

/**
 * 	@fn
 *
 * 	@brief
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */

void GPIO_Deinit(GPIO_RegDef_t *pGpiox)
{
	if(pGpiox == GPIOA){

		GPIOA_REG_RESET();
	}
	else if(pGpiox == GPIOB){

		GPIOB_REG_RESET();
	}
	else if(pGpiox == GPIOC){

		GPIOC_REG_RESET();
	}
	else if(pGpiox == GPIOD){

		GPIOD_REG_RESET();
	}
	else if(pGpiox == GPIOE){

		GPIOE_REG_RESET();
	}
	else if(pGpiox == GPIOF){

		GPIOF_REG_RESET();
	}
	else if(pGpiox == GPIOG){

		GPIOG_REG_RESET();
	}
	else if(pGpiox == GPIOH){

		GPIOH_REG_RESET();
	}
	else if(pGpiox == GPIOI){

		GPIOI_REG_RESET();
	}


}
/*
 * Data read and write
 */
/**
 * 	@fn
 *
 * 	@brief
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGpiox, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGpiox->IDR >> PinNumber) & 0x00000001); // right shifting the input data register to least significant bit acording to the pin number and doing AND(&) operation to get the value of pinnumber value.

	return value;
}
/**
 * 	@fn
 *
 * 	@brief
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGpiox){

	uint16_t value;

	value = (uint16_t)(pGpiox->IDR); // return entire port value.

	return value;

}
/**
 * 	@fn
 *
 * 	@brief
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */
void GPIO_WriteTOOutputPin(GPIO_RegDef_t *pGpiox,uint8_t PinNumber, uint8_t Value ){


	if(Value ==  GPIO_SET)
	{
		pGpiox->ODR  |= (1 << PinNumber);
	}
	else{
		pGpiox->ODR  &=  ~(1 << PinNumber);
	}
}
/**
 * 	@fn
 *
 * 	@brief
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */
void GPIO_WriteToOutpitPort(GPIO_RegDef_t *pGpiox,uint16_t Value ){

	pGpiox->ODR = Value;
}
/**
 * 	@fn
 *
 * 	@brief
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGpiox, uint8_t PinNumber){


	pGpiox->ODR ^= (1 << PinNumber);

}
/*
 * Irq configuration and ISR handling
 */
/**
 * 	@fn
 *
 * 	@brief
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */
void IRQ_Config(uint8_t IRQNumber,uint8_t IRQPriority ,uint8_t EnorDi)
{


}
/**
 * 	@fn
 *
 * 	@brief
 *
 *
 *  @param[1]
 *
 *
 *  @param[2]
 *
 *  @return .
 */
void IRQ_Handling(uint8_t PinNumber){



}

