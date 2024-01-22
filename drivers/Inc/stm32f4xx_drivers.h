/*
 * stm32f4xx_drivers.h
 *
 *  Created on: Jan 20, 2024
 *      Author: yadav
 */

#ifndef INC_STM32F4XX_DRIVERS_H_
#define INC_STM32F4XX_DRIVERS_H_
#include <stdio.h>

#define _vo  	volatile

#define FLASH_MEMORY_BASEADDR 		0x08000000U			//flash memory base address
#define SRAM1_BASEADDR 				0x20000000U			//SRAM1 base address
#define SRAM2_BASEADDR				0x2001C000U			//sram2 base address
#define ROM_BASEADDR				0x1FFF0000U			//system memory base address
#define SRAM_BASEADDR				SRAM_BASE_ADDR
/*
 * AHB and APB base addresses
 */

#define PERIPH_BASEADDR				0x40000000U
#define	APB1_BASEADDR				PERIPH_BASE
#define	APB2_BASEADDR				0x40010000u
#define	AHB1_BASEADDR				0x40020000u
#define	AHB2_BASEADDR				0x50000000U

/*
 * GPIO base addresses
 */
#define	GPIOA_BASEADDR				(AHB1_BASE + 0x0000)
#define	GPIOB_BASEADDR				(AHB1_BASE + 0x0400)
#define	GPIOC_BASEADDR				(AHB1_BASE + 0x0800)
#define	GPIOD_BASEADDR				(AHB1_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB1_BASE + 0x1000)
#define GPIOF_BASEADDR				(AHB1_BASE + 0x1400)
#define GPIOG_BASEADDR				(AHB1_BASE + 0x1800)
#define GPIOH_BASEADDR				(AHB1_BASE + 0x1C00)
#define GPIOI_BASEADDR				(AHB1_BASE + 0x2000)

/*
 * APB1 hanging base addressess
 */
#define I2C1_BASEADDR 				(APB1_BASE + 0x5400)
#define I2C2_BASEADDR 				(APB1_BASE + 0x5800)
#define I2C3_BASEADDR 				(APB1_BASE + 0x5C00)
#define SPI2_BASEADDR				(APB1_BASE + 0x3800)
#define SPI3_BASEADDR				(APB1_BASE + 0x3C00)
#define USART2_BASEADDR				(APB1_BASE + 0x4400)
#define USART3_BASEADDR				(APB1_BASE + 0x4800)
#define UART4_BASEADDR				(APB1_BASE + 0x4C00)
#define UART5_BASEADDR				(APB1_BASE + 0x5000)
/*
 * APB2 hanging base addressess
 */

#define USART1_BASEADDR				(APB2_BASE + 0x1000)
#define USART6_BASEADDR				(APB2_BASE + 0x1400)
#define SPI1_BASEADDR				(APB2_BASE + 0x3000)
#define EXTI_BASEADDR				(APB2_BASE + 0x3C00)
#define SYSCFG_BASEADDR				(APB2_BASE + 0x3800)

/*****************************Peripheral register definition structure*************************/
/*
 * Note the registers are specific for MCU
 */



typedef struct {

	_vo uint32_t MODER;			/* GPIO port mode register, Address offset: 0x00 */
	_vo uint32_t OTYPER;		/* GPIO port output type register, Address offset: 0x04 */
	_vo uint32_t OSPEEDR;		/* GPIO port output speed register,  Address offset: 0x08 */
	_vo uint32_t PUPDR;			/* GPIO port pull-up/pull-down register, Address offset: 0x0C */
	_vo uint32_t IDR;			/* GPIO port input data register, Address offset: 0x10 */
	_vo uint32_t ODR;			/* GPIO port output data register, Address offset: 0x14 */
	_vo uint32_t BSRR;			/* GPIO port bit set/reset register, Address offset: 0x18 */
	_vo uint32_t LCKR;			/* GPIO port configuration lock register, Address offset: 0x1C */
	_vo uint32_t AFR[2];		/* AFR[0] GPIO port  alternate function low register,AFR[1] GPIO port  alternate function high register, Address offset: 0x20 & 0x24 */

}GPIO_RegDef_t;


/*To do RCC register map */


/*
 * Peripheral definition macros (peripheral base address typecasted to xxx_RegDef_t )
 */
//GPIO_RegDef_t *pGpioa = (GPIO_RegDef_t *)GPIOA_BASEADDR;
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR)
#endif /* INC_STM32F4XX_DRIVERS_H_ */
