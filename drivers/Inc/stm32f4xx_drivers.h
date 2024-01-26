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
#define RCC_BASEADDR				0x40023800U
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
/*
 *RCC base address
 */

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


/* RCC register map */

typedef struct{

	_vo uint32_t CR;			/* RCC clock control register, Address offset: 0x00 */
	_vo uint32_t PLLCFGR;		/* RCC PLL configuration register, Address offset: 0x04 */
	_vo uint32_t CFGR;			/* RCC clock configuration register, Address offset: 0x08 */
	_vo uint32_t CIR;			/* RCC clock interrupt register, Address offset: 0x0C */
	_vo uint32_t AHB1RSTR;		/* RCC AHB1 peripheral reset register, Address offset: 0x10 */
	_vo uint32_t AHB2RSTR;		/* RCC AHB2 peripheral reset register, Address offset: 0x14 */
	_vo uint32_t AHB3RSTR;		/* RCC AHB3 peripheral reset register, Address offset: 0x18 */
	uint32_t  	Reserved0;		/* Reserved , Address offset: 0x1C */
	_vo uint32_t APB1RSTR;		/* RCC APB1 peripheral reset register, Address offset: 0x20 */
	_vo uint32_t APB2RSTR;		/* RCC APB2 peripheral reset register, Address offset: 0x24 */
	uint32_t  	Reserved1;		/* Reserved , Address offset: 0x28 */
	uint32_t  	Reserved2;		/* Reserved , Address offset: 0x2C */
	_vo uint32_t AHB1ENR;		/* RCC AHB1 peripheral clock register, Address offset: 0x30 */
	_vo uint32_t AHB2ENR;		/* RCC AHB2 peripheral clock register, Address offset: 0x34 */
	_vo uint32_t AHB3ENR;		/* RCC AHB3 peripheral clock register, Address offset: 0x38 */
	uint32_t  	Reserved3;		/* Reserved , Address offset: 0x3C */
	_vo uint32_t APB1ENR;		/* RCC APB1 peripheral clock register, Address offset: 0x40 */
	_vo uint32_t APB2ENR;		/* RCC APB2 peripheral clock register, Address offset: 0x44 */
	uint32_t  	Reserved4;		/* Reserved , Address offset: 0x48 */
	uint32_t  	Reserved5;		/* Reserved , Address offset: 0x4C */
	_vo uint32_t AHB1LPENR;		/* RCC  AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	_vo uint32_t AHB2LPENR;		/* RCC  AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	_vo uint32_t AHB3LPENR;		/* RCC  AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	uint32_t  	Reserved6;		/* Reserved , Address offset: 0x5C */
	_vo uint32_t APB1LPENR;		/* RCC  APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	_vo uint32_t APB2LPENR;		/* RCC  APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	uint32_t  	Reserved7;		/* Reserved , Address offset: 0x68 */
	uint32_t  	Reserved8;		/* Reserved , Address offset: 0x6C */
	_vo uint32_t BDCR;			/* RCC Backup domain control register, Address offset: 0x70 */
	_vo uint32_t CSR;			/* RCC clock control & status register, Address offset: 0x74 */
	uint32_t  	Reserved9;		/* Reserved , Address offset: 0x78 */
	uint32_t  	Reserved10;		/* Reserved , Address offset: 0x7C */
	_vo uint32_t SSCGR;			/* RCC clock control & status register, Address offset: 0x80 */
	_vo uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register, Address offset: 0x70 */
	_vo uint32_t PLLSAICFGR;	/* RCC  PLL configuration register, Address offset: 0x70 */
	_vo uint32_t DCKCFGR;		/* RCC  Dedicated Clock Configuration Register, Address offset: 0x70 */
}RCC_Refdef_t;



/*
 * Peripheral definition macros (peripheral base address typecasted to xxx_RegDef_t )
 */
//GPIO_RegDef_t *pGpioa = (GPIO_RegDef_t *)GPIOA_BASEADDR;
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)	//GPIOA base addresses dereferenced to GPIOA and typecasted with structure GPIO_Refdef_t
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 		(RCC_Refdef_t*)RCC_BASEADDR			// RCC base address dereferenced to preprocessor RCC and typecasted with structure RCC_Refdef_t

/*
 * Clock enable macros for GPIO.
 */
#define GPIOA_PCLK_EN() RCC->AHB1ENR |=(1<<0)			//Enables the GPIOA clock
#define GPIOB_PCLK_EN() RCC->AHB1ENR |=(1<<1)			//Enables the GPIOB clock
#define GPIOC_PCLK_EN() RCC->AHB1ENR |=(1<<2)			//Enables the GPIOC clock
#define GPIOD_PCLK_EN() RCC->AHB1ENR |=(1<<3)			//Enables the GPIOD clock
#define GPIOE_PCLK_EN() RCC->AHB1ENR |=(1<<4)			//Enables the GPIOE clock
#define GPIOF_PCLK_EN() RCC->AHB1ENR |=(1<<5)			//Enables the GPIOF clock
#define GPIOG_PCLK_EN() RCC->AHB1ENR |=(1<<6)			//Enables the GPIOG clock
#define GPIOH_PCLK_EN() RCC->AHB1ENR |=(1<<7)			//Enables the GPIOH clock
#define GPIOI_PCLK_EN() RCC->AHB1ENR |=(1<<8)			//Enables the GPIOI clock
/*
 * To do all gpio and i2c,spi and uart.
 */

/*
 * Clock enable macros for I2C.
 */
#define I2C1_PCLK_EN() RCC->APB1ENR |=(1<<21) 			//Enables the I2c 1 clock
#define I2C2_PCLK_EN() RCC->APB1ENR |=(1<<22) 			//Enables the I2c 2 clock
#define I2C3_PCLK_EN() RCC->APB1ENR |=(1<<23)			//Enables the I2c 3 clock

/*
 * Clock enable macros for SPI.
 */
#define SPI1_PCLK_EN() RCC->APB2ENR |=(1<<12) 			//Enables the SPI 1 clock
#define SPI2_PCLK_EN() RCC->APB1ENR |=(1<<14) 			//Enables the SPI 2 clock
#define SPI3_PCLK_EN() RCC->APB1ENR |=(1<<15)			//Enables the SPI 3 clock

/*
 * Clock enable macros for UART.
 */
#define USART1_PCLK_EN() RCC->APB2ENR |=(1<<4) 			//Enables the USART 1 clock
#define USART2_PCLK_EN() RCC->APB1ENR |=(1<<17) 		//Enables the USART 2 clock
#define USART3_PCLK_EN() RCC->APB1ENR |=(1<<18) 		//Enables the USART 3 clock
#define UART4_PCLK_EN() RCC->APB1ENR |=(1<<19) 			//Enables the UART 4 clock
#define UART5_PCLK_EN() RCC->APB1ENR |=(1<<20) 			//Enables the UART 5 clock
#define USART6_PCLK_EN() RCC->APB2ENR |=(1<<4) 			//Enables the USART 6 clock
/*
 * Clock Disable macros for GPIO.
 */
#define GPIOA_PCLK_DI() RCC->AHB1ENR |=(0<<0)			//Disable  the GPIOA clock
#define GPIOB_PCLK_DI() RCC->AHB1ENR |=(0<<1)			//Disable the GPIOB clock
#define GPIOC_PCLK_DI() RCC->AHB1ENR |=(0<<2)			//Disable the GPIOC clock
#define GPIOD_PCLK_DI() RCC->AHB1ENR |=(0<<3)			//Disable the GPIOD clock
#define GPIOE_PCLK_DI() RCC->AHB1ENR |=(0<<4)			//Disable the GPIOE clock
#define GPIOF_PCLK_DI() RCC->AHB1ENR |=(0<<5)			//Disable the GPIOF clock
#define GPIOG_PCLK_DI() RCC->AHB1ENR |=(0<<6)			//Disable the GPIOG clock
#define GPIOH_PCLK_DI() RCC->AHB1ENR |=(0<<7)			//Disable the GPIOH clock
#define GPIOI_PCLK_DI() RCC->AHB1ENR |=(0<<8)			//Disable the GPIOI clock


/*
 * Clock Disable macros for I2C.
 */
#define I2C1_PCLK_DI() RCC->APB1ENR |=(0<<21) 			//Disable the I2c 1 clock
#define I2C2_PCLK_DI() RCC->APB1ENR |=(0<<22) 			//Disable the I2c 2 clock
#define I2C3_PCLK_DI() RCC->APB1ENR |=(0<<23)			//Disable the I2c 3 clock


/*
 * Clock Disable macros for SPI.
 */
#define SPI1_PCLK_DI() RCC->APB2ENR |=(0<<12) 			//Disable the SPI 1 clock
#define SPI2_PCLK_DI() RCC->APB1ENR |=(0<<14) 			//Disable the SPI 2 clock
#define SPI3_PCLK_DI() RCC->APB1ENR |=(0<<15)			//Disable the SPI 3 clock

/*
 * Clock Disable macros for UART.
 */

#define USART1_PCLK_DI() RCC->APB2ENR |=(0<<4) 			//Disable the USART 1 clock
#define USART2_PCLK_DI() RCC->APB1ENR |=(0<<17) 		//Disable the USART 2 clock
#define USART3_PCLK_DI() RCC->APB1ENR |=(0<<18) 		//Disable the USART 3 clock
#define UART4_PCLK_DI() RCC->APB1ENR |=(0<<19) 			//Disable the UART 4 clock
#define UART5_PCLK_DI() RCC->APB1ENR |=(0<<20) 			//Disable the UART 5 clock
#define USART6_PCLK_DI() RCC->APB2ENR |=(0<<4) 			//Disable the USART 6 clock


#endif /* INC_STM32F4XX_DRIVERS_H_ */
