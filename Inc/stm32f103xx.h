/*
 * stm32f103xx.h
 *
 *  Created on: Nov 23, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include <stdint.h>
#include <string.h>


 /* ARM Cortex Mx Processor NVIC ISERx register Addresses */

#define HAL_NVIC_ISER0          ( (volatile uint32_t*)0xE000E100 )
#define HAL_NVIC_ISER1          ( (volatile uint32_t*)0xE000E104 )
#define HAL_NVIC_ISER2          ( (volatile uint32_t*)0xE000E108 )
#define HAL_NVIC_ISER3          ( (volatile uint32_t*)0xE000E10C )

#define HAL_NVIC_ICER0          ( (volatile uint32_t*)0xE000E180 )
#define HAL_NVIC_ICER1          ( (volatile uint32_t*)0xE000E184 )
#define HAL_NVIC_ICER2          ( (volatile uint32_t*)0xE000E188 )
#define HAL_NVIC_ICER3          ( (volatile uint32_t*)0xE000E18C )

#define HAL_NVIC_PR_BASE_ADDR 	( (volatile uint32_t*)0xE000E400 )

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/* base address of Flash and SRAM */
#define HAL_FLASH_BASEADDR					(0x08000000U)
#define HAL_SRAM_BASEADDR					(0x20000000U)
#define HAL_ROM_BASEADDR					(0x1FFFF000U)

/* base address of AHB and APB */
#define HAL_PERIPH_BASEADDR					(0x40000000U)
#define HAL_APB1_BASEADDR					(HAL_PERIPH_BASEADDR)
#define HAL_APB2_BASEADDR					(0x40010000U)
#define HAL_AHB_BASEADDR					(0x40018000U)

/* base address of peripheral in APB2 bus (72MHz) */

#define HAL_AFIO_BASEADDR					(HAL_APB2_BASEADDR + 0x0000)
#define HAL_EXTI_BASEADDR					(HAL_APB2_BASEADDR + 0x0400)

#define HAL_GPIOA_BASEADDR					(HAL_APB2_BASEADDR + 0x0800)
#define HAL_GPIOB_BASEADDR					(HAL_APB2_BASEADDR + 0x0C00)
#define HAL_GPIOC_BASEADDR					(HAL_APB2_BASEADDR + 0x1000)
#define HAL_GPIOD_BASEADDR					(HAL_APB2_BASEADDR + 0x1400)
#define HAL_GPIOE_BASEADDR					(HAL_APB2_BASEADDR + 0x1800)
#define HAL_GPIOF_BASEADDR					(HAL_APB2_BASEADDR + 0x1C00)
#define HAL_GPIOG_BASEADDR					(HAL_APB2_BASEADDR + 0x2000)


#define HAL_ADC1_BASEADDR					(HAL_APB2_BASEADDR + 0x2400)
#define HAL_ADC2_BASEADDR					(HAL_APB2_BASEADDR + 0x2800)
#define HAL_ADC3_BASEADDR					(HAL_APB2_BASEADDR + 0x3C00)


#define HAL_SPI1_BASEADDR					(HAL_APB2_BASEADDR + 0x3000)


#define HAL_USART1_BASEADDR					(HAL_APB2_BASEADDR + 0x3800)


/* base address of peripheral in APB1 bus (36MHz) */

#define HAL_SPI2_BASEADDR					(HAL_APB1_BASEADDR + 0x3800)
#define HAL_SPI3_BASEADDR					(HAL_APB1_BASEADDR + 0x3C00)


#define HAL_USART2_BASEADDR					(HAL_APB1_BASEADDR + 0x4400)
#define HAL_USART3_BASEADDR					(HAL_APB1_BASEADDR + 0x4800)
#define HAL_UART4_BASEADDR					(HAL_APB1_BASEADDR + 0x4C00)
#define HAL_UART5_BASEADDR					(HAL_APB1_BASEADDR + 0x5000)



/* base address of peripheral in AHB bus */
#define HAL_RCC_BASEADDR					(0x40021000)


/************************************************PERIPHERAL REGISTER DEFINITION STRUCTURE************************************************/

typedef struct
{
	volatile uint32_t CR[2];
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t BRR;
	volatile uint32_t LCKR;
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
}RCC_RegDef_t;

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;

typedef struct
{
	volatile uint32_t EVCR;
	volatile uint32_t MAPR;
	volatile uint32_t EXTICR[4];
	volatile uint32_t MAPR2;
}AFIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SSPR;
}SPI_RegDef_t;

typedef struct
{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_RegDef_t;


/*
 * GPIO_port_def reference
 */
#define GPIOA 								( (GPIO_RegDef_t*) HAL_GPIOA_BASEADDR )
#define GPIOB 								( (GPIO_RegDef_t*) HAL_GPIOB_BASEADDR )
#define GPIOC 								( (GPIO_RegDef_t*) HAL_GPIOC_BASEADDR )
#define GPIOD 								( (GPIO_RegDef_t*) HAL_GPIOD_BASEADDR )
#define GPIOE 								( (GPIO_RegDef_t*) HAL_GPIOE_BASEADDR )
#define GPIOF 								( (GPIO_RegDef_t*) HAL_GPIOF_BASEADDR )
#define GPIOG 								( (GPIO_RegDef_t*) HAL_GPIOG_BASEADDR )

#define RCC									( (RCC_RegDef_t*) HAL_RCC_BASEADDR)

#define EXTI								( (EXTI_RegDef_t*) HAL_EXTI_BASEADDR)

#define AFIO								( (AFIO_RegDef_t*) HAL_AFIO_BASEADDR)

#define USART1								( (USART_RegDef_t*) HAL_USART1_BASEADDR)
#define USART2								( (USART_RegDef_t*) HAL_USART2_BASEADDR)
#define USART3								( (USART_RegDef_t*) HAL_USART3_BASEADDR)
#define UART4								( (USART_RegDef_t*) HAL_UART4_BASEADDR)
#define UART5								( (USART_RegDef_t*) HAL_UART5_BASEADDR)

/*Clock Enable Macro for GPIOx peripheral*/

#define GPIOA_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 2) )
#define GPIOB_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 3) )
#define GPIOC_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 4) )
#define GPIOD_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 5) )
#define GPIOE_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 6) )

/*Clock Enable Macro for AFIO and EXTI (somehow it's stupid)*/
#define AFIO_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 0) )

/*Clock Enable Macro for I2Cx peripheral*/

#define I2C1_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 21) )
#define I2C2_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 22) )

/*Clock Enable Macro for SPIx peripheral*/
#define SPI1_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 12) )
#define SPI2_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 14) )
#define SPI3_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 15) )

/*Clock Enable Macro for UART peripheral*/
#define USART1_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 14) )
#define USART2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 17) )
#define USART3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 18) )
#define UART4_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 19) )
#define UART5_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 20) )

/*Clock Enable Macro for ADC, DAC peripheral*/
#define DAC_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 29) )
#define ADC1_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 9) )
#define ADC2_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 10) )

/*Clock Enable Macro for SYSCFG peripheral*/
/******************************************************************************/

/*Clock Disable Macro for GPIOx peripheral*/

#define GPIOA_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 2) )
#define GPIOB_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 3) )
#define GPIOC_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 4) )
#define GPIOD_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 5) )
#define GPIOE_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 6) )

/*Clock Disable Macro for I2Cx peripheral*/

#define I2C1_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 21) )
#define I2C2_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 22) )

/*Clock Disable Macro for SPIx peripheral*/

#define SPI1_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 12) )
#define SPI2_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 14) )
#define SPI3_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 15) )

/*Clock Disable Macro for UART peripheral*/

#define USART1_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 14) )
#define USART2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 17) )
#define USART3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 18) )
#define UART4_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 19) )
#define UART5_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 20) )

/*Clock Disable Macro for ADC, DAC peripheral*/
#define DAC_PCLK_DI()							( RCC->APB1ENR &= ~( 1 << 29) )
#define ADC1_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 9) )
#define ADC2_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 10) )
/*Clock Disable Macro for SYSCFG peripheral*/

/******************************************************************************/

/*Reset GPIOx peripheral*/

#define GPIOA_REG_RESET()						do{ (RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(uint32_t)(1 << 2)); }while(0)
#define GPIOB_REG_RESET()						do{ (RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(uint32_t)(1 << 3)); }while(0)
#define GPIOC_REG_RESET()						do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(uint32_t)(1 << 4)); }while(0)
#define GPIOD_REG_RESET()						do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~((uint32_t)1 << 5)); }while(0)
#define GPIOE_REG_RESET()						do{ (RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~((uint32_t)1 << 6)); }while(0)

/*Reset I2Cx peripheral*/

/*Reset SPIx peripheral*/

/*Reset for UART peripheral*/

/*Reset for SYSCFG peripheral*/

/*
 * Macro to return a number (0 - 7) according to GPIOx
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:0 )
/*
 * IRQ(Interrupt Request) Numbers of STM32F103 MCU. IRQNumber_def
 */

#define IRQ_NO_EXTI0 		6U
#define IRQ_NO_EXTI1 		7U
#define IRQ_NO_EXTI2 		8U
#define IRQ_NO_EXTI3 		9U
#define IRQ_NO_EXTI4 		10U
#define IRQ_NO_EXTI9_5 		23U
#define IRQ_NO_EXTI15_10 	40U

#define IRQ_NO_I2C1_EV     	31U
#define IRQ_NO_I2C1_ER     	32U
#define IRQ_NO_I2C2_EV     	33U
#define IRQ_NO_I2C2_ER     	34U

#define IRQ_NO_SPI1			35U
#define IRQ_NO_SPI2         36U

#define IRQ_NO_USART1	    37U
#define IRQ_NO_USART2	    38U
#define IRQ_NO_USART3	    39U
#define IRQ_NO_UART4	    52U
#define IRQ_NO_UART5	    53U



/*
 * Priority macros IRQPrority_def
 */
#define NVIC_IRQ_PRI0    	0U
#define NVIC_IRQ_PRI1    	1U
#define NVIC_IRQ_PRI2    	2U
#define NVIC_IRQ_PRI3    	3U
#define NVIC_IRQ_PRI4    	4U
#define NVIC_IRQ_PRI5    	5U
#define NVIC_IRQ_PRI6    	6U
#define NVIC_IRQ_PRI7    	7U
#define NVIC_IRQ_PRI8    	8U
#define NVIC_IRQ_PRI9    	9U
#define NVIC_IRQ_PRI10    	10U
#define NVIC_IRQ_PRI11    	11U
#define NVIC_IRQ_PRT12    	12U
#define NVIC_IRQ_PRI13    	13U
#define NVIC_IRQ_PRI14    	14U
#define NVIC_IRQ_PRI15    	15U


/*
 * General definition General_def
 */
#define ENABLE								(1)
#define DISABLE								(0)
#define SET 								(ENABLE)
#define RESET 								(DISABLE)
#define GPIO_PIN_SET        				(SET)
#define GPIO_PIN_RESET      				(RESET)
#define NONE								(0)
/*
 * Bit position definitions of USART peripheral
 */
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_CLKEN					11
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include "stm32f103xx_gpio_driver.h"
//#include "stm32f103xx_uart_driver.h"

#endif /* INC_STM32F103XX_H_ */
