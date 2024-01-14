/*
 * stm32f103xx_gpio.c
 *
 *  Created on: Nov 23, 2023
 *      Author: Admin
 */

#include "stm32f103xx_gpio_driver.h"


/*********************************************************************
 * @fn      		  - GPIO::GPIO
 *
 * @brief             - The constructor of class GPIO
 *
 * @param[in]         - GPIO_RegDef_t *pGPIOxIn is for which GPIO port. Check GPIO_port_def in stm32f103xx.h
 * @param[in]         -	uint8_t Pin for Pin number. Check Pin_number_def in stm32f103xx_gpio_driver.h
 * @param[in]         - uint32_t Mode for chosen pin. Check Mod_def in stm32f103xx_gpio_driver.h
 * @param[in]         - uint8_t Speed for Output speed. Check Speed_def in stm32f103xx_gpio_driver.h
 * @param[in]         - uint8_t Pull for Pullup and down. Check Pull_def in stm32f103xx_gpio_driver.h
 * @return            -
 *
 * @Note              -

 */
GPIO::GPIO(GPIO_RegDef_t *pGPIOxIn, uint8_t Pin, uint32_t Mode, uint8_t Speed, uint8_t Pull)
{
	GPIO_Handle_t GPIO_InitStructure;
	GPIO_InitStructure.pGPIOx = pGPIOxIn;

	GPIO_PeriClk_Ctrl(pGPIOxIn, ENABLE);

	GPIO_InitStructure.GPIO_PinConfig.GPIO_PinNumber = Pin;
	GPIO_InitStructure.GPIO_PinConfig.GPIO_PinMode = Mode;
	GPIO_InitStructure.GPIO_PinConfig.GPIO_PinSpeed = Speed;
	GPIO_InitStructure.GPIO_PinConfig.GPIO_PinPull = Pull;

	GPIO_Init(&GPIO_InitStructure);
}

/*********************************************************************
 * @fn      		  - GPIO::GPIO_PeriClk_Ctrl
 *
 * @brief             - Enable or Disable a APB clock to the port
 *
 * @param[in]         - GPIO_RegDef_t *pGPIOxIn is for which GPIO port. Check GPIO_port_def in stm32f103xx.h
 * @param[in]         - uint8_t ENorDI is to choose between ENABLE or DISABLE. Check General_def in stm32f103xx.h
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_PeriClk_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else
		{

		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO::GPIO_Init
 *
 * @brief             - Initialize a GPIO pin
 *
 * @param[in]         - GPIO_Handle_t *pGPIOHandle, to control overall GPIO mode... Check GPIO_Handle_t in stm32f103xx_gpio_driver.h
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp;
	uint8_t tmp1;
	uint8_t tmp2;
	tmp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber/8;
	tmp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 8;
	if ( (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_OUTPUT_PP )
	{
		temp = ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed) + (GPIO_CR_CNF_GP_OUTPUT_PP)) << (4 * tmp2) ;
		pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
		pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
	}
	else if ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_OUTPUT_OD)
	{
		temp = ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed) + (GPIO_CR_CNF_GP_OUTPUT_OD)) << (4 * tmp2) ;
		pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
		pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
	}
	else if ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_AF_PP)
	{
		temp = ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed) + (GPIO_CR_CNF_AF_OUTPUT_PP)) << (4 * tmp2) ;
		pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
		pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
	}
	else if ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_AF_OD)
	{
		temp = ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed) + (GPIO_CR_CNF_AF_OUTPUT_OD)) << (4 * tmp2) ;
		pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
		pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
	}
	else if ( ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_INPUT) ||
			((pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_AF_INPUT) )
	{
		if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPull == GPIO_NOPULL)
		{
			temp = GPIO_CR_CNF_INPUT_FLOATING << (4 * tmp2) ;
			pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
			pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPull == GPIO_PULLUP)
		{
			temp = GPIO_CR_CNF_INPUT_PU_PD << (4 * tmp2) ;
			pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
			pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
			pGPIOHandle -> pGPIOx -> BSRR |= (SET << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPull == GPIO_PULLDOWN)
		{
			temp = GPIO_CR_CNF_INPUT_PU_PD << (4 * tmp2) ;
			pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
			pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
			pGPIOHandle -> pGPIOx -> BSRR |= (SET << (16 + pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		}
	}
	else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ANALOG)
	{
		temp = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_ANALOG;
	}
	else if ( ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_IN_IT_FT) ||
			  ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_IN_IT_RT) ||
			  ((pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode) == GPIO_IN_IT_RFT))
	{

		if( pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_IN_IT_FT)
		{
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if( pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_IN_IT_RT)
		{
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if( pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_IN_IT_RFT)
		{
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		uint8_t tmp1_exti = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t tmp2_exti = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		AFIO_PCLK_EN();
		AFIO->EXTICR[tmp1_exti] = portcode << (tmp2_exti * 4);

		EXTI->IMR |= 1 << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber;

		if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPull == GPIO_NOPULL)
		{
			temp = GPIO_CR_CNF_INPUT_FLOATING << (4 * tmp2) ;
			pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
			pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPull == GPIO_PULLUP)
		{
			temp = GPIO_CR_CNF_INPUT_PU_PD << (4 * tmp2) ;
			pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
			pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
			pGPIOHandle -> pGPIOx -> BSRR |= (SET << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinPull == GPIO_PULLDOWN)
		{
			temp = GPIO_CR_CNF_INPUT_PU_PD << (4 * tmp2) ;
			pGPIOHandle -> pGPIOx -> CR[tmp1] &= ~((0xF) << (4 * tmp2));
			pGPIOHandle -> pGPIOx -> CR[tmp1] |= temp;
			pGPIOHandle -> pGPIOx -> BSRR |= (SET << (16 + pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber));
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO::GPIO_DeInit
 *
 * @brief             - De-initialize a GPIO pin
 *
 * @param[in]         - GPIO_Handle_t *pGPIOHandle, to control overall GPIO mode... Check GPIO_Handle_t in stm32f103xx_gpio_driver.h
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA) GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB) GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC) GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD) GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE) GPIOE_REG_RESET();
	else
	{

	}
}

/*********************************************************************
 * @fn      		  - GPIO::GPIO_ReadFromInputPin
 *
 * @brief             - To read data from GPIO pin
 *
 * @param[in]         - GPIO_RegDef_t *pGPIOxIn is for which GPIO port. Check GPIO_port_def in stm32f103xx.h
 * @param[in]         -	uint8_t Pin for Pin number. Check Pin_number_def in stm32f103xx_gpio_driver.h
 *
 * @return            - An interger in Hex (value)
 *
 * @Note              -

 */
uint8_t GPIO::GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)( ((pGPIOx -> IDR) >> PinNumber) & 0x00000001 );
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO::GPIO_ReadFromInputPort
 *
 * @brief             - To read data from GPIO port
 *
 * @param[in]         - GPIO_RegDef_t *pGPIOxIn is for which GPIO port. Check GPIO_port_def in stm32f103xx.h
 *
 *
 * @return            - An interger in Hex (value)
 *
 * @Note              -

 */
uint16_t GPIO::GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx -> IDR);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO::GPIO_WriteToOutputPin
 *
 * @brief             - To write to GPIO pin
 *
 * @param[in]         - GPIO_RegDef_t *pGPIOxIn is for which GPIO port. Check GPIO_port_def in stm32f103xx.h
 * @param[in]         - uint8_t Value: 0 or 1
 * @param[in]         -	uint8_t Pin for Pin number. Check Pin_number_def in stm32f103xx_gpio_driver.h
 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx -> ODR |= ( 1 << PinNumber );
	}else
	{
		pGPIOx -> ODR &= ~( 1 << PinNumber );
	}
}

/*********************************************************************
 * @fn      		  - GPIO::GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         - GPIO_RegDef_t *pGPIOxIn is for which GPIO port. Check GPIO_port_def in stm32f103xx.h
 * @param[in]         - uint16_t Value: interger in form of Hex
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx -> ODR = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - Toggle o GPIO pin SET or RESET
 *
 * @param[in]         - GPIO_RegDef_t *pGPIOxIn is for which GPIO port. Check GPIO_port_def in stm32f103xx.h
 * @param[in]         -	uint8_t Pin for Pin number. Check Pin_number_def in stm32f103xx_gpio_driver.h
 * @param[in]         - uint16_t Value: interger in form of Hex

 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR  ^= ( 1 << PinNumber);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         - uint8_t IRQNumber to choose IRQ number accordingly, check IRQNumber_def in stm32f103xx.h
 * @param[in]         - uint8_t ENorDI. ENABLE or DISABLE

 *
 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI)
{
	if (ENorDI == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			*HAL_NVIC_ISER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber <= 64)
		{
			*HAL_NVIC_ISER1 |= ( 1 << (IRQNumber % 32));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			*HAL_NVIC_ICER0 |= ( 1 << IRQNumber);
		}
		else if (IRQNumber >= 32 && IRQNumber <= 64)
		{
			*HAL_NVIC_ICER1 |= ( 1 << (IRQNumber % 32));
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         - uint8_t IRQNumber to choose IRQ number accordingly, check IRQNumber_def in stm32f103xx.h
 * @param[in]         - uint32_t IRQPriority to choose IRQ priority, check IRQPrority_def in stm32f103xx.h
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_sector = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_sector) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*( HAL_NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -	uint8_t Pin for Pin number. Check Pin_number_def in stm32f103xx_gpio_driver.h
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO::GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & ( 1 << PinNumber))
	{

		EXTI->PR |= ( 1 << PinNumber);
	}

}


