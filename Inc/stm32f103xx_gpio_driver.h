/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Nov 23, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint32_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPull;

}GPIO_PinConfig_t;

typedef struct
{
	//pointer to hole the base address of GPIO peripherals
	GPIO_RegDef_t *pGPIOx;  /* Hold base address of GPIO port which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

class GPIO
{
public:
	GPIO_PinConfig_t GPIO_config;
	GPIO_RegDef_t *pGPIOx;

	/*	The Constructor of the Class	*/
	GPIO(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint32_t Mode, uint8_t Speed, uint8_t Pull);

	/*
	 * Peripheral Clock Setup
	 */
	void GPIO_PeriClk_Ctrl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

	/*
	 * Peripheral Init and De-Init
	 */
	void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
	void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

	/*
	 * Data read and write
	 */
	uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
	uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
	void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
	void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

	void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
	/*
	 * IRQ Config and Handling
	 */


	void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
	void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
	void GPIO_IRQHandling(uint8_t PinNumber);
};


/*
 * Pin_number_def, Pin number define
 */
#define GPIO_PIN_NO_0				(0)
#define GPIO_PIN_NO_1				(1)
#define GPIO_PIN_NO_2				(2)
#define GPIO_PIN_NO_3				(3)
#define GPIO_PIN_NO_4				(4)
#define GPIO_PIN_NO_5				(5)
#define GPIO_PIN_NO_6				(6)
#define GPIO_PIN_NO_7				(7)
#define GPIO_PIN_NO_8				(8)
#define GPIO_PIN_NO_9				(9)
#define GPIO_PIN_NO_10				(10)
#define GPIO_PIN_NO_11				(11)
#define GPIO_PIN_NO_12				(12)
#define GPIO_PIN_NO_13				(13)
#define GPIO_PIN_NO_14				(14)
#define GPIO_PIN_NO_15				(15)

/*
 * Mod_def Mode definition
 */
#define  GPIO_MODE_INPUT                        0x00000000u   /*!< Input Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    0x00000001u   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    0x00000011u   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        0x00000002u   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        0x00000012u   /*!< Alternate Function Open Drain Mode    */
#define  GPIO_MODE_AF_INPUT                     GPIO_MODE_INPUT          /*!< Alternate Function Input Mode         */

#define  GPIO_MODE_ANALOG                       0x00000003u   /*!< Analog Mode  */

#define  GPIO_IN_IT_RT                    		0x10110000u   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_IN_IT_FT                   		0x10210000u   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_IN_IT_RFT            				0x10310000u   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

/*
 * Speed_def definition
 */
#define  GPIO_SPEED_FREQ_MEDIUM              	(0x01U) /*!< Medium speed */
#define  GPIO_SPEED_FREQ_LOW           			(0x02U) /*!< Low speed */
#define  GPIO_SPEED_FREQ_HIGH             		(0x03U)   /*!< High speed */

/* Definitions for bit manipulation of CRL and CRH register */
#define  GPIO_CR_MODE_INPUT         			0x00000000u /*!< 00: Input mode (reset state)  */
#define  GPIO_CR_CNF_ANALOG         			0x00000000u /*!< 00: Analog mode  */
#define  GPIO_CR_CNF_INPUT_FLOATING 			0x00000004u /*!< 01: Floating input (reset state)  */
#define  GPIO_CR_CNF_INPUT_PU_PD    			0x00000008u /*!< 10: Input with pull-up / pull-down  */
#define  GPIO_CR_CNF_GP_OUTPUT_PP   			0x00000000u /*!< 00: General purpose output push-pull  */
#define  GPIO_CR_CNF_GP_OUTPUT_OD   			0x00000004u /*!< 01: General purpose output Open-drain  */
#define  GPIO_CR_CNF_AF_OUTPUT_PP   			0x00000008u /*!< 10: Alternate function output Push-pull  */
#define  GPIO_CR_CNF_AF_OUTPUT_OD   			0x0000000Cu /*!< 11: Alternate function output Open-drain  */

/*Definitions for GPIO_PinPull and Pull_def params */
#define  GPIO_NOPULL        0x00000000u   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        0x00000001u   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      0x00000002u   /*!< Pull-down activation                */

/*************************************************************************************
			This driver API, for more, check the function definition
*************************************************************************************/




#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
