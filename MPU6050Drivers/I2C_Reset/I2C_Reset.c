#include "I2C_Reset.h"


/**
  * @brief  Performs a software reset for the I2C peripheral.
  * @param  I2C_Handle Pointer to the I2C handle structure.
  * @retval None
  * @note   This function sets and clears the SWRST bit in the CR1 register of the I2C peripheral.
  */

static void I2C_SoftwareReset(I2C_HandleTypeDef *I2C_Handle) {
	I2C_Handle->Instance->CR1 |= 0x8000;
	HAL_Delay(4);
	I2C_Handle->Instance->CR1 &= ~0x8000;
}


/**
  * @brief  Resets the I2C peripheral and clears any stuck conditions on the I2C bus.
  * @param  I2C_Handle Pointer to the I2C handle structure.
  * @retval None
  * @note   This function disables the I2C peripheral, configures the GPIO pins as outputs to manually toggle the SCL line,
  *         and performs a software reset to reinitialize the I2C peripheral.
  */

void I2C_ResetI2C(I2C_HandleTypeDef *I2C_Handle) {

	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_I2C_DISABLE(I2C_Handle);

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	for (int i = 0; i < 10; i++) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_Delay(1);
	}

	I2C_SoftwareReset(I2C_Handle);
}
