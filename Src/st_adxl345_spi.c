#include <stdint.h>
#include "st_adxl345_spi.h"
#include "main.h"

#define hspi hspi3

extern SPI_HandleTypeDef hspi3;

/*
 * @brief Writes a Byte in a ADXL345's Register
 * @param ADXL345 Register, Byte to write
 * @retval adxl345_spi_status_t
 */
adxl345_spi_status_t ADXL345_Write(uint8_t Register, uint8_t Byte) {

	adxl345_spi_status_t exit_code= ADXL345_SPI_OK;

	HAL_GPIO_WritePin(CS_ADXL345_GPIO_Port, CS_ADXL345_Pin, GPIO_PIN_RESET);

	if( HAL_SPI_Transmit(&hspi, (uint8_t *)&Register, 1, ADXL345_TIMEOUT) != HAL_OK)	exit_code= ADXL345_SPI_ERROR;
	if( HAL_SPI_Transmit(&hspi, (uint8_t *)&Byte, 1, ADXL345_TIMEOUT) != HAL_OK) 		exit_code= ADXL345_SPI_ERROR;

	HAL_GPIO_WritePin(CS_ADXL345_GPIO_Port, CS_ADXL345_Pin, GPIO_PIN_SET);

	return exit_code;
}

/*
 * @brief Reads a single Byte from a ADXL345's Register
 * @param ADXL345 Register, pointer to a byte
 * @retval adxl345_spi_status_t
 */
adxl345_spi_status_t ADXL345_Single_Read(uint8_t Register, uint8_t* Buffer){

	adxl345_spi_status_t exit_code= ADXL345_SPI_OK;
	uint8_t REG = (SINGLE_READ|Register);

	HAL_GPIO_WritePin(CS_ADXL345_GPIO_Port, CS_ADXL345_Pin, GPIO_PIN_RESET);

	if( HAL_SPI_Transmit(&hspi, (uint8_t *)&REG, 1, ADXL345_TIMEOUT) != HAL_OK)		exit_code= ADXL345_SPI_ERROR;
	if(	HAL_SPI_Receive(&hspi, Buffer, 1, ADXL345_TIMEOUT) != HAL_OK) 				exit_code= ADXL345_SPI_ERROR;

	HAL_GPIO_WritePin(CS_ADXL345_GPIO_Port, CS_ADXL345_Pin, GPIO_PIN_SET);

	return exit_code;
}

/*
 * @brief Reads multiple Bytes from a ADXL345's Register (auto incrementing register value)
 * @param ADXL345 Register, pointer to a byte (array), Number of bytes to read
 * @retval adxl345_spi_status_t
 */
adxl345_spi_status_t ADXL345_Multiple_Read(uint8_t Register, uint8_t* Buffer, int num_bytes){

	adxl345_spi_status_t exit_code= ADXL345_SPI_OK;
	uint8_t REG = (MULTIPLE_READ|Register);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

	if( HAL_SPI_Transmit(&hspi, (uint8_t *)&REG, 1, ADXL345_TIMEOUT) != HAL_OK) 	exit_code= ADXL345_SPI_ERROR;
	if(	HAL_SPI_Receive(&hspi, Buffer, num_bytes, ADXL345_TIMEOUT) != HAL_OK)		exit_code= ADXL345_SPI_ERROR;

	HAL_GPIO_WritePin(CS_ADXL345_GPIO_Port, CS_ADXL345_Pin, GPIO_PIN_SET);

	return exit_code;
}
