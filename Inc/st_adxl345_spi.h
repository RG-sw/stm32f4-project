#ifndef __ST_ADXL345_SPI_H
#define __ST_ADXL345_SPI_H

typedef enum
{
	ADXL345_SPI_OK       = 0x00U,
	ADXL345_SPI_ERROR    = 0x01U
} adxl345_spi_status_t;

#define SINGLE_READ 			((uint8_t)0x80) //( READ_bits | 0x32); /*0xB2*/
#define MULTIPLE_READ 			((uint8_t)0xC0)

#define ADXL345_TIMEOUT 		1000
#define CS_ADXL345_GPIO_Port 	GPIOE
#define CS_ADXL345_Pin			GPIO_PIN_3

adxl345_spi_status_t ADXL345_Write(uint8_t Register, uint8_t Byte);
adxl345_spi_status_t ADXL345_Single_Read(uint8_t Register, uint8_t* Buffer);
adxl345_spi_status_t ADXL345_Multiple_Read(uint8_t Register, uint8_t* Buffer, int num_bytes);

#endif
