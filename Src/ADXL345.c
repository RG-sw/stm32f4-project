
#include "main.h"
#include "ADXL345.h"
#include "st_adxl345_spi.h"

/*
 * @brief Initialize ADXL345
 * @param None
 * @retval adxl345_status_t
 */
adxl345_status_t ADXL345_Init(){

	  adxl345_status_t exit_code = ADXL345_OK;

	  if( ADXL345_TapDetection_Setup() != ADXL345_OK)	exit_code= ADXL345_ERROR;

	  if( ADXL345_Write(DATA_FORMAT_REG_ADDR, DATA_FORMAT) != ADXL345_SPI_OK) 	exit_code= ADXL345_ERROR;
	  if( ADXL345_Write(INIT_ADDR, ADXL345_INIT_VAL) != ADXL345_SPI_OK)		exit_code= ADXL345_ERROR;
	  if( ADXL345_Write(INT_ENABLE_REG, INT_TAPS_ENABLE) != ADXL345_SPI_OK) 	exit_code= ADXL345_ERROR;

	  return exit_code;
}

/*
 * @brief Enables ADXL345 tap detection functionality
 * @param None
 * @retval adxl345_status_t
 */
adxl345_status_t ADXL345_TapDetection_Setup(){

	adxl345_status_t exit_code= ADXL345_OK;

	if( ADXL345_Write(TAP_AXES_REG, TAP_Z_ENABLE) != ADXL345_SPI_OK) exit_code= ADXL345_ERROR;
	if( ADXL345_Write(THRESH_TAP_REG, 0x40) != ADXL345_SPI_OK) exit_code= ADXL345_ERROR;
	if( ADXL345_Write(DUR_REG, 0x20) != ADXL345_SPI_OK) exit_code= ADXL345_ERROR;
	if( ADXL345_Write(LATENT_REG, 0x20) != ADXL345_SPI_OK) exit_code= ADXL345_ERROR;
	if( ADXL345_Write(WINDOW_REG, 0x50) != ADXL345_SPI_OK) exit_code= ADXL345_ERROR;
	if( ADXL345_Write(INT_MAP_REG, EVERY_INT_ON_INT1) != ADXL345_SPI_OK) exit_code= ADXL345_ERROR;

	return exit_code;
}

/*
 * @brief Retrieves ADXL345 device identifier
 * @param byte pointer
 * @retval adxl345_status_t
 */
adxl345_status_t Device_ID(uint8_t* id ){

	adxl345_status_t exit_code= ADXL345_OK;

	if( ADXL345_Single_Read(ADXL345_WHO_AM_I, id) != ADXL345_SPI_OK) exit_code= ADXL345_ERROR;

	return exit_code;

}

/*
 * @brief Reads Axis(X,Y,Z) Acceleration
 * @param ADXL345_t pointer to data structure to fill
 * @retval adxl345_status_t
 */
adxl345_status_t ADXL345_Read_Axis(ADXL345_t* Data){

	adxl345_status_t exit_code=ADXL345_OK;
	uint8_t Letture[6];

	if( ADXL345_Multiple_Read(X_L_ADDR, Letture, 6) != ADXL345_SPI_OK) exit_code=ADXL345_ERROR;

	Data->X = (int16_t)((Letture[1]<<8) | Letture[0]);
	Data->Y = (int16_t)((Letture[3]<<8) | Letture[2]);
	Data->Z = (int16_t)((Letture[5]<<8) | Letture[4]);

	return exit_code;
}

/*
 * @brief Reads ADXL345's INT_SOURCE_REGISTER when an interrupt has occurred
 * @param byte pointer
 * @retval adxl345_status_t
 */
adxl345_status_t ADXL345_Taps_Detected(uint8_t *ret){

	adxl345_status_t exit_code=ADXL345_OK;

	if( ADXL345_Single_Read(INT_SOURCE_REG, ret) != ADXL345_SPI_OK) exit_code=ADXL345_ERROR;

	return exit_code;
}
