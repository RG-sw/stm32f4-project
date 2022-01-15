#ifndef __ADXL345_H
#define __ADXL345_H

#include <stdint.h>

/* ADXL345 REGISTERS */
#define ADXL345_WHO_AM_I  		((uint8_t)0x00)
#define INIT_ADDR  				((uint8_t)0x2D)
#define DATA_FORMAT_REG_ADDR  	((uint8_t)0x31)
#define DATA_FORMAT 			((uint8_t)0x08) //2g
#define ADXL345_INIT_VAL 		((uint8_t)0x08)
#define X_L_ADDR	 			((uint8_t)0x32)

/* ADXL345 REGISTER VALUES */
#define TAP_AXES_REG 			((uint8_t)0x2A) //AXIS CONTROL FOR SINGLE/DOUBLE TAP
#define THRESH_TAP_REG 			((uint8_t)0x1D) //TAP THRESHOLD
#define DUR_REG 				((uint8_t)0x21) //TAP DURATION
#define LATENT_REG				((uint8_t)0x22) //TAP LATENCY
#define WINDOW_REG				((uint8_t)0x23) //TAP WINDOW
#define INT_MAP_REG				((uint8_t)0x2F) //INTERRUPT MAPPING CONTROL
#define INT_ENABLE_REG 			((uint8_t)0x2E) //INTERRUPT ENABLE
#define INT_SOURCE_REG 			((uint8_t)0x30) //SOURCE_OF_INTERRUPTS

#define EVERY_INT_ON_INT1 		((uint8_t)0x00)
#define INT_TAPS_ENABLE 		((uint8_t)0x60)
#define TAP_Z_ENABLE 			((uint8_t)0x01)

#define SCALE_FACTOR 			((float)0.0039)

typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z;
} ADXL345_t;

typedef enum
{
	ADXL345_OK       = 0x00U,
	ADXL345_ERROR    = 0x01U
} adxl345_status_t;

adxl345_status_t Device_ID(uint8_t*);
adxl345_status_t ADXL345_Init(void);
adxl345_status_t ADXL345_TapDetection_Setup(void);
adxl345_status_t ADXL345_Read_Axis(ADXL345_t */*float*, float*, float**/);

adxl345_status_t ADXL345_Taps_Detected(uint8_t*);

#endif
