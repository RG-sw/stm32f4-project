#ifndef INC_SD_UTILS_H_
#define INC_SD_UTILS_H_

#include "ADXL345.h"

void StrToNum(char*, ADXL345_t*);

int is_SD_FULL(void);
int is_SD_EMPTY(void);

void set_SD_Status(char, int);

#endif /* INC_SD_UTILS_H_ */
