#include "main.h"
#include "fatfs.h"
#include "sd_utils.h"
#include <errno.h>

/*
 * @brief Function translating a String line into ADXL345 data
 * @param String (containing data), ADXL345 data struct pointer
 * @retval None
 */
void StrToNum(char* buf, ADXL345_t* data) {

    char *p = buf;
    long tmp[3];
    int i = 0;

    while (*p) {
        long ret = strtol(p, &p, 10);

        // add error checking
        if (ret == 0 && errno == EINVAL) {
            //No conversion done
            break;
        }
        if (errno == ERANGE) {
            break;
        }

        tmp[i] = ret;
        i++;
        /* When \n found -> stop */
        if(*p=='\n') break;
    }
    data->X = (int16_t)tmp[0];
    data->Y = (int16_t)tmp[1];
    data->Z = (int16_t)tmp[2];
}

/*
 * @brief SD Global Variables setting Function
 * @param Char indicating global variable to be set(SD_FULL/SD_EMPTY)
 * 		  integer value assigned to a global variable
 * @retval None
 */
void set_SD_Status(char SD_Status, int cmd){

	extern osMutexId_t SD_MutexHandle;

	/* Setting Global Variables in a SAFE way */
	osMutexAcquire(SD_MutexHandle, osWaitForever);

	if(SD_Status=='F'){
		SD_FULL=cmd;
	}
	if(SD_Status=='E'){
		SD_EMPTY=cmd;
	}

	osMutexRelease(SD_MutexHandle);

}

/*
 * @brief Function checking if SD is full
 * @param None
 * @retval SD Full status
 */
int is_SD_FULL(){

	/* This Function checks how much free space is available in SD
	 * and sets the Global Variable SD_FULL to its logical value */

	FATFS *fs;
	DWORD fre_clust, fre_sect;// tot_sect;

	if( f_getfree("", &fre_clust, &fs) == FR_OK ){

		fre_sect = fre_clust * fs->csize;

		if(fre_sect/2 > 128)
			set_SD_Status('F',0); /*Reset SD_FULL global variable*/
		else
			set_SD_Status('F',1); /*Reset SD_FULL global variable*/

	}
	return SD_FULL;
}

/*
 * @brief Function checking if SD is Empty
 * @param None
 * @retval SD Empty status
 */
int is_SD_EMPTY(){
	/*
	FATFS *fs;
	DWORD fre_clust, fre_sect, tot_sect;  //Unsigned 32 bit
	int res=0;
	DWORD TOT_SPACE = 3899388;

	if( f_getfree("", &fre_clust, &fs) == FR_OK ){

		tot_sect = (fs->n_fatent - 2) * fs->csize;
		fre_sect = fre_clust * fs->csize;

		if(fre_sect == TOT_SPACE)
			res= 1;
		else
			res= 0;
	}
	return res;
	*/
	return SD_EMPTY;
}

