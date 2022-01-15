
#include "eth_utils.h"
#include "main.h"
#include "lwip.h"

/** @brief ETH connection status Function
  * @param None
  * @retval ETH status
  */
int is_ETH_UP(void){
	uint32_t ret = 0;
	int status;

	/* Polls PHY_BSR register to detect ETH Connectivity */
	do{
		HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &ret);

		if((ret & PHY_LINKED_STATUS) == PHY_LINKED_STATUS){
			status=1;
		}
		else{
			status=0;
		}
	}while(ret!=0x782d && ret!=0x7809);
	return status;
}
