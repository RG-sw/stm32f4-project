
#include "lwip.h"
#include "lwip/sockets.h"
#include "socket_utils.h"
#include <stdio.h>

/**
  * @brief Server Address Initialization Function
  * @param Empty struct sockaddr_in*
  * @retval None
  */
void init_sock_addr(struct sockaddr_in* addr){

	/* Setting Server Address(Static) & Port */
	addr->sin_family = AF_INET;
	addr->sin_port = htons(SERVER_PORT);
	addr->sin_addr.s_addr = inet_addr(SERVER_ADDR);
}

/*
 * @brief Socket Creation & Connection Function
 * @param Server Information
 * @retval Socket Descriptor
 */
int start_TCP_socket(struct sockaddr_in* addr){

	int sd;
	/* TCP IPV4 Socket Creation */
	if( (sd=socket(AF_INET, SOCK_STREAM, 0)) < 0 )
		myErrorHandler();

	/* Socket Connection */
	if( connect(sd, (struct sockaddr*)addr, sizeof(*addr))< 0)
		myErrorHandler();

	return sd;
}

/*
 * @brief Function sending Accelerometer data via Socket
 * @param Socket Descriptor, ADXL345 Data
 * @retval Char written on socket
 */
int socket_write(int sd, ADXL345_t Data){

	int ret;
	char buf[64];
	memset(buf, 0 ,sizeof(buf));

	osThreadId_t id; // id of the currently running thread
	extern osThreadId_t AccToEthHandle;

	/* Formatting ADXL345 Data for Socket Transmission..
	 * The message starts with "SD" or "ADXL345" depending on the thread calling this function */

	id = osThreadGetId();
	if(id == AccToEthHandle) {
		sprintf(buf,"ADXL345: X=%.4f Y=%.4f Z=%.4f\n", (float)Data.X*SCALE_FACTOR, (float)Data.Y*SCALE_FACTOR, (float)Data.Z*SCALE_FACTOR);
	}else{
		sprintf(buf,"SD : X=%.4f Y=%.4f Z=%.4f\n", (float)Data.X*SCALE_FACTOR, (float)Data.Y*SCALE_FACTOR, (float)Data.Z*SCALE_FACTOR);
	}

	if( (ret = write(sd, buf, strlen(buf))) < 0)
		myErrorHandler();

	return ret;
}

/*
 * @brief Function closing Socket
 * @param Socket Descriptor
 * @retval None
 */
void close_socket(int sd){
	close(sd);
}
