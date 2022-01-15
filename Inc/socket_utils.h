#ifndef __SOCKET_UTILS_H
#define __SOCKET_UTILS_H

#include "ADXL345.h"

#define SERVER_ADDR		"192.168.1.11"
#define SERVER_PORT		50000

void init_sock_addr(struct sockaddr_in*);

int start_TCP_socket(struct sockaddr_in*);

int socket_write(int, ADXL345_t);

void close_socket(int);

#endif
