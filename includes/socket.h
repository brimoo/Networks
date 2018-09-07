#ifndef SOCKET_H
#define SOCKET_H
#define BUFFER_SIZE 64

#define CLOSED      0
#define SYN_SENT    1
#define SYN_RCVD    2
#define ESTABLISHED 3
#define FIN_WAIT1   4
#define FIN_WAIT2   5
#define TIME_WAIT   6
#define CLOSE_WAIT  7
#define LAST_ACK    8
#define LISTEN      9

#include "protocol.h"
#include "channels.h"

typedef nx_struct socket_addr_t{
    nx_uint16_t location;
    nx_uint8_t port;
} socket_addr_t;

typedef nx_struct socket_t{
    socket_addr_t dest;
    socket_addr_t src;
    nx_uint16_t transfer;
    nx_uint16_t seq;
    nx_uint8_t CONN;
    nx_uint8_t advertisedWindow;
    nx_uint16_t nextExp;
    nx_uint16_t lastRCVD;
    nx_uint16_t lastACKED;
    nx_uint8_t sendBuffer[BUFFER_SIZE];
    nx_uint16_t rcvdBuffer[BUFFER_SIZE];
    nx_uint16_t sendBuffCounter;
} socket_t;

#endif
