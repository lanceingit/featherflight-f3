#pragma once

#include "mavlink.h"
#include "serial.h"


class LINK_MAVLINK
{
public:
    //LINK_MAVLINK();

    void init();
    void msg_send(mavlink_message_t &msg);
    bool recv(mavlink_message_t &msg);


private:
    #define RX_BUF_SIZE 300    
    #define TX_BUF_SIZE 300    

    uint8_t sendbuf[300];
    serialPort_t * _port;
    uint8_t _rxBuf[RX_BUF_SIZE]; 
    uint8_t _txBuf[TX_BUF_SIZE];

    mavlink_status_t _r_mavlink_status;
    mavlink_message_t msg;

    void handle_log_request_list(mavlink_message_t *msg);
};
