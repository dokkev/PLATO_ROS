#ifndef MAIN_SOCKET_CAN_H   
#define MAIN_SOCKET_CAN_H


#include <iostream>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include <vector>
#include <fcntl.h>


namespace plato{

class SocketCAN
{
    public:

        explicit SocketCAN();
        ~SocketCAN() = default;

        void init();

        void pack_can_msg();
        void send_can_msg();

        void receive_can_msg();
        void unpack_can_msg();

    private:

        int socket_;
        struct ifreq ifr_;
        struct sockaddr_can addr_;
        const std::string CAN_CHANNEL;

        int scale_factor = 10000;
    









};



}


#endif // MAIN_SOCKET_CAN_H