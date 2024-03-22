#include "plato_socket_can.hpp"


namespace plato{

    plato::SocketCAN::SocketCAN() : CAN_CHANNEL("can0")
    {
        init();
    }

    plato::SocketCAN::init(){

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    // Specify the CAN interface
    std::strcpy(ifr_.ifr_name, "can0");
    ioctl(socket_, SIOCGIFINDEX, &ifr_);
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    // Non-blocking
    // int flags = fcntl(socket_, F_GETFL, 0);
    // fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

    bind(socket_, (struct sockaddr *)&addr_, sizeof(addr_));


    }
}