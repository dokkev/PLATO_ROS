#ifndef PLATO_HARDWARE_INTERFACE__PLATO_SOCKET_CAN_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO_SOCKET_CAN_HPP_



#include <iostream>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "rclcpp/rclcpp.hpp"

#include "plato_hardware_interface/plato_common.hpp"




namespace plato_socket_can{

static constexpr double MAX_EFFORT = 1.5;


class PlatoSocketCAN{
public:
    explicit PlatoSocketCAN();

    void init();

    void send_can_tx_msg(const std::vector<double>& motor_effort_commands);
    void receive_can_rx_msg(std::vector<double>& motor_position_states);


private:
    void setup_can_ids();
    
    int socket_;
    struct ifreq ifr_;
    struct sockaddr_can addr_;
    const std::string CAN_CHANNEL;
    

    std::vector<int> can_tx_id_;
    std::vector<int> can_rx_id_;

    std::vector<double> can_rx_msg_;

    void write_can(int socket, int id, double data);
    double read_can(int socket ,int id);


    

};

} // namespace can_comms
#endif  // PLATO_HARDWARE_INTERFACE__CAN_COMMS_HPP_