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
#include <chrono>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>


#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "rclcpp/rclcpp.hpp"

#include "plato_hardware_interface/plato_common.hpp"


namespace plato_socket_can{




class PlatoSocketCAN{
public:
    explicit PlatoSocketCAN();
    ~PlatoSocketCAN() = default;

    void init();

    void receive_can(std::vector<double>& motor_position_states);
    void send_can(std::vector<double>& motor_effort_commands);

    void send_can_zero_effort();
                       

private:
    void setup_can_ids();
    
    // SocketCAN
    int socket_;
    struct ifreq ifr_;
    struct sockaddr_can addr_;
    const std::string CAN_CHANNEL;

    // CAN Message Packing/Unpacking Scale Factors
    double encoder_scale_factor_;
    double command_scale_factor_;
    double encoder_scale_offset_;
    double command_scale_offset_;

    const std::vector<long int> can_tx_id_list_;
    const std::vector<long int> can_rx_id_list_;

    std::vector<double> can_rx_msg_;
    std::vector<double> can_tx_msg_;

    void write_can(int socket, int id, int8_t data);

    void debug_can(can_frame frame);


    struct can_frame frame_;



    

};

} // namespace can_comms
#endif  // PLATO_HARDWARE_INTERFACE__CAN_COMMS_HPP_