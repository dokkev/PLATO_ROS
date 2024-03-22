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

static constexpr double MAX_CURRENT = 0.8;
static constexpr double ZERO_CURRENT = 0.0001; 

class PlatoSocketCAN{
public:
    explicit PlatoSocketCAN();
    ~PlatoSocketCAN() = default;

    void init();

    void send_can_tx_msg();
    void receive_can_rx_msg(std::vector<double>& motor_position_states,
                            std::vector<bool>& can_error);
    
    void set_can_tx_msg(std::vector<double>& msg);

    void begin_send_thread();
    void stop_send_thread(); 


private:
    void setup_can_ids();
    
    // SocketCAN
    int socket_;
    struct ifreq ifr_;
    struct sockaddr_can addr_;
    const std::string CAN_CHANNEL;

    int send_timer;

    // Multithreading
    std::thread send_thread;
    bool send_thread_active = false;
    std::mutex mtx;
    std::condition_variable cv;
    

    std::vector<int> can_tx_id_;
    std::vector<int> can_rx_id_;


    std::vector<double> can_rx_msg_;
    std::vector<double> can_tx_msg_;


    void write_can(int socket, int id, double data);
    std::optional<std::tuple<int, double, bool, bool>> read_can(int socket);



    struct can_frame frame_;



    

};

} // namespace can_comms
#endif  // PLATO_HARDWARE_INTERFACE__CAN_COMMS_HPP_