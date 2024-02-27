#include "plato_hardware_interface/plato_socket_can.hpp"
#include <fcntl.h>
#include <iostream>

namespace plato_socket_can{

PlatoSocketCAN::PlatoSocketCAN() : CAN_CHANNEL("can0") {
}




void PlatoSocketCAN::init() {
    // Setup ID mappings for Motor and CAN IDs
    setup_can_ids();

    // Create a socket
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    // Specify the CAN interface
    std::strcpy(ifr_.ifr_name, "can0");
    ioctl(socket_, SIOCGIFINDEX, &ifr_); 
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;

    // Buffer size
    // int rcvbuf_size = 524288; // Example size, adjust based on your needs
    // if (setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
    //     perror("Setting receive buffer size failed");
    //     // Handle error

    // Non-blocking
    // int flags = fcntl(socket_, F_GETFL, 0);
    // fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

    bind(socket_, (struct sockaddr *)&addr_, sizeof(addr_));

    // Timeout
    // struct timeval tv;
    // tv.tv_sec = 0; // 5 second timeout
    // tv.tv_usec = 10; // 0 microseconds
    // if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    //     perror("Setting socket timeout failed");
    //     return; // Early return on failure
    // }


    RCLCPP_INFO(rclcpp::get_logger("PlatoSocketCAN"), "SocketCAN initialized!");
    
    begin_send_thread();

}
    
    
void PlatoSocketCAN::setup_can_ids() {
    can_tx_id_ = {MOTOR_0_CAN_TX_ID, MOTOR_1_CAN_TX_ID, MOTOR_2_CAN_TX_ID, 
                  MOTOR_3_CAN_TX_ID, MOTOR_4_CAN_TX_ID, MOTOR_5_CAN_TX_ID, 
                  MOTOR_6_CAN_TX_ID, MOTOR_7_CAN_TX_ID, MOTOR_8_CAN_TX_ID};

    can_rx_id_ = {MOTOR_0_CAN_RX_ID, MOTOR_1_CAN_RX_ID, MOTOR_2_CAN_RX_ID, 
                  MOTOR_3_CAN_RX_ID, MOTOR_4_CAN_RX_ID, MOTOR_5_CAN_RX_ID, 
                  MOTOR_6_CAN_RX_ID, MOTOR_7_CAN_RX_ID, MOTOR_8_CAN_RX_ID};

}

void PlatoSocketCAN::begin_send_thread() {
    send_thread_active = true;
    send_thread = std::thread(&PlatoSocketCAN::send_can_tx_msg, this);
}

void PlatoSocketCAN::stop_send_thread() {
    send_thread_active = false;
    if (send_thread.joinable()) {
        send_thread.join();
    }
}

void PlatoSocketCAN::set_can_tx_msg(std::vector<double>& msg) {
    std::unique_lock<std::mutex> lock(mtx);
    can_tx_msg_ = msg;
    lock.unlock();
    cv.notify_one();
}

void PlatoSocketCAN::send_can_tx_msg() {
    while (send_thread_active) {
        std::unique_lock<std::mutex> lock(mtx);
        // Wait for either send_thread_active to become false or a notification from another thread
        cv.wait(lock, [this]{
            // This condition ensures that the thread only proceeds if send_thread_active is true
            // or if it's been notified via cv.notify_one()/cv.notify_all().
            return !send_thread_active || !can_tx_msg_.empty();
        });

        if (!send_thread_active) {
            // If send_thread_active has been set to false, exit the loop
            break;
        }

        // Copy shared data to local variable to minimize time spent under lock
        std::vector<double> msg = can_tx_msg_;
        lock.unlock(); // Unlock as soon as the shared data is copied

        // Send logic using local copy of data
        for (size_t i = 0; i < msg.size(); ++i) {
            // Assuming write_can is your method to send CAN messages
            // and can_tx_id_[i] is the CAN ID for the ith motor
            write_can(socket_, can_tx_id_[i], msg[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Optionally, use condition variable to control rate
        // For example, to send messages every 10 milliseconds
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        
    }
}

void PlatoSocketCAN::receive_can_rx_msg(std::vector<double>& motor_position_states,
                                        std::vector<bool> &can_error) {
 
    auto rx_data = read_can(socket_);
    if (!rx_data.has_value()) {
        // RCLCPP_ERROR(rclcpp::get_logger("PlatoSocketCAN"), "Failed to read CAN frame");
        return;
    }

    auto [id, data, b_err, b_rtr] = rx_data.value();


    // TODO: I could change to unordered_map to make it more efficient
    switch (id){
    case MOTOR_0_CAN_RX_ID:
        motor_position_states[0] = data;
        can_error[0] = b_err;
        break;
    case MOTOR_1_CAN_RX_ID:
        motor_position_states[1] = data;
        can_error[1] = b_err;
        break;
    case MOTOR_2_CAN_RX_ID:
        motor_position_states[2] = data;
        break;
    case MOTOR_3_CAN_RX_ID:
        motor_position_states[3] = data;
        can_error[3] = b_err;
        break;
    case MOTOR_4_CAN_RX_ID:
        motor_position_states[4] = data;
        can_error[4] = b_err;
        break;
    case MOTOR_5_CAN_RX_ID:
        motor_position_states[5] = data;
        can_error[5] = b_err;
        break;
    case MOTOR_6_CAN_RX_ID:
        motor_position_states[6] = data;
        can_error[6] = b_err;
        break;
    case MOTOR_7_CAN_RX_ID:
        motor_position_states[7] = data;
        can_error[7] = b_err;
        break;
    case MOTOR_8_CAN_RX_ID:
        motor_position_states[8] = data;
        can_error[8] = b_err;
        break;

    default:   
        break;
    }

}

void PlatoSocketCAN::write_can(int socket, int id, double data) {
    struct can_frame frame;

    // if data is NaN send 0.0
    if (std::isnan(data)) {
        data = ZERO_CURRENT;
        RCLCPP_WARN(rclcpp::get_logger("PlatoSocketCAN::write_can"), "NaN data detected, sending 0.0 Effort instead");    
    }

    // Current Limiting 
    // data = std::clamp(data, -MAX_CURRENT, MAX_CURRENT); //disable for position

    if (almost_zero(data)) {
        data = ZERO_CURRENT;
    }


    frame.can_id = id;
    frame.can_dlc = 8; // Explicitly set to 8 bytes for clarity

    std::memcpy(frame.data, &data, sizeof(double));
    if (write(socket, &frame, sizeof(frame)) != sizeof(frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("PlatoSocketCAN"), "Failed to send CAN frame");
    }
    


    // RCLCPP_INFO(rclcpp::get_logger("PlatoSocketCAN"), "Sent CAN frame with ID: %d and Data: %f", id, data);

}

std::optional<std::tuple<int, double, bool, bool>> PlatoSocketCAN::read_can(int socket) {

        struct can_frame frame;
        int nbytes = read(socket, &frame, sizeof(frame));

        if (nbytes > 0 && frame.can_dlc == 8) {
            // Successfully read a frame with expected DLC
            int id = frame.can_id;
            double data;
            bool b_err = (data || 0x10000000) == 0x10000000;
            bool b_rtr = (data || 0x20000000) == 0x20000000;
            std::memcpy(&data, frame.data, sizeof(double));
            return {{id, data, b_err, b_rtr}}; // Successfully received data within the timeout
        }
   
    // Log timeout or error only if no valid data received
    RCLCPP_ERROR(rclcpp::get_logger("PlatoSocketCAN"), "Timeout or error reading CAN frame, returning std::nullopt");
    
    return std::nullopt; // Indicate failure to read valid data within the timeout

}



}// namespace plato_socket_can