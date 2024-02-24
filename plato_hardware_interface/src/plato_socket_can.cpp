#include "plato_hardware_interface/plato_socket_can.hpp"

#include <iostream>

namespace plato_socket_can{

PlatoSocketCAN::PlatoSocketCAN() : CAN_CHANNEL("can0") {
    
}


void PlatoSocketCAN::init() {
    // Setup ID mappings for Motor and CAN IDs
    setup_can_ids();

    // Create a socket
    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) {
        perror("Socket creation failed");
        return; // Early return on failure
    }

    // Specify the CAN interface
    std::strcpy(ifr_.ifr_name, CAN_CHANNEL.c_str());
    if (ioctl(socket_, SIOCGIFINDEX, &ifr_) < 0) {
        perror("Getting IF index failed");
        close(socket_); // Clean up socket on failure
        return; // Early return on failure
    }

    // Bind the socket to the CAN interface
    std::memset(&addr_, 0, sizeof(addr_));
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(socket_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
        perror("Socket bind failed");
        close(socket_); // Clean up socket on failure
        return; // Early return on failure
    }

    // Timeout
    // struct timeval tv;
    // tv.tv_sec = 0; // 5 second timeout
    // tv.tv_usec = 10; // 0 microseconds
    // if (setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    //     perror("Setting socket timeout failed");
    //     return; // Early return on failure
    // }

    RCLCPP_INFO(rclcpp::get_logger("PlatoSocketCAN"), "SocketCAN initialized!");
}
    
    
void PlatoSocketCAN::setup_can_ids() {
    can_tx_id_ = {MOTOR_0_CAN_TX_ID, MOTOR_1_CAN_TX_ID, MOTOR_2_CAN_TX_ID, 
                  MOTOR_3_CAN_TX_ID, MOTOR_4_CAN_TX_ID, MOTOR_5_CAN_TX_ID, 
                  MOTOR_6_CAN_TX_ID, MOTOR_7_CAN_TX_ID, MOTOR_8_CAN_TX_ID};

    can_rx_id_ = {MOTOR_0_CAN_RX_ID, MOTOR_1_CAN_RX_ID, MOTOR_2_CAN_RX_ID, 
                  MOTOR_3_CAN_RX_ID, MOTOR_4_CAN_RX_ID, MOTOR_5_CAN_RX_ID, 
                  MOTOR_6_CAN_RX_ID, MOTOR_7_CAN_RX_ID, MOTOR_8_CAN_RX_ID};
}

void PlatoSocketCAN::send_can_tx_msg(const std::vector<double>& motor_effort_commands) {
    if (motor_effort_commands.size() > can_tx_id_.size()) {
        // RCLCPP_ERROR(rclcpp::get_logger("PlatoSocketCAN"), "Motor command size exceeds CAN ID size!");
        return;
    }

    for (size_t i = 0; i < motor_effort_commands.size(); ++i) {
        write_can(socket_, can_tx_id_[i], motor_effort_commands[i]);
        #ifdef DEBUG_MODE
            // RCLCPP_INFO(rclcpp::get_logger("PlatoSocketCAN"), "Sent CAN message: %f", motor_effort_commands[i]);
        #endif
    
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

    #ifdef DEBUG_MODE
        // RCLCPP_INFO(rclcpp::get_logger("PlatoSocketCAN::receive_can_rx_msg"), "Received CAN ID %d message: %f", id, data);
    #endif

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
    std::memset(&frame, 0, sizeof(frame)); // Clear the frame

    // if data is NaN send 0.0
    if (std::isnan(data)) {
        data = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("PlatoSocketCAN::write_can"), "NaN data detected, sending 0.0 Effort instead");    
    }

    // Current Limiting 
    data = std::clamp(data, -MAX_CURRENT, MAX_CURRENT);
    // through warning if data is clamped
    if (data != std::clamp(data, -MAX_CURRENT, MAX_CURRENT)) {
        RCLCPP_WARN(rclcpp::get_logger("PlatoSocketCAN::write_can"), "Current clamped to %f", data);
    }


    frame.can_id = id;
    frame.can_dlc = 8; // Explicitly set to 8 bytes for clarity
    std::memcpy(frame.data, &data, sizeof(double));
    if (write(socket, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("CAN Frame write failed");
    }
}

std::optional<std::tuple<int, double, bool, bool>> PlatoSocketCAN::read_can(int socket) {

        struct can_frame frame;
        int nbytes = read(socket, &frame, sizeof(frame));

        if (nbytes > 0 && frame.can_dlc == 8) {
            // Successfully read a frame with expected DLC
            int id = frame.can_id;
            double data;
            bool b_err = data || 0x10000000;
            bool b_rtr = data || 0x20000000;
            std::memcpy(&data, frame.data, sizeof(double));
            return {{id, data, b_err, b_rtr}}; // Successfully received data within the timeout
        }
   
    // Log timeout or error only if no valid data received
    RCLCPP_ERROR(rclcpp::get_logger("PlatoSocketCAN"), "Timeout or error reading CAN frame, returning std::nullopt");
    
    return std::nullopt; // Indicate failure to read valid data within the timeout

}

}// namespace plato_socket_can