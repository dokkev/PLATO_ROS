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
        std::cerr << "Error: More commands than available TX IDs.\n";
        return;
    }

    for (size_t i = 0; i < motor_effort_commands.size(); ++i) {
        write_can(socket_, can_tx_id_[i], motor_effort_commands[i]);
    
    }
}


void PlatoSocketCAN::receive_can_rx_msg(std::vector<double>& motor_position_states) {
   for (size_t i = 0; i < motor_position_states.size(); ++i) {
        double rx_msg = read_can(socket_, can_rx_id_[i]);
        if (i < motor_position_states.size()){
            motor_position_states[i] = rx_msg;
        }         
    }
}

void PlatoSocketCAN::write_can(int socket, int id, double data) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame)); // Clear the frame

    frame.can_id = id;
    frame.can_dlc = 8; // Explicitly set to 8 bytes for clarity
    std::memcpy(frame.data, &data, sizeof(double));
    if (write(socket, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("CAN Frame write failed");
    }
}

double PlatoSocketCAN::read_can(int socket, int id) {
    struct can_frame frame;
    int nbytes = read(socket, &frame, sizeof(frame));
    if (nbytes > 0) {
        if (frame.can_dlc == 8 && frame.can_id == static_cast<canid_t>(id)) {
            double data;
            std::memcpy(&data, frame.data, sizeof(double));
            return data;
        }
    } else {
        perror("Failed to read CAN frame");
    }

    return 0.0; // Return a default or indicate error/missing data differently


}  

}// namespace plato_socket_can