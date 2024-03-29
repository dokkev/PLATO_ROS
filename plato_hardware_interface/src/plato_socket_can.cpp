#include "plato_hardware_interface/plato_socket_can.hpp"
#include <fcntl.h>
#include <iostream>

namespace plato_socket_can {

PlatoSocketCAN::PlatoSocketCAN() : CAN_CHANNEL("can0"),
  encoder_scale_factor_(SCALE_INT / (ENCODER_MAX_SCALE_VALUE - ENCODER_MIN_SCALE_VALUE)),
  command_scale_factor_(SCALE_INT / (COMMAND_MAX_SCALE_VALUE - COMMAND_MIN_SCALE_VALUE)),
  can_tx_id_list_({ESP0_CAN_TX_ID, ESP1_CAN_TX_ID, ESP2_CAN_TX_ID}),
  can_rx_id_list_({ESP0_CAN_RX_ID, ESP1_CAN_RX_ID, ESP2_CAN_RX_ID})
{}

void PlatoSocketCAN::init() {
  // Setup ID mappings for Motor and CAN IDs
  // setup_can_ids();

  // Create a socket
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  // Specify the CAN interface
  std::strcpy(ifr_.ifr_name, "can0");
  ioctl(socket_, SIOCGIFINDEX, &ifr_);
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;

  // Buffer size
  // int rcvbuf_size = 524288; // Example size, adjust based on your needs
  // if (setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size,
  // sizeof(rcvbuf_size)) < 0) {
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


}


void PlatoSocketCAN::write_can(int socket, int id, int8_t data) {
  // sleep for 1us
  // usleep(1);
  struct can_frame frame;

  frame.can_id = id;
  frame.can_dlc = 6; 

  frame.data[0] = data;
  frame.data[1] = data;
  frame.data[2] = data;
  frame.data[3] = data;
  frame.data[4] = data;
  frame.data[5] = data;



  if (write(socket, &frame, sizeof(frame)) != sizeof(frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("PlatoSocketCAN"),
                 "Failed to send CAN frame");
  }

  // RCLCPP_INFO(rclcpp::get_logger("PlatoSocketCAN"),
              // "Sent CAN frame with ID: %d and Data: %f", id, data);
  //sleep
  
}


void PlatoSocketCAN::send_can_tx_msg() {



  write_can(socket_, can_tx_id_list_[0], 0x00);
  std::this_thread::sleep_for(std::chrono::microseconds(1));
  write_can(socket_, can_tx_id_list_[1], 0x00);
  std::this_thread::sleep_for(std::chrono::microseconds(1));
  write_can(socket_, can_tx_id_list_[2], 0x00);
  std::this_thread::sleep_for(std::chrono::microseconds(1));
  
}




void PlatoSocketCAN::receive_can(std::vector<double> &motor_position_states) {
    // Initialize the CAN frame
    struct can_frame frame;
    int nbytes = read(socket_, &frame, sizeof(frame));

    // check if the frame has an error
    // debug_can(frame);

    long id = frame.can_id;
    // Check if the frame is not empty and contains at least 6 bytes of data
    if (nbytes > 0 && frame.can_dlc >= 6) {
        // Unpack the data from the frame assuming it is in little endian and each value is 16 bits
        // Note: Adjust the indices if your data starts at a different byte within the frame
        
        double decoded_value0 = static_cast<double>(frame.data[0] | (frame.data[1] << 8)) / encoder_scale_factor_;
        double decoded_value1 = static_cast<double>(frame.data[2] | (frame.data[3] << 8)) / encoder_scale_factor_;
        double decoded_value2 = static_cast<double>(frame.data[4] | (frame.data[5] << 8)) / encoder_scale_factor_;

        if (id == can_rx_id_list_[0]){
          motor_position_states[0] = decoded_value0;
          motor_position_states[1] = decoded_value1;
          motor_position_states[2] = decoded_value2;

        } else if (id == can_rx_id_list_[1]){
          motor_position_states[3] = decoded_value0;
          motor_position_states[4] = decoded_value1;
          motor_position_states[5] = decoded_value2;

        } else if (id == can_rx_id_list_[2]){
          motor_position_states[6] = decoded_value0;
          motor_position_states[7] = decoded_value1;
          motor_position_states[8] = decoded_value2;
        } else {
          // Handle unexpected CAN ID
          RCLCPP_ERROR(rclcpp::get_logger("PlatoSocket::receive_can"), "Unexpected CAN ID: %ld", id);
        }

        
    }
}

void PlatoSocketCAN::send_can(std::vector<double> &motor_effort_commands) {

  // Send CAN messages for each ESP32 (3 times)
  for (long unsigned int i = 0; i < can_tx_id_list_.size(); i++) {
    struct can_frame frame;
    frame.can_dlc = 6; 
    frame.can_id = can_tx_id_list_[i];

    int offset;
    if (frame.can_id == can_tx_id_list_[0]) {
        offset = 0;
    } else if (frame.can_id == can_tx_id_list_[1]) {
        offset = 3;
    } else if (frame.can_id == can_tx_id_list_[2]) {
        offset = 6;
    } else {
        // Handle unexpected CAN ID
        RCLCPP_ERROR(rclcpp::get_logger("PlatoSocketCAN::send_can"), "Unexpected CAN ID: %d", frame.can_id);
        continue; // Skip this iteration

    }

    int16_t encoded_data0 = static_cast<int16_t>(motor_effort_commands[0+offset] * command_scale_factor_);
    int16_t encoded_data1 = static_cast<int16_t>(motor_effort_commands[1+offset] * command_scale_factor_);
    int16_t encoded_data2 = static_cast<int16_t>(motor_effort_commands[2+offset] * command_scale_factor_);

    frame.data[0] = encoded_data0 & 0xFF;
    frame.data[1] = (encoded_data0 >> 8) & 0xFF;
    frame.data[2] = encoded_data1 & 0xFF;
    frame.data[3] = (encoded_data1 >> 8) & 0xFF;
    frame.data[4] = encoded_data2 & 0xFF;
    frame.data[5] = (encoded_data2 >> 8) & 0xFF;

    // write(socket_, &frame, sizeof(frame));
    if (write(socket_, &frame, sizeof(frame)) != sizeof(frame)) {
    RCLCPP_ERROR(rclcpp::get_logger("PlatoSocketCAN::send_can"),
                 "Failed to send CAN frame");
    }

    #ifdef DEBUG_MODE
    RCLCPP_INFO(rclcpp::get_logger("PlatoSocketCAN::send_can"),
              "Sent CAN frame with ID: %x and Motor Command Data: [%f, %f, %f] and CAN Frame: [0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x]", 
              frame.can_id, motor_effort_commands[0+offset], motor_effort_commands[1+offset], motor_effort_commands[2+offset],
              frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5]);
    #endif

    // sleep for 1us
    std::this_thread::sleep_for(std::chrono::microseconds(1)); 
  }


}


void PlatoSocketCAN::debug_can(can_frame frame) {


    bool b_rtr = (frame.can_id & CAN_RTR_FLAG) != 0; // Remote transmission request flag
    bool b_err = (frame.can_id & CAN_ERR_FLAG) != 0; // Error frame flag

    if (b_err) {
        // Error frame
        can_err_mask_t mask = frame.can_id & CAN_ERR_MASK;
        RCLCPP_INFO(rclcpp::get_logger("PlatoSocket CAN::debug_can"), "Error frame: %x", mask);
    } 
    else if (b_rtr) {
        // Remote transmission request
        RCLCPP_INFO(rclcpp::get_logger("PlatoSocket CAN::debug_can"), "Remote transmission request");
    }

}

} // namespace plato_socket_can