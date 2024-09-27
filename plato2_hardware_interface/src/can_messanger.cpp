#include "plato2_hardware_interface/can_messanger.hpp"


namespace can_messanger{

void MessageManager::init_message_(TPCANMsg &msg){
    msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
    msg.LEN = 8;
    std::memset(msg.DATA, 0, 8);
}

void MessageManager::send_message(const TPCANMsg &msg){
    // pcan_interface_.write_message(&msg);
}

void MessageManager::receive_message(){

    TPCANMsg rx_msg;

    // pcan_interface_.read_message();

    // pcan_interface_.get_buffer_message(rx_msg);
    
}


}; // namespace can_messanger