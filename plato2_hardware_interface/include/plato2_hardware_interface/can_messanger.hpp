#ifndef PLATO_HARDWARE_INTERFACE__CAN_MESSANGER_HPP_
#define PLATO_HARDWARE_INTERFACE__CAN_MESSANGER_HPP_

#include "plato2_hardware_interface/pcan_interface.hpp"
#include "plato2_hardware_interface/can_protocol.hpp"

namespace can_messanger{

class MessageManager{
public:
    MessageManager() = default;

    TPCANMsg init_message();

    /// @brief Send the TPCANMsg using pcan_interface
    /// @param msg 
    void send_message(const TPCANMsg &msg);

private:
    pcan_interface::PCANInterface pcan_interface_;
    can_protocol::ControlMessage control_msg_;
    can_protocol::StateMessage state_msg_;
    can_protocol::GainMessage gain_msg_;


    void init_message_(TPCANMsg &msg);

};

} // namespace can_messanger







#endif // PLATO_HARDWARE_INTERFACE__CAN_MESSANGER_HPP_