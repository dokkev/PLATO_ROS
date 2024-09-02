#ifndef PLATO_HARDWARE_INTERFACE__PCAN_INTERFACE_HPP_
#define PLATO_HARDWARE_INTERFACE__PCAN_INTERFACE_HPP_

#include <PCANBasic.h>
#include <cstdint>
#include "linux_interop.h"


namespace pcan_interface {


class PCANInterface {

private:
    // @bridf CAN FD bool indicator
    const bool b_is_fd = false;

    /// @brief Sets PCANHanddle (Hardware Chaneel)
    const TPCANHandle pcan_handle = PCAN_USBBUS1;

    /// @brief Sets PCANBaudrate (Baudrate)
    const TPCANBaudrate bitrate = PCAN_BAUD_1M;


public:
    /// @brief Constructor
    PCANInterface();

    /// @brief Destructor
    ~PCANInterface();

    /// @brief Write messages on CAN devices
    TPCANStatus write_message(TPCANMsg* msg);

    /// @brief Read messages from CAN devices
    TPCANStatus read_message();

    /// @brief Print a message on the console for debugging
    void print_message(const TPCANMsg msg); 



private:

    /// @brief Shows formatted status
    /// @param status will be formatted
    void show_status(TPCANStatus status);
  
};

} // namespace pcan_interface

#endif // PLATO_HARDWARE_INTERFACE__PCAN_INTERFACE_HPP_