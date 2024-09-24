#ifndef PLATO_HARDWARE_INTERFACE__PCAN_INTERFACE_HPP_
#define PLATO_HARDWARE_INTERFACE__PCAN_INTERFACE_HPP_

#include <PCANBasic.h>
#include <cstdint>
#include "linux_interop.h"
#include "utils/ring_buf.h"

#define CAN_RX_BUFFER_SIZE 10

// #define DEBUG_MODE

namespace pcan_interface {


class PCANInterface {

private:
    // @bridf CAN FD bool indicator
    const bool b_fd = false;

    /// @brief Sets PCANHanddle (Hardware Chaneel)
    const TPCANHandle pcan_handle = PCAN_USBBUS1;

    /// @brief Sets PCANBaudrate (Baudrate)
    const TPCANBaudrate bitrate = PCAN_BAUD_1M;

    TPCANMsg can_rx_buffer_storage[CAN_RX_BUFFER_SIZE];
    RingBuf can_rx_buffer;


public:
    /// @brief Constructor
    PCANInterface();

    /// @brief Destructor
    ~PCANInterface();

    /// @brief Write messages on CAN devices
    /// @param msg TPCANMsg to write
    /// @return TPCANStatus of the write operation
    TPCANStatus write_message(TPCANMsg* msg);

    /// @brief Read messages from CAN devices
    /// @return TPCANStatus of the read operation
    TPCANStatus read_message();

    /// @brief Process a message to store the data in to a buffer
    /// @param msg TPCANMsg to process
    /// @param timestamp TPCANTimestamp of the message
    void process_message(const TPCANMsg msg, TPCANTimestamp timestamp);

    /// @brief Get the RX message from the buffer
    /// @param msg TPCANMsg to store the message
    /// @return bool true if there is a message in the buffer
    bool get_buffer_message(TPCANMsg& msg);

    /// @brief Print a message on the console for debugging
    /// @param msg TPCANMsg to print
    void print_message(const TPCANMsg msg); 

private:

    /// @brief Shows formatted status
    /// @param status will be formatted
    void show_status(TPCANStatus status);

    /// @brief print PCAN Configuration
    void print_current_config();

    /// @brief gets the formatted text from a PCAN-Basic channel handle
    /// @param handle PCAN-Basic Handle to format
    /// @param buffer Buffer to store the formatted text
    /// @param b_fd If the channel is FD capable
    void format_channel_name(TPCANHandle handle, LPSTR buffer, bool b_fd);

    /// @brief Gets name of a TPCANHandle
    /// @param handle TPCANHandle to get the name of
    /// @param buffer Buffer to store the name
    void get_handle_name(TPCANHandle handle, LPSTR buffer);

    /// @brief Gets the formatted error text from a PCAN-Basic error code
    /// @param error PCAN-Basic error code to format
    /// @param buffer A string buffer for the translated error
    void get_formatted_error(TPCANStatus error, LPSTR buffer);

    /// @brief Covernt bitrate c_short value to readable string
    /// @param bitrate TPCANBaudrate to convert
    /// @param buffer Buffer to store the converted string
    void convert_bitrate_to_string(TPCANBaudrate bitrate, LPSTR buffer);
  
};

} // namespace pcan_interface

#endif // PLATO_HARDWARE_INTERFACE__PCAN_INTERFACE_HPP_