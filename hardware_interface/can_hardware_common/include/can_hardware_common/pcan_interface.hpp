#ifndef CAN_HARDWARE_COMMON__PCAN_INTERFACE_HPP_
#define CAN_HARDWARE_COMMON__PCAN_INTERFACE_HPP_

#include <PCANBasic.h>

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

namespace pcan_interface
{

class PCANInterface
{
public:
  PCANInterface();
  ~PCANInterface() noexcept;

  PCANInterface(const PCANInterface &) = delete;
  PCANInterface & operator=(const PCANInterface &) = delete;
  PCANInterface(PCANInterface &&) = delete;
  PCANInterface & operator=(PCANInterface &&) = delete;

  TPCANStatus write(const TPCANMsg & msg);
  TPCANStatus read(TPCANMsg & msg, TPCANTimestamp * timestamp = nullptr);

  /// Blocking read: tries an immediate read, then waits on the receive-event fd
  /// via ppoll() for up to @p timeout before retrying once.
  /// Returns PCAN_ERROR_OK on success or PCAN_ERROR_QRCVEMPTY on timeout.
  TPCANStatus read_with_timeout(TPCANMsg & msg, std::chrono::microseconds timeout);

  /// Query the CAN controller error state via CAN_GetStatus().
  TPCANStatus get_bus_status();

  /// Read a PCAN channel parameter via CAN_GetValue().
  TPCANStatus get_value(TPCANParameter parameter, void * buffer, uint32_t buffer_length);

  static std::string format_error(TPCANStatus status);

private:
  static constexpr TPCANHandle kChannelHandle_ = PCAN_USBBUS1;
  static constexpr TPCANBaudrate kChannelBitrate_ = PCAN_BAUD_1M;
  mutable std::mutex io_mutex_;
  int receive_event_fd_ = -1;
};

}  // namespace pcan_interface

#endif  // CAN_HARDWARE_COMMON__PCAN_INTERFACE_HPP_
