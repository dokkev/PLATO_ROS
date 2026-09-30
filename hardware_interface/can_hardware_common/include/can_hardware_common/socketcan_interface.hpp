#ifndef CAN_HARDWARE_COMMON__SOCKETCAN_INTERFACE_HPP_
#define CAN_HARDWARE_COMMON__SOCKETCAN_INTERFACE_HPP_

#include <chrono>
#include <mutex>
#include <string>

#include "can_hardware_common/can_interface.hpp"

namespace socketcan_interface
{

class SocketCANInterface final : public can_hardware_common::CanInterface
{
public:
  explicit SocketCANInterface(std::string interface_name);
  ~SocketCANInterface() noexcept override;

  SocketCANInterface(const SocketCANInterface &) = delete;
  SocketCANInterface & operator=(const SocketCANInterface &) = delete;
  SocketCANInterface(SocketCANInterface &&) = delete;
  SocketCANInterface & operator=(SocketCANInterface &&) = delete;

  TPCANStatus write(const TPCANMsg & tx_frame) override;
  TPCANStatus read(TPCANMsg & rx_frame, TPCANTimestamp * timestamp = nullptr) override;
  TPCANStatus read_with_timeout(
    TPCANMsg & rx_frame,
    std::chrono::microseconds timeout) override;
  TPCANStatus get_bus_status() override;
  TPCANStatus get_value(
    TPCANParameter parameter,
    void * buffer,
    uint32_t buffer_length) override;

private:
  static TPCANStatus map_read_errno_(int error_number);
  static TPCANStatus map_write_errno_(int error_number);
  bool interface_is_up_() const;

  mutable std::mutex io_mutex_;
  std::string interface_name_;
  int socket_fd_ = -1;
};

}  // namespace socketcan_interface

#endif  // CAN_HARDWARE_COMMON__SOCKETCAN_INTERFACE_HPP_
