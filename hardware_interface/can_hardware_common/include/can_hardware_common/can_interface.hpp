#ifndef CAN_HARDWARE_COMMON__CAN_INTERFACE_HPP_
#define CAN_HARDWARE_COMMON__CAN_INTERFACE_HPP_

#include <PCANBasic.h>

#include <chrono>
#include <cstdint>

namespace can_hardware_common
{

class CanInterface
{
public:
  virtual ~CanInterface() noexcept = default;

  CanInterface(const CanInterface &) = delete;
  CanInterface & operator=(const CanInterface &) = delete;
  CanInterface(CanInterface &&) = delete;
  CanInterface & operator=(CanInterface &&) = delete;

  virtual TPCANStatus write(const TPCANMsg & tx_frame) = 0;
  virtual TPCANStatus read(TPCANMsg & rx_frame, TPCANTimestamp * timestamp = nullptr) = 0;
  virtual TPCANStatus read_with_timeout(
    TPCANMsg & rx_frame,
    std::chrono::microseconds timeout) = 0;
  virtual TPCANStatus get_bus_status() = 0;
  virtual TPCANStatus get_value(
    TPCANParameter parameter,
    void * buffer,
    uint32_t buffer_length) = 0;

protected:
  CanInterface() = default;
};

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__CAN_INTERFACE_HPP_
