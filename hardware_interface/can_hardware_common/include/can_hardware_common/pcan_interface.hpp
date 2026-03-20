#ifndef CAN_HARDWARE_COMMON__PCAN_INTERFACE_HPP_
#define CAN_HARDWARE_COMMON__PCAN_INTERFACE_HPP_

#include <PCANBasic.h>

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

  static std::string format_error(TPCANStatus status);

private:
  static constexpr TPCANHandle kChannelHandle_ = PCAN_USBBUS1;
  static constexpr TPCANBaudrate kChannelBitrate_ = PCAN_BAUD_1M;
  mutable std::mutex io_mutex_;
};

}  // namespace pcan_interface

#endif  // CAN_HARDWARE_COMMON__PCAN_INTERFACE_HPP_
