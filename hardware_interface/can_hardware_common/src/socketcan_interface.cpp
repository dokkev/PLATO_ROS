#include "can_hardware_common/socketcan_interface.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace socketcan_interface
{

namespace
{

constexpr unsigned char kPcanMessageRtr = 0x01;
constexpr unsigned char kPcanMessageExtended = 0x02;
constexpr unsigned char kPcanMessageErrorFrame = 0x40;

void throw_socket_error(const std::string & operation, const std::string & interface_name)
{
  throw std::runtime_error(
          operation + " for SocketCAN interface '" + interface_name + "' failed: " +
          std::strerror(errno));
}

int poll_timeout_milliseconds(std::chrono::microseconds timeout)
{
  if (timeout.count() <= 0) {
    return 0;
  }

  const auto milliseconds = (timeout.count() + 999) / 1000;
  return static_cast<int>(std::min<long long>(
           milliseconds, std::numeric_limits<int>::max()));
}

}  // namespace

SocketCANInterface::SocketCANInterface(std::string interface_name)
: interface_name_(std::move(interface_name))
{
  if (interface_name_.empty()) {
    throw std::invalid_argument("SocketCAN interface name must not be empty");
  }
  if (interface_name_.size() >= IFNAMSIZ) {
    throw std::invalid_argument("SocketCAN interface name is too long: " + interface_name_);
  }

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw_socket_error("Opening socket", interface_name_);
  }

  try {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
      throw_socket_error("Making socket non-blocking", interface_name_);
    }

    struct ifreq interface_request{};
    std::strncpy(
      interface_request.ifr_name,
      interface_name_.c_str(),
      sizeof(interface_request.ifr_name) - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &interface_request) < 0) {
      throw_socket_error("Resolving interface index", interface_name_);
    }

    struct sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_request.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
      throw_socket_error("Binding socket", interface_name_);
    }
  } catch (...) {
    (void)::close(fd);
    throw;
  }

  socket_fd_ = fd;
}

SocketCANInterface::~SocketCANInterface() noexcept
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (socket_fd_ >= 0) {
    (void)::close(socket_fd_);
    socket_fd_ = -1;
  }
}

TPCANStatus SocketCANInterface::write(const TPCANMsg & tx_frame)
{
  struct can_frame frame{};
  frame.can_id = tx_frame.ID;
  if ((tx_frame.MSGTYPE & kPcanMessageExtended) != 0U) {
    frame.can_id |= CAN_EFF_FLAG;
  }
  if ((tx_frame.MSGTYPE & kPcanMessageRtr) != 0U) {
    frame.can_id |= CAN_RTR_FLAG;
  }
  frame.can_dlc = std::min<unsigned char>(tx_frame.LEN, CAN_MAX_DLEN);
  std::memcpy(frame.data, tx_frame.DATA, frame.can_dlc);

  std::lock_guard<std::mutex> lock(io_mutex_);
  if (socket_fd_ < 0) {
    return PCAN_ERROR_UNKNOWN;
  }

  const auto bytes_written = ::send(
    socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT | MSG_NOSIGNAL);
  if (bytes_written < 0) {
    return map_write_errno_(errno);
  }
  return bytes_written == static_cast<ssize_t>(sizeof(frame)) ?
    PCAN_ERROR_OK : PCAN_ERROR_ILLDATA;
}

TPCANStatus SocketCANInterface::read(TPCANMsg & rx_frame, TPCANTimestamp * timestamp)
{
  struct can_frame frame{};
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (socket_fd_ < 0) {
    return PCAN_ERROR_UNKNOWN;
  }

  const auto bytes_read = ::recv(socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT);
  if (bytes_read < 0) {
    return map_read_errno_(errno);
  }
  if (bytes_read != static_cast<ssize_t>(sizeof(frame))) {
    return PCAN_ERROR_ILLDATA;
  }

  rx_frame = {};
  if ((frame.can_id & CAN_ERR_FLAG) != 0U) {
    rx_frame.ID = frame.can_id & CAN_ERR_MASK;
    rx_frame.MSGTYPE = kPcanMessageErrorFrame;
  } else if ((frame.can_id & CAN_EFF_FLAG) != 0U) {
    rx_frame.ID = frame.can_id & CAN_EFF_MASK;
    rx_frame.MSGTYPE = kPcanMessageExtended;
  } else {
    rx_frame.ID = frame.can_id & CAN_SFF_MASK;
    rx_frame.MSGTYPE = PCAN_MESSAGE_STANDARD;
  }
  if ((frame.can_id & CAN_RTR_FLAG) != 0U) {
    rx_frame.MSGTYPE |= kPcanMessageRtr;
  }
  rx_frame.LEN = std::min<unsigned char>(frame.can_dlc, CAN_MAX_DLEN);
  std::memcpy(rx_frame.DATA, frame.data, rx_frame.LEN);
  if (timestamp != nullptr) {
    *timestamp = {};
  }
  return PCAN_ERROR_OK;
}

TPCANStatus SocketCANInterface::read_with_timeout(
  TPCANMsg & rx_frame,
  std::chrono::microseconds timeout)
{
  const auto initial_status = read(rx_frame);
  if (initial_status != PCAN_ERROR_QRCVEMPTY || timeout.count() <= 0) {
    return initial_status;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
      return PCAN_ERROR_QRCVEMPTY;
    }

    struct pollfd poll_fd{};
    poll_fd.fd = socket_fd_;
    poll_fd.events = POLLIN;
    const int poll_status = ::poll(&poll_fd, 1, poll_timeout_milliseconds(remaining));
    if (poll_status == 0) {
      return PCAN_ERROR_QRCVEMPTY;
    }
    if (poll_status < 0) {
      if (errno == EINTR) {
        continue;
      }
      return map_read_errno_(errno);
    }
    if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      return PCAN_ERROR_BUSOFF;
    }
    return read(rx_frame);
  }
}

TPCANStatus SocketCANInterface::get_bus_status()
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (socket_fd_ < 0) {
    return PCAN_ERROR_UNKNOWN;
  }
  return interface_is_up_() ? PCAN_ERROR_OK : PCAN_ERROR_BUSOFF;
}

TPCANStatus SocketCANInterface::get_value(
  TPCANParameter parameter,
  void * buffer,
  uint32_t buffer_length)
{
  if (buffer == nullptr) {
    return PCAN_ERROR_ILLPARAMVAL;
  }

  std::lock_guard<std::mutex> lock(io_mutex_);
  if (socket_fd_ < 0) {
    return PCAN_ERROR_UNKNOWN;
  }

  switch (parameter) {
    case PCAN_CHANNEL_CONDITION:
      if (buffer_length < sizeof(uint32_t)) {
        return PCAN_ERROR_ILLPARAMVAL;
      }
      *static_cast<uint32_t *>(buffer) =
        interface_is_up_() ? PCAN_CHANNEL_AVAILABLE : PCAN_CHANNEL_UNAVAILABLE;
      return PCAN_ERROR_OK;
    case PCAN_RECEIVE_STATUS:
      if (buffer_length < sizeof(uint32_t)) {
        return PCAN_ERROR_ILLPARAMVAL;
      }
      *static_cast<uint32_t *>(buffer) = PCAN_PARAMETER_ON;
      return PCAN_ERROR_OK;
    case PCAN_RECEIVE_EVENT:
      if (buffer_length < sizeof(int)) {
        return PCAN_ERROR_ILLPARAMVAL;
      }
      *static_cast<int *>(buffer) = socket_fd_;
      return PCAN_ERROR_OK;
    default:
      return PCAN_ERROR_ILLPARAMTYPE;
  }
}

TPCANStatus SocketCANInterface::map_read_errno_(int error_number)
{
  switch (error_number) {
    case EAGAIN:
#if EWOULDBLOCK != EAGAIN
    case EWOULDBLOCK:
#endif
      return PCAN_ERROR_QRCVEMPTY;
    case ENETDOWN:
    case ENETUNREACH:
    case ENETRESET:
    case ENODEV:
      return PCAN_ERROR_BUSOFF;
    default:
      return PCAN_ERROR_UNKNOWN;
  }
}

TPCANStatus SocketCANInterface::map_write_errno_(int error_number)
{
  switch (error_number) {
    case EAGAIN:
#if EWOULDBLOCK != EAGAIN
    case EWOULDBLOCK:
#endif
    case ENOBUFS:
      return PCAN_ERROR_QXMTFULL;
    case ENETDOWN:
    case ENETUNREACH:
    case ENETRESET:
    case ENODEV:
      return PCAN_ERROR_BUSOFF;
    default:
      return PCAN_ERROR_UNKNOWN;
  }
}

bool SocketCANInterface::interface_is_up_() const
{
  struct ifreq interface_request{};
  std::strncpy(
    interface_request.ifr_name,
    interface_name_.c_str(),
    sizeof(interface_request.ifr_name) - 1);
  return ::ioctl(socket_fd_, SIOCGIFFLAGS, &interface_request) == 0 &&
         (interface_request.ifr_flags & IFF_UP) != 0;
}

}  // namespace socketcan_interface
