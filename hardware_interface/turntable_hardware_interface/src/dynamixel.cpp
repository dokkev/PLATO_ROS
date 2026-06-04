#include "turntable_hardware_interface/dynamixel.hpp"

#include <stdexcept>
#include <utility>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace turntable_hardware_interface
{
namespace
{
constexpr uint8_t kNoError = 0;
}  // namespace

Dynamixel::Dynamixel(Config config)
: config_(std::move(config)),
  port_handler_(dynamixel::PortHandler::getPortHandler(config_.device_name.c_str())),
  packet_handler_(dynamixel::PacketHandler::getPacketHandler(config_.protocol_version))
{
  if (port_handler_ == nullptr || packet_handler_ == nullptr) {
    throw std::runtime_error("Failed to create Dynamixel SDK handlers.");
  }
}

Dynamixel::~Dynamixel()
{
  close();
}

Dynamixel::Dynamixel(Dynamixel && other) noexcept
{
  *this = std::move(other);
}

Dynamixel & Dynamixel::operator=(Dynamixel && other) noexcept
{
  if (this != &other) {
    close();
    config_ = std::move(other.config_);
    port_handler_ = other.port_handler_;
    packet_handler_ = other.packet_handler_;
    is_open_ = other.is_open_;
    last_error_ = std::move(other.last_error_);

    other.port_handler_ = nullptr;
    other.packet_handler_ = nullptr;
    other.is_open_ = false;
  }
  return *this;
}

bool Dynamixel::open()
{
  if (is_open_) {
    return true;
  }

  if (!port_handler_->openPort()) {
    set_error_("Failed to open Dynamixel port: " + config_.device_name);
    return false;
  }

  if (!port_handler_->setBaudRate(config_.baudrate)) {
    port_handler_->closePort();
    set_error_("Failed to set Dynamixel baudrate: " + std::to_string(config_.baudrate));
    return false;
  }

  is_open_ = true;
  clear_error_();
  return true;
}

void Dynamixel::close()
{
  if (port_handler_ != nullptr && is_open_) {
    port_handler_->closePort();
  }
  is_open_ = false;
}

bool Dynamixel::is_open() const
{
  return is_open_;
}

int32_t Dynamixel::get_current_position()
{
  return read_int32_(config_.present_position_address, "present position");
}

int32_t Dynamixel::get_current_velocity()
{
  return read_int32_(config_.present_velocity_address, "present velocity");
}

void Dynamixel::set_operating_mode(uint8_t mode)
{
  write_uint8_(config_.operating_mode_address, mode, "operating mode");
}

void Dynamixel::set_torque_enabled(bool enabled)
{
  write_uint8_(config_.torque_enable_address, enabled ? 1 : 0, "torque enable");
}

void Dynamixel::set_desired_position(int32_t position)
{
  write_int32_(config_.goal_position_address, position, "goal position");
}

int32_t Dynamixel::read_int32_(uint16_t address, const char * signal_name)
{
  if (!is_open_ && !open()) {
    throw std::runtime_error(last_error_);
  }

  uint8_t dynamixel_error = kNoError;
  uint32_t value = 0;
  const int communication_result = packet_handler_->read4ByteTxRx(
    port_handler_, config_.id, address, &value, &dynamixel_error);
  if (communication_result != COMM_SUCCESS) {
    set_error_(
      std::string("Failed to read Dynamixel ") + signal_name + ": " +
      packet_handler_->getTxRxResult(communication_result));
    throw std::runtime_error(last_error_);
  }
  if (dynamixel_error != kNoError) {
    set_error_(
      std::string("Dynamixel reported an error while reading ") + signal_name + ": " +
      packet_handler_->getRxPacketError(dynamixel_error));
    throw std::runtime_error(last_error_);
  }

  clear_error_();
  return static_cast<int32_t>(value);
}

void Dynamixel::write_uint8_(uint16_t address, uint8_t value, const char * signal_name)
{
  if (!is_open_ && !open()) {
    throw std::runtime_error(last_error_);
  }

  uint8_t dynamixel_error = kNoError;
  const int communication_result = packet_handler_->write1ByteTxRx(
    port_handler_,
    config_.id,
    address,
    value,
    &dynamixel_error);

  if (communication_result != COMM_SUCCESS) {
    set_error_(
      std::string("Failed to write Dynamixel ") + signal_name + ": " +
      packet_handler_->getTxRxResult(communication_result));
    throw std::runtime_error(last_error_);
  }
  if (dynamixel_error != kNoError) {
    set_error_(
      std::string("Dynamixel reported an error while writing ") + signal_name + ": " +
      packet_handler_->getRxPacketError(dynamixel_error));
    throw std::runtime_error(last_error_);
  }

  clear_error_();
}

void Dynamixel::write_int32_(uint16_t address, int32_t value, const char * signal_name)
{
  if (!is_open_ && !open()) {
    throw std::runtime_error(last_error_);
  }

  uint8_t dynamixel_error = kNoError;
  const int communication_result = packet_handler_->write4ByteTxRx(
    port_handler_,
    config_.id,
    address,
    static_cast<uint32_t>(value),
    &dynamixel_error);

  if (communication_result != COMM_SUCCESS) {
    set_error_(
      std::string("Failed to write Dynamixel ") + signal_name + ": " +
      packet_handler_->getTxRxResult(communication_result));
    throw std::runtime_error(last_error_);
  }
  if (dynamixel_error != kNoError) {
    set_error_(
      std::string("Dynamixel reported an error while writing ") + signal_name + ": " +
      packet_handler_->getRxPacketError(dynamixel_error));
    throw std::runtime_error(last_error_);
  }

  clear_error_();
}

void Dynamixel::set_error_(const std::string & message)
{
  last_error_ = message;
}

void Dynamixel::clear_error_()
{
  last_error_.clear();
}

}  // namespace turntable_hardware_interface
