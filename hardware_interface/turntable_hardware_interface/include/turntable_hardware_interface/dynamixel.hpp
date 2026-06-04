#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace dynamixel
{
class PacketHandler;
class PortHandler;
}  // namespace dynamixel

namespace turntable_hardware_interface
{

class Dynamixel
{
public:
  struct Config
  {
    std::string device_name = "/dev/ttyUSB0";
    int baudrate = 57600;
    uint8_t id = 1;
    float protocol_version = 2.0F;

    // Protocol 2.0 X-series defaults. Override these for other models.
    uint16_t operating_mode_address = 11;
    uint16_t torque_enable_address = 64;
    uint16_t goal_position_address = 116;
    uint16_t present_velocity_address = 128;
    uint16_t present_position_address = 132;
  };

  explicit Dynamixel(Config config);
  ~Dynamixel();

  Dynamixel(const Dynamixel &) = delete;
  Dynamixel & operator=(const Dynamixel &) = delete;

  Dynamixel(Dynamixel &&) noexcept;
  Dynamixel & operator=(Dynamixel &&) noexcept;

  bool open();
  void close();
  bool is_open() const;

  int32_t get_current_position();
  int32_t get_current_velocity();
  void set_operating_mode(uint8_t mode);
  void set_torque_enabled(bool enabled);
  void set_desired_position(int32_t position);

  const Config & config() const {return config_;}
  const std::string & last_error() const {return last_error_;}

private:
  int32_t read_int32_(uint16_t address, const char * signal_name);
  void write_uint8_(uint16_t address, uint8_t value, const char * signal_name);
  void write_int32_(uint16_t address, int32_t value, const char * signal_name);
  void set_error_(const std::string & message);
  void clear_error_();

  Config config_;
  dynamixel::PortHandler * port_handler_ = nullptr;
  dynamixel::PacketHandler * packet_handler_ = nullptr;
  bool is_open_ = false;
  std::string last_error_;
};

}  // namespace turntable_hardware_interface
