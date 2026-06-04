#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace
{

constexpr float kProtocolVersion = 2.0F;
const char * kDefaultDevice =
  "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9BTGF3-if00-port0";

std::vector<int> default_baudrates()
{
  return {
    1000000,
    57600,
    115200,
    2000000,
    3000000,
    4000000,
  };
}

void print_usage(const char * program_name)
{
  std::cout
    << "Usage: " << program_name << " [device_name] [baudrate]\n"
    << "\n"
    << "Examples:\n"
    << "  " << program_name << "\n"
    << "  " << program_name << " /dev/ttyUSB0\n"
    << "  " << program_name << " /dev/ttyUSB0 1000000\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc > 1 && std::string(argv[1]) == "--help") {
    print_usage(argv[0]);
    return 0;
  }

  const std::string device_name = argc > 1 ? argv[1] : kDefaultDevice;
  std::vector<int> baudrates = default_baudrates();
  if (argc > 2) {
    baudrates = {std::atoi(argv[2])};
  }

  auto * port_handler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
  auto * packet_handler = dynamixel::PacketHandler::getPacketHandler(kProtocolVersion);
  if (port_handler == nullptr || packet_handler == nullptr) {
    std::cerr << "Failed to create Dynamixel SDK handlers.\n";
    return 1;
  }

  if (!port_handler->openPort()) {
    std::cerr << "Failed to open port: " << device_name << "\n";
    return 1;
  }

  bool found_any = false;
  std::cout << "Scanning " << device_name << " with Protocol 2.0\n";
  for (const int baudrate : baudrates) {
    if (!port_handler->setBaudRate(baudrate)) {
      std::cerr << "  " << baudrate << " baud: failed to set baudrate\n";
      continue;
    }

    std::vector<uint8_t> ids;
    const int result = packet_handler->broadcastPing(port_handler, ids);
    if (result != COMM_SUCCESS) {
      std::cout
        << "  " << baudrate << " baud: "
        << packet_handler->getTxRxResult(result) << "\n";
      continue;
    }

    if (ids.empty()) {
      std::cout << "  " << baudrate << " baud: no Dynamixels found\n";
      continue;
    }

    found_any = true;
    std::cout << "  " << baudrate << " baud: found ID";
    if (ids.size() > 1) {
      std::cout << "s";
    }
    std::cout << " ";
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (i > 0) {
        std::cout << ", ";
      }
      std::cout << static_cast<int>(ids[i]);
    }
    std::cout << "\n";
  }

  port_handler->closePort();
  return found_any ? 0 : 2;
}
