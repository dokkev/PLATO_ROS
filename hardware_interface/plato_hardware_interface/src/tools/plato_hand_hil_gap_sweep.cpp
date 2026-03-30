#include "can_hardware_common/can_transport.hpp"
#include "can_hardware_common/command_scheduler.hpp"
#include "plato_hardware_interface/actuator.hpp"
#include "plato_hardware_interface/utils/plato_hand_config_loader.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace
{
using Scheduler = can_hardware_common::CanCommandScheduler;
using ResultStatus = Scheduler::TransactionResult::Status;

struct Options
{
  std::vector<int> gaps_us{1000, 500, 300, 200, 100, 50, 0};
  int timeout_ms = 40;
  int retries = 2;
  int cycles = 200;
  int loop_period_us = 10000;
  float servo_position_rad = 0.0f;
  bool disable_on_exit = true;
};

bool parse_gap_list(const std::string & raw, std::vector<int> & gaps_us)
{
  std::vector<int> parsed;
  std::stringstream ss(raw);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) {
      continue;
    }
    parsed.push_back(std::stoi(item));
  }
  if (parsed.empty()) {
    return false;
  }
  gaps_us = std::move(parsed);
  return true;
}

bool parse_options(int argc, char ** argv, Options & options)
{
  std::vector<std::string> args(argv + 1, argv + argc);
  for (size_t i = 0; i < args.size(); ++i) {
    if (args[i] == "--gaps-us" && i + 1 < args.size()) {
      if (!parse_gap_list(args[++i], options.gaps_us)) {
        return false;
      }
      continue;
    }
    if (args[i] == "--timeout-ms" && i + 1 < args.size()) {
      options.timeout_ms = std::stoi(args[++i]);
      continue;
    }
    if (args[i] == "--retries" && i + 1 < args.size()) {
      options.retries = std::stoi(args[++i]);
      continue;
    }
    if (args[i] == "--cycles" && i + 1 < args.size()) {
      options.cycles = std::stoi(args[++i]);
      continue;
    }
    if (args[i] == "--loop-period-us" && i + 1 < args.size()) {
      options.loop_period_us = std::stoi(args[++i]);
      continue;
    }
    if (args[i] == "--servo-pos-rad" && i + 1 < args.size()) {
      options.servo_position_rad = std::stof(args[++i]);
      continue;
    }
    if (args[i] == "--keep-enabled") {
      options.disable_on_exit = false;
      continue;
    }
    if (args[i] == "--help" || args[i] == "-h") {
      std::cout
        << "Usage: plato_hand_hil_gap_sweep [options]\n"
        << "  --gaps-us <csv>          comma-separated gap sweep (default: 1000,500,300,200,100,50,0)\n"
        << "  --timeout-ms <int>       blocking transaction timeout in ms (default: 40)\n"
        << "  --retries <int>          blocking transaction retries (default: 2)\n"
        << "  --cycles <int>           streaming write/read cycles per gap (default: 200)\n"
        << "  --loop-period-us <int>   delay between cycles (default: 10000)\n"
        << "  --servo-pos-rad <float>  servo hold target during sweep (default: 0.0)\n"
        << "  --keep-enabled           skip disable commands at the end of each gap case\n";
      return false;
    }
    std::cerr << "Unknown argument: " << args[i] << "\n";
    return false;
  }
  return true;
}

std::string join_indices(const std::vector<size_t> & indices)
{
  std::ostringstream oss;
  for (size_t i = 0; i < indices.size(); ++i) {
    if (i != 0) {
      oss << ",";
    }
    oss << (indices[i] + 1);
  }
  return oss.str();
}

bool enable_all(
  Scheduler & scheduler,
  std::vector<plato_actuator::Actuator> & actuators,
  std::chrono::microseconds timeout,
  size_t retries)
{
  for (size_t i = 0; i < actuators.size(); ++i) {
    const auto req = actuators[i].make_enable_request(static_cast<uint32_t>(i));
    const auto res = scheduler.execute_blocking(req, timeout, retries);
    if (res.status != ResultStatus::kConfirmed) {
      std::cerr << "[gap_sweep] enable failed for actuator " << (i + 1) << "\n";
      return false;
    }
  }
  return true;
}

void disable_all(
  Scheduler & scheduler,
  std::vector<plato_actuator::Actuator> & actuators,
  std::chrono::microseconds timeout,
  size_t retries)
{
  for (size_t i = 0; i < actuators.size(); ++i) {
    const auto req = actuators[i].make_disable_request(static_cast<uint32_t>(100 + i));
    (void)scheduler.execute_blocking(req, timeout, retries);
  }
}

std::vector<size_t> initialized_indices(const std::vector<plato_actuator::Actuator> & actuators)
{
  std::vector<size_t> indices;
  for (size_t i = 0; i < actuators.size(); ++i) {
    if (actuators[i].is_initialized()) {
      indices.push_back(i);
    }
  }
  return indices;
}

void send_streaming_frame(
  can_hardware_common::CanTransport & transport,
  const TPCANMsg & frame)
{
  const auto next_send = transport.next_send_time();
  const auto now = std::chrono::steady_clock::now();
  if (next_send > now) {
    std::this_thread::sleep_until(next_send);
  }
  (void)transport.send_if_ready(frame);
}

void run_gap_case(const Options & options, int gap_us)
{
  auto config = plato_hand::load_default_plato_hand_config();
  std::vector<plato_actuator::Actuator> actuators;
  actuators.reserve(config.actuator_configs.size());
  for (const auto & cfg : config.actuator_configs) {
    actuators.emplace_back(cfg);
  }

  can_hardware_common::CanTransport transport;
  transport.set_min_inter_frame_gap(std::chrono::microseconds(gap_us));
  transport.add_rx_observer([&actuators](const TPCANMsg & frame) {
    for (auto & actuator : actuators) {
      if (actuator.get_rx_id() == frame.ID) {
        actuator.process_message(frame);
        return;
      }
    }
  });

  Scheduler scheduler(transport);
  const auto timeout = std::chrono::milliseconds(options.timeout_ms);
  const auto retries = static_cast<size_t>(options.retries);

  std::cout << "[CASE] gap_us=" << gap_us << "\n";
  if (!enable_all(scheduler, actuators, timeout, retries)) {
    std::cout << "  enable=FAIL\n";
    return;
  }

  size_t rx_frames = 0;
  for (int cycle = 0; cycle < options.cycles; ++cycle) {
    for (size_t i = 0; i < actuators.size(); ++i) {
      TPCANMsg frame{};
      if (i < 2) {
        frame = actuators[i].set_servo_hold(options.servo_position_rad).frame;
      } else {
        frame = actuators[i].set_joint_torque(0.0f).frame;
      }
      send_streaming_frame(transport, frame);
    }

    const auto rx = transport.process_rx();
    rx_frames += rx.processed_frames;
    std::this_thread::sleep_for(std::chrono::microseconds(options.loop_period_us));
  }

  const auto initialized = initialized_indices(actuators);
  std::cout
    << "  initialized=[" << join_indices(initialized) << "]"
    << " rx_frames=" << rx_frames
    << " total_init=" << initialized.size() << "/" << actuators.size() << "\n";

  if (options.disable_on_exit) {
    disable_all(scheduler, actuators, timeout, retries);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  if (!parse_options(argc, argv, options)) {
    return 2;
  }

  rclcpp::init(argc, argv);
  try {
    for (const int gap_us : options.gaps_us) {
      run_gap_case(options, gap_us);
    }
  } catch (const std::exception & e) {
    std::cerr << "[gap_sweep] exception: " << e.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
