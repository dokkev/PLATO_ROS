// Copyright 2024 Your Organization
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLATO2_HARDWARE_INTERFACE__FT_SENSOR_CAN_HPP_
#define PLATO2_HARDWARE_INTERFACE__FT_SENSOR_CAN_HPP_

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring>
#include <string>
#include <errno.h>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace plato2_hardware_interface
{

class FTSensorCAN {
public:
  // Struct to hold 6-axis force/torque data
  struct Wrench {
    double fx;
    double fy;
    double fz;
    double tx;
    double ty;
    double tz;
    uint64_t timestamp;  // Microseconds since start
  };

  /**
   * @brief Constructor for FTSensorCAN
   * @param interface CAN interface name (default: "can0")
   */
  explicit FTSensorCAN(const std::string& interface = "can0");

  /**
   * @brief Destructor ensures proper cleanup of resources
   */
  ~FTSensorCAN();

  /**
   * @brief Initialize the CAN communication
   * @return true if initialization successful, false otherwise
   */
  bool init();

  /**
   * @brief Get the latest wrench data without blocking
   * @return Wrench struct containing latest force/torque data
   */
  Wrench getLatestWrench();

  /**
   * @brief Get the latest wrench data with age checking
   * @param wrench Reference to store the wrench data
   * @param max_age_us Maximum age of data in microseconds
   * @return true if fresh data available, false if data too old
   */
  bool getLatestWrench(Wrench& wrench, uint64_t max_age_us = 10000);

private:
  // CAN IDs for force and torque messages
  static constexpr canid_t FORCE_MSG_ID = 0x01A;
  static constexpr canid_t TORQUE_MSG_ID = 0x01B;

  int socket_;
  std::string interface_;
  std::thread read_thread_;
  std::atomic<bool> running_;
  std::mutex wrench_mutex_;
  Wrench latest_wrench_;
  bool have_force_;
  bool have_torque_;

  /**
   * @brief Background thread for reading CAN messages
   */
  void readLoop();
};

}  // namespace plato2_hardware_interface

#endif  // PLATO2_HARDWARE_INTERFACE__FT_SENSOR_CAN_HPP_