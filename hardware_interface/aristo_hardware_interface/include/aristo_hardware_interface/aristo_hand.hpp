#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_

#include <array>
#include <chrono>
#include <mutex>
#include <vector>

#include <geometry_msgs/msg/wrench.hpp>

#include "aristo_hardware_interface/actuator.hpp"
#include "aristo_hardware_interface/aristo_model.hpp"
#include "aristo_hardware_interface/aristo_protocol.hpp"
#include "can_hardware_common/can_transport.hpp"
#include "can_hardware_common/core/can_hand_base.hpp"
#include "can_hardware_common/ft_sensor.hpp"
#include "plato_hardware_interface/utils/can_ids.hpp"

namespace aristo_hand
{

class Hand : public can_hardware_common::core::CanHandBase
{
public:
  static constexpr size_t kNumActuators = 8;
  static constexpr size_t kNumFtSensors = 3;

  explicit Hand(std::vector<aristo_actuator::Config> actuator_configs);
  Hand(const Hand &) = delete;
  Hand & operator=(const Hand &) = delete;
  Hand(Hand &&) = delete;
  Hand & operator=(Hand &&) = delete;
  ~Hand() = default;

  bool write_joint_commands() { return write(); }

  size_t get_num_actuators() const { return kNumActuators; }
  size_t get_num_ft_sensors() const { return ft_sensors_.size(); }
  void update_ft_sensor_states(std::vector<geometry_msgs::msg::Wrench> & ft_sensor_states);

private:
  using LifecyclePlan = can_hardware_common::core::LifecyclePlan;
  using SensorStatus = can_hardware_common::core::StateSnapshot::SensorStatus;
  using WritePlan = can_hardware_common::core::WritePlan;

  static constexpr size_t kThumbRollIndex = 0;
  static constexpr size_t kThumbYawIndex = 1;
  static constexpr size_t kThumbMcpIndex = 2;
  static constexpr size_t kThumbPipIndex = 3;
  static constexpr size_t kIndexMcpIndex = 4;
  static constexpr size_t kIndexPipIndex = 5;
  static constexpr size_t kMiddleMcpIndex = 6;
  static constexpr size_t kMiddlePipIndex = 7;
  static constexpr std::chrono::microseconds kDirectTxInterFrameGap{100};
  static constexpr std::chrono::microseconds kDirectTxFrameTimeout{2000};
  static constexpr std::chrono::milliseconds kRxStaleTimeout{20};
  static constexpr std::chrono::milliseconds kFtSensorFreshnessTimeout{200};

  bool update_measurements_() override;
  void refresh_state_snapshot_() override;
  void build_ready_write_plan_(WritePlan & plan) override;
  bool execute_write_plan_(const WritePlan & plan) override;
  LifecyclePlan build_lifecycle_plan_(can_hardware_common::core::LifecycleOperation operation) override;
  bool execute_lifecycle_plan_(const LifecyclePlan & plan) override;
  bool execute_standard_lifecycle_(const LifecyclePlan & plan);
  bool execute_direct_frames_(
    const std::vector<TPCANMsg> & frames,
    std::chrono::microseconds timeout);
  can_hardware_common::CanTransport::RxResult poll_can_bus();
  TPCANStatus enable_all_actuators();
  TPCANStatus disable_all_actuators();
  TPCANStatus set_current_position_as_zero();
  TPCANStatus set_default_can_limits();
  TPCANStatus send_frame_blocking_(const TPCANMsg & frame, std::chrono::microseconds timeout);
  TPCANStatus send_command_(const actuator::TxCommand & command);
  void update_ft_sensor_wrenches_(std::vector<geometry_msgs::msg::Wrench> & ft_sensor_states);
  void print_hardware_info_(const char * actuator_total_label = "Total Actuators") const;
  void mark_rx_frame_();
  bool has_fresh_rx_(std::chrono::steady_clock::time_point now) const;
  SensorStatus summarize_ft_sensor_status_(sensor::FTSensor::SteadyClock::time_point now) const;
  void print_actuator_info_() const;

  can_hardware_common::CanTransport transport_;
  mutable std::mutex state_mutex_;
  AristoProtocol protocol_;
  AristoModel model_;
  std::vector<aristo_actuator::Actuator> actuators_;
  std::vector<sensor::FTSensor> ft_sensors_;
  std::vector<aristo_actuator::Config> actuator_configs_;
  can_hardware_common::CanTransport::RxResult last_rx_result_{};
  std::chrono::steady_clock::time_point last_rx_time_{};
  bool has_observed_rx_ = false;

  std::vector<sensor::Config> ft_sensor_configs_ = {
    {FTSensorID::THUMB_FORCE, FTSensorID::THUMB_TORQUE},
    {FTSensorID::INDEX_FORCE, FTSensorID::INDEX_TORQUE},
    {FTSensorID::MIDDLE_FORCE, FTSensorID::MIDDLE_TORQUE},
  };
};

}  // namespace aristo_hand

#endif  // ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_
