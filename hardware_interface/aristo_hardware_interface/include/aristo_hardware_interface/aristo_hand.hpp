#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_

#include <array>
#include <mutex>
#include <vector>

#include <geometry_msgs/msg/wrench.hpp>

#include "aristo_hardware_interface/actuator.hpp"
#include "can_hardware_common/can_bus_manager.hpp"
#include "can_hardware_common/ft_sensor.hpp"
#include "can_hardware_common/robot.hpp"
#include "plato_hardware_interface/utils/can_ids.hpp"

namespace aristo_hand
{

class Hand : public can_hardware_common::RobotIO
{
public:
  static constexpr size_t kNumActuators = 8;
  static constexpr size_t kNumFtSensors = 3;

  Hand();
  ~Hand() = default;

  void enable();
  void disable();
  void set_current_position_as_zero();
  void set_default_can_limits();

  size_t get_num_actuators() const { return kNumActuators; }
  size_t get_num_ft_sensors() const { return ft_sensors_.size(); }
  can_hardware_common::CanBusManager::PollResult poll_can_bus();
  TPCANStatus write_joint_commands();
  void read_joint_states();
  void update_ft_sensor_states(std::vector<geometry_msgs::msg::Wrench> & ft_sensor_states);

private:
  enum class DispatchTargetKind
  {
    kActuator,
    kForceSensor,
    kTorqueSensor,
  };

  struct RxDispatchEntry
  {
    uint32_t rx_id = 0;
    DispatchTargetKind target_kind = DispatchTargetKind::kActuator;
    size_t target_index = 0;
  };

  static constexpr size_t kThumbRollIndex = 0;
  static constexpr size_t kThumbYawIndex = 1;
  static constexpr size_t kThumbMcpIndex = 2;
  static constexpr size_t kThumbPipIndex = 3;
  static constexpr size_t kIndexMcpIndex = 4;
  static constexpr size_t kIndexPipIndex = 5;
  static constexpr size_t kMiddleMcpIndex = 6;
  static constexpr size_t kMiddlePipIndex = 7;

  void enable_all_actuators();
  void disable_all_actuators();
  TPCANStatus send_command_(const actuator::TxCommand & command);
  void update_ft_sensor_wrenches_(std::vector<geometry_msgs::msg::Wrench> & ft_sensor_states);
  void print_hardware_info_(const char * actuator_total_label = "Total Actuators") const;
  void initialize_rx_dispatch_table_();
  static void dispatch_rx_frame_static_(void * context, const TPCANMsg & frame);
  void dispatch_rx_frame_(const TPCANMsg & frame);
  void print_actuator_info_() const;

  can_hardware_common::CanBusManager can_bus_manager_;
  mutable std::mutex state_mutex_;
  std::vector<aristo_actuator::Actuator> actuators_;
  std::vector<sensor::FTSensor> ft_sensors_;
  std::array<RxDispatchEntry, kNumActuators + (2 * kNumFtSensors)> rx_dispatch_table_{};

  std::vector<aristo_actuator::Config> actuator_configs_ = {
    aristo_actuator::ActuatorConfigFactory::create_thumb_roll_config(),
    aristo_actuator::ActuatorConfigFactory::create_thumb_yaw_config(),
    aristo_actuator::ActuatorConfigFactory::create_mcp_config(3),
    aristo_actuator::ActuatorConfigFactory::create_pip_config(4),
    aristo_actuator::ActuatorConfigFactory::create_mcp_config(5),
    aristo_actuator::ActuatorConfigFactory::create_pip_config(6),
    aristo_actuator::ActuatorConfigFactory::create_mcp_config(7),
    aristo_actuator::ActuatorConfigFactory::create_pip_config(8),
  };

  std::vector<sensor::Config> ft_sensor_configs_ = {
    {FTSensorID::THUMB_FORCE, FTSensorID::THUMB_TORQUE},
    {FTSensorID::INDEX_FORCE, FTSensorID::INDEX_TORQUE},
    {FTSensorID::MIDDLE_FORCE, FTSensorID::MIDDLE_TORQUE},
  };
};

}  // namespace aristo_hand

#endif  // ARISTO_HARDWARE_INTERFACE__ARISTO_HAND_HPP_
