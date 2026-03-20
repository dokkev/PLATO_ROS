#ifndef PLATO_HARDWARE_INTERFACE__PLATO_HAND_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO_HAND_HPP_

#include <array>
#include <mutex>
#include <vector>

#include "can_hardware_common/can_bus_manager.hpp"
#include "can_hardware_common/robot.hpp"
#include "plato_hardware_interface/actuator.hpp"
#include "plato_hardware_interface/five_bar_linkage.hpp"
#include "plato_hardware_interface/plato_hand_config.hpp"

namespace plato_hand
{

class Hand : public can_hardware_common::RobotIO
{
public:
  static constexpr size_t kNumJoints = 8;
  static constexpr size_t kNumActuators = 8;

  explicit Hand(PlatoHandConfig config);
  Hand(const Hand &) = delete;
  Hand & operator=(const Hand &) = delete;
  Hand(Hand &&) = delete;
  Hand & operator=(Hand &&) = delete;
  ~Hand() = default;

  void enable(bool automatic_zeroing = false);
  void disable();

  size_t get_num_actuators() const { return kNumActuators; }
  bool read();
  bool write_joint_commands();

  void print_motor_positions();

private:
  enum class ZeroingResult
  {
    kFailed,
    kRuntimeOnly,
    kRuntimeAndPersisted,
  };

  struct RxDispatchEntry
  {
    uint32_t rx_id = 0;
    size_t actuator_index = 0;
  };

  static constexpr size_t kThumbRollIndex = 0;
  static constexpr size_t kThumbYawIndex = 1;
  static constexpr size_t kThumbMcpIndex = 2;
  static constexpr size_t kThumbPipIndex = 3;
  static constexpr size_t kIndexMcpIndex = 4;
  static constexpr size_t kIndexPipIndex = 5;
  static constexpr size_t kMiddleMcpIndex = 6;
  static constexpr size_t kMiddlePipIndex = 7;

  bool enable_all_actuators();
  bool disable_all_actuators();
  TPCANStatus send_command_(const actuator::TxCommand & command);
  void read_joint_states_();
  bool has_zeroing_feedback_() const;
  void request_feedback_probe_();
  ZeroingResult set_current_position_as_zero_(bool persist_offsets = true);
  void print_hardware_info_(const char * actuator_total_label = "Total Actuators") const;
  void initialize_rx_dispatch_table_();
  static void dispatch_rx_frame_static_(void * context, const TPCANMsg & frame);
  void dispatch_rx_frame_(const TPCANMsg & frame);
  void print_actuator_info_() const;

  can_hardware_common::CanBusManager can_bus_manager_;
  mutable std::mutex state_mutex_;
  std::vector<plato_actuator::Actuator> actuators_;
  std::array<RxDispatchEntry, kNumActuators> rx_dispatch_table_{};

  FiveBarLinkage::Transmission transmission_;

  std::string actuator_offset_yaml_path_;
  std::vector<plato_actuator::Config> actuator_configs_;
  size_t consecutive_read_failures_ = 0;
  bool zeroing_pending_ = false;
  size_t zeroing_probe_cooldown_cycles_ = 0;
};

}  // namespace plato_hand

#endif  // PLATO_HARDWARE_INTERFACE__PLATO_HAND_HPP_
