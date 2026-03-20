#ifndef CAN_HARDWARE_COMMON__ROBOT_HPP_
#define CAN_HARDWARE_COMMON__ROBOT_HPP_

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

namespace can_hardware_common
{

class RobotIO
{
public:
  struct Command
  {
    static constexpr double kInvalidValue = std::numeric_limits<double>::quiet_NaN();

    struct ConstView
    {
      explicit ConstView(const Command & command)
      : size_(command.size()),
        position(command.position_data(), static_cast<Eigen::Index>(size_)),
        velocity(command.velocity_data(), static_cast<Eigen::Index>(size_)),
        effort(command.effort_data(), static_cast<Eigen::Index>(size_)),
        stiffness(command.stiffness_data(), static_cast<Eigen::Index>(size_)),
        damping(command.damping_data(), static_cast<Eigen::Index>(size_))
      {
        command.validate_sizes();
      }

      bool all_finite() const
      {
        return size_ > 0 &&
               position.allFinite() &&
               velocity.allFinite() &&
               effort.allFinite() &&
               stiffness.allFinite() &&
               damping.allFinite();
      }

      size_t size() const { return size_; }

    private:
      size_t size_;

    public:
      Eigen::Map<const Eigen::ArrayXd> position;
      Eigen::Map<const Eigen::ArrayXd> velocity;
      Eigen::Map<const Eigen::ArrayXd> effort;
      Eigen::Map<const Eigen::ArrayXd> stiffness;
      Eigen::Map<const Eigen::ArrayXd> damping;
    };

    struct PreviousConstView
    {
      explicit PreviousConstView(const Command & command)
      : size_(command.previous_size()),
        position(
          command.previous_position_data(),
          static_cast<Eigen::Index>(size_)),
        velocity(
          command.previous_velocity_data(),
          static_cast<Eigen::Index>(size_)),
        effort(
          command.previous_effort_data(),
          static_cast<Eigen::Index>(size_))
      {
        command.validate_previous_sizes();
      }

      bool all_finite() const
      {
        return size_ > 0 &&
               position.allFinite() &&
               velocity.allFinite() &&
               effort.allFinite();
      }

      size_t size() const { return size_; }

    private:
      size_t size_;

    public:
      Eigen::Map<const Eigen::ArrayXd> position;
      Eigen::Map<const Eigen::ArrayXd> velocity;
      Eigen::Map<const Eigen::ArrayXd> effort;
    };

    void assign(size_t size, double value = 0.0)
    {
      position_.assign(size, value);
      velocity_.assign(size, value);
      effort_.assign(size, value);
      stiffness_.assign(size, value);
      damping_.assign(size, value);
      invalidate_previous(size);
    }

    void fill(double value = 0.0)
    {
      validate_sizes();
      std::fill(position_.begin(), position_.end(), value);
      std::fill(velocity_.begin(), velocity_.end(), value);
      std::fill(effort_.begin(), effort_.end(), value);
      std::fill(stiffness_.begin(), stiffness_.end(), value);
      std::fill(damping_.begin(), damping_.end(), value);
      invalidate_previous();
    }

    void capture_previous()
    {
      validate_sizes();
      previous_position_ = position_;
      previous_velocity_ = velocity_;
      previous_effort_ = effort_;
      previous_valid_ = true;
    }

    void invalidate_previous()
    {
      std::fill(previous_position_.begin(), previous_position_.end(), kInvalidValue);
      std::fill(previous_velocity_.begin(), previous_velocity_.end(), kInvalidValue);
      std::fill(previous_effort_.begin(), previous_effort_.end(), kInvalidValue);
      previous_valid_ = false;
    }

    void invalidate_previous(size_t size)
    {
      previous_position_.assign(size, kInvalidValue);
      previous_velocity_.assign(size, kInvalidValue);
      previous_effort_.assign(size, kInvalidValue);
      previous_valid_ = false;
    }

    bool sizes_consistent() const
    {
      return position_.size() == velocity_.size() &&
             position_.size() == effort_.size() &&
             position_.size() == stiffness_.size() &&
             position_.size() == damping_.size();
    }

    bool previous_sizes_consistent() const
    {
      return previous_position_.size() == previous_velocity_.size() &&
             previous_position_.size() == previous_effort_.size();
    }

    void validate_sizes() const
    {
      if (!sizes_consistent()) {
        throw std::logic_error("RobotIO::Command vectors have inconsistent sizes");
      }
    }

    void validate_previous_sizes() const
    {
      if (!previous_valid_) {
        throw std::logic_error("RobotIO::Command previous command history is not valid");
      }
      if (!previous_sizes_consistent()) {
        throw std::logic_error("RobotIO::Command previous vectors have inconsistent sizes");
      }
    }

    size_t size() const
    {
      validate_sizes();
      return position_.size();
    }

    size_t previous_size() const
    {
      validate_previous_sizes();
      return previous_position_.size();
    }

    bool has_previous() const { return previous_valid_; }
    // These mapped views alias the internal storage. Use them ephemerally and do not keep them
    // across assign/fill/reset operations that may reinitialize the underlying buffers.
    ConstView const_view() const { return ConstView(*this); }
    PreviousConstView previous_const_view() const { return PreviousConstView(*this); }

    double & position_at(size_t index) { return position_.at(index); }
    const double & position_at(size_t index) const { return position_.at(index); }
    double & velocity_at(size_t index) { return velocity_.at(index); }
    const double & velocity_at(size_t index) const { return velocity_.at(index); }
    double & effort_at(size_t index) { return effort_.at(index); }
    const double & effort_at(size_t index) const { return effort_.at(index); }
    double & stiffness_at(size_t index) { return stiffness_.at(index); }
    const double & stiffness_at(size_t index) const { return stiffness_.at(index); }
    double & damping_at(size_t index) { return damping_.at(index); }
    const double & damping_at(size_t index) const { return damping_.at(index); }

    double * position_data() { return position_.data(); }
    const double * position_data() const { return position_.data(); }
    double * velocity_data() { return velocity_.data(); }
    const double * velocity_data() const { return velocity_.data(); }
    double * effort_data() { return effort_.data(); }
    const double * effort_data() const { return effort_.data(); }
    double * stiffness_data() { return stiffness_.data(); }
    const double * stiffness_data() const { return stiffness_.data(); }
    double * damping_data() { return damping_.data(); }
    const double * damping_data() const { return damping_.data(); }

    const double * previous_position_data() const { return previous_position_.data(); }
    const double * previous_velocity_data() const { return previous_velocity_.data(); }
    const double * previous_effort_data() const { return previous_effort_.data(); }

  private:
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> effort_;
    std::vector<double> stiffness_;
    std::vector<double> damping_;
    std::vector<double> previous_position_;
    std::vector<double> previous_velocity_;
    std::vector<double> previous_effort_;
    bool previous_valid_ = false;
  };

  struct State
  {
    struct ConstView
    {
      explicit ConstView(const State & state)
      : size_(state.size()),
        position(state.position_data(), static_cast<Eigen::Index>(size_)),
        velocity(state.velocity_data(), static_cast<Eigen::Index>(size_)),
        effort(state.effort_data(), static_cast<Eigen::Index>(size_))
      {
        state.validate_sizes();
      }

      bool all_finite() const
      {
        return size_ > 0 &&
               position.allFinite() &&
               velocity.allFinite() &&
               effort.allFinite();
      }

      size_t size() const { return size_; }

    private:
      size_t size_;

    public:
      Eigen::Map<const Eigen::ArrayXd> position;
      Eigen::Map<const Eigen::ArrayXd> velocity;
      Eigen::Map<const Eigen::ArrayXd> effort;
    };

    struct View
    {
      explicit View(State & state)
      : size_(state.size()),
        position(state.position_data(), static_cast<Eigen::Index>(size_)),
        velocity(state.velocity_data(), static_cast<Eigen::Index>(size_)),
        effort(state.effort_data(), static_cast<Eigen::Index>(size_))
      {
        state.validate_sizes();
      }

      bool all_finite() const
      {
        return size_ > 0 &&
               position.allFinite() &&
               velocity.allFinite() &&
               effort.allFinite();
      }

      size_t size() const { return size_; }

    private:
      size_t size_;

    public:
      Eigen::Map<Eigen::ArrayXd> position;
      Eigen::Map<Eigen::ArrayXd> velocity;
      Eigen::Map<Eigen::ArrayXd> effort;
    };

    void assign(
      size_t size,
      double value = std::numeric_limits<double>::quiet_NaN())
    {
      position_.assign(size, value);
      velocity_.assign(size, value);
      effort_.assign(size, value);
    }

    void fill(double value)
    {
      validate_sizes();
      std::fill(position_.begin(), position_.end(), value);
      std::fill(velocity_.begin(), velocity_.end(), value);
      std::fill(effort_.begin(), effort_.end(), value);
    }

    bool sizes_consistent() const
    {
      return position_.size() == velocity_.size() &&
             position_.size() == effort_.size();
    }

    void validate_sizes() const
    {
      if (!sizes_consistent()) {
        throw std::logic_error("RobotIO::State vectors have inconsistent sizes");
      }
    }

    size_t size() const
    {
      validate_sizes();
      return position_.size();
    }

    // These mapped views alias the internal storage. Use them ephemerally and do not keep them
    // across assign/fill/reset operations that may reinitialize the underlying buffers.
    ConstView const_view() const { return ConstView(*this); }
    View view() { return View(*this); }

    double & position_at(size_t index) { return position_.at(index); }
    const double & position_at(size_t index) const { return position_.at(index); }
    double & velocity_at(size_t index) { return velocity_.at(index); }
    const double & velocity_at(size_t index) const { return velocity_.at(index); }
    double & effort_at(size_t index) { return effort_.at(index); }
    const double & effort_at(size_t index) const { return effort_.at(index); }

    double * position_data() { return position_.data(); }
    const double * position_data() const { return position_.data(); }
    double * velocity_data() { return velocity_.data(); }
    const double * velocity_data() const { return velocity_.data(); }
    double * effort_data() { return effort_.data(); }
    const double * effort_data() const { return effort_.data(); }

  private:
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> effort_;
  };

  struct JointCommand : Command {};
  struct JointState : State {};
  struct ActuatorCommand : Command {};
  struct ActuatorState : State {};

  RobotIO() = default;
  virtual ~RobotIO() = default;

  void initialize_joint_buffers(
    size_t num_joints,
    double state_initial_value = std::numeric_limits<double>::quiet_NaN())
  {
    joint_commands_.assign(num_joints, 0.0);
    joint_states_.assign(num_joints, state_initial_value);
  }

  void initialize_actuator_buffers(
    size_t num_actuators,
    double state_initial_value = std::numeric_limits<double>::quiet_NaN())
  {
    actuator_commands_.assign(num_actuators, 0.0);
    actuator_states_.assign(num_actuators, state_initial_value);
  }

  void reset_joint_commands(double value = 0.0)
  {
    joint_commands_.fill(value);
  }

  void reset_actuator_commands(double value = 0.0)
  {
    actuator_commands_.fill(value);
  }

  void capture_joint_commands()
  {
    joint_commands_.capture_previous();
  }

  void invalidate_joint_command_history()
  {
    joint_commands_.invalidate_previous();
  }

  void capture_actuator_commands()
  {
    actuator_commands_.capture_previous();
  }

  void invalidate_actuator_command_history()
  {
    actuator_commands_.invalidate_previous();
  }

  size_t get_num_joints() const { return joint_states_.size(); }
  size_t get_num_actuators() const { return actuator_states_.size(); }

  JointCommand & joint_commands() { return joint_commands_; }
  const JointCommand & joint_commands() const { return joint_commands_; }
  bool has_previous_joint_command() const { return joint_commands_.has_previous(); }
  JointCommand::ConstView joint_command_view() const { return joint_commands_.const_view(); }
  JointCommand::PreviousConstView previous_joint_command_view() const
  {
    return joint_commands_.previous_const_view();
  }

  JointState & joint_states() { return joint_states_; }
  const JointState & joint_states() const { return joint_states_; }
  JointState::View joint_state_view() { return joint_states_.view(); }
  JointState::ConstView joint_state_view() const { return joint_states_.const_view(); }

  ActuatorCommand & actuator_commands() { return actuator_commands_; }
  const ActuatorCommand & actuator_commands() const { return actuator_commands_; }
  bool has_previous_actuator_command() const { return actuator_commands_.has_previous(); }
  ActuatorCommand::ConstView actuator_command_view() const { return actuator_commands_.const_view(); }
  ActuatorCommand::PreviousConstView previous_actuator_command_view() const
  {
    return actuator_commands_.previous_const_view();
  }

  ActuatorState & actuator_states() { return actuator_states_; }
  const ActuatorState & actuator_states() const { return actuator_states_; }
  ActuatorState::View actuator_state_view() { return actuator_states_.view(); }
  ActuatorState::ConstView actuator_state_view() const { return actuator_states_.const_view(); }

protected:
  JointCommand joint_commands_;
  JointState joint_states_;
  ActuatorCommand actuator_commands_;
  ActuatorState actuator_states_;
};

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__ROBOT_HPP_
