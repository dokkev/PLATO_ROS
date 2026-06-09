// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>

#include "plato_robot_system/control/state_machine/fsm_handler.hpp"
#include "plato_robot_system/robot/robot_system.hpp"

namespace plato_robot_system {

enum class ControlMode {
  kIdle = 10000,
  kHold = 10001,
  kJointImpedance = 10002,
  kFault = 10003,
};

const char* ControlModeName(ControlMode mode);

struct ArchTimingStats {
  double model_us{0.0};
  double fsm_us{0.0};
  double command_us{0.0};
  double output_us{0.0};
};

struct DriverPdGainsConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::VectorXd kp;
  Eigen::VectorXd kd;

  void Resize(int nv);
  bool IsValid() const;
  bool HasValidDimensions(int nv) const;
};

struct ControlArchitectureConfig {
  bool compute_impedance_torque{false};
  DriverPdGainsConfig driver_pd_gains;
};

struct ImpedanceSetpoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::VectorXd q_cmd;
  Eigen::VectorXd qdot_cmd;
  Eigen::VectorXd tau_ff_cmd;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;

  void Resize(int nq, int nv = -1);
  bool HasValidDimensions() const;
  bool AllFinite() const;
};

struct ControlState {
  ControlMode mode{ControlMode::kIdle};
  ControlMode requested_mode{ControlMode::kHold};
  bool command_valid{false};
  bool faulted{false};
  std::string status{"idle"};
};

// ControlArchitecture owns the per-tick control orchestration.
//
// It follows the rpc/control_architecture shape:
//   Update(state, dt)
//     -> Step(dt)
//       -> UpdateModelTerms()
//       -> UpdateStateMachine()
//       -> EvaluateCommand()
//
// The current implementation emits RobotCommand directly.
class ControlArchitecture {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ControlArchitecture() = default;
  explicit ControlArchitecture(ControlArchitectureConfig config);
  ControlArchitecture(
      ControlArchitectureConfig config,
      std::shared_ptr<RobotSystem> robot);

  void Configure(int nq, int nv = -1);
  void SetRobot(std::shared_ptr<RobotSystem> robot);
  void Initialize();
  void Update(const RobotState& state, double dt);

  bool IsConfigured() const { return nq_ > 0 && nv_ > 0; }
  bool initialized() const { return initialized_; }
  int nq() const { return nq_; }
  int nv() const { return nv_; }

  RobotSystem* robot() const { return robot_.get(); }
  const ControlArchitectureConfig& config() const { return config_; }
  ControlArchitectureConfig& config() { return config_; }

  const ControlState& control_state() const { return control_state_; }
  ControlMode mode() const { return control_state_.mode; }
  FSMHandler* fsmHandler() { return &fsm_handler_; }
  const FSMHandler* fsmHandler() const { return &fsm_handler_; }
  const RobotCommand& command() const { return cmd_; }
  const RobotCommand& robot_command() const { return cmd_; }

  void SetComputeImpedanceTorque(bool enabled);
  bool compute_impedance_torque() const { return config_.compute_impedance_torque; }
  void SetDriverPdGainsConfig(const DriverPdGainsConfig& config);
  const DriverPdGainsConfig& driver_pd_gains_config() const {
    return config_.driver_pd_gains;
  }

  void SetImpedanceSetpoint(const ImpedanceSetpoint& setpoint);
  ImpedanceSetpoint ClampImpedanceSetpoint(const ImpedanceSetpoint& setpoint) const;
  void ClearImpedanceSetpoint();
  void RegisterState(std::unique_ptr<State> state);
  bool SetStartState(StateId id);
  bool RequestState(StateId id);
  bool RequestState(const std::string& name);
  void RequestMode(ControlMode mode);
  void RequestFault(const std::string& reason);
  void ClearFault();

  ImpedanceSetpoint MakeHoldSetpoint(
      const Eigen::Ref<const Eigen::VectorXd>& q_current) const;
  ImpedanceSetpoint MakeZeroSetpoint() const;

  void setTimingEnabled(bool enabled) { timing_enabled_ = enabled; }
  bool timingEnabled() const { return timing_enabled_; }
  const ArchTimingStats& timingStats() const { return timing_stats_; }

 private:
  void Step(double dt);
  void UpdateModelTerms();
  void InitializeStateMachine();
  void UpdateStateMachine(double current_time, double dt);
  StateId RequestedStateId() const;
  void UpdateControlStateFromFsm();
  void EvaluateCommand();
  void InitializeCommandFromRobotState();

  void Fail(const std::string& error);
  void ApplyDriverPdGains(RobotCommand* command) const;
  bool CheckSetpoint(
      const ImpedanceSetpoint& setpoint,
      const RobotState& state,
      std::string* error) const;

  ControlArchitectureConfig config_;
  std::shared_ptr<RobotSystem> robot_;
  bool initialized_{false};
  int nq_{0};
  int nv_{0};

  ControlState control_state_;
  FSMHandler fsm_handler_;
  bool fsm_initialized_{false};
  StateId requested_state_id_{static_cast<StateId>(ControlMode::kHold)};
  ImpedanceSetpoint pending_impedance_;
  bool has_pending_impedance_{false};

  RobotCommand cmd_;
  bool command_initialized_{false};

  bool timing_enabled_{false};
  ArchTimingStats timing_stats_;
};

using control_architecture = ControlArchitecture;

}  // namespace plato_robot_system
