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
  DriverPdGainsConfig driver_pd_gains;
};

struct ControlState {
  StateId current_state_id{-1};
  StateId requested_state_id{-1};
  bool command_valid{false};
  std::string status{"idle"};
};

struct ControlUpdateResult {
  bool ok{false};
  std::string reason;
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
  ControlUpdateResult Update(const RobotState& state, double dt);

  bool IsConfigured() const { return nq_ > 0 && nv_ > 0; }
  bool initialized() const { return initialized_; }
  int nq() const { return nq_; }
  int nv() const { return nv_; }

  RobotSystem* robot() const { return robot_.get(); }
  const ControlArchitectureConfig& config() const { return config_; }
  ControlArchitectureConfig& config() { return config_; }

  const ControlState& control_state() const { return control_state_; }
  StateId current_state_id() const { return control_state_.current_state_id; }
  FSMHandler* fsmHandler() { return &fsm_handler_; }
  const FSMHandler* fsmHandler() const { return &fsm_handler_; }
  const RobotCommand& command() const { return cmd_; }
  const RobotCommand& robot_command() const { return cmd_; }

  void SetDriverPdGainsConfig(const DriverPdGainsConfig& config);
  const DriverPdGainsConfig& driver_pd_gains_config() const {
    return config_.driver_pd_gains;
  }

  void RegisterState(std::unique_ptr<State> state);
  bool SetStartState(StateId id);
  bool RequestState(StateId id);
  bool RequestState(const std::string& name);
  void FinalizeCommand(RobotCommand* command) const;

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
  void EvaluateCommand(double current_time, double dt);

  void Fail(const std::string& error);
  bool PopulateIdleCommand(double stamp_sec, RobotCommand* command) const;
  bool SwitchToIdleAndPopulateCommand(
      double current_time,
      double dt,
      RobotCommand* command);
  bool ValidatePopulatedCommand(
      const RobotCommand& command,
      std::string* error) const;
  bool ValidateFinalCommand(
      const RobotCommand& command,
      std::string* error) const;

  ControlArchitectureConfig config_;
  std::shared_ptr<RobotSystem> robot_;
  bool initialized_{false};
  int nq_{0};
  int nv_{0};

  ControlState control_state_;
  FSMHandler fsm_handler_;
  bool fsm_initialized_{false};
  StateId requested_state_id_{-1};

  RobotCommand cmd_;

  bool timing_enabled_{false};
  ArchTimingStats timing_stats_;
};

using control_architecture = ControlArchitecture;

}  // namespace plato_robot_system
