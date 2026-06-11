// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/control/plato_control_architecture.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace plato_robot_system {
namespace {

constexpr const char* kIdleStateName = "idle";

class ScopedPhaseTimer {
 public:
  ScopedPhaseTimer(bool enabled, double& destination)
      : enabled_(enabled), destination_(destination) {
    if (enabled_) {
      start_ = Clock::now();
    }
  }

  ~ScopedPhaseTimer() {
    if (!enabled_) {
      return;
    }
    destination_ =
        std::chrono::duration<double, std::micro>(Clock::now() - start_)
            .count();
  }

 private:
  using Clock = std::chrono::steady_clock;

  bool enabled_{false};
  double& destination_;
  Clock::time_point start_;
};

void validate_nonnegative_size(int nq, int nv, const char* caller) {
  if (nq < 0) {
    throw std::invalid_argument(std::string(caller) + ": nq must be nonnegative");
  }
  if (nv < 0) {
    throw std::invalid_argument(std::string(caller) + ": nv must be nonnegative");
  }
}

bool IsIdleState(const State* state) {
  return state != nullptr && state->name() == kIdleStateName;
}

}  // namespace

void DriverPdGainsConfig::Resize(int nv) {
  validate_nonnegative_size(0, nv, "DriverPdGainsConfig::Resize");
  kp = Eigen::VectorXd::Zero(nv);
  kd = Eigen::VectorXd::Zero(nv);
}

bool DriverPdGainsConfig::IsValid() const {
  return kp.allFinite() && kd.allFinite();
}

bool DriverPdGainsConfig::HasValidDimensions(int nv) const {
  return nv > 0 && kp.size() == nv && kd.size() == nv;
}

ControlArchitecture::ControlArchitecture(
    ControlArchitectureConfig config)
    : config_(config) {}

ControlArchitecture::ControlArchitecture(
    ControlArchitectureConfig config,
    std::shared_ptr<RobotSystem> robot)
    : config_(config) {
  SetRobot(std::move(robot));
}

void ControlArchitecture::Configure(int nq, int nv) {
  if (nv < 0) {
    nv = nq;
  }
  if (nq <= 0) {
    throw std::invalid_argument(
        "ControlArchitecture::Configure: nq must be positive");
  }
  if (nv <= 0) {
    throw std::invalid_argument(
        "ControlArchitecture::Configure: nv must be positive");
  }

  nq_ = nq;
  nv_ = nv;
  cmd_.Resize(nq_, nv_);
  config_.driver_pd_gains.Resize(nv_);
  cmd_.valid = false;
  control_state_ = ControlState{};
  fsm_handler_.Clear();
  fsm_initialized_ = false;
  requested_state_id_ = -1;
  control_state_.requested_state_id = requested_state_id_;
}

void ControlArchitecture::SetRobot(std::shared_ptr<RobotSystem> robot) {
  if (!robot) {
    throw std::invalid_argument("ControlArchitecture::SetRobot: robot is null");
  }
  robot_ = std::move(robot);
  if (robot_->hasModel()) {
    Configure(robot_->nq(), robot_->nv());
  }
  initialized_ = false;
}

void ControlArchitecture::Initialize() {
  if (initialized_) {
    return;
  }
  if (!IsConfigured()) {
    if (robot_ && robot_->hasModel()) {
      Configure(robot_->nq(), robot_->nv());
    } else {
      throw std::logic_error("ControlArchitecture::Initialize: not configured");
    }
  }
  InitializeStateMachine();
  initialized_ = true;
}

ControlUpdateResult ControlArchitecture::Update(const RobotState& state, double dt) {
  if (robot_) {
    robot_->UpdateState(state);
  }
  if (!initialized_) {
    Initialize();
  }
  if (!robot_ && !IsValid(state)) {
    Fail("input robot state is invalid");
    return {false, control_state_.status};
  }

  Step(dt);
  if (!cmd_.IsUsable()) {
    return {false, control_state_.status};
  }
  return {true, control_state_.status};
}

void ControlArchitecture::SetDriverPdGainsConfig(
    const DriverPdGainsConfig& config) {
  if (!config.IsValid()) {
    throw std::invalid_argument(
        "ControlArchitecture::SetDriverPdGainsConfig: invalid driver PD gains");
  }
  if (IsConfigured() && !config.HasValidDimensions(nv_)) {
    throw std::invalid_argument(
        "ControlArchitecture::SetDriverPdGainsConfig: driver PD gain dimension mismatch");
  }
  config_.driver_pd_gains = config;
}

void ControlArchitecture::RegisterState(std::unique_ptr<State> state) {
  InitializeStateMachine();
  if (!state) {
    throw std::invalid_argument("ControlArchitecture::RegisterState: state is null");
  }
  const StateId id = state->id();
  fsm_handler_.RegisterState(id, std::move(state));
}

bool ControlArchitecture::SetStartState(const StateId id) {
  InitializeStateMachine();
  if (!fsm_handler_.SetStartState(id)) {
    return false;
  }
  requested_state_id_ = id;
  control_state_.current_state_id = id;
  control_state_.requested_state_id = id;
  return true;
}

bool ControlArchitecture::RequestState(const StateId id) {
  InitializeStateMachine();
  if (!fsm_handler_.RequestState(id)) {
    return false;
  }
  requested_state_id_ = id;
  control_state_.requested_state_id = id;
  return true;
}

bool ControlArchitecture::RequestState(const std::string& name) {
  InitializeStateMachine();
  const auto id = fsm_handler_.FindStateIdByName(name);
  if (!id) {
    return false;
  }
  return RequestState(*id);
}

void ControlArchitecture::Step(double dt) {
  if (!initialized_) {
    Initialize();
  }
  const double current_time =
      robot_ && robot_->hasState() ? robot_->state().time_s : 0.0;
  UpdateModelTerms();
  UpdateStateMachine(current_time, dt);
  EvaluateCommand(current_time, dt);
}

void ControlArchitecture::UpdateModelTerms() {
  ScopedPhaseTimer timer(timing_enabled_, timing_stats_.model_us);
  if (robot_ && robot_->hasModel() && robot_->hasState()) {
    robot_->UpdateKinematics();
  }
}

void ControlArchitecture::InitializeStateMachine() {
  if (fsm_initialized_) {
    return;
  }
  fsm_initialized_ = true;
}

void ControlArchitecture::UpdateStateMachine(double current_time, double dt) {
  ScopedPhaseTimer timer(timing_enabled_, timing_stats_.fsm_us);
  InitializeStateMachine();

  const StateId requested_state_id = RequestedStateId();
  if (requested_state_id >= 0) {
    (void)fsm_handler_.RequestState(requested_state_id);
    if (requested_state_id_ == requested_state_id) {
      requested_state_id_ = -1;
      control_state_.requested_state_id = -1;
    }
  }
  fsm_handler_.Update(current_time, dt);
  UpdateControlStateFromFsm();
}

StateId ControlArchitecture::RequestedStateId() const {
  if (requested_state_id_ >= 0) {
    return requested_state_id_;
  }
  if (fsm_handler_.GetCurrentStateId() >= 0) {
    return -1;
  }
  const auto idle_id = fsm_handler_.FindStateIdByName(kIdleStateName);
  return idle_id ? *idle_id : -1;
}

void ControlArchitecture::UpdateControlStateFromFsm() {
  const auto* current_state = fsm_handler_.GetCurrentState();
  if (!current_state) {
    control_state_.current_state_id = -1;
    control_state_.status = "fsm has no active state";
    return;
  }

  control_state_.current_state_id = current_state->id();
  if (IsIdleState(current_state)) {
    control_state_.status = "idle";
    return;
  }

  control_state_.status = "running state: " + current_state->name();
}

void ControlArchitecture::EvaluateCommand(double current_time, double dt) {
  ScopedPhaseTimer timer(timing_enabled_, timing_stats_.command_us);

  if (!robot_ || !robot_->hasState()) {
    Fail("robot state is not available");
    return;
  }

  const RobotState& state = robot_->state();
  if (!IsValid(state)) {
    Fail("robot state is invalid");
    return;
  }

  RobotCommand next_command;
  std::string error;
  std::string fallback_status;
  const auto* active_state = fsm_handler_.GetCurrentState();
  const bool command_populated =
      active_state != nullptr && active_state->PopulateCommand(&next_command);
  if (!command_populated) {
    error = active_state == nullptr
        ? "fsm has no active state"
        : "state '" + active_state->name() + "' did not populate command";
    fallback_status = "Command populate failed; switched to idle: " + error;
  }

  if (fallback_status.empty() && !ValidatePopulatedCommand(next_command, &error)) {
    fallback_status = "Invalid command; switched to idle: " + error;
  }

  const bool switched_to_idle_due_to_command = !fallback_status.empty();
  if (switched_to_idle_due_to_command) {
    if (!SwitchToIdleAndPopulateCommand(current_time, dt, &next_command)) {
      Fail(error + "; failed to switch to idle.");
      return;
    }
    control_state_.status = fallback_status;
  }

  FinalizeCommand(&next_command);
  if (!ValidateFinalCommand(next_command, &error)) {
    Fail(error);
    return;
  }
  if (IsIdleState(fsm_handler_.GetCurrentState()) &&
      !switched_to_idle_due_to_command) {
    control_state_.status = "idle";
  }

  cmd_ = std::move(next_command);
  control_state_.command_valid = true;
}

void ControlArchitecture::FinalizeCommand(RobotCommand* command) const {
  if (command == nullptr) {
    return;
  }

  // Driver-local gains are attached only here. Tasks, states, and planners must
  // not tune or publish RobotCommand.kp/kd directly.
  if (IsIdleState(fsm_handler_.GetCurrentState())) {
    command->kp.setZero();
    command->kd.setZero();
  } else if (config_.driver_pd_gains.HasValidDimensions(nv_)) {
    command->kp = config_.driver_pd_gains.kp;
    command->kd = config_.driver_pd_gains.kd;
  }
  command->valid =
      command->valid && command->HasValidDimensions() && command->AllFinite();
}

void ControlArchitecture::Fail(const std::string& error) {
  if (cmd_.q_cmd.size() != nq_ || cmd_.qdot_cmd.size() != nv_) {
    cmd_.Resize(nq_, nv_);
  }
  cmd_.valid = false;
  control_state_.command_valid = false;
  control_state_.status = error;
}

bool ControlArchitecture::PopulateIdleCommand(double stamp_sec, RobotCommand* command) const {
  if (command == nullptr) {
    return false;
  }
  const auto* active_state = fsm_handler_.GetCurrentState();
  if (!IsIdleState(active_state)) {
    return false;
  }
  if (!active_state->PopulateCommand(command)) {
    return false;
  }
  command->stamp_sec = stamp_sec;
  return true;
}

bool ControlArchitecture::SwitchToIdleAndPopulateCommand(
    double current_time,
    double dt,
    RobotCommand* command) {
  const auto idle_id = fsm_handler_.FindStateIdByName(kIdleStateName);
  if (!idle_id) {
    return false;
  }
  requested_state_id_ = *idle_id;
  control_state_.requested_state_id = *idle_id;
  if (!fsm_handler_.RequestState(*idle_id)) {
    return false;
  }
  fsm_handler_.Update(current_time, dt);
  requested_state_id_ = -1;
  control_state_.requested_state_id = -1;
  UpdateControlStateFromFsm();
  return PopulateIdleCommand(current_time, command);
}

bool ControlArchitecture::ValidatePopulatedCommand(
    const RobotCommand& command,
    std::string* error) const {
  if (command.q_cmd.size() != nq_ || command.qdot_cmd.size() != nv_ ||
      command.tau_cmd.size() != nv_) {
    if (error) {
      *error = "robot command dimension mismatch";
    }
    return false;
  }
  if (!command.valid || !command.q_cmd.allFinite() ||
      !command.qdot_cmd.allFinite() || !command.tau_cmd.allFinite()) {
    if (error) {
      *error = "robot command is invalid or contains non-finite q/qdot/tau values";
    }
    return false;
  }
  return true;
}

bool ControlArchitecture::ValidateFinalCommand(
    const RobotCommand& command,
    std::string* error) const {
  if (command.q_cmd.size() != nq_ || command.qdot_cmd.size() != nv_ ||
      command.tau_cmd.size() != nv_ || command.kp.size() != nv_ ||
      command.kd.size() != nv_) {
    if (error) {
      *error = "robot command dimension mismatch";
    }
    return false;
  }
  if (!command.IsUsable()) {
    if (error) {
      *error = "robot command is invalid or contains non-finite values";
    }
    return false;
  }
  return true;
}

}  // namespace plato_robot_system
