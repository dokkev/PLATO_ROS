// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/control/plato_control_architecture.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace plato_robot_system {
namespace {

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

StateId StateIdFromMode(ControlMode mode) {
  return static_cast<StateId>(mode);
}

class ControlModeState final : public State {
 public:
  ControlModeState(ControlMode mode, std::string name, std::string status)
      : State(StateIdFromMode(mode), std::move(name)),
        mode_(mode),
        status_(std::move(status)) {}

  ControlMode mode() const { return mode_; }
  const std::string& status() const { return status_; }

 private:
  ControlMode mode_;
  std::string status_;
};

const ControlModeState* AsControlModeState(const State* state) {
  return dynamic_cast<const ControlModeState*>(state);
}

}  // namespace

const char* ControlModeName(ControlMode mode) {
  switch (mode) {
    case ControlMode::kIdle:
      return "idle";
    case ControlMode::kHold:
      return "hold";
    case ControlMode::kJointImpedance:
      return "joint_impedance";
    case ControlMode::kFault:
      return "fault";
  }
  return "unknown";
}

void ImpedanceSetpoint::Resize(int nq, int nv) {
  if (nv < 0) {
    nv = nq;
  }
  validate_nonnegative_size(nq, nv, "ImpedanceSetpoint::Resize");

  q_cmd = Eigen::VectorXd::Zero(nq);
  qdot_cmd = Eigen::VectorXd::Zero(nv);
  tau_ff_cmd = Eigen::VectorXd::Zero(nv);
  kp = Eigen::VectorXd::Zero(nv);
  kd = Eigen::VectorXd::Zero(nv);
}

bool ImpedanceSetpoint::HasValidDimensions() const {
  const Eigen::Index nq = q_cmd.size();
  const Eigen::Index nv = qdot_cmd.size();
  return nq > 0 && nv > 0 && tau_ff_cmd.size() == nv && kp.size() == nv &&
         kd.size() == nv;
}

bool ImpedanceSetpoint::AllFinite() const {
  return q_cmd.allFinite() && qdot_cmd.allFinite() && tau_ff_cmd.allFinite() &&
         kp.allFinite() && kd.allFinite();
}

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
  pending_impedance_.Resize(nq_, nv_);
  cmd_.Resize(nq_, nv_);
  config_.driver_pd_gains.Resize(nv_);
  cmd_.valid = false;
  control_state_ = ControlState{};
  fsm_handler_.Clear();
  fsm_initialized_ = false;
  requested_state_id_ = StateIdFromMode(ControlMode::kHold);
  command_initialized_ = false;
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

void ControlArchitecture::Update(const RobotState& state, double dt) {
  if (robot_) {
    robot_->UpdateState(state);
  }
  if (!initialized_) {
    Initialize();
  }
  if (!command_initialized_) {
    InitializeCommandFromRobotState();
  }
  if (!robot_ && !IsValid(state)) {
    Fail("input robot state is invalid");
    return;
  }

  Step(dt);
}

void ControlArchitecture::SetComputeImpedanceTorque(bool enabled) {
  config_.compute_impedance_torque = enabled;
}

void ControlArchitecture::SetDriverPdGainsConfig(
    const DriverPdGainsConfig& config) {
  if (!config.IsValid()) {
    throw std::invalid_argument(
        "ControlArchitecture::SetDriverPdGainsConfig: invalid driver PD gains");
  }
  config_.driver_pd_gains = config;
}

void ControlArchitecture::SetImpedanceSetpoint(
    const ImpedanceSetpoint& setpoint) {
  std::string error;
  RobotState state;
  if (robot_ && robot_->hasState()) {
    state = robot_->state();
  } else {
    state.q = Eigen::VectorXd::Zero(nq_);
    state.qdot = Eigen::VectorXd::Zero(nv_);
    state.tau = Eigen::VectorXd::Zero(nv_);
    state.valid = true;
  }
  if (!CheckSetpoint(setpoint, state, &error)) {
    has_pending_impedance_ = false;
    control_state_.command_valid = false;
    control_state_.status = error;
    return;
  }
  pending_impedance_ = ClampImpedanceSetpoint(setpoint);
  has_pending_impedance_ = true;
  control_state_.command_valid = true;
}

ImpedanceSetpoint ControlArchitecture::ClampImpedanceSetpoint(
    const ImpedanceSetpoint& setpoint) const {
  return setpoint;
}

void ControlArchitecture::ClearImpedanceSetpoint() {
  has_pending_impedance_ = false;
  control_state_.command_valid = false;
}

void ControlArchitecture::RegisterState(std::unique_ptr<State> state) {
  InitializeStateMachine();
  const StateId id = state ? state->id() : -1;
  fsm_handler_.RegisterState(id, std::move(state));
}

bool ControlArchitecture::SetStartState(const StateId id) {
  InitializeStateMachine();
  if (!fsm_handler_.SetStartState(id)) {
    return false;
  }
  requested_state_id_ = id;
  return true;
}

bool ControlArchitecture::RequestState(const StateId id) {
  InitializeStateMachine();
  if (!fsm_handler_.RequestState(id)) {
    return false;
  }
  requested_state_id_ = id;
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

void ControlArchitecture::RequestMode(ControlMode mode) {
  control_state_.requested_mode = mode;
  requested_state_id_ = StateIdFromMode(mode);
  (void)RequestState(requested_state_id_);
}

void ControlArchitecture::RequestFault(const std::string& reason) {
  control_state_.mode = ControlMode::kFault;
  control_state_.requested_mode = ControlMode::kFault;
  control_state_.faulted = true;
  control_state_.status = reason.empty() ? "fault requested" : reason;
}

void ControlArchitecture::ClearFault() {
  if (control_state_.mode == ControlMode::kFault) {
    control_state_ = ControlState{};
  }
}

ImpedanceSetpoint ControlArchitecture::MakeHoldSetpoint(
    const Eigen::Ref<const Eigen::VectorXd>& q_current) const {
  if (!IsConfigured()) {
    throw std::logic_error(
        "ControlArchitecture::MakeHoldSetpoint: not configured");
  }
  if (q_current.size() != nq_ || !q_current.allFinite()) {
    throw std::invalid_argument(
        "ControlArchitecture::MakeHoldSetpoint: invalid q_current");
  }

  ImpedanceSetpoint setpoint;
  setpoint.Resize(nq_, nv_);
  setpoint.q_cmd = q_current;
  return setpoint;
}

ImpedanceSetpoint ControlArchitecture::MakeZeroSetpoint() const {
  if (!IsConfigured()) {
    throw std::logic_error(
        "ControlArchitecture::MakeZeroSetpoint: not configured");
  }

  ImpedanceSetpoint setpoint;
  setpoint.Resize(nq_, nv_);
  return setpoint;
}

void ControlArchitecture::Step(double dt) {
  if (!initialized_) {
    Initialize();
  }
  const double current_time =
      robot_ && robot_->hasState() ? robot_->state().time_s : 0.0;
  UpdateModelTerms();
  UpdateStateMachine(current_time, dt);
  EvaluateCommand();
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

  StateLifecycle stay_here;
  stay_here.stay_here = true;

  auto idle = std::make_unique<ControlModeState>(
      ControlMode::kIdle, "idle", "idle request uses hold output");
  idle->ConfigureLifecycle(stay_here);
  fsm_handler_.RegisterState(StateIdFromMode(ControlMode::kIdle), std::move(idle));

  auto hold = std::make_unique<ControlModeState>(ControlMode::kHold, "hold", "hold");
  hold->ConfigureLifecycle(stay_here);
  fsm_handler_.RegisterState(StateIdFromMode(ControlMode::kHold), std::move(hold));

  auto joint_impedance = std::make_unique<ControlModeState>(
      ControlMode::kJointImpedance, "joint_impedance", "running joint impedance");
  joint_impedance->ConfigureLifecycle(stay_here);
  fsm_handler_.RegisterState(
      StateIdFromMode(ControlMode::kJointImpedance), std::move(joint_impedance));

  auto fault = std::make_unique<ControlModeState>(ControlMode::kFault, "fault", "fault");
  fault->ConfigureLifecycle(stay_here);
  fsm_handler_.RegisterState(StateIdFromMode(ControlMode::kFault), std::move(fault));

  if (!fsm_handler_.SetStartState(StateIdFromMode(ControlMode::kHold))) {
    throw std::logic_error("ControlArchitecture::InitializeStateMachine: failed to set hold start state");
  }
  fsm_initialized_ = true;
}

void ControlArchitecture::UpdateStateMachine(double current_time, double dt) {
  ScopedPhaseTimer timer(timing_enabled_, timing_stats_.fsm_us);
  InitializeStateMachine();

  if (control_state_.faulted ||
      control_state_.requested_mode == ControlMode::kFault) {
    requested_state_id_ = StateIdFromMode(ControlMode::kFault);
    (void)fsm_handler_.RequestState(requested_state_id_);
    fsm_handler_.Update(current_time, dt);
    UpdateControlStateFromFsm();
    control_state_.faulted = true;
    if (control_state_.status.empty()) {
      control_state_.status = "fault requested";
    }
    return;
  }

  const StateId requested_state_id = RequestedStateId();
  (void)fsm_handler_.RequestState(requested_state_id);
  fsm_handler_.Update(current_time, dt);
  UpdateControlStateFromFsm();

  if (control_state_.requested_mode == ControlMode::kJointImpedance &&
      control_state_.mode == ControlMode::kHold &&
      (!has_pending_impedance_ || !control_state_.command_valid))
  {
    control_state_.status = "holding because impedance setpoint is invalid";
  }
}

StateId ControlArchitecture::RequestedStateId() const {
  if (control_state_.requested_mode == ControlMode::kFault || control_state_.faulted) {
    return StateIdFromMode(ControlMode::kFault);
  }

  if (requested_state_id_ == StateIdFromMode(ControlMode::kJointImpedance)) {
    if (has_pending_impedance_ && control_state_.command_valid) {
      return StateIdFromMode(ControlMode::kJointImpedance);
    }
    return StateIdFromMode(ControlMode::kHold);
  }

  return requested_state_id_;
}

void ControlArchitecture::UpdateControlStateFromFsm() {
  const auto* current_state = fsm_handler_.GetCurrentState();
  const auto* mode_state = AsControlModeState(current_state);
  if (!mode_state) {
    if (current_state) {
      control_state_.mode = ControlMode::kHold;
      control_state_.status = "running state: " + current_state->name();
      return;
    }
    control_state_.mode = ControlMode::kFault;
    control_state_.status = "fsm has no active state";
    control_state_.faulted = true;
    return;
  }

  if (mode_state->mode() == ControlMode::kIdle) {
    control_state_.mode = ControlMode::kHold;
  } else {
    control_state_.mode = mode_state->mode();
  }
  control_state_.status = mode_state->status();
}

void ControlArchitecture::EvaluateCommand() {
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

  if (const auto* active_state = fsm_handler_.GetCurrentState()) {
    RobotCommand state_command;
    if (active_state->GetCommand(&state_command)) {
      cmd_ = state_command;
      ApplyDriverPdGains(&cmd_);
      cmd_.valid = cmd_.HasValidDimensions() && cmd_.AllFinite();
      command_initialized_ = cmd_.valid;
      if (!cmd_.valid) {
        Fail("state machine produced an invalid robot command");
      }
      return;
    }
  }

  ImpedanceSetpoint setpoint;
  if (control_state_.mode == ControlMode::kJointImpedance && has_pending_impedance_) {
    setpoint = pending_impedance_;
  } else {
    setpoint = MakeHoldSetpoint(state.q);
  }
  std::string error;
  if (!CheckSetpoint(setpoint, state, &error)) {
    Fail(error);
    return;
  }

  cmd_.Resize(nq_, nv_);
  cmd_.q_cmd = setpoint.q_cmd;
  cmd_.qdot_cmd = setpoint.qdot_cmd;
  cmd_.kp.setZero();
  cmd_.kd.setZero();
  cmd_.tau_cmd = setpoint.tau_ff_cmd;

  if (config_.compute_impedance_torque) {
    cmd_.tau_cmd += setpoint.kp.cwiseProduct(setpoint.q_cmd - state.q) +
                    setpoint.kd.cwiseProduct(setpoint.qdot_cmd - state.qdot);
  }
  ApplyDriverPdGains(&cmd_);

  cmd_.stamp_sec = state.time_s;
  cmd_.valid = cmd_.HasValidDimensions() && cmd_.AllFinite();
  command_initialized_ = cmd_.valid;
  if (!cmd_.valid) {
    Fail("evaluated robot command is invalid");
  }
}

void ControlArchitecture::InitializeCommandFromRobotState() {
  if (!IsConfigured()) {
    return;
  }
  if (cmd_.q_cmd.size() != nq_ || cmd_.qdot_cmd.size() != nv_) {
    cmd_.Resize(nq_, nv_);
    cmd_.valid = false;
  }
  if (!robot_ || !robot_->hasState() || robot_->q().size() != nq_ ||
      robot_->qdot().size() != nv_) {
    return;
  }

  cmd_.q_cmd = robot_->q();
  cmd_.qdot_cmd = robot_->qdot();
  cmd_.tau_cmd.setZero(nv_);
  cmd_.valid = cmd_.HasValidDimensions() && cmd_.AllFinite();
  command_initialized_ = true;
}

void ControlArchitecture::Fail(const std::string& error) {
  if (cmd_.q_cmd.size() != nq_ || cmd_.qdot_cmd.size() != nv_) {
    cmd_.Resize(nq_, nv_);
  }
  cmd_.valid = false;
  control_state_.status = error;
}

void ControlArchitecture::ApplyDriverPdGains(RobotCommand* command) const {
  if (command == nullptr) {
    return;
  }
  if (!config_.driver_pd_gains.IsValid()) {
    throw std::logic_error(
        "ControlArchitecture::ApplyDriverPdGains: invalid driver PD gains");
  }
  if (!config_.driver_pd_gains.HasValidDimensions(nv_)) {
    return;
  }
  command->kp = config_.driver_pd_gains.kp;
  command->kd = config_.driver_pd_gains.kd;
}

bool ControlArchitecture::CheckSetpoint(
    const ImpedanceSetpoint& setpoint,
    const RobotState& state,
    std::string* error) const {
  if (!setpoint.HasValidDimensions()) {
    if (error) {
      *error = "impedance setpoint has invalid dimensions";
    }
    return false;
  }
  if (setpoint.q_cmd.size() != nq_ || setpoint.qdot_cmd.size() != nv_ ||
      setpoint.tau_ff_cmd.size() != nv_ || setpoint.kp.size() != nv_ ||
      setpoint.kd.size() != nv_) {
    if (error) {
      *error = "impedance setpoint dimension mismatch";
    }
    return false;
  }
  if (state.q.size() != nq_ || state.qdot.size() != nv_ ||
      state.tau.size() != nv_) {
    if (error) {
      *error = "robot state dimension mismatch";
    }
    return false;
  }
  if (!setpoint.AllFinite() || !IsValid(state)) {
    if (error) {
      *error = "setpoint or state contains non-finite values";
    }
    return false;
  }
  return true;
}

}  // namespace plato_robot_system
