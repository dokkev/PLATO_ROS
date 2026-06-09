#include "aristo_state_machines/state_machines/initialize.hpp"

#include <utility>

namespace aristo_state_machines::state_machines
{

InitializeState::InitializeState(
  const plato_robot_system::StateId id,
  std::string name,
  const plato_robot_system::RobotSystem * robot)
: plato_robot_system::State(id, std::move(name)),
  robot_(robot)
{
}

void InitializeState::SetTargetPosition(const Eigen::Ref<const Eigen::VectorXd> & target_jpos)
{
  target_jpos_ = target_jpos;
}

void InitializeState::SetFeedbackGains(
  const Eigen::Ref<const Eigen::VectorXd> & kp,
  const Eigen::Ref<const Eigen::VectorXd> & kd)
{
  kp_ = kp;
  kd_ = kd;
}

bool InitializeState::GetCommand(plato_robot_system::RobotCommand * command) const
{
  if (command == nullptr || !ResizeCommandFromRobotState()) {
    return false;
  }

  const auto & state = robot_->state();
  const auto nq = static_cast<int>(state.q.size());
  const auto nv = static_cast<int>(state.qdot.size());
  if (target_jpos_.size() != nq || kp_.size() != nv || kd_.size() != nv) {
    return false;
  }

  command_.q_cmd = target_jpos_;
  command_.qdot_cmd.setZero();
  command_.kp.setZero();
  command_.kd.setZero();
  command_.tau_cmd = kp_.cwiseProduct(command_.q_cmd - state.q) -
                     kd_.cwiseProduct(state.qdot);
  command_.stamp_sec = state.time_s;
  command_.valid = command_.HasValidDimensions() && command_.AllFinite();
  if (!command_.valid) {
    return false;
  }

  *command = command_;
  return true;
}

bool InitializeState::ResizeCommandFromRobotState() const
{
  if (robot_ == nullptr || !robot_->hasState()) {
    return false;
  }

  const auto & state = robot_->state();
  const auto nq = static_cast<int>(state.q.size());
  const auto nv = static_cast<int>(state.qdot.size());
  if (nq <= 0 || nv <= 0) {
    return false;
  }

  if (command_.q_cmd.size() != nq || command_.qdot_cmd.size() != nv) {
    command_.Resize(nq, nv);
  }
  return true;
}

}  // namespace aristo_state_machines::state_machines
