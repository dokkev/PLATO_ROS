// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/control/state_machine/fsm_handler.hpp"

#include <stdexcept>
#include <utility>

namespace plato_robot_system
{

void FSMHandler::RegisterState(const StateId id, std::unique_ptr<State> state)
{
  if (!state) {
    throw std::invalid_argument("FSMHandler::RegisterState: state is null");
  }
  if (state->id() != id) {
    throw std::invalid_argument("FSMHandler::RegisterState: id does not match state id");
  }
  if (!state_map_.emplace(id, std::move(state)).second) {
    throw std::invalid_argument("FSMHandler::RegisterState: duplicate state id");
  }
}

void FSMHandler::Clear()
{
  current_state_id_.store(-1);
  current_state_ = nullptr;
  is_first_visit_ = false;
  requested_state_.store(-1);
  state_map_.clear();
}

bool FSMHandler::SetStartState(const StateId id)
{
  const auto it = state_map_.find(id);
  if (it == state_map_.end()) {
    return false;
  }
  current_state_id_.store(id);
  current_state_ = it->second.get();
  is_first_visit_ = true;
  return true;
}

bool FSMHandler::RequestState(const StateId id)
{
  if (state_map_.find(id) == state_map_.end()) {
    return false;
  }
  requested_state_.store(id);
  return true;
}

std::optional<StateId> FSMHandler::FindStateIdByName(const std::string & name) const
{
  for (const auto & [id, state] : state_map_) {
    if (state->name() == name) {
      return id;
    }
  }
  return std::nullopt;
}

void FSMHandler::Update(const double global_time, const double dt)
{
  if (!current_state_) {
    return;
  }

  const StateId requested = requested_state_.exchange(-1);
  if (requested >= 0 && requested != current_state_id_.load()) {
    SwitchToState(requested, global_time);
  }

  EnterCurrentStateIfNeeded(global_time);
  current_state_->UpdateTime(global_time);

  if (current_state_->IsFinished()) {
    const StateId next = current_state_->NextState();
    if (next >= 0 && SwitchToState(next, global_time)) {
      current_state_->UpdateTime(global_time);
    }
  }

  current_state_->Tick(dt);
}

void FSMHandler::EnterCurrentStateIfNeeded(const double global_time)
{
  if (!is_first_visit_ || !current_state_) {
    return;
  }
  current_state_->Enter(global_time);
  is_first_visit_ = false;
}

bool FSMHandler::SwitchToState(const StateId id, const double global_time)
{
  const auto it = state_map_.find(id);
  if (it == state_map_.end()) {
    return false;
  }
  if (current_state_) {
    current_state_->Exit();
  }
  current_state_ = it->second.get();
  current_state_id_.store(id);
  is_first_visit_ = true;
  EnterCurrentStateIfNeeded(global_time);
  return true;
}

}  // namespace plato_robot_system
