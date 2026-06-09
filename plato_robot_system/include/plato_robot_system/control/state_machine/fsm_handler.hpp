// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "plato_robot_system/control/state_machine/state_machine.hpp"

namespace plato_robot_system
{

class FSMHandler
{
public:
  using StateMap = std::unordered_map<StateId, std::unique_ptr<State>>;

  void RegisterState(StateId id, std::unique_ptr<State> state);
  void Clear();
  bool SetStartState(StateId id);
  bool RequestState(StateId id);
  std::optional<StateId> FindStateIdByName(const std::string & name) const;

  StateId GetCurrentStateId() const { return current_state_id_.load(); }
  State * GetCurrentState() const { return current_state_; }

  void Update(double global_time, double dt = 0.0);

  const StateMap & states() const { return state_map_; }

private:
  void EnterCurrentStateIfNeeded(double global_time);
  bool SwitchToState(StateId id, double global_time);

  std::atomic<StateId> current_state_id_{-1};
  State * current_state_{nullptr};
  bool is_first_visit_{false};
  std::atomic<StateId> requested_state_{-1};
  StateMap state_map_;
};

}  // namespace plato_robot_system
