// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/control/state_machine/state_machine.hpp"

#include <utility>

namespace plato_robot_system
{

State::State(const StateId id, std::string name)
: state_id_(id), state_name_(std::move(name))
{
}

void State::ConfigureLifecycle(const StateLifecycle & lifecycle)
{
  lifecycle_ = lifecycle;
}

bool State::IsFinished() const
{
  if (lifecycle_.stay_here) {
    return false;
  }
  return current_time_ >= lifecycle_.duration + lifecycle_.wait_time;
}

StateId State::NextState() const
{
  return lifecycle_.next_state_id;
}

void State::Enter(const double global_time)
{
  start_time_ = global_time;
  current_time_ = 0.0;
  OnEnter();
}

void State::UpdateTime(const double global_time)
{
  current_time_ = global_time - start_time_;
}

void State::Tick(const double dt)
{
  dt_ = dt;
  OnUpdate();
}

void State::Exit()
{
  OnExit();
}

}  // namespace plato_robot_system
