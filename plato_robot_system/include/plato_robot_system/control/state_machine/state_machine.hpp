// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <string>

#include "plato_robot_system/robot/robot_system.hpp"

namespace plato_robot_system
{

using StateId = int;

struct StateLifecycle
{
  double duration{0.0};
  double wait_time{0.0};
  StateId next_state_id{-1};
  bool stay_here{true};
};

class State
{
public:
  State(StateId id, std::string name);
  virtual ~State() = default;

  virtual void OnEnter() {}
  virtual void OnUpdate() {}
  virtual void OnExit() {}
  virtual bool PopulateCommand(RobotCommand * command) const
  {
    (void)command;
    return false;
  }

  void ConfigureLifecycle(const StateLifecycle & lifecycle);

  virtual bool IsFinished() const;
  virtual StateId NextState() const;

  void Enter(double global_time);
  void UpdateTime(double global_time);
  void Tick(double dt);
  void Exit();

  StateId id() const { return state_id_; }
  const std::string & name() const { return state_name_; }
  double elapsed_time() const { return current_time_; }
  double dt() const { return dt_; }

protected:
  StateId state_id_;
  std::string state_name_;
  StateLifecycle lifecycle_;

  double start_time_{-1.0};
  double current_time_{0.0};
  double dt_{0.0};
};

}  // namespace plato_robot_system
