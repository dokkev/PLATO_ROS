// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <string>
#include <utility>

namespace mppi_core {

class ActionSequence {
 public:
  ActionSequence() = default;
  ActionSequence(std::size_t action_dim, std::size_t horizon_steps) {
    Resize(action_dim, horizon_steps);
  }

  void Resize(std::size_t action_dim, std::size_t horizon_steps);
  void SetZero();
  void ShiftAndRepeatLast();

  std::size_t actionDim() const {
    return static_cast<std::size_t>(values_.rows());
  }
  std::size_t horizonSteps() const {
    return static_cast<std::size_t>(values_.cols());
  }

  const Eigen::MatrixXd& values() const { return values_; }
  Eigen::MatrixXd& values() { return values_; }
  const std::string& name() const { return name_; }
  void setName(std::string name) { name_ = std::move(name); }

  Eigen::VectorXd action(std::size_t step) const;
  Eigen::VectorXd firstAction() const;
  void setAction(std::size_t step, const Eigen::Ref<const Eigen::VectorXd>& u);

 private:
  Eigen::MatrixXd values_;
  std::string name_;
};

}  // namespace mppi_core
