// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cstddef>

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

  Eigen::VectorXd action(std::size_t step) const;
  Eigen::VectorXd firstAction() const;
  void setAction(std::size_t step, const Eigen::Ref<const Eigen::VectorXd>& u);

 private:
  Eigen::MatrixXd values_;
};

}  // namespace mppi_core
