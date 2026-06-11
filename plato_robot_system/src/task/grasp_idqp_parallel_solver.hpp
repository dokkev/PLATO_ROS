#ifndef PLATO_ROBOT_SYSTEM__SRC__TASK__GRASP_IDQP_PARALLEL_SOLVER_HPP_
#define PLATO_ROBOT_SYSTEM__SRC__TASK__GRASP_IDQP_PARALLEL_SOLVER_HPP_

#include <Eigen/Core>

#include <array>

#include <pinocchio/multibody/model.hpp>

#include "plato_robot_system/task/grasp_idqp.hpp"
#include "plato_robot_system/task/thumb_index_grasp_constants.hpp"

namespace plato_robot_system::task
{

bool ResolveGraspIDQPParallelFrames(
  const pinocchio::Model & model,
  const GraspIDQPConfig & config,
  pinocchio::FrameIndex * index_contact_point_frame_id,
  pinocchio::FrameIndex * thumb_contact_point_frame_id);

bool RefineGraspIDQPParallelTarget(
  const pinocchio::Model & model,
  const GraspIDQPConfig & config,
  const std::array<int, kThumbIndexActiveJoints.size()> & active_q_indices,
  int active_dof,
  pinocchio::FrameIndex index_contact_point_frame_id,
  pinocchio::FrameIndex thumb_contact_point_frame_id,
  const Eigen::VectorXd & q_seed,
  const Eigen::VectorXd & smooth_reference,
  bool has_smooth_reference,
  Eigen::VectorXd * q_target,
  GraspIDQPStatus * status);

}  // namespace plato_robot_system::task

#endif  // PLATO_ROBOT_SYSTEM__SRC__TASK__GRASP_IDQP_PARALLEL_SOLVER_HPP_
