#include "grasp_idqp_parallel_solver.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>

namespace plato_robot_system::task
{
namespace
{

constexpr double kMinAxisNorm = 1.0e-12;

bool IsFinite(const double value)
{
  return std::isfinite(value);
}

double Clamp(const double value, const double lower, const double upper)
{
  return std::clamp(value, lower, upper);
}

bool ResolveFrameId(
  const pinocchio::Model & model,
  const std::string & frame_name,
  pinocchio::FrameIndex * frame_id)
{
  if (frame_id == nullptr || frame_name.empty()) {
    return false;
  }
  const auto resolved_id = model.getFrameId(frame_name);
  if (resolved_id >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
    return false;
  }
  *frame_id = resolved_id;
  return true;
}

double ClampToFinitePositionLimit(
  const pinocchio::Model & model,
  const int q_index,
  const double value)
{
  if (
    q_index < 0 ||
    q_index >= model.nq ||
    model.lowerPositionLimit.size() != model.nq ||
    model.upperPositionLimit.size() != model.nq)
  {
    return value;
  }

  const double lower = model.lowerPositionLimit[q_index];
  const double upper = model.upperPositionLimit[q_index];
  if (!IsFinite(lower) || !IsFinite(upper) || lower >= upper) {
    return value;
  }
  return Clamp(value, lower, upper);
}

struct ParallelGeometry
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p_index_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d p_thumb_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d n_index_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d n_thumb_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d n_pair_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d r_perp_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d parallel_residual{Eigen::Vector3d::Zero()};
  double aperture_m{0.0};
  double moment_arm_m{0.0};
  bool valid{false};
};

struct ParallelResidualEvaluation
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ParallelGeometry geometry;
  Eigen::VectorXd residual;
  double cost{0.0};
  bool valid{false};
};

bool EvaluateParallelGeometry(
  const pinocchio::Model & model,
  const pinocchio::FrameIndex index_contact_point_frame_id,
  const pinocchio::FrameIndex thumb_contact_point_frame_id,
  const Eigen::Vector3d & index_contact_normal_axis_frame,
  const Eigen::Vector3d & thumb_contact_normal_axis_frame,
  const Eigen::VectorXd & q_candidate,
  ParallelGeometry * geometry)
{
  if (
    geometry == nullptr ||
    model.nq <= 0 ||
    model.nv <= 0 ||
    q_candidate.size() != model.nq ||
    !q_candidate.allFinite() ||
    index_contact_point_frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes) ||
    thumb_contact_point_frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes) ||
    !index_contact_normal_axis_frame.allFinite() ||
    !thumb_contact_normal_axis_frame.allFinite())
  {
    return false;
  }

  const double index_axis_norm = index_contact_normal_axis_frame.norm();
  const double thumb_axis_norm = thumb_contact_normal_axis_frame.norm();
  if (index_axis_norm <= kMinAxisNorm || thumb_axis_norm <= kMinAxisNorm) {
    return false;
  }

  pinocchio::Data data(model);
  const Eigen::VectorXd zero_qdot = Eigen::VectorXd::Zero(model.nv);
  pinocchio::forwardKinematics(model, data, q_candidate, zero_qdot);
  pinocchio::updateFramePlacements(model, data);

  const pinocchio::SE3 & index_pose = data.oMf[index_contact_point_frame_id];
  const pinocchio::SE3 & thumb_pose = data.oMf[thumb_contact_point_frame_id];

  ParallelGeometry result;
  result.p_index_world = index_pose.translation();
  result.p_thumb_world = thumb_pose.translation();
  result.n_index_world =
    index_pose.rotation() * (index_contact_normal_axis_frame / index_axis_norm);
  result.n_thumb_world =
    thumb_pose.rotation() * (thumb_contact_normal_axis_frame / thumb_axis_norm);

  const Eigen::Vector3d r_world = result.p_index_world - result.p_thumb_world;
  const Eigen::Vector3d n_pair_raw = result.n_index_world - result.n_thumb_world;
  const double n_pair_norm = n_pair_raw.norm();
  if (n_pair_norm <= kMinAxisNorm) {
    return false;
  }
  result.n_pair_world = n_pair_raw / n_pair_norm;
  result.aperture_m = r_world.dot(result.n_pair_world);
  result.r_perp_world = r_world - result.aperture_m * result.n_pair_world;
  result.parallel_residual = result.n_index_world + result.n_thumb_world;
  result.moment_arm_m = result.r_perp_world.norm();
  result.valid =
    result.p_index_world.allFinite() &&
    result.p_thumb_world.allFinite() &&
    result.n_index_world.allFinite() &&
    result.n_thumb_world.allFinite() &&
    result.n_pair_world.allFinite() &&
    result.r_perp_world.allFinite() &&
    result.parallel_residual.allFinite() &&
    IsFinite(result.aperture_m) &&
    IsFinite(result.moment_arm_m);
  if (!result.valid) {
    return false;
  }

  *geometry = result;
  return true;
}

}  // namespace

bool ResolveGraspIDQPParallelFrames(
  const pinocchio::Model & model,
  const GraspIDQPConfig & config,
  pinocchio::FrameIndex * index_contact_point_frame_id,
  pinocchio::FrameIndex * thumb_contact_point_frame_id)
{
  return ResolveFrameId(
    model,
    config.index_contact_point_frame,
    index_contact_point_frame_id) &&
         ResolveFrameId(
    model,
    config.thumb_contact_point_frame,
    thumb_contact_point_frame_id);
}

bool RefineGraspIDQPParallelTarget(
  const pinocchio::Model & model,
  const GraspIDQPConfig & config,
  const std::array<int, kThumbIndexActiveJoints.size()> & active_q_indices,
  const int active_dof,
  const pinocchio::FrameIndex index_contact_point_frame_id,
  const pinocchio::FrameIndex thumb_contact_point_frame_id,
  const Eigen::VectorXd & q_seed,
  const Eigen::VectorXd & smooth_reference,
  const bool has_smooth_reference,
  Eigen::VectorXd * q_target,
  GraspIDQPStatus * status)
{
  if (
    q_target == nullptr ||
    status == nullptr ||
    model.nq <= 0 ||
    model.nv <= 0 ||
    q_seed.size() != model.nq ||
    !q_seed.allFinite())
  {
    return false;
  }

  ParallelGeometry seed_geometry;
  if (
    !EvaluateParallelGeometry(
      model,
      index_contact_point_frame_id,
      thumb_contact_point_frame_id,
      config.index_contact_normal_axis_frame,
      config.thumb_contact_normal_axis_frame,
      q_seed,
      &seed_geometry))
  {
    return false;
  }
  const double aperture_des_m = seed_geometry.aperture_m;

  auto build_residual =
    [&config, &active_q_indices, active_dof, aperture_des_m, &q_seed,
      &smooth_reference, has_smooth_reference](
      const ParallelGeometry & geometry,
      const Eigen::VectorXd & q) -> Eigen::VectorXd
    {
      const int smooth_dim = has_smooth_reference ? active_dof : 0;
      Eigen::VectorXd residual(1 + 3 + 3 + active_dof + smooth_dim);
      int row = 0;

      residual[row++] =
        std::sqrt(config.parallel_solver_w_aperture) *
        (geometry.aperture_m - aperture_des_m);
      residual.segment<3>(row) =
        std::sqrt(config.parallel_solver_w_parallel) *
        geometry.parallel_residual;
      row += 3;
      residual.segment<3>(row) =
        std::sqrt(config.parallel_solver_w_moment) *
        geometry.r_perp_world;
      row += 3;

      const double posture_scale = std::sqrt(config.parallel_solver_w_posture);
      for (int i = 0; i < active_dof; ++i) {
        const int q_index = active_q_indices[static_cast<std::size_t>(i)];
        residual[row++] = posture_scale * (q[q_index] - q_seed[q_index]);
      }

      if (has_smooth_reference) {
        const double smooth_scale = std::sqrt(config.parallel_solver_w_smooth);
        for (int i = 0; i < active_dof; ++i) {
          const int q_index = active_q_indices[static_cast<std::size_t>(i)];
          residual[row++] = smooth_scale * (q[q_index] - smooth_reference[q_index]);
        }
      }
      return residual;
    };

  auto evaluate =
    [&model, &config, index_contact_point_frame_id, thumb_contact_point_frame_id,
      &build_residual](const Eigen::VectorXd & q)
    {
      ParallelResidualEvaluation evaluation;
      if (
        !EvaluateParallelGeometry(
          model,
          index_contact_point_frame_id,
          thumb_contact_point_frame_id,
          config.index_contact_normal_axis_frame,
          config.thumb_contact_normal_axis_frame,
          q,
          &evaluation.geometry))
      {
        return evaluation;
      }
      evaluation.residual = build_residual(evaluation.geometry, q);
      evaluation.cost = evaluation.residual.squaredNorm();
      evaluation.valid =
        evaluation.residual.allFinite() &&
        IsFinite(evaluation.cost);
      return evaluation;
    };

  Eigen::VectorXd q_current = q_seed;
  ParallelResidualEvaluation best = evaluate(q_current);
  if (!best.valid) {
    return false;
  }
  Eigen::VectorXd best_q = q_current;

  int solver_iters = 0;
  for (int iter = 0; iter < config.parallel_solver_max_iters; ++iter) {
    const ParallelResidualEvaluation current = evaluate(q_current);
    if (!current.valid) {
      break;
    }
    if (current.cost < best.cost) {
      best = current;
      best_q = q_current;
    }

    Eigen::MatrixXd jacobian(current.residual.size(), active_dof);
    jacobian.setZero();
    for (int col = 0; col < active_dof; ++col) {
      const int q_index = active_q_indices[static_cast<std::size_t>(col)];
      Eigen::VectorXd q_perturbed = q_current;
      q_perturbed[q_index] = ClampToFinitePositionLimit(
        model,
        q_index,
        q_perturbed[q_index] + config.parallel_solver_fd_eps_rad);
      const double actual_eps = q_perturbed[q_index] - q_current[q_index];
      if (std::abs(actual_eps) <= 0.1 * config.parallel_solver_fd_eps_rad) {
        continue;
      }

      const ParallelResidualEvaluation perturbed = evaluate(q_perturbed);
      if (perturbed.valid && perturbed.residual.size() == current.residual.size()) {
        jacobian.col(col) = (perturbed.residual - current.residual) / actual_eps;
      }
    }

    const Eigen::MatrixXd hessian =
      jacobian.transpose() * jacobian +
      config.parallel_solver_damping *
      Eigen::MatrixXd::Identity(active_dof, active_dof);
    const Eigen::VectorXd gradient = jacobian.transpose() * current.residual;
    const Eigen::LDLT<Eigen::MatrixXd> ldlt(hessian);
    if (ldlt.info() != Eigen::Success) {
      break;
    }

    Eigen::VectorXd dq = -ldlt.solve(gradient);
    if (dq.size() != active_dof || !dq.allFinite()) {
      break;
    }

    const double dq_norm = dq.norm();
    if (dq_norm > config.parallel_solver_max_step_rad) {
      dq *= config.parallel_solver_max_step_rad / dq_norm;
    }
    if (!dq.allFinite()) {
      break;
    }
    if (dq.norm() <= 1.0e-12) {
      solver_iters = iter + 1;
      break;
    }

    Eigen::VectorXd q_next = q_current;
    for (int i = 0; i < active_dof; ++i) {
      const int q_index = active_q_indices[static_cast<std::size_t>(i)];
      q_next[q_index] = ClampToFinitePositionLimit(
        model,
        q_index,
        q_next[q_index] + dq[i]);
    }
    if (!q_next.allFinite()) {
      break;
    }

    q_current = q_next;
    solver_iters = iter + 1;
  }

  const ParallelResidualEvaluation final = evaluate(q_current);
  if (final.valid && final.cost < best.cost) {
    best = final;
    best_q = q_current;
  }

  status->aperture_des_m = aperture_des_m;
  status->aperture_m = best.geometry.aperture_m;
  status->parallel_axis_error = best.geometry.parallel_residual.norm();
  status->moment_arm_m = best.geometry.moment_arm_m;
  status->solver_cost = best.cost;
  status->solver_iters = solver_iters;
  status->used_pinocchio_parallel_solver = true;
  *q_target = best_q;
  return q_target->allFinite();
}

}  // namespace plato_robot_system::task
