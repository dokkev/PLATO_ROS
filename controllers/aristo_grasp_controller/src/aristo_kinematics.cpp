#include "aristo_grasp_controller/aristo_kinematics.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace aristo_grasp_controller
{

AristoKinematics::AristoKinematics(std::string urdf_path)
: robot_(std::move(urdf_path))
{
}

Eigen::VectorXd AristoKinematics::make_configuration(
  const std::vector<double> & joint_positions) const
{
  Eigen::VectorXd q = robot_.q();
  if (q.size() != robot_.nq()) {
    q = Eigen::VectorXd::Zero(robot_.nq());
  }

  for (Eigen::Index i = 0; i < q.size(); ++i) {
    q[i] = i < static_cast<Eigen::Index>(joint_positions.size())
      ? joint_positions[static_cast<std::size_t>(i)]
      : 0.0;
  }
  return q;
}

void AristoKinematics::update(const Eigen::Ref<const Eigen::VectorXd> & q)
{
  if (q.size() != robot_.nq()) {
    throw std::invalid_argument("AristoKinematics::update: q dimension mismatch");
  }

  Eigen::VectorXd qdot = Eigen::VectorXd::Zero(robot_.nv());
  robot_.UpdateState(q, qdot);
  robot_.UpdateKinematics();
}

pinocchio::SE3 AristoKinematics::frame_pose(const std::string & frame_name) const
{
  return robot_.FramePoseWorld(frame_name);
}

Eigen::Matrix<double, 6, Eigen::Dynamic> AristoKinematics::frame_jacobian(
  const std::string & frame_name)
{
  return robot_.FrameJacobianWorld(frame_name);
}

double AristoKinematics::frame_distance(
  const std::string & first_frame,
  const std::string & second_frame) const
{
  return (frame_pose(first_frame).translation() - frame_pose(second_frame).translation()).norm();
}

double AristoKinematics::frame_axis_alignment(
  const std::string & first_frame,
  const std::string & second_frame,
  int axis_index) const
{
  if (axis_index < 0 || axis_index > 2) {
    throw std::invalid_argument("AristoKinematics::frame_axis_alignment: axis index must be 0, 1, or 2");
  }

  const Eigen::Vector3d first_axis = frame_pose(first_frame).rotation().col(axis_index);
  const Eigen::Vector3d second_axis = frame_pose(second_frame).rotation().col(axis_index);
  return std::clamp(first_axis.dot(second_axis), -1.0, 1.0);
}

}  // namespace aristo_grasp_controller
