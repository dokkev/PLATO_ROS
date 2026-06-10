#ifndef PLATO_ROBOT_SYSTEM__TASK__THUMB_INDEX_GRASP_CONSTANTS_HPP_
#define PLATO_ROBOT_SYSTEM__TASK__THUMB_INDEX_GRASP_CONSTANTS_HPP_

#include <array>
#include <string_view>

namespace plato_robot_system::task
{

inline constexpr std::string_view kThumbIndexFrameA = "index_distal_tactile";
inline constexpr std::string_view kThumbIndexFrameB = "thumb_distal_tactile";

// Active joint order is also the order expected by active-space posture vectors:
//   [index_mcp, index_pip, thumb_mcp, thumb_ip]
inline constexpr std::array<std::string_view, 4> kThumbIndexActiveJoints = {
  "joint5",
  "joint6",
  "joint3",
  "joint4",
};

}  // namespace plato_robot_system::task

#endif  // PLATO_ROBOT_SYSTEM__TASK__THUMB_INDEX_GRASP_CONSTANTS_HPP_
