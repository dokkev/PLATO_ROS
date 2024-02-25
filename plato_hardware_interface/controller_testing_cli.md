ros2 topic pub /plato_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}},
  joint_names: ['plato_joint0','plato_joint1', 'plato_joint2', 'plato_joint3', 'plato_joint4', 'plato_joint5', 'plato_joint6', 'plato_joint7', 'plato_joint8'],
  points: [
    {
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      velocities: [],
      accelerations: [],
      effort: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"

  ros2 topic pub /plato/plato_effort_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [0.00, 0.00, 0.01, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]}"


  ros2 topic pub /servo_node/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}},
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'],
  points: [
    {
      positions: [0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"


