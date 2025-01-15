ros2 topic pub /plato/plato_joint_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {stamp: {sec: 0, nanosec: 0}},
  joint_names: ['plato_joint0','plato_joint1', 'plato_joint2', 'plato_joint3', 'plato_joint4', 'plato_joint5', 'plato_joint6', 'plato_joint7', 'plato_joint8'],
  points: [
    {
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"

  ros2 topic pub /plato/plato_effort_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]}"


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

<!-- Pinch Open -->
ros2 topic pub /plato2/plato2_position_controller/commands std_msgs/msg/Float64MultiArray "{
data: [0.0, 0.0, 0.518, -0.46, 0.864, -0.005, 1.067, 0.655]}"

<!-- Pinch Close -->
ros2 topic pub /plato2/plato2_position_controller/commands std_msgs/msg/Float64MultiArray "{
data: [0.0, 0.0, -0.03, -0.209, 1.422, -0.635, 1.067, 0.655]}"

-------------------

<!-- Open -->
ros2 topic pub /plato2/plato2_position_controller/commands std_msgs/msg/Float64MultiArray "{
data: [-0.048, -0.206, 0.367,-0.557, 0.397, 0.684, 0.201, 0.667]}"

<!-- Close -->
ros2 topic pub /plato2/plato2_position_controller/commands std_msgs/msg/Float64MultiArray "{
data: [-0.057, -0.224, 0.054, -0.386, 0.863, 0.507, 0.67, 0.337]}"

-------------------

<!-- Power Grasp Open -->
ros2 topic pub /plato2/plato2_position_controller/commands std_msgs/msg/Float64MultiArray "{
data: [-0.029, -0.327, 0.338, -0.747, -0.323, 0.808, -0.6, 1.035]}"

<!-- Power Grasp Close -->
ros2 topic pub /plato2/plato2_position_controller/commands std_msgs/msg/Float64MultiArray "{
data: [-0.033, -0.34, -0.243, -0.679, 1.285, 0.549, 0.667, 1.297]}"

-------------------

<!-- Point-->
ros2 topic pub /plato2/plato2_position_controller/commands std_msgs/msg/Float64MultiArray "{
data: [0.0, 0.0, -0.176, -1.227, 0.0, 0.0, 1.085, 1.054]}"

ros2 topic pub  /plato2/joint_impedance_controller/commands plato2_interfaces/msg/ImpedanceCommands "
{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
stiffness: [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0], 
damping: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5], 
effort_ff: [0.0, 0.0, 0.0, -0.6, 0.0, 0.6, 0.0, 0.0]}"

ros2 topic pub  /plato2/joint_impedance_controller/commands plato2_interfaces/msg/ImpedanceCommands "
{position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
stiffness: [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0], 
damping: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
effort_ff: [0.0, 0.0, -0.4, -0.4, 0.4, 0.4, 0.4, 0.4]}"

