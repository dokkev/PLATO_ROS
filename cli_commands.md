ros2 topic pub /plato2/joint_impedance_controller/commands plato2_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, -0.398, 0.398, 0.992, -0.992, 0.785, 1.57],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [3.0, 3.0, 2.0, 2.0, 2.0, 2.0, 1.0, 1.0],
  damping: [0.2, 0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
}"


ros2 topic pub /plato2/joint_impedance_controller/commands plato2_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [3.0, 3.0, 2.0, 2.0, 2.0, 2.0, 1.0, 1.0],
  damping: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"


ros2 topic pub /plato2/joint_impedance_controller/commands plato2_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [3.0, 3.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
  damping: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"


<!-- // damping only -->
ros2 topic pub /plato2/joint_impedance_controller/commands plato2_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  damping: [0.2, 0.2, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15]
}"

<!-- // damping only -->
ros2 topic pub /plato2/joint_impedance_controller/commands plato2_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  damping: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"