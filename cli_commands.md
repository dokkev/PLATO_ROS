# Joint impedance command examples

`ImpedanceCommands` requires all five arrays to have 8 values. Publishing only the
topic and type sends empty arrays, which the controller rejects.

## Zero-command / compliant mode

ros2 topic pub --once /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  damping: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"

## Example pose command

ros2 topic pub --once /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, -0.398, 0.398, 0.992, -0.992, 0.785, 1.57],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [3.0, 3.0, 2.0, 2.0, 2.0, 2.0, 1.0, 1.0],
  damping: [0.2, 0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
}"

ros2 topic pub /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  damping: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"

ros2 topic pub /plato2/parallel_grasp_controller/command std_msgs/msg/Float64 "{data: 1.0}"

ros2 topic pub /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [3.0, 3.0, 2.0, 2.0, 2.0, 2.0, 1.0, 1.0],
  damping: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"


ros2 topic pub /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [0.0, 0.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
  damping: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"


<!-- // damping only -->
ros2 topic pub /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  damping: [0.2, 0.2, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15]
}"

<!-- // damping only -->
ros2 topic pub /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  damping: [0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"


ros2 service call /naritouch_grasp_demo/start_grasp std_srvs/srv/Empty



ros2 topic pub --rate 100 /plato2/parallel_grasp_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.0]"


ros2 topic pub --once /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.193, -0.265, 0.251, 0.811, 0.884, 1.154],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
  damping: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
  effort_ff: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"


ros2 topic pub /plato2/joint_impedance_controller/commands plato_interfaces/msg/ImpedanceCommands "{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  effort_ff: [0.0, 0.0, 0.0, -40.0, 0.0, 0.0, 0.0, 0.0],
  stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  damping: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"
