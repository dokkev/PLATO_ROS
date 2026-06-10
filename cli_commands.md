ros2 service call /plato2/aristo_controller/request_state \
  plato_interfaces/srv/RequestState "{state_id: 3}"

ros2 topic pub --once /grasp_force_reference/target_normal_force_n \
  std_msgs/msg/Float64 "{data: 0.4}"

ros2 topic pub --once /grasp_force_reference/reference_valid \
  std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /plato2/parallel_grasp_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.7, 0.2]}"