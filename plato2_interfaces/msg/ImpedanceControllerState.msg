# Header timestamp should be update time of controller state
std_msgs/Header header
string[] joint_names

# JointTrajectoryPoint
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual
trajectory_msgs/JointTrajectoryPoint error  

# Stiffness
float64[] stiffness
# Damping
float64[] damping

# total effort
float64[] effort
# feedforward components
float64[] effort_ff
# feedback components
float64[] effort_fb

