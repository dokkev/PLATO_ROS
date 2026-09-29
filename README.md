# PLATO_ROS

`PLATO_ROS` is a ROS 2 Humble codebase for PLATO and ARISTO robotic hands. It
contains ros2_control hardware plugins, ROS bringup and observation packages,
and a ROS-free C++ MPPI planning core with object-contact belief support. The
workspace is intended for Ubuntu 22.04 with ROS 2 Humble.

## Four research areas

- **PLATO Hardware** implements the PLATO hand's ros2_control interface,
  actuator communication, five-bar transmission, and hardware configuration.
- **ARISTO Hardware** implements the ARISTO hand's ros2_control interface,
  actuator protocol, and configuration.
- **MPPI** in `mppi_core` provides reusable joint-acceleration sampling,
  grasp-state rollout, cost evaluation, and `RobotCommand` generation.
- **Object Belief** uses tactile contacts and an object pose prior to form
  plausible object-pose scenarios for geometric grasp-support evaluation.

The PLATO and ARISTO hardware stacks can be brought up independently. The
planning and object-belief code is separate research infrastructure; hardware
bringup does not require MPPI.

## Architecture and data flow

```text
PLATO / ARISTO hardware
  -> joint state + tactile observations
      ├ -> plato_state_estimator: configured object prior + tactile contacts
      │      -> representative pose, pose particles, status, visualization
      └ -> application adapter builds mppi_core::GraspObservation
             (robot state + tactile state + optional VirtualObjectBelief)
               -> qddot_sol rollouts and grasp/object-support costs
                 -> RobotCommand
                   -> application adapter -> joint-space controller
                     -> bringup -> hardware interface -> hand
```

The diagram shows the package-level data contract. `mppi_core` has no ROS nodes
or message adapters: this repository does not include an adapter that converts
the estimator's ROS pose/particle topics into a `VirtualObjectBelief`, or one
that connects `RobotCommand` to the controller command topic. Object particles
are plausible pose scenarios used for contact and support evaluation, not
simulated rigid-body object trajectories.

## Package map

| Capability | Implementation |
| --- | --- |
| PLATO-specific hardware, CAN protocol, transmission, and configuration | [`hardware_interface/plato_hardware_interface`](hardware_interface/plato_hardware_interface/) |
| ARISTO-specific hardware, protocol, and configuration | [`hardware_interface/aristo_hardware_interface`](hardware_interface/aristo_hardware_interface/) |
| Shared generic CAN transport and utilities | [`hardware_interface/can_hardware_common`](hardware_interface/can_hardware_common/) |
| PLATO ROS launch and controller configuration | [`bringup/plato_bringup`](bringup/plato_bringup/) |
| ARISTO ROS launch and controller configuration | [`bringup/aristo_bringup`](bringup/aristo_bringup/) |
| Hand URDF/Xacro descriptions and meshes | [`plato_description`](plato_description/) |
| Shared joint-space impedance controller | [`controllers/ros2_control/joint_impedance_controller`](controllers/ros2_control/joint_impedance_controller/) |
| MPPI, rollout, costs, and object-belief core | [`mppi_core`](mppi_core/), especially [`mppi_core/include/mppi_core/object`](mppi_core/include/mppi_core/object/) and [`mppi_core/src/object`](mppi_core/src/object/) |
| ROS-facing joint/tactile observation and object-prior node | [`plato_state_estimator`](plato_state_estimator/) |

The hardware boundary keeps shared CAN utilities in `can_hardware_common`,
robot-specific protocols, transmissions, and actuator configuration in the
matching hardware package, and controllers in joint space without CAN details.
Bringup packages own ROS launch and controller wiring.

The PLATO hardware implementation is distinct from the additional ARISTO hand
support and the later/general MPPI and object-belief planning infrastructure.
They share useful interfaces, but an experiment should depend only on the
packages it uses.

## Setup and further documentation

- [Installation and platform notes](installation.md)
- [Build, test, and run commands](docs/COMMANDS.md)
- [Repository architecture and package boundaries](docs/ARCHITECTURE.md)
- [MPPI package overview](mppi_core/README.md)
- [MPPI rollout and command contract](mppi_core/docs/grasp_state_mppi.md)
- [Object-belief ownership and semantics](mppi_core/docs/object_belief.md)
- [Robust grasp policy](mppi_core/docs/robust_grasp_policy.md)
- [MPPI file structure](mppi_core/docs/file_structure.md)
- [USB-CAN hardware setup](docs/motor/usbcan_setup.md)
- [PLATO2 firmware notes](docs/frimware/plato2/README.md)
