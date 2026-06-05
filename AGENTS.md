# AGENTS.md

This is the entry map for agents working in `PLATO_ROS`, a ROS 2 package
collection for PLATO/Aristo hands, CAN hardware interfaces, controllers,
teleoperation, state estimation, and MPPI grasp planning.

Read first:

- `docs/ARCHITECTURE.md`
- `docs/COMMANDS.md`
- `docs/DECISIONS.md`
- `docs/PLANS.md`

Useful deeper runbooks:

- `docs/motor/usbcan_setup.md`
- `docs/frimware/plato2/README.md`
- `installation.md`
- `cli_commands.md`

Guidelines:

- Keep this file short. Put durable project knowledge in `docs/`.
- Work from the workspace root `~/workspace/plato_ws` for `colcon` commands.
- Prefer small, reviewable diffs and preserve behavior unless the task asks for
  behavior changes.
- Treat hardware-facing changes conservatively. Use fake hardware, unit tests,
  launch dry runs, or limited-scope checks before real hardware when practical.
- Preserve the ros2_control boundary: controllers speak joint-space interfaces;
  hardware packages absorb robot-specific actuator, CAN, and transmission detail.
- Run the narrowest relevant validation command from `docs/COMMANDS.md` when
  practical. If validation is not run, say exactly what was not verified.
