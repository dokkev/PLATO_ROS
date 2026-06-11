# PLATO ROS Grasp Task 코드 리뷰

작성일: 2026-06-10

## 후속 반영

2026-06-10에 아래 항목은 코드에 반영했다.

- `grasp_task.q_ready`를 `GraspTeleopState::ConfigureTask()` 전에 model `q` 순서로 mapping한다.
- `GraspTask`에서 ID-QP/task-space branch를 제거하고, `parallel_grasp_controller`의 직접 joint-space geometry만 사용하도록 단순화했다.
- `aristo.yaml`의 `grasp_teleop`에서 더 이상 쓰지 않는 ID-QP/task-space 튜닝 키를 제거했다.
- `grasp_teleop` 진입 기본 command를 ready pose와 맞추기 위해 `default_u: 0.0`, `default_phi: 0.0`으로 바꿨다.
- ROS controller 내부 `GraspTeleopCommand::u` 기본값을 새 convention에 맞게 `0.0`으로 바꿨다.

## 리뷰 범위

이번 리뷰는 최근 변경된 Aristo grasp 제어 흐름을 중심으로 보았다.

- `plato_robot_system/task/grasp_task.*`
- `controllers/aristo_controller/config/aristo.yaml`
- `controllers/aristo_controller/src/config/aristo_config.cpp`
- `controllers/aristo_controller/state_machines/grasp_teleop.*`
- `controllers/ros2_control/plato_ros_controller/src/plato_ros_controller.cpp`
- 관련 테스트: `grasp_task_test.cpp`, `aristo_config_test.cpp`

핵심 목표는 `grasp_ready` joint pose를 직접 command endpoint로 쓰지 않고, `GraspTask`의 `q_ready` FK 기준 geometry로 사용하는 설계가 코드에 안전하게 반영되었는지 확인하는 것이다.

## 결론

큰 방향은 맞다. `q_ready` FK로 ready/open fingertip distance를 만들고, `close_travel_m`/`min_closed_distance_m`/`lateral_offset_limit_m`로 normalized command range를 정의하는 구조는 이전 `distance_open_m`/`distance_closed_m` 고정값보다 실험 의미가 더 명확하다.

다만 하드웨어 구동 전에 바로 고치는 것이 좋은 항목이 있다. 가장 중요한 문제는 `q_ready`가 controller YAML joint order에서 Pinocchio model `q` order로 mapping되지 않은 채 `GraspTask`에 전달될 수 있다는 점이다. 현재 URDF order가 우연히 YAML order와 같으면 문제가 드러나지 않지만, 순서가 달라지면 FK 기준 geometry와 posture bias가 모두 틀어진다.

## 주요 발견 사항

### 1. 높음: `q_ready`가 model q 순서로 mapping되지 않음

위치:

- `controllers/aristo_controller/src/config/aristo_config.cpp:391`
- `controllers/ros2_control/plato_ros_controller/src/plato_ros_controller.cpp:1001`
- `controllers/ros2_control/plato_ros_controller/src/plato_ros_controller.cpp:1187`
- `plato_robot_system/src/task/grasp_task.cpp:378`

현재 `aristo_config.cpp`는 `grasp_ready.target_jpos`를 그대로 `grasp_task.q_ready`에 복사한다. 이 값은 YAML 주석 기준으로 `[joint1, joint2, ..., joint8]` 순서다. 반면 `GraspTask::CaptureReadyGeometryFromConfig()`는 `config_.q_ready`를 그대로 Pinocchio FK에 넣는다. 즉 이 벡터는 model `q` 순서여야 한다.

`initialize`, `poke`, `grasp_ready` state target은 ROS controller에서 `map_joint_positions_to_model_q(...)`를 거친다. 하지만 `grasp_teleop_config.grasp_task.q_ready`는 `GraspTeleopState::ConfigureTask()` 전에 같은 mapping을 거치지 않는다.

영향:

- URDF/model `q` order와 YAML joint order가 다르면 ready fingertip FK가 잘못 계산된다.
- `q_posture`를 생략했을 때 `q_ready`에서 active joint posture를 추출하는 로직도 같이 틀어진다.
- 현재 `aristo_config_test`는 `q_ready`가 YAML 값과 같은지만 확인하고, ROS controller에서 model coordinate로 mapping되는지는 확인하지 못한다.

권장 수정:

```cpp
auto grasp_teleop_config = aristo_config.grasp_teleop.state;
if (grasp_teleop_config.grasp_task.q_ready.size() > 0) {
  grasp_teleop_config.grasp_task.q_ready =
    map_joint_positions_to_model_q(grasp_teleop_config.grasp_task.q_ready, robot);
}
```

이 처리는 `GraspTeleopState::ConfigureTask()` 호출 전에 해야 한다. 그리고 controller-level test 또는 작은 helper test로 `q_ready`가 model `q` index에 맞게 들어가는지 확인하는 것이 좋다.

### 2. 중간: 상태 진입 첫 tick에서 최신 grasp teleop command가 default로 덮일 수 있음

위치:

- `controllers/ros2_control/plato_ros_controller/src/plato_ros_controller.cpp:309`
- `controllers/aristo_controller/src/state_machines/grasp_teleop.cpp:74`

현재 update loop 순서는 대략 다음과 같다.

1. pending state request 적용
2. `sync_joint_teleop_input()`
3. `sync_grasp_teleop_input()`
4. `control_architecture_.Update(...)`

그런데 FSM이 이 update 안에서 `grasp_teleop`에 새로 진입하면 `GraspTeleopState::OnEnter()`가 `input_`을 YAML default로 다시 설정한다. 따라서 사용자가 이미 `/plato2/parallel_grasp_controller/commands`에 command를 publish해 둔 상태여도, 진입 첫 control tick은 `default_u`/`default_phi`을 사용할 수 있다.

현재 YAML default는 `default_u: 0.8`, `default_phi: 0.0`이다. 즉 `grasp_ready -> grasp_teleop` 자동 전환 직후 첫 tick에서 의도보다 강하게 close 또는 lateral command가 들어갈 가능성이 있다.

권장 수정 방향:

- `OnEnter()`에서 command input을 무조건 default로 덮지 않고, command가 아직 없을 때만 default를 적용한다.
- 또는 `control_architecture_.Update()` 이후에 `sync_grasp_teleop_input()`을 한 번 더 적용할 수 있는 구조를 만든다.
- 가장 단순한 방법은 `GraspTeleopState`에 `has_external_input_` 같은 flag를 두고 `SetInput()` 호출 이력이 있으면 `OnEnter()`에서 close/lateral을 유지하는 것이다.

테스트 권장:

- state request 전에 grasp command를 publish한 상황을 흉내내고, `grasp_teleop` 첫 command가 default가 아니라 최신 input을 쓰는지 확인한다.

### 3. 중간: `q_ready` geometry가 enabled여도 fallback 거리 검증에 묶임

위치:

- `plato_robot_system/src/task/grasp_task.cpp:93`

`IsValidConfig()`는 항상 다음 조건을 요구한다.

```cpp
config.distance_open_m > config.distance_closed_m
config.min_closed_distance_m < config.distance_open_m
```

`q_ready` geometry가 활성화된 경우 실제 runtime open distance는 FK에서 계산한 `distance_ready_m_`이다. 따라서 `distance_open_m`은 fallback 값인데, 이 fallback 값이 조금 이상하면 유효한 `q_ready` 설정도 configure에서 실패할 수 있다.

현재 YAML 값에서는 문제가 없다. 하지만 설계상 `q_ready`가 우선순위 1이라면 검증도 다음처럼 분리하는 편이 더 자연스럽다.

- fallback mode: `distance_open_m > distance_closed_m` 검증
- q_ready mode: `close_travel_m > 0`, `min_closed_distance_m >= 0` 검증 후 FK runtime에서 `distance_open_runtime_m_ > distance_closed_runtime_m_` 검증

권장 수정:

- `use_q_ready_geometry && q_ready.size() == model.nq`일 때 fallback 거리 검증을 완화한다.
- 지금처럼 runtime FK 이후 `distance_open_runtime_m_ > distance_closed_runtime_m_`를 검증하는 것은 유지한다.

### 4. 중간: 직접 `grasp_teleop` 진입 방지는 ROS service 경로에만 적용됨

위치:

- `controllers/ros2_control/plato_ros_controller/src/plato_ros_controller.cpp:883`

ROS service 요청이 `grasp_teleop` 이름의 state를 가리키면 `grasp_ready` 이름의 state로 redirect하는 것은 좋다. state id 변경에도 안전하다.

다만 이 guard는 `request_state_callback()`에만 있다. 코드 내부에서 `ControlArchitecture::RequestState(grasp_teleop_id)`를 직접 호출하거나, 향후 다른 transition path가 추가되면 `grasp_ready`를 건너뛸 수 있다.

현재 구조에서는 큰 문제는 아니지만, 설계 의도가 "grasp_teleop은 항상 grasp_ready를 거쳐야 한다"라면 FSM layer나 state policy layer에 넣는 것이 더 강하다.

권장 수정:

- 단기: 현재 service guard 유지.
- 중기: `grasp_teleop`에 `requires_entry_state: grasp_ready` 같은 config/policy를 두거나, `ControlArchitecture::RequestState(name)` 경로에서 name 기반 redirect를 공통 처리한다.

### 5. 낮음: `ready_center_base_`는 계산되지만 아직 사용되지 않음

위치:

- `plato_robot_system/src/task/grasp_task.cpp:392`
- `plato_robot_system/src/task/grasp_task.cpp:595`

`ready_center_base_`는 runtime field로 추가되어 계산된다. 하지만 실제 task error는 `r = p_b - p_a`의 close/lateral projection만 사용한다. 즉 현재 제어는 두 fingertip 사이 상대 vector를 맞추는 구조이고, fingertip pair의 absolute center position은 제어하지 않는다.

지금 목표가 "fingertip 간 거리와 상대 lateral alignment"라면 괜찮다. 그러나 앞으로 "두 fingertip virtual point의 lateral position"이 hand base 기준 절대 위치를 의미한다면 center task가 별도로 필요하다.

권장 수정:

- 현 의도가 상대 alignment라면 `ready_center_base_`를 제거하거나 주석으로 "future absolute-center task용"이라고 명시한다.
- 절대 위치까지 제어하려면 `center = 0.5 * (p_a + p_b)`에 대한 Jacobian과 target을 별도 task row로 추가한다.

### 6. 낮음: ROS wrapper 내부 command default가 새 semantics와 다름

위치:

- `controllers/ros2_control/plato_ros_controller/include/plato_ros_controller/plato_ros_controller.hpp:76`

`GraspTaskCommand`와 `GraspTeleopInput`의 `u` default는 새 convention에 맞게 `0.0`으로 바뀌었다. 하지만 ROS controller 내부 `GraspTeleopCommand`는 아직 `u{1.0}`이다.

현재 callback에서는 메시지 값을 바로 덮어쓰기 때문에 실사용 영향은 작다. 그래도 parallel grasp semantics에서 `0.0`은 closed, `1.0`은 open이므로, default 값이 남아 있으면 테스트나 future code에서 헷갈릴 수 있다.

권장 수정:

```cpp
double u{0.0};
```

### 7. 낮음: `q_ready` fallback 시 warning이 없음

위치:

- `plato_robot_system/src/task/grasp_task.cpp:216`
- `controllers/aristo_controller/src/config/aristo_config.cpp:391`

요구사항에는 `q_ready`가 없거나 disabled일 때 old behavior로 fallback하되, ROS node/config path에서는 clear warning을 내는 것이 좋다고 되어 있다. 현재 Aristo YAML path에서는 `grasp_ready.target_jpos`가 자동 복사되므로 대부분 문제는 없다.

하지만 다른 config에서 `use_q_ready_geometry: true`인데 `q_ready`가 비어 있으면 조용히 old distance fallback으로 동작한다.

권장 수정:

- `GraspTeleopState::ConfigureTask()` 또는 ROS controller configure 단계에서 `use_q_ready_geometry && q_ready.empty()`이면 `RCLCPP_WARN`을 낸다.
- core `GraspTask`는 logger가 없으므로 fail-fast 또는 status field만 유지하는 편이 낫다.

## 잘 된 부분

- `q_ready` FK는 `pinocchio::Data data(model)`를 새로 만들어 계산하므로 live `RobotSystem` state/data를 오염시키지 않는다.
- `q_posture` 생략 시 `q_ready`에서 active joint posture를 추출하는 방향은 중복 YAML을 줄여준다.
- `next_state: grasp_teleop` 이름 기반 transition은 state id 변경에 강하다.
- `grasp_teleop` 직접 service request를 `grasp_ready`로 redirect하는 방어는 실험 중 실수 방지에 유용하다.
- `w_close`, `w_lateral`, `w_posture`가 YAML에서 tunable하게 정리된 것은 bring-up 중 튜닝에 좋다.

## 테스트 커버리지 평가

현재 통과한 테스트:

```bash
colcon build --symlink-install --packages-up-to plato_ros_controller
./build/plato_robot_system/grasp_task_test
./build/aristo_controller/aristo_config_test
```

현재 테스트가 잘 잡는 것:

- `q_ready` FK 기준 ready/open distance가 status에 들어가는지
- `u=0.5`, `phi=0.0`에서 ready pose task error가 거의 0인지
- 잘못된 크기의 `q_ready`를 reject하는지
- YAML `next_state: grasp_teleop`이 id로 resolve되는지
- `grasp_task.q_ready`가 `grasp_ready.target_jpos`에서 채워지는지

추가하면 좋은 테스트:

1. ROS controller configure path에서 `q_ready`가 `map_joint_positions_to_model_q()`를 거쳐 model q order로 들어가는지 확인.
2. `grasp_ready -> grasp_teleop` 자동 전환 후 첫 tick에서 latest teleop command가 default로 덮이지 않는지 확인.
3. `use_q_ready_geometry: false`일 때 old `distance_open_m`/`distance_closed_m` fallback이 새 normalized convention으로 정상 동작하는지 확인.
4. `q_ready` pose에서 `phi=0.0`/`1.0`이 예상한 sign으로 lateral offset을 만드는지 확인.

## 권장 작업 순서

1. `grasp_teleop_config.grasp_task.q_ready`를 `map_joint_positions_to_model_q()`로 변환한 뒤 `ConfigureTask()`에 넘긴다.
2. `GraspTeleopState::OnEnter()`가 최신 external input을 첫 tick에서 덮지 않도록 수정한다.
3. `PlatoRosController::GraspTeleopCommand::u` default를 `0.0`으로 바꾼다.
4. `q_ready` fallback warning을 ROS configure path에 추가한다.
5. `ready_center_base_`의 의도를 결정한다. 상대 fingertip alignment만 제어할 거면 주석/제거, absolute center도 필요하면 별도 task row 추가.

## 하드웨어 실행 전 체크리스트

- `grasp_ready` state id가 바뀌어도 `next_state: grasp_teleop`이 유지되는지 확인한다.
- `/plato2/parallel_grasp_controller/commands` publisher가 새 semantics를 쓰는지 확인한다.
  - `u=0`: closed
  - `u=0.5`: ready/parallel
  - `u=1`: open
  - `phi=0`: extended/max lateral
  - `phi=0.5`: centered
  - `phi=1`: flexed/min lateral
- `grasp_ready`에 도달한 뒤 `grasp_teleop` 진입 로그가 `[FSM] entering state ... name=grasp_teleop`으로 찍히는지 확인한다.
- 첫 command는 작은 값으로 시작하거나 torque/velocity limits를 낮게 유지한 상태에서 확인한다.
