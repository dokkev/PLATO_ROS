# Deep Code Review - Jenga Continuous qddot MPPI

작성일: 2026-06-11

## 결론

이번 변경으로 robust grasp의 main path는 discrete candidate selector가 아니라 continuous qddot MPPI가 되었다. 컨트롤러는 qddot sequence를 샘플링하고, horizon 전체 cost를 누적하고, min-cost offset을 둔 soft MPPI weight로 전체 sequence를 평균낸 뒤 첫 qddot만 command로 내보낸다. Jenga object prior와 geometric contact-support evaluator는 그대로 사용하되, object disturbance가 horizon rollout에 들어가고 hemisphere surface gap 및 penetration cost가 추가되었다.

아직 하드웨어/빌드 검증은 하지 않았다. repo 지침상 명시 요청 없이 `colcon` build/test를 실행하지 않았다.

## High

1. Runtime 성능은 아직 미검증이다.

   기본 Aristo 설정이 `num_rollouts: 128`, `horizon_steps: 3`으로 바뀌었다. 각 sample-step마다 object support evaluator가 Pinocchio FK, candidate hemisphere vector, per-sensor accumulator를 만든다. 100 Hz 목표에서 충분한지는 아직 측정되지 않았다. 최소 검증은 `mppi_core` 단독 test 이후 fake hardware launch에서 solve time log를 보는 것이다.

2. Continuous MPPI policy-level behavioral tests는 아직 일부만 있다.

   추가된 테스트는 zero-noise stable contact가 near-zero qddot를 내는지, soft weight가 정상인지, qdot/q limit이 적용되는지, radius/gap/penetration cost가 계산되는지를 잠근다. 하지만 low preload가 closing qddot를 만드는지, weak-side force가 side bias를 만드는지, edge contact가 align qddot로 이어지는지는 아직 회귀 테스트가 없다.

## Medium

3. Contact geometry는 아직 primitive box proxy다.

   `jenga_block.urdf` URI는 config에 있지만 surface query는 `primitive_size_m` box를 사용한다. MVP에는 적합하지만 URDF mesh와 YAML primitive size가 달라지면 cost/visualization이 어긋난다.

4. Hemisphere radius 기본값은 0이다.

   evaluator는 `HemisphereGeometry::radius_m` 또는 `object_support.hemisphere_radius_m`로 gap을 계산한다. 현재 Aristo YAML은 `hemisphere_radius_m: 0.0`이라 기존 center-distance behavior와 같다. 실제 tactile dome 반지름을 알면 config에 넣어야 penetration/contact margin 의미가 좋아진다.

5. RViz visualization은 main continuous rollout 전체를 아직 보여주지 않는다.

   Jenga prior box, measured tactile contact, contact-force arrow, selected one-step motion marker는 있다. 하지만 MPPI sample cloud, weighted rollout trace, predicted 3x4 contact mask, distance field, ESS/cost overlay는 아직 없다.

## Low

6. Discrete selector는 baseline으로 남아 있지만 튜닝 상태가 main path와 다를 수 있다.

   `control_mode: discrete_action_selector`는 parse되고 기존 action library도 살아 있다. 다만 continuous MPPI 튜닝이 진행되면 discrete baseline의 cost/threshold가 drift할 수 있으므로 비교 실험 전에 별도 config snapshot이 필요하다.

## 구현 요약

- `ContinuousQddotMppiController` 추가: qddot sequence sampling, rollout, soft weighting, nominal shift, first-control output.
- `RobustGraspPolicy`에 `control_mode` 추가: `continuous_qddot_mppi` 기본, `discrete_action_selector` baseline.
- MPPI config 확장: `action_noise_clip`, `qdot_limit`/qdot bounds parsing.
- Object support evaluator 확장: hemisphere radius 기반 `gap = signed_distance - radius`, penetration cost.
- Status/log 확장: sample count, horizon, lambda, best sample cost, weighted cost, min/mean/max, ESS, qddot nominal/best/weighted, penetration/control/rate cost.
- Aristo/generic robust grasp YAML을 horizon 3, 128 samples continuous MPPI로 변경.
- Unit tests 추가/수정: soft weights, qdot/q limit, radius/gap/penetration, config parsing, stable zero-noise continuous MPPI.

## 검증

실행함:

```bash
git diff --check -- mppi_core controllers/aristo_controller
rg -n "disturbed_rollout|tactile_transition:|StepGraspStateWithDisturbance\\(|horizon_steps: 1|control_mode" mppi_core controllers/aristo_controller -g'*.yaml' -g'*.cpp' -g'*.hpp'
```

실행하지 않음:

```bash
cd ~/workspace/plato_ws
colcon test --packages-select mppi_core
colcon build --symlink-install --packages-up-to plato_ros_controller
```

## 다음 검증 순서

1. `colcon test --packages-select mppi_core`
2. `colcon build --symlink-install --packages-up-to plato_ros_controller`
3. fake hardware에서 robust grasp 진입 후 `solve_ms`, `ess`, `qddot_norm`, `penetration_cost` 로그 확인
4. RViz에서 Jenga prior와 tactile/contact markers frame alignment 확인
