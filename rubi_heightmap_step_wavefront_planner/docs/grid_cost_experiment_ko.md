# Grid A*의 기존 height cost와 CoT cost 비교

2026-10-26. 작업 브랜치: `codex/cot-grid-comparison-20261026`.

**현재 frozen 지도 실험은 [Frozen 실행 안내](grid_cost_frozen_experiment_ko.md)를 따른다.** 아래 온라인 FastDEM 예시는 별도 입력 방식이다. 기존 frozen 사이트에 이 예시의 5 cm-only world를 혼용하지 않는다.

## 기존 실험 보존

다음 기존 명령, launch, 설정 파일은 수정하지 않았다.

```bash
ros2 launch rubi_heightmap_step_wavefront_planner hybrid_grid_navigation.launch.py use_sim_time:=true
ros2 launch rubi_heightmap_step_wavefront_planner hybrid_sampling_navigation.launch.py use_sim_time:=true
ros2 launch rubi_heightmap_step_wavefront_planner hybrid_grid_trg_comparison.launch.py use_sim_time:=true
```

기존 `hybrid_grid_cot_cost.launch.py`와 lookup calibration도 보존했다. **이전 lookup 모델은 이번 Flat–11 cm, 240회 polynomial 모델과 다르다.** 이번 비교에는 아래 새 명령을 사용한다. 공통 C++ evaluator에 선택 기능을 추가했지만 새 polynomial 파라미터가 없는 기존 실행의 비용식은 유지된다. Sampling의 비용식도 유지된다.

이번 전용 파일은 `grid_cost_experiment.launch.py`, `grid_cost_experiment.yaml`, `grid_cost_experiment_controller.yaml`, `grid_cost_experiment_costmap.yaml`이다. 세 설정은 기존 설정을 복사해 분리했고, planner에는 기존 Local Relief 실험의 10 cm hard rejection을 두 모드 공통으로 활성화했다. 이후 비교 실험의 설정을 바꿀 때 이 복사본만 수정한다. 기존 FastDEM/localization/Gazebo 소스는 변경하지 않았다.

## 온라인 height map을 사용하는 별도 실행 예시

Gazebo → FastLIO localization + FastDEM → costmap/heightmap → 8-neighbor grid A* → 검증된 경로 후처리 → Simple Pure Pursuit → `/cmd_vel`.

두 비용 모드를 동시에 제어기로 실행하지 않는다. 기존 DWB/다른 navigation controller도 종료한 상태에서 한 모드씩 실행한다. planner-only 비교는 `launch_controller:=false`로 가능하다.

### 1. 환경: 각 터미널에서

```bash
source /opt/ros/humble/setup.bash
source ~/rubi_ws/src/ws_livox/install/setup.bash
source ~/rubi_ws/install/setup.bash
source ~/ros2_ws_nav/install/setup.bash
ros2 pkg prefix rubi_heightmap_step_wavefront_planner
```

마지막 prefix가 `/home/kim/ros2_ws_nav/install/rubi_heightmap_step_wavefront_planner`인지 확인한다. 다른 workspace를 뒤에 source하면 다른 패키지를 실행할 수 있다. FastDEM 터미널은 기존 `grid_map_ws` 환경을 사용한다.

### 2. 기존 시뮬레이션 / localization / heightmap 명령 사용

각각 별도 터미널에서 실행한다. 기존 world 경로는 유지하며 비교 A/B에 같은 world를 사용한다.

```bash
ros2 launch rubi_gazebo_sim rubi_gazebo_terrain_lidar.launch.py \
  world:=/home/kim/worlds_height5_layout_clean.world \
  contact_logging:=true gui:=true paused:=false launch_joy_node:=false

ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  use_sim_time:=true publish_2d_map:=false use_rviz:=true

ros2 launch fastdem_ros2 run.launch.py \
  global_mapping:=true input_scan:=/livox/lidar_PointCloud2 \
  use_sim_time:=true rviz:=true
```

`walk_ready`, `policy_on`은 기존 절차대로 수행한다. 이 작업에서는 시뮬레이터나 policy를 자동으로 켜거나 이동 명령을 보내지 않았다. 시작 자세·reset·policy·command scale을 동일하게 유지한다.

### 3. 새 navigation 명령: 둘 중 하나

```bash
# A. 기존 geometric height cost
ros2 launch rubi_heightmap_step_wavefront_planner grid_cost_experiment.launch.py \
  use_sim_time:=true cost_mode:=vanilla \
  scenario:=layout5 run_id:=layout5_vanilla_01

# B. 240회 데이터의 3차 excess-CoT cost
ros2 launch rubi_heightmap_step_wavefront_planner grid_cost_experiment.launch.py \
  use_sim_time:=true cost_mode:=cot cot_degree:=3 \
  scenario:=layout5 run_id:=layout5_cot3_01
```

`cot_degree:=1`, `2`, `3`을 지원한다. 기본 height weight는 기존 값 5.0이며, 두 모드에서 동일하다. 바꾸려면 **양쪽 명령에 동일하게** `height_weight:=...`를 넣는다. 물리적 등가 거리 해석에 가까운 기본 스케일은 weight=1이지만, 기존 설정과 같은 비교를 위해 자동으로 weight를 바꾸지는 않았다. 테스트한 synthetic map은 경로 선택 검증을 위해 양쪽 weight=1을 사용했다.

같은 weight만으로 두 높이 점수의 크기까지 같아지는 것은 아니다. CoT 변환값이 기존 geometric score보다 커서 우회가 늘 수 있다. 따라서 “fitting된 곡선의 모양이 좋아서 개선됐다”까지 주장하려면 별도의 검증 시나리오에서 **vanilla와 CoT 모두 weight sweep을 수행**하거나 기준 높이에서 penalty 크기를 맞춘 ablation도 함께 비교한다. 본 실험의 결과를 보고 유리한 weight만 고르는 것은 피한다.

각 실행에는 새로운 `run_id`가 필요하다. 이미 존재하는 결과 폴더는 덮어쓰지 않고 측정 노드가 시작을 거부한다. 새 goal을 보내기 전 측정 노드가 정상인지 확인한다.

### 4. 기록 → 시작 → goal → 종료

```bash
# navigation 명령의 run_id와 동일하게 입력. 별도 터미널에서 실행하고 유지.
ros2 run rubi_heightmap_step_wavefront_planner record_grid_cost_experiment.sh layout5_vanilla_01

# 센서·지도 준비 완료 후, goal을 보내기 직전에 측정 시작
ros2 service call /rubi/experiment/start std_srvs/srv/Trigger '{}'

# RViz의 기존 goal 도구 또는 /rubi/planner_comparison/goal 사용
# 도착을 확인한 후:
ros2 service call /rubi/experiment/finish_success std_srvs/srv/Trigger '{}'

# 넘어짐·timeout·진행 불가라면 위 success 대신:
# ros2 service call /rubi/experiment/finish_failure std_srvs/srv/Trigger '{}'

ros2 topic echo /rubi/experiment/run_metrics --once
```

종료 서비스는 종료 시각을 고정하고, joint/odom이 그 시각을 포함할 때까지 최대 3초 기다린 후 결과를 발행한다. **서비스가 success=true라는 것은 종료 요청을 받았다는 뜻**이며, 실제 데이터 정상성은 `run_metrics.qc_pass`로 확인한다. 결과를 확인한 후 recorder를 종료한다.

성공 여부는 현재 operator service로 기록한다. 자동 goal 판정 구현이 아니므로 두 방법 모두 같은 도착 기준(예: 목표까지 0.20 m 이내, 동일 timeout)을 사전에 고정한다. goal 이후 수동 종료 지연도 work에 포함되므로 짧고 일정하게 유지해야 한다. 같은 시작/끝 경계를 적용하고, 지연이 크면 bag의 동일한 사전 정의 기준으로 재분석한다. 이 방식으로 수집한 결과를 자동 종료 기준으로 수집했다고 서술하지 않는다.

한 launch는 한 trial이다. 다음 trial은 reset하고 새 run_id로 실행한다. run 중 simulator reset/역행 시각은 QC 실패다. 측정 시작 후는 모든 주행·정지·재계획·실패 회복의 work가 포함된다. step calibration의 `x=-1→0 m`와 달리 navigation 전체를 평가한다.

결과 위치:

```text
~/cot_experiment/navigation_cost_comparison/<run_id>/events.jsonl
~/cot_experiment/navigation_cost_comparison/<run_id>/result.json
~/cot_experiment/navigation_cost_comparison/bags/<run_id>/
```

`output_dir:=...`로 측정 결과 위치 변경 가능. recorder는 `RUBI_COST_EXPERIMENT_DIR` 환경 변수로 루트를 변경한다. bag에는 지도를 포함하므로 용량이 커질 수 있다.

### 5. 결과 집계

```bash
ros2 run rubi_heightmap_step_wavefront_planner summarize_navigation_experiments.py \
  ~/cot_experiment/navigation_cost_comparison
```

`comparison_trials.csv`, `comparison_summary.csv`, `comparison_summary.json`을 만든다. 성공+QC 통과 trial의 mean±sample SD를 계산하고, 실패 수·QC 실패 수·미완료 run을 별도로 남긴다. **실패한 run을 없애고 성공률을 계산하지 않는다.** 미완료 run의 실제 결과를 확인하기 전에는 finalized-run 성공률을 최종 논문 수치로 쓰지 않는다. model hash/degree/weight가 다른 결과는 합치지 않는다. 전체 비교 조건이 같은지는 JSON의 parameters와 기록한 실험 조건을 확인해야 한다.

## 실제 비용식

공통 edge 비용:

```text
c(u,v) = w_d * length_xy + w_h * height_score
       + w_i * inflation_score + w_s * Sobel_exposure
```

공통 hard gate, 장애물 costmap, grid, 후처리, 제어기, 재계획 조건을 동일하게 유지한다. 거리에 대한 octile heuristic을 사용하며 추가 비용이 음수가 되지 않도록 polynomial의 전체 사용 구간을 검사한다.

Vanilla의 높이 점수는 단순 `|Δz|`가 아니다. 기존 코드는:

```text
q = clamp((|Δz| - h_noise)/(h_max - h_noise), 0, 1)
height_score = h_max * q^p   if |Δz| > h_noise
height_score = 0            otherwise
```

현재 공통 설정은 `h_max=0.10 m`, `h_noise=0.01 m`, `p=2`. 상승·하강에 동일하게 적용된다.

CoT 모드에서는 상승 구간만 교체한다:

```text
f(h_cm) = predicted excess CoT_abs
equivalent_distance(H) = 1.0 m * f(100 * H_m) / 0.2397005087210085
H = max(|Δz|, local_relief_at_edge_endpoints)
height_score_ascent = equivalent_distance(H) * clamp(Δz/H, 0, 1)
```

`H > h_noise`이고 `Δz > 0`일 때 적용한다. 주변 relief와 상승 분배는 FastDEM에서 퍼진 단차를 다루기 위한 기존 Grid lookup 방식의 휴리스틱을 따른다. 한 단차가 여러 cell로 퍼졌을 때 각 cell마다 전체 단차 비용을 중복 부과하지 않으려는 목적이다. **개별 물리적 단차를 완벽히 분리한 event detector도, 일반적인 grid 해상도 불변성의 보장도 아니다.** 가까운 여러 단차·연속 경사·잡음 지형에서는 추가 검증이 필요하다. `height_edge_event_count`는 물리적인 계단 개수가 아니다.

하강 데이터는 fitting하지 않았으므로 하강 비용은 vanilla 식을 그대로 쓴다. Grid 경로 smoothing/validation에서도 동일한 상승 식을 사용하며 Sampling evaluator는 바꾸지 않는다.

모델 파일 `cot_fit_240trials_20261019.json`에는 원본 fitting JSON의 SHA256, input SHA256, 1/2/3차 계수와 RMSE를 보존했다. 입력 높이 단위는 **cm**, heightmap 입력은 **m**이므로 100배 변환한다. 11 cm 초과는 extrapolation하지 않고 해당 상승 edge를 유효하지 않게 처리한다.

CoT 모드만 모델 범위 밖을 막으면 A/B의 통과 가능 영역까지 달라지므로, **새 launch는 두 모드 모두 동일하게 `H≤0.11 m`인 상승 후보로 제한**한다(`evaluation.grid_comparison_max_event_height_m`). 이 범위는 이번 모델 비교의 적용 범위이며 안전성 실험으로 얻은 한계가 아니다. 기본값 0은 기능 비활성화이므로 기존 launch에는 적용되지 않는다.

| Degree | Fitting RMSE | Leave-one-height-out RMSE |
|---|---:|---:|
| 1 | 0.01663171 | 0.01761973 |
| 2 | 0.01654194 | 0.01907216 |
| 3 | 0.01286193 | 0.02527201 |

3차는 fitting RMSE 최소인 후보라 기본 선택으로 넣었다. 교차검증은 1차가 더 낮다. **3차가 navigation에 최적이라는 결론은 아직 없다.** 3차는 약 10.89–11 cm에서 아주 조금 감소하므로 단조 증가 모델이라고 표현하지 않는다. 검증용 시나리오에서 degree/weight를 결정한 후 본 비교에서는 고정한다.

두 가지 현재 설정의 경계를 유의해야 한다. calibration은 11 cm까지지만 공통 planner의 기존 인접 높이 통과 한계는 **10 cm**이다. 또 1 cm 이하 local height는 noise floor로 비용이 생략될 수 있다. 11 cm 통과나 1 cm 감지까지 평가하려면 기존 launch가 아닌 이번 공통 실험 설정에서 양쪽 기준을 함께 바꾸고, 지도 noise와 로봇 통과 가능성을 먼저 검증한다. fitting 범위를 통과 안전성의 근거로 사용하지 않는다.

### Local Relief 재검토 및 설정 누락 수정 — 2026-10-26

초기 새 비교 설정은 일반 navigation YAML을 복사하면서 `evaluation.local_relief_hard_reject_enabled`를 누락했다. C++ 기본값이 false였기 때문에 CoT 비용 산출에는 주변 relief가 들어가지만, **Local Relief에 의한 10 cm hard reject는 꺼진 상태**였다. 기존 `hybrid_grid_cot_cost.yaml`과 relief 전용 실험 설정에는 true로 설정되어 있었다. 이 누락을 새 비교 YAML에만 수정했다. 이전 launch/YAML은 변경하지 않았다.

현재 vanilla/CoT 공통 설정은 다음과 같다.

```yaml
evaluation.local_relief_hard_reject_enabled: true
evaluation.local_relief_threshold_m: 0.10
evaluation.local_relief_first_window_radius_m: 0.10
evaluation.local_relief_second_window_radius_m: 0.10
evaluation.local_relief_lower_quantile: 0.10
evaluation.local_relief_upper_quantile: 0.90
evaluation.local_relief_min_observed_ratio: 0.70
evaluation.local_relief_critical_cell_count: 3
```

1차 원형 창에서 관측률 70% 이상인 경우 `R(c)=Q90(z)-Q10(z)`를 계산한다. 이것은 max−min보다 극단값의 영향을 줄이는 높이 범위다. 2차 원형 창에서는 유효한 R의 최대값 `R_max`와 `R>0.10 m`인 cell 수 `n`을 구한다. 코드의 supported 값은 `R_supported = min(R_max, (n/3)*R_max)`이며, `R_supported>0.10 m`이면 hard reject한다. 이 조건은 단순히 “항상 3개 이상이어야 차단”이라는 논리식과는 다르다. 이 값은 cell별 주변 통계이지, 물리적 계단마다 고유 ID를 붙이는 segmentation 결과는 아니다.

Grid는 edge 양 끝의 증거를 검사하고, smoothing/revalidation도 같은 Local Relief hard gate를 사용한다. CoT 비용에는 supported 값이 아닌 `max_raw_relief_m`이 사용되므로, hard reject 높이와 모델 입력 H는 항상 동일한 값이라고 쓰면 안 된다. 국소 창에 전체 단차가 잘 포함되고 상승 중 H가 일정하면 `sum(Δz/H)=1`이 되어 단일 상승 비용으로 분배된다. 창 밖까지 퍼지거나 복수 단차가 겹치면 이 조건이 성립하지 않을 수 있다.

실제 C++ evaluator를 이용한 합성 검증(해상도 5 cm, 현재 반경/분위수 설정):

| 전체 상승 | 선형으로 퍼진 폭 | 최대 인접 변화 | 최대 local relief | Local Relief 결과 |
|---|---:|---:|---:|---|
| 11 cm | 10 cm (2개 edge) | 5.5 cm | 11 cm | 꺼짐: 통과 / 켜짐: 차단 |
| 11 cm | 15 cm (3개 edge) | 3.67 cm | 7.33 cm | 켜져 있어도 차단되지 않음 |

두 비용 모드 및 off-grid 경로 검증에서 위 결과를 확인한다. 따라서 “Local Relief가 여러 cell에 퍼진 단차를 검출하도록 설계되었다”는 설명은 맞지만, “퍼짐 폭과 관측 상태에 관계없이 원래 물리적 단차 높이를 완벽히 복원한다”는 설명은 맞지 않는다. 반경/분위수는 실제 FastDEM의 퍼짐 폭을 확인하고 정해야 한다. 이번 수정에서는 기존 값들을 복구했으며 임의로 반경을 키우지 않았다.

추가 관찰: 같은 합성 데이터에서 급격한 9 cm 경계는 현재 node evidence outlier 규칙으로도 거절되는 경우가 있었다. 이는 Local Relief와 별도인 증거 일관성 조건이다. 모든 통과/차단을 Local Relief 때문이라고 해석하지 말고 rejection reason을 함께 확인한다. Local Relief 위험 영역과 로봇 사이에 20 cm clearance를 두는 별도 기능은 이번 일반 navigation 기반 프로필에서 여전히 비활성화되어 있다. 이것은 Local Relief 자체의 10 cm 높이 차단과 별개의 옵션이다.

## 무엇을 논문에서 비교할까

핵심 주장은 **동일 시작·목표에서, 성공률을 유지하면서 실제 절대 mechanical work를 줄이는가**이다. own objective의 숫자만 낮아졌다는 것은 이 주장의 근거가 아니다.

| 지표 | 용도 | 데이터 |
|---|---|---|
| Success rate, failure/timeout count | 도착 가능성 우선 | 명시적인 동일 도착/실패 기준 |
| Total absolute mechanical work [J] | 핵심 효율 지표 | 6관절 `sum(abs(torque*velocity))` 시간 적분 |
| Executed path length [m] | 우회와 추종 오차 설명 | ground truth BODY XY 누적 이동거리 |
| Planned / tracking path length [m] | 계획·후처리의 경로 차이 | 첫 accepted raw/tracking path |
| Traversal time [s] | 느려져서 얻은 효과인지 확인 | 동일 시작/종료 이벤트의 simulation time |
| Mechanical CoT [-] | 이동거리 대비 비용 | `W_abs / (m*g*executed_length_xy)` |
| Mean / peak absolute power [W] | 부하의 크기 설명 | joint samples |
| Planning time [ms], replans | 계산 부담 | plan_metrics, wall computation time |
| Cumulative ascent / descent [m] | 지형 선택 설명 | raw planned path의 Z 변화 합 |

`W_abs`와 CoT를 같이 봐야 한다. 긴 평지 우회로 CoT가 낮아져도 총 work는 늘 수 있다. `CoT_net_displacement`는 실제 시작–끝 XY 직선 변위를 분모로 하는 보조값이며, 정확히 공통 목표 거리로 정규화한 값과는 구분한다. 필요하다면 동일 시작–목표 거리 D를 고정해 `W_abs/(mgD)`를 후처리한다. 로봇의 electrical energy나 battery consumption을 측정한 것은 아니다.

```text
P_abs(t) = sum_i |tau_i(t) * qdot_i(t)|
P_pos(t) = sum_i max(tau_i*qdot_i, 0)
P_neg_magnitude(t) = sum_i max(-tau_i*qdot_i, 0)
W_abs = W_pos + W_neg_magnitude
W_signed = W_pos - W_neg_magnitude
```

`abs(sum_i tau_i*qdot_i)`와 구별한다. JointState의 이름으로 여섯 관절을 정렬하고, effort와 velocity를 같은 message에서 곱한다. header simulation time의 사다리꼴 적분과 경계값 선형 보간을 사용한다. 기본 QC는 joint gap 0.05 s, odom gap 0.10 s, 누락 관절/NaN/역행·중복 시간/경계 미도달/frame 변경을 검사한다. 이 gap 값은 문헌에서 얻은 표준이 아니라 **로그 유효성 검사 설정**이며 결과 parameters에 저장한다. QC 실패 시 work를 정상 수치로 발행하지 않는다.

계획 비용은 `weighted_equivalent_m`인 목적함수다. `total_cost`, `height_score_m`의 모드별 값은 같은 energy 척도라고 직접 비교하지 않는다. `planned_cumulative_ascent_m`은 두 방법에서 같은 기하학적 정의이므로 표의 누적 상승량에 사용할 수 있다. 반복 replan의 경로 비용이나 길이를 모두 더하지 않는다. 처음 계획 길이와 실제 주행 길이를 별도로 보고한다.

## 추천 실험 구성

1. 같은 frozen map의 계획 비교: 비용식만 바꿔 경로·지형 선택·계산 시간을 확인한다. 같은 map hash가 기록되어야 한다.
2. Gazebo 실제 추종 비교: 동일 world, 시작 자세, goal, policy checkpoint, physics, gain, command scale, 제어기 및 지도 파라미터를 사용한다. A/B 순서를 번갈아 실행하고 매번 reset한다. online mapping일 때 지도 hash가 서로 달라질 수 있으므로 비용식만의 효과와 지도 변화 효과를 구분한다.
3. 평지 대조군, 짧은 높은 단차 경로 vs 긴 평지 우회, 여러 낮은 단차 vs 적은 높은 단차 등 서로 다른 trade-off를 포함한다. fitting에 쓴 단일 1 m step 실험만 재현하는 것으로 navigation 검증을 대신하지 않는다.
4. 가능하면 시나리오별 같은 횟수의 반복 시행을 확보한다. 먼저 짧은 pilot로 endpoint/로그를 검증하고, degree/weight를 고정한 후 본 실험을 수집한다. 예쁜 결과만 선별하지 않고 명확한 QC 제외 이유와 실패를 보존한다.
5. 표에는 성공률 + W_abs + 실제 거리 + 주행 시간 + CoT + planning time을 우선 넣는다. 평면도에 두 경로와 단차 높이를 표시하고, W_abs/거리 분포를 추가하면 경로 선택이 work 차이로 연결되는지 설명할 수 있다.

실물에서는 실제 effort 신호가 어떤 토크(명령/추정/측정)를 의미하는지 별도로 확인하고, 시뮬레이션 값과 같은 이름으로 혼합하지 않는다. 현재 fitting은 고정 속도의 단일 상승 실험에 기반한다. 회전·가감속·여러 근접 단차·policy 변경에 대한 정확한 energy 예측 모델로 검증된 상태는 아니다.

문헌상 동기는 에너지 모델과 통과 가능성을 분리해 계획하고 실제 로봇에서 검증하는 접근이다. [Brandão et al., Humanoids 2015](https://www.martimbrandao.com/papers/Brandao2015_humanoids.pdf)는 학습한 에너지/feasibility/slippage 모델을 footstep planning에 사용한다. [VOCALoco, RA-L 2025](https://arxiv.org/abs/2510.23997)는 viability와 고정 horizon의 예상 CoT를 함께 평가한다. 두 논문은 에너지와 성공 가능성을 함께 평가할 근거이며, **이번 grid edge 분배식이나 1 m calibration의 navigation 가산성 자체를 검증해 주는 자료는 아니다.**

## 토픽

| Topic | Type | 내용 |
|---|---|---|
| `/rubi/experiment/plan_metrics` | std_msgs/String (JSON) | 모드·계수·hash·지도 hash·성공/채택·길이·비용 성분·계산 시간·설정 |
| `/rubi/experiment/power_abs_W` | std_msgs/Float64 | 여섯 관절 절대 power 합, 표시용 |
| `/rubi/experiment/power_signed_W` | std_msgs/Float64 | 여섯 관절 signed power 합, 표시용 |
| `/rubi/experiment/run_metrics` | std_msgs/String (JSON) | 구간·outcome·QC·실제 work/CoT/거리·계획 요약 |
| `/rubi/experiment/events` | std_msgs/String (JSON) | ready/start/finish/plan 이벤트 |

Float64 power에는 header가 없으므로 정밀 재적분은 원본 JointState header를 사용한다. 관절별 power는 bag의 원본 여섯 관절 신호로 재구성한다.

## 검증 상태

빌드, 기존 evaluator/A* 테스트, 새 polynomial 단위 변환·하강 동일성·후처리 일관성 테스트, mixed-sign work·정확한 경계·결측/역행 QC 테스트를 수행했다. 합성 ROS 테스트에서 같은 costmap/heightmap의 두 경로, JSON 비용 합, 6 W 입력의 W_abs 및 odom 이동거리, `/cmd_vel` 발행 0개를 확인했다. 이는 **Gazebo 보행 실험 결과가 아니다**.

기존 `test_config_contract.py`의 설정 기대값 두 건은 수정 전 HEAD에서도 실패한다: controller의 20 Hz 기대값과 실제 10 Hz, 구형 CoT YAML의 height weight 1 기대값과 실제 0. 사용자 요청대로 기존 설정/테스트를 바꾸어 덮지 않고 차이를 보존했다. 다른 기존 검사와 이번 새 기능 검사는 통과했다. 상세 실행 로그는 작업 결과 폴더의 verification 파일을 참조한다.
