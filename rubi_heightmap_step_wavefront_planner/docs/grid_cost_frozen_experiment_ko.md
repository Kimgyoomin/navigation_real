# Frozen height map 기반 Grid A* 비용 비교 실행 안내

2026-10-26 검토. 이번 안내는 `ros2_ws_nav`의 새 polynomial/vanilla 비교용이다.
기존 `legged_bringup navigate.launch.py`, hybrid navigation 명령, 지도 원본은 보존했다.

## 1. 찾은 world와 지도

이전 shell 실행 기록과 2026-10-13/14 지도 평가 기록에 나타난 world는
`/home/kim/worlds_height515_layout_clean.world`이다. 현재 파일 SHA-256은
`200acc1fe7b0bce8717ac49162a0dd50eaeacc37caf6e79b92466ad9da4eb7ea`이며 당시 기록과 같다.
이 world는 **5 cm 단차 3개, 15 cm 단차 2개**와 복도로 구성된다.
`worlds_height5_layout_clean.world`는 다섯 단차가 모두 5 cm인 다른 world이다.

`rubi_gazebo_terrain_lidar.launch.py`를 인자 없이 실행하면 기본
`rubi_navigation_mid360_imu.world`를 선택한다. 이는 위 단차 world와 다르고,
기본 `paused=true`다. 따라서 world와 paused를 명시한다.
`scenario:=layout5`는 결과 파일에 남기는 라벨이며 world/지도를 선택하지 않는다.

| 자료 | 확인된 파일 |
|---|---|
| Frozen 사이트 묶음 | `/home/kim/legged_robot_ws/maps/sim_site_v1/site.yaml` |
| Height map | 같은 폴더의 `height.rghm` |
| Localization PCD | 같은 폴더의 `localization.pcd` |
| 2D occupancy | 같은 폴더의 `occupancy.yaml` + `floor.pgm` |
| Height 원 취득본 | `/home/kim/legged_robot_ws/maps/sim_lio_snapshot/run_1792052351250393480/map.rghm` |

높이 지도는 5 cm 해상도, usable cell 52,827개, frame `map`, 원 취득 sim stamp
197.502 s이다. 사이트 파일의 네 자산 SHA-256과 실제 파일이 일치하며 height 원본과도 같다.
Height SHA-256: `45a5785bea97b9e71b4c03d79be753d75dbbbb775838886c177a82f21a9b0fdc`.
PCD는 기존 `2026_07_22_10_43_57.pcd`와 같은 해시다.

**World 연결의 근거와 한계:** 과거 명령과 관련 지도 평가 기록으로 위 world 사용을
확인했다. `sim_site_v1/site.yaml` 자체에는 world 해시가 없다. 파일 해시 검증이
실행 중 Gazebo world를 자동으로 인증하는 것은 아니다. 새 run의 `map_manifest.json`은
사용자가 지정한 `world_file` 경로/해시를 기록하며, Gazebo 실행에도 같은 파일을 지정한다.

기존 정합 검토는 `simulation_integration_trial_only`다. 육안 XY overlay 검사이며,
랜드마크/단차 높이 오차의 정량 인증, 미관측 영역 전체 검토, 실물 승인 기록은 없다.
따라서 현재 지도는 시뮬레이션 통합 시험용이다. 첫 경로에서는 PCD/height/occupancy
중첩과 15 cm 금지 단차의 실제 우회 여부를 확인한다. `map`과 `world`를 임의의
identity TF로 연결하거나 spawn world 좌표를 map goal 좌표로 그대로 사용하지 않는다.

## 2. 실행 구조

```text
Gazebo ──LiDAR/IMU──> FAST-LIO localization ──> map→body TF
                                  │
site.yaml ── occupancy ──> static + inflation costmap
          └─ frozen height ──────────────────────────────┐
                                                        ↓
                  Grid A* (vanilla 또는 fitted CoT) → Pure Pursuit → /cmd_vel
joint_states_500hz + ground_truth/odom + plan/events → 측정 JSON + rosbag
```

온라인 FastDEM/mapping을 켜지 않는다. 새 frozen publisher는 같은 관측 셀만
발행하고 원 취득 stamp를 유지한다. Frozen 모드에서는 height age/skew 검사만
제외하며 map frame과 live costmap freshness 검사는 유지한다. 실행 도중 다른 높이
데이터로 교체하려 하면 planner가 거부한다. 두 비용 모드는 같은 frozen height 해시를 쓴다.

현재 새 비교용 costmap은 **static + inflation**이다. 이전 `legged_bringup`의
obstacle cloud/collision monitor/watchdog를 포함한 전체 navigation stack과는 다르다.
이번 launch는 기존 `ros2_ws_nav` Grid + Simple Pure Pursuit 흐름에 frozen 입력을
연결한 것이다. 이동 장애물 대응이나 이전 legged 제어기와 동일한 성능을 주장하지 않는다.
기존 legged 실행은 보존되어 있으나, 그 설정의 옛 lookup CoT와 이번 240회 polynomial은 다르다.

## 3. 각 터미널 환경

새 터미널에서 아래 순서로 source한다. 같은 실험의 모든 터미널은 같은
`ROS_DOMAIN_ID`를 사용한다. 과거 사용한 45는 선택값이지 필수값이 아니다.

```bash
source /opt/ros/humble/setup.bash
source ~/rubi_ws/src/ws_livox/install/setup.bash
source ~/rubi_ws/install/setup.bash
source ~/ros2_ws_nav/install/setup.bash
ros2 pkg prefix rubi_heightmap_step_wavefront_planner
```

마지막 출력은 `/home/kim/ros2_ws_nav/install/rubi_heightmap_step_wavefront_planner`여야 한다.
이 실험 터미널에서 `legged_robot_ws/install/setup.bash`를 뒤에 source하면 같은 이름의
다른 planner를 실행할 수 있다. Frozen 파일은 경로로 읽으므로 legged overlay가 필요 없다.

## 4. Simulator → localization → frozen navigation

### 터미널 A: simulator

```bash
ros2 launch rubi_gazebo_sim rubi_gazebo_terrain_lidar.launch.py \
  world:=/home/kim/worlds_height515_layout_clean.world \
  contact_logging:=true gui:=true paused:=false \
  auto_demo:=false launch_joy_node:=false
```

`contact_logging:=true`는 controller sample/실험 metadata 등 추가 진단 기록을 켠다.
현재 plugin의 `joint_states_500hz` publisher는 이 옵션과 별개로 생성된다.
Simulator 실행만으로 policy on이 되지는 않는다. 기존 준비 절차는 다음과 같다.
`walk_ready`가 완료되고 상태가 준비됐는지 확인한 뒤 policy를 켠다.

```bash
ros2 service call /rubi_gazebo_legacy/legacy_policy_controller/walk_ready std_srvs/srv/Trigger '{}'
ros2 service call /rubi_gazebo_legacy/legacy_policy_controller/policy_on std_srvs/srv/Trigger '{}'
```

### 터미널 B: localization

```bash
ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  use_sim_time:=true publish_2d_map:=false use_rviz:=true \
  pcd_map:=/home/kim/legged_robot_ws/maps/sim_site_v1/localization.pcd
```

RViz의 **2D Pose Estimate**로 초기 정합을 수행한다. `/localization`과
`map → base_link` TF가 갱신되는지 확인한다. 높이 지도는 로봇 위치 추정의 대체물이 아니다.

### 터미널 C: vanilla

```bash
ros2 launch rubi_heightmap_step_wavefront_planner grid_cost_frozen_experiment.launch.py \
  use_sim_time:=true cost_mode:=vanilla \
  site:=/home/kim/legged_robot_ws/maps/sim_site_v1/site.yaml \
  world_file:=/home/kim/worlds_height515_layout_clean.world \
  scenario:=height515_frozen run_id:=height515_vanilla_01
```

이 명령이 frozen publisher + occupancy map server + costmap + Grid A* +
Pure Pursuit + 측정 node + RViz를 실행한다. 별도 FastDEM은 필요 없다.
`world_file`은 기록용이며 Gazebo를 시작하거나 바꾸지 않는다.

### CoT 비교 run

Vanilla navigation과 recorder를 끝내고, 동일 world/시작 조건으로 reset 또는 재시작하여
localization을 다시 확인한 뒤 아래를 실행한다. 두 navigation을 동시에 켜지 않는다.

```bash
ros2 launch rubi_heightmap_step_wavefront_planner grid_cost_frozen_experiment.launch.py \
  use_sim_time:=true cost_mode:=cot cot_degree:=3 \
  site:=/home/kim/legged_robot_ws/maps/sim_site_v1/site.yaml \
  world_file:=/home/kim/worlds_height515_layout_clean.world \
  scenario:=height515_frozen run_id:=height515_cot3_01
```

두 모드의 hard threshold와 Local Relief threshold는 모두 0.10 m이며,
height weight 기본값도 5.0으로 동일하다. 15 cm 단차는 지도에서 충분히 관측되면
양쪽 모두 우회 대상이다. 이 world에서 cost 비교 대상이 되는 통과 가능한 단차는
주로 5 cm이므로, **같은 goal에서 5 cm를 지나는 짧은 길과 평지 우회길이 실제로
경쟁하는지** 먼저 살핀다. 동일 경로가 나오는 시작/goal에서는 경로 차이가 없어도 이상이 아니다.

첫 연결 확인만 하려면 `launch_controller:=false`를 추가한다. 이 경우 실제 이동을 하지 않으므로
navigation energy 비교 run으로 집계하지 않는다.

## 5. 기록 → 측정 시작 → goal → 측정 종료 → recorder 종료

`ros2 bag record -a`는 현재 및 새로 발견되는 일반 토픽을 기록한다.
서비스 호출 자체가 자동으로 분석용 시작/종료 이벤트가 되는 것은 아니다.
또한 모든 토픽의 데이터가 무손실임을 보장하지 않는다. 그래서 **raw bag과 명시적인
sim-time 이벤트를 함께** 남긴다. 시작 전에 recorder의 subscription 로그를 확인한다.

터미널 D, navigation과 같은 run_id:

```bash
ros2 run rubi_heightmap_step_wavefront_planner record_grid_cost_all.sh height515_vanilla_01
```

내부 명령은 다음과 같다. 둘 중 하나만 실행한다.

```bash
SHARE=$(ros2 pkg prefix --share rubi_heightmap_step_wavefront_planner)
ros2 bag record -a --use-sim-time \
  --qos-profile-overrides-path "$SHARE/config/grid_cost_record_qos.yaml" \
  -o /home/kim/cot_experiment/navigation_cost_comparison/bags/height515_vanilla_01
```

`--use-sim-time`은 bag 수신시각을 sim time으로 맞춘다. 적분은 joint/odom header stamp를
사용한다. `/clock` 최초 수신 전에는 기록하지 않으며, 한 bag/run 안에서 simulator를 reset하지 않는다.
QoS 파일은 `/tf_static`, `/map`, frozen 지도, 실험 이벤트 등 latched 데이터를 포함하기 위한 것이다.
`-a`의 기본 범위는 일반 토픽이다. `--include-hidden-topics`는 추가 선택사항이며
서비스 호출을 데이터셋의 start/finish marker로 바꾸는 옵션이 아니다.

터미널 E, goal 직전:

```bash
ros2 service call /rubi/experiment/start std_srvs/srv/Trigger '{}'
```

**`success: true`를 확인한 다음** 새 navigation RViz의 **2D Goal Pose**를 지정한다.
Goal topic은 `/rubi/planner_comparison/goal`이다. 이전 legged의 `/goal_pose`나
Nav2 Goal action과 다르다. 반복 비교에는 같은 **map-frame goal 좌표와 yaw**를 재사용한다.
초기 위치 오차와 실제 주행 시작 위치도 bag에 남겨 확인한다.

도착 기준을 만족했을 때:

```bash
ros2 service call /rubi/experiment/finish_success std_srvs/srv/Trigger '{}'
ros2 topic echo /rubi/experiment/run_metrics --once --qos-durability transient_local
```

넘어짐/timeout/진행 불가:

```bash
ros2 service call /rubi/experiment/finish_failure std_srvs/srv/Trigger '{}'
```

종료 서비스는 로봇 정지 명령이 아니라 **적분 종료시각 기록**이다. 실패 시에는
해당 상황에 맞는 기존 정지 절차를 함께 사용한다. 종료 서비스를 부르자마자 Gazebo를
pause/종료하지 말고, joint/odom이 종료 경계를 포함해 결과를 낼 때까지 유지한다.
결과의 `qc_pass`를 확인하고 recorder 터미널에서 Ctrl+C로 정상 종료한 뒤 navigation을 종료한다.
서비스 응답 success와 `run_metrics.qc_pass`는 다른 값이다.

성공 판정과 종료는 현재 수동이다. Pure Pursuit 목표 거리 tolerance는 0.20 m이지만,
측정 node가 이를 자동 감지하지 않는다. 두 방법에 같은 도착 기준과 timeout을 미리 정한다.
`start` 후 goal을 늦게 누르거나 도착 후 종료를 늦게 누른 시간의 보행 work도 현재 적분에
포함된다. 논문 최종 결과에서는 지연을 보고하고, 필요하면 기록된 goal/pose를 이용해 동일한
자동 경계로 후처리한다. 수동 측정값을 자동 goal 종료 결과로 서술하지 않는다.

## 6. 무엇을 기록하고 계산하는가

| 구분 | 핵심 토픽/파일 |
|---|---|
| 토크·관절 속도 | `/rubi_gazebo_legacy/joint_states_500hz` |
| 실제 이동 거리 | `/ground_truth/odom` (world, BODY) |
| 추정 위치·TF | `/localization`, `/Odometry`, `/tf`, `/tf_static` |
| 명령·정책 상태 | `/cmd_vel`, `/rubi_gazebo_legacy/status`, `/rubi/experiment/controller_sample`, metadata |
| 시작·종료 | `/rubi/experiment/events` + `events.jsonl` |
| Goal·경로 | `/rubi/planner_comparison/goal`, `grid/path`, `grid/tracking_path` |
| 계획 지표 | `/rubi/experiment/plan_metrics` |
| Frozen 근거 | `/rubi/experiment/frozen_heightmap`, `/rubi/experiment/frozen_map_info`, `map_manifest.json` |
| 측정 결과 | `/rubi/experiment/run_metrics` + `result.json` |

총 주행에 대해 `P_abs = sum_i |tau_i * qdot_i|`, `W_abs = integral P_abs dt`,
`L_exec = integral sqrt(dx_B^2 + dy_B^2)`, `CoT_abs = W_abs/(m*g*L_exec)`를 계산한다.
거리 분모는 돌아가는 경로도 포함하는 실제 XY arc length다. Calibration의 고정 1 m와는 다르다.

논문 핵심 비교는 **성공률, 총 W_abs, 실제 이동 거리, 완료 시간**이며 계획 경로 길이,
실제 CoT, 계획 시간, 재계획 수를 함께 제시한다. 돌아가는 길은 CoT가 작아도 총 work가
클 수 있으므로 총 work를 반드시 비교한다. 서로 다른 비용함수의 내부 total_cost 숫자끼리만
비교해서 에너지 절감을 주장하지 않는다.

`-a`에는 LiDAR/영상도 포함될 수 있어 기록 부하가 커진다. 짧은 pilot bag에서 주행 구간의
500 Hz joint/odom header 누락을 먼저 확인한다. 필요한 입력이 없는 경우 recorder 옵션으로
새 데이터가 생기지는 않는다. `contact_logging:=true`, 실제 topic 존재 및 수신을 확인한다.

저장:

```text
~/cot_experiment/navigation_cost_comparison/<run_id>/map_manifest.json
~/cot_experiment/navigation_cost_comparison/<run_id>/events.jsonl
~/cot_experiment/navigation_cost_comparison/<run_id>/result.json
~/cot_experiment/navigation_cost_comparison/bags/<run_id>/
```

Raw bag의 종료 후 `ros2 bag info ...`로 topic 수신을 확인한다. 온라인 result의 QC는
측정 node가 수신한 표본에 대한 검사이므로, recorder의 별도 누락 검증을 대신하지 않는다.

## 7. 이번 검증 범위

실제 frozen 파일의 셀 수·원 timestamp·SHA-256을 두 모드 모두에서 확인했다.
별도 ROS domain에서 synthetic planner 입력으로 frozen old/future stamp 허용,
온라인 height stale 거부, live costmap stale 거부, 잘못된 frame 거부를 검사했다.
사용자 domain에 goal/cmd_vel을 보내거나 Gazebo 실제 주행을 수행하지 않았다.
따라서 지금은 연결 검증이 끝난 상태이며, 동일 시작/goal의 짧은 pilot을 한 뒤 본 반복 실험을 시작한다.
