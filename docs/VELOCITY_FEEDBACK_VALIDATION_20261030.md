# FAST-LIO 속도 피드백과 DWB 급격한 명령 변화 검증

작성: 2026-10-30 KST. 대상: `navigation_real`, ROS 2 Humble / Nav2 1.1.20.

## 결론과 적용 범위

- 이전에 좁은 복도 이상 주행을 분석한 bag은 `rosbag2_2026_10_29-01_50_03`이다.
  사용자가 이번에 언급한 `rosbag2_2026_10_29-01_35_55`도 실제로 있어서 두 기록을 비교했다.
- 두 bag의 `/Odometry.twist` 6개 성분은 모두 0이다. 현재 Nav2의 `odom_topic`이
  `/Odometry`이므로 이 연결에서는 실제 속도를 받지 못한다.
- `/fastlio/odometry_velocity`에는 유효한 속도 추정치가 기록되어 있다. 특히 `01:50:03`
  기록에서 주기·지연·pose 변화와의 내부 일관성을 확인하여 **피드백 비교 실험용 연결**을 구현했다.
- 현재 StandardTrajectoryGenerator에서는 피드백만 연결해도 한 제어 주기당 가속도 제한을
  보장하지 않는다. 실험 옵션은 피드백과 LimitedAccelGenerator를 함께 활성화한다.
- 기본값은 `use_fastlio_velocity:=false`이다. 기존 실행 명령과 사용자가 수정한 YAML은 유지한다.
  실차 폐루프 안정성, LCM 실제 송수신, 모드 전환, 장애물 상황 검증까지 통과했다는 의미는 아니다.

## 데이터와 계산 방법

원본 폴더:

- `/home/rclab/rosbag_record/rosbag2_2026_10_29-01_35_55` — 259.045초
- `/home/rclab/rosbag_record/rosbag2_2026_10_29-01_50_03` — 292.874초

SQLite `mode=ro`, `PRAGMA query_only=ON`으로 읽었다. `ros2 bag play`를 실행하지 않았다.
분석 스크립트는 `tools/analyze_velocity_feedback.py`이며 ROS 초기화/발행을 하지 않는다.
큰 점군·지도 payload를 읽지 않고 속도·pose·IMU·TF·로그를 분석한다.

`/cmd_vel`은 Header 없는 Twist이다. 따라서 명령 변화율 `Δv/Δt`, `Δω/Δt`의 dt는
**bag 기록 시각**이다. 송신 시각 또는 메인제어기 수신 시각이 아니다. 5Hz의 일반적인
연속 구간을 보기 위해 dt가 0.10~0.35초인 구간을 따로 집계했다. 극단적인 짧은 수신
간격과 긴 단절을 섞어 미분값을 부풀리지 않았다. 전체 표본은 CSV에 남겼다.

추정 속도는 Header의 관측 시각으로 비교했다. 연속 5표본(약 0.4초 범위)에 선형회귀를
적용해 위치·회전 변화율을 구했다. 간격이 0.16초를 넘거나 역전되면 그 창은 제외했다.
회전은 quaternion 상대 회전벡터를 사용했다. 선가속도는 먼저 속도를 고정 world 좌표로
회전하여 미분한 뒤 body/base 좌표로 바꾸었다. body 속도를 단순 미분하면 회전 효과가
누락되므로 그렇게 계산하지 않았다. 이는 FAST-LIO 기반 추정 가속도이며 독립 센서 실측값은 아니다.

## 관측 결과

| 항목 | 01:35:55 | 01:50:03 |
|---|---:|---:|
| `/cmd_vel` 표본 | 647 | 1,385 |
| 기록된 최대 선속도 명령 | 0.750m/s | 0.750m/s |
| 일반 간격에서 최대 양의 선속도 명령 변화율 | 1.604m/s² | 3.763m/s² |
| 일반 간격에서 yaw 명령 변화율 범위 | −0.880~+0.796rad/s² | −22.602~+15.583rad/s² |
| 일반 간격에서 선속도 명령 변화율 >0.51 | 59/643구간 | 100/1,375구간 |
| native 속도 평균 수신율 | 9.157Hz | 10.000Hz |
| native 속도 age 중앙값 / p95 | 19.03 / 28.35ms | 17.03 / 25.76ms |
| native 속도 최대 수신 간격 | 11.794초 | 0.156초 |
| `/Odometry` twist | 모두 0 | 모두 0 |
| 추정 선가속도 x 범위(약0.4초 fit) | −0.754~+0.714m/s² | −1.081~+1.088m/s² |
| 추정 yaw 각가속도 범위(약0.4초 fit) | −0.694~+0.750rad/s² | −2.811~+2.643rad/s² |

현재 YAML의 가속도 0.5와 비교할 수 있지만, 이를 bag 당시 런타임 설정으로 소급해서
확정하지 않는다. 01:35:55의 parameter_events에도 필요한 controller 설정이 없고,
01:50:03에는 parameter_events 메시지가 없다. 현재 sim_time은 2.0초이며 당시와 같다는
보장은 없다. 관측된 선속도 상한 0.75 역시 현재 설정 max_vel_x=1.0과 구분해야 한다.

01:35:55의 IMU 녹화율은 약33.15Hz, age 중앙값은 약1.274초다. `/Odometry`의 age p95는
약1.615초지만 native 속도의 p95는 약28ms다. 서로 다른 QoS/구독 전달 경로의 녹화 결과이므로
이것만으로 FAST-LIO 내부 IMU 전체가 1.274초 늦었다고 단정하지 않는다. 이 bag 전체를
안정적인 실시간 피드백 검증으로 인정하지 않았다.

01:50:03은 IMU 약200Hz, Odometry/native 속도 약10Hz로 기록됐다. 수신 연속성이 더 좋다.
다만 Navigation flag, joystick 개입 시각, LCM 송수신, 모터/관절 상태는 없다. 특히 사용자는
벽으로 향하는 것을 보고 메인제어기 Navigation flag를 끄고 수동으로 움직였다고 했다.
따라서 명령과 측정이 다른 전체 구간을 자율주행 추종 오류나 속도 추정 오류로 집계할 수 없다.

### 급격한 명령 복귀 사례

01:50:03 bag 시작 후:

- 211.557초: `No valid trajectories out of 263!` 로그.
- 약211.557초: 선속도 명령 0.
- 211.757초: 선속도 명령 0.75m/s.
- 두 명령 간격 약0.1993초, 변화율 약3.763m/s².

이 시점의 native 선속도는 약0.15~0.16m/s다. 실제 로봇이 매번 완전히 정지했다가
출발했다는 뜻은 아니다. **경로 평가 실패에 따른 0 명령과 다음 정상 계산의 큰 명령 복귀**가
발생했다. 이 bag에는 `No valid trajectories`가 17회 기록돼 있다. 속도 피드백 연결은 필요한
개선이지만 경로 실패/복구와 LCM 전달 문제까지 한 번에 해결했다는 결론은 내리지 않는다.

### 최초 이상 좌회전과 명령 점프의 시간 순서 재확인

사용자의 추가 질문에 따라 같은 bag의 시각을 직접 대조했다. 모든 절대 시각은 KST다.

| 사건 | bag 시작 후 | 절대 시각 |
|---|---:|---|
| 최초 이상 좌회전 이전의 선속도 명령 점프: 0.205→0.614m/s, 약0.200초 | 26.937초 | 01:50:30.846 |
| 우회전 명령(음의 yaw)과 좌회전 추정(양의 yaw)이 지속적으로 불일치 | 약124.3~126.9초 | 약01:52:08.211~01:52:10.806 |
| ROS controller 취소 후 0 명령 | 128.080초 | 01:52:11.989 |
| 앞서 최대값으로 보고한 0→0.75m/s 명령 점프 | 211.757초 | 01:53:35.666 |

따라서 **최대 3.763m/s² 명령 점프는 최초 이상 좌회전보다 약85~88초 뒤**다.
사용자는 이상 동작을 본 뒤 main-PC Navigation flag를 끄고 수동 조작했다고 했으므로,
이 최대값을 최초 벽 방향 좌회전의 원인으로 해석하면 안 된다. 그 뒤 flag 재활성화 여부와
정확한 시각은 기록에 없어서 211.757초 당시 자동/수동 상태도 확정할 수 없다.
128.080초의 ROS 취소와 main-PC flag OFF는 서로 다른 이벤트다.

다만 명령 점프가 모두 사건 이후에만 발생한 것은 아니다. 사건 이전 26.937초에도
명령 증가율 약2.047m/s²가 있었다. 최초 이상 좌회전 구간(124~128.08초, 취소 0 명령 제외)의
최대 양의 선속도 명령 변화율은 약0.683m/s²였다. 이 구간의 핵심 증거는 큰 선속도 점프보다
**지속적인 회전 명령/응답 부호 불일치**다. LCM TX/RX 미기록으로 통신 단절 자체는 여전히 미확정이다.

## FAST-LIO 속도 출력 검증

코드의 선속도는 `R_world_body.transpose() * state_point.vel`, 각속도는 scan-end propagation에
사용한 gyro에서 추정 bias를 뺀 값이다. 추정기 자체의 상태 갱신식은 이번 작업에서 바꾸지 않았다.
원본 `/Odometry`의 pose/TF도 그대로 유지한다.

01:50:03에서 순간 native 출력과 약0.4초 pose 변화율의 차이:

| 비교 | RMSE | 절대 차이 p95 |
|---|---:|---:|
| 선속도 x | 0.0282m/s | 0.0564m/s |
| 각속도 z | 0.0674rad/s | 0.1566rad/s |

native 출력도 같은 5표본 평균으로 비교하면 RMSE는 각각 약0.0241m/s, 0.0507rad/s다.
이 값은 동일 추정기의 pose/twist 간 **내부 일관성**이다. 외부 ground truth에 대한 절대 오차가
아니며 평활화 창 차이와 진동이 포함된다. 이번 adapter에는 새 저역통과 필터를 넣지 않았다.
추후 실제 로봇의 제자리 회전/직선 이동에서 부호·단위·진동 민감도를 확인해야 한다.

## 원점·좌표계·QoS 변환

새 노드: `pongbot_navigation/fastlio_velocity_feedback`

```text
/fastlio/odometry_velocity  (camera_init / body, BEST_EFFORT)
    -> TF로 body에서 base_link까지 변환
/fastlio/odometry_base      (camera_init / base_link, RELIABLE depth 1)
    -> controller_server 및 bt_navigator odom_topic
```

bag의 TF와 현재 launch에서 body 기준 base 원점은 `[-0.1105, 0.02329, 0.09338]m`,
회전은 단위 회전이다. adapter는 이 값을 하드코딩하지 않고 TF를 조회한다. 일반 회전도 지원한다.

body 좌표의 base 원점 벡터를 r, body에서 base로 벡터를 표현하는 회전을 R이라 할 때:

```text
v_base = R * (v_body + omega_body × r)
omega_base = R * omega_body
T_world_base = T_world_body * T_body_base
```

pose와 twist 공분산도 원점·좌표 변환 Jacobian으로 전달한다. 장착 TF는 정확하다고 가정하며
원본 추정 공분산의 보정/검증을 의미하지 않는다. 관측 timestamp를 now()로 바꾸지 않는다.
01:50:03에서 원점 보정량의 축별 절댓값 최대는 x 약0.074, y 약0.099m/s여서 단순 frame 이름
변경으로 생략하지 않았다. 이 최대에는 roll/pitch 회전의 영향도 포함된다.

설치된 Humble Nav2 OdomSubscriber는 twist의 x/y/z각속도를 직접 읽으며 자체 TF 변환을 하지 않는다.
또한 SystemDefaultsQoS를 사용한다. 기존 native 출력은 BEST_EFFORT이므로 RELIABLE 기본 구독과
호환되지 않을 수 있다. adapter는 BEST_EFFORT depth 1 입력과 RELIABLE depth 1 출력을 사용한다.

250ms보다 오래됐거나 50ms보다 미래인 입력, 순서 역전/중복, 잘못된 frame, NaN/Inf,
없는 TF는 발행하지 않는다. 측정 부재를 가짜 0 속도로 대체하거나 최신 시각으로 재발행하지 않는다.
**이 거절 동작은 명령 watchdog이 아니다.** 입력이 사라지면 Nav2는 마지막 twist를 보유할 수 있다.

## DWB의 가속도 의미와 변경

설치된 Nav2 1.1.20 라이브러리를 로봇과 분리된 domain 216에서 직접 호출했다.
5Hz, sim_time=2.0초, acc_lim_x/theta=0.5, decel_lim_x/theta=-0.5, 최대1.0 설정이다.

| 현재 추정 vx | Standard 후보 vx | LimitedAccel 후보 vx (sim_period=0.2초) |
|---:|---:|---:|
| 0.00 | 0~1.00 | 0~0.10 |
| 0.15 | 0~1.00 | 0.05~0.25 |
| 0.50 | 0~1.00 | 0.40~0.60 |
| 0.90 | 0~1.00 | 0.80~1.00 |

현재 yaw 속도0이면 Standard yaw 후보는 −1~+1rad/s, LimitedAccel은 −0.1~+0.1rad/s다.
실제 후보 선택은 장애물과 critic 평가 결과에 따라 달라진다. `sim_time=2.0`은 계속 경로 예측
시간으로 유지하고, LimitedAccel의 가속 가능 시간만 제어 주기 `1/controller_frequency`로 둔다.
launch에서 주파수를 읽으므로 10Hz로 바꾸면 sim_period도0.1초로 바뀐다.

피드백이 정확하고 실제 속도가 명령을 따라가며 정상 제어가 이어진다면, 정지에서
0.5m/s²로 1m/s까지 약2초라는 목표가 성립한다. 그러나 LimitedAccel은 **현재 측정 속도 기준의
도달 가능한 후보 범위**이며 이전 송신 명령 대비 전역 slew-rate 제한기가 아니다. 추정 잡음,
수신 지연, 제어 주기 변동, 입력 속도가 min/max 밖인 경우의 clamp, 실패/취소 정지 명령,
별도 recovery behavior, LCM/메인제어기 동작은 별개다. 실제 가속도0.5 이내를 보증하지 않는다.
추가 출력 rate limiter/velocity smoother와 명령 timeout은 별도 검증 대상이다.

공식 소스:

- [StandardTrajectoryGenerator](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp)
- [LimitedAccelGenerator](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/dwb_plugins/src/limited_accel_generator.cpp)
- [Humble OdomSubscriber](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_dwb_controller/nav_2d_utils/include/nav_2d_utils/odom_subscriber.hpp)
- [ControllerServer의 실패 시 0 명령 처리](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_controller/src/controller_server.cpp)

## 빌드·검증 결과와 실행

- `pongbot_navigation` Release 빌드 성공. 시스템 패키지 업그레이드 없음.
- colcon test-result: 31 tests, 0 errors, 0 failures.
- 실제 장착 offset의 cross product 부호, 90도 회전 장착, pose/stamp 유지, 공분산,
  NaN/Inf/잘못된 quaternion·frame 거절 단위 테스트 통과.
- 로봇과 분리한 ROS domain216의 `/validation/input`, `/validation/output`만 사용하여 기록된
  30개 twist를 주입했다. stamp·원점 보정 일치 및 SystemDefaultsQoS 수신을 확인했다.
  이 연결 테스트는 bag의 twist 값에 테스트 당시 시각을 붙여 입력하고, 출력이 그 입력 stamp를
  보존하는지 확인한다. 원본 bag을 재시각화하거나 실제 로봇에 재생한 것이 아니다.
- stale/duplicate/future/wrong-frame/nonfinite 입력과 TF 미준비 시 발행 거절, 이후 정상 복귀 확인.
- Nav2 launch의 false/true 분기와 5Hz/10Hz 제어 주기 대응을 **프로세스 실행 없이** 검증했다.
- 실제 Nav2 goal, `/cmd_vel` 발행, LCM 송신, live 로봇 프로세스 재시작은 하지 않았다.

구동을 차단하고 Livox/localization을 먼저 실행한 뒤, Goal 없이 연결 확인:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

taskset -c 6,7 ros2 launch pongbot_navigation rubi_navigation_dwb.launch.py \
  dds_profile:=navigation use_sim_time:=false use_fastlio_velocity:=true
```

별도 같은 환경 터미널:

```bash
ros2 param get /controller_server odom_topic
ros2 param get /controller_server FollowPath.trajectory_generator_name
ros2 param get /controller_server FollowPath.sim_period
ros2 topic echo /fastlio/odometry_base --field twist.twist --qos-reliability reliable
```

기대 파라미터는 `/fastlio/odometry_base`, `dwb_plugins::LimitedAccelGenerator`, `0.2`다.
원래 비교 조건은 `use_fastlio_velocity:=false` 또는 인자 생략이다. 옵션 변경은 Nav2 재시작에 적용된다.
현재 최대 선속도/각속도와 가감속 값은 사용자 YAML을 그대로 따른다.

**주행 재개 판단에는 명령 전달 검증이 남아 있다.** 현재 LCM 코드의 `/cmd_vel` 수신 timeout 부재와
마지막 명령 무기한 재송신은 이번 속도 피드백 작업에서 수정하지 않았다. ROS bag에는 그 출력이 없다.
속도 피드백을 켰다는 이유만으로 이전 급출발·반대 방향 움직임의 원인이 해결됐다고 판단하지 않는다.

## 산출물 위치

`/home/rclab/ros2_ws/log/velocity_feedback_20261030/`

- `results/bag_013555/`, `results/bag_015003/`: summary.json, CSV, velocity_comparison.png
- `generator_envelopes.txt`, `generator_probe/`: 설치된 DWB 라이브러리 후보 범위 검증
- `adapter_probe_result.json`, `adapter_probe.log`, `probe_feedback.py`: 분리된 ROS 연결 검증
- `build_final.log`, `tests_final.log`: 빌드/테스트 결과

재분석 예(출력은 새 폴더여야 함):

```bash
source /opt/ros/humble/setup.bash
python3 tools/analyze_velocity_feedback.py \
  /home/rclab/rosbag_record/rosbag2_2026_10_29-01_50_03 \
  --output /home/rclab/ros2_ws/log/velocity_feedback_repeat
```
