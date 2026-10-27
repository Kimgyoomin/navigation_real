# navigation_real

## RUBI 실물 Navigation: footprint / 비대칭 관측 필터 (2026-10-28)

현재 대상은 `pongbot_navigation/launch/rubi_navigation_dwb.launch.py`와
`pongbot_navigation/config/nav2_rubi_dwb.yaml`이다. `legged_robot_ws`나 다른
Navigation launch의 설정은 이 변경으로 바뀌지 않는다.

### 적용 설정

| 항목 | 이전 | 현재 |
| --- | --- | --- |
| Global / local footprint | 반지름 0.25 m 원 | base_link 중심 0.55 × 0.55 m 사각형 |
| Footprint padding | 0.01 m | 유지: 검사 시 외곽 크기는 0.57 × 0.57 m |
| Global costmap 갱신 / 발행 | 2 / 1 Hz | 1 / 1 Hz |
| Local costmap 갱신 / 발행 | 5 / 5 Hz | 유지 |
| Global 재계획 / local 제어 | BT 1 Hz / controller 5 Hz | 유지 |
| Global / local obstacle_min_range | 0.6 / 0.5 m | 모두 0.3 m + 상류 비대칭 필터 |
| Nav2 odom_topic | /Odometry | 유지; 새 FAST-LIO 속도 토픽은 아직 연결하지 않음 |

Global layer 순서는 `static_layer → obstacle_layer → inflation_layer`, local은
`obstacle_layer → inflation_layer`를 유지한다. Inflation radius, 속도·가속도 제한,
Rotation Shim 및 나머지 DWB 평가 계수는 유지한다.

사각형 모서리도 검사하도록 DWB에 `ObstacleFootprint` critic을 추가했다.
기존 `BaseObstacle` 중심 셀 검사도 유지한다. 따라서 단순 표시 모양만 바뀌는 것은
아니며 DWB 계산량도 일부 증가한다. Global A* 자체를 방향별 사각형 충돌 검색으로
변경한 것은 아니다. Global A*는 여전히 footprint 크기가 반영된 inflation costmap의
셀 비용을 사용한다.
참고: [Nav2 Humble ObstacleFootprint 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/obstacle_footprint.cpp).

### 필터 데이터 경로와 좌표 기준

```text
/livox/lidar            (CustomMsg) ──→ FAST-LIO             [변경 없음]
/livox/imu                          ──→ FAST-LIO             [변경 없음]
/livox/lidar_pointcloud (PointCloud2)
    └─ costmap_sector_filter ─→ /livox/lidar_costmap
                                  ├─ global obstacle_layer
                                  └─ local obstacle_layer
```

`costmap_sector_filter`는 Navigation launch가 한 개만 실행하는 C++ 노드다.

- `livox_frame` 원점의 수평거리 `sqrt(x²+y²)`를 사용한다. 로봇 중심 거리나 3D
  직선거리가 아니다.
- +X 전방 기준 **-60° ≤ 각도 ≤ +60°**는 **0.3 m 이상**만 유지한다.
- 나머지 방향은 **1.0 m 이상**만 유지한다. 최대거리는 기존 costmap 설정이 제한한다.
- 현재 RUBI static TF는 `livox_frame`과 `base_link`의 축 방향이 같다. 원점은 다르다:
  현재 base_link→LiDAR 위치는 `(0.0995, 0, -0.1375)` m이다. 장착 방향/프레임을
  바꾸면 필터의 전방 기준도 재검토해야 한다. 필터가 임의로 TF 회전을 추정하지 않는다.
- 원래 `header.stamp`, frame, point fields와 남은 점의 바이트/순서를 보존한다.
  현재 시각으로 restamp하거나 stamp=0으로 바꾸지 않는다.
- 입력/출력 모두 BEST_EFFORT, KEEP_LAST 1이다. 입력이 없으면 재발행하지 않는다.
  NaN/Inf 좌표는 제외하고, 잘못된 구조·예상 밖 frame·stamp=0 메시지는 거부한다.
- 점당 sqrt/atan2 없이 제곱거리·곱셈 비교로 한 번 순회한다. ICP나 voxel grid,
  별도 TF 조회를 추가하지 않는다.

설정값은 YAML 최상단 `costmap_sector_filter.ros__parameters`에 있다.
필터 파라미터는 시작 시 읽으며 변경 후 Navigation을 재시작한다. 별도의 예전
`params_file`을 넘기면 그 파일의 costmap 입력 토픽도 확인해야 한다.

### 관측 제한과 주기 해석

이 필터는 사람 검출기가 아니다. **측후방 1 m 이내의 사람뿐 아니라 실제 장애물도
제외한다.** 전방도 0.3 m 이내 관측은 제외된다. 특히 후진·제자리 회전에 대한
충돌 안전을 보장하지 않으며, footprint를 키워도 보이지 않는 장애물은 복구되지 않는다.
처음에는 구동을 차단한 관측 검사 후, 비상정지가 가능한 통제된 공간에서 낮은 속도로
검증해야 한다. 이번 변경으로 기존 최대속도 0.7 m/s를 낮춘 것은 아니다.

두 obstacle layer는 필터 출력으로 marking과 clearing을 수행한다. 버린 점을 가상의
먼 점이나 free space로 바꾸지 않는다. 기존 obstacle mark가 삭제되는지 여부는 이후
유효한 관측의 ray clearing 등에 달려 있다. 필터가 오래된 costmap을 자동으로 지우지는
않는다. 또한 기존 `expected_update_rate: 0.0`을 유지하므로, 필터 입력 단절 시 자동
정지가 보장된다고 가정하면 안 된다.

1 Hz / 5 Hz로 숫자를 맞춰도 planner, controller, costmap 타이머는 동기화되지 않는다.
Global 갱신 1 Hz는 계산량을 줄이는 대신 관측 반영까지 거의 1초를 기다릴 수 있고,
재계획 대기·실제 처리시간도 별도로 더해질 수 있다. Local 5 Hz 갱신을 함께 확인한다.
`publish_frequency`는 외부/RViz 전달 빈도이며 내부 갱신 빈도와 다르다.
`expected_planner_frequency: 1.0`은 처리시간 경고 기준일 뿐, 재계획을 구동하는
타이머가 아니다. 실제 재계획 주기는 기존 BT의 `RateController hz="1.0"`가 지정한다.
참고: [Nav2 Humble PlannerServer 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_planner/src/planner_server.cpp).

### 실행 및 관측 확인

기존 Livox, FAST-LIO **localization** 실행 및 초기 위치 정합 후 Navigation을 실행한다.
단독 `fast_lio mapping.launch.py` 속도 시험만으로 Nav2용 map→base_link TF 구성이
완성되는 것은 아니다. 아래 명령은 사용자가 안전 상태를 확인한 뒤 직접 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/rclab/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
taskset -c 6,7 ros2 launch pongbot_navigation rubi_navigation_dwb.launch.py
```

필터는 함께 실행되므로 별도 실행하지 않는다. 기존 Navigation 프로세스는 종료한 뒤
재시작해야 새 설정이 적용된다. 다른 터미널도 같은 workspace/domain을 사용한다.

```bash
ros2 topic hz /livox/lidar_costmap --window 30
ros2 topic echo /livox/lidar_costmap --field header --once --qos-reliability best_effort
ros2 param get /local_costmap/local_costmap footprint
ros2 param get /global_costmap/global_costmap footprint
ros2 param get /controller_server odom_topic
```

현재 driver의 보조 PointCloud2는 약 5 Hz이므로 필터 출력도 약 5 Hz를 기대한다.
RViz에 `/livox/lidar_costmap` PointCloud2를 BEST_EFFORT로 추가하여 마스크를 확인하고,
`/local_costmap/published_footprint`, `/global_costmap/published_footprint`를 Polygon으로
보며 실제 로봇 외곽과 비교한다. 필터 로그에는 5초 간격으로 입력/유지/폐기 점 수와
`filter_ms`가 나온다. 메시지 header가 오래되어도 현재 시각으로 덮어 숨기지 않는다.

### 단계별 비교 실험

1. **Footprint·비대칭 필터·costmap 주기 변경 단계**: 기존 `/Odometry` 입력을 유지한다.
   정지 상태 관측, 초기 위치/TF, local costmap 연속성부터 확인한 뒤 제한된 주행을
   한다. `/Odometry`, `/localization`의 Hz/지연과 controller missed-rate, TF 오류,
   `/cmd_vel` 끊김도 확인한다. 뒤에서 따라오는 사람이 제거된 것만으로 성공 판정하지 않는다.
2. **속도 피드백 비교 단계**: 다른 설정·경로·속도 제한을 고정하고, FAST-LIO 속도의
   축/부호·정지 잡음 및 body→base_link 변환을 검증한 뒤 Nav2 입력을 연결한다.
   기존 YAML에 `closed_loop: true`가 이미 있어도 그것만으로 새 속도가 공급되는 것은
   아니다. `/fastlio/odometry_velocity` 연결 작업은 이번 변경에 포함하지 않았다.

### 검증 범위

- `pongbot_navigation` Release 빌드 성공.
- C++ 필터 12개 + YAML/launch 구성 7개 테스트 통과.
- 별도 localhost ROS_DOMAIN_ID=167에서 설치된 실행 파일과 YAML로 synthetic
  PointCloud2 송수신 검사: 마스크, timestamp/fields/point bytes 보존, 잘못된 입력
  거부, 무입력 시 재발행 없음, 빈 점군 처리를 확인했다. 실제 Nav2/LCM/주행 명령은
  실행하지 않았다.
- Jetson에서 20,000점/100회 필터 계산 시험: 중앙값 약 0.237 ms, p95 약 0.255 ms.
  **DDS 복사·전송, Nav2와 추가 footprint critic 비용, 실제 주행 성능은 포함하지 않는다.**
  통합 부하는 실험 중 별도 확인해야 한다.

테스트 재실행:

```bash
cd /home/rclab/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon test --packages-select pongbot_navigation --executor sequential
colcon test-result --test-result-base build/pongbot_navigation --verbose
```
