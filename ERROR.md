# RUBI 실차 Localization / Navigation 지연 문제 분석 및 변경 보고서

## 1. 문서 목적

이 문서는 RUBI에서 Livox MID360, FAST-LIO Localization, Nav2(DWB), 조이스틱/LCM을 함께 실행했을 때 발생한 다음 문제를 재현하고, 원인을 분리하여, `jetson/260904` 브랜치에서 적용한 변경의 의도와 결과를 원작성자가 추적할 수 있도록 기록한 보고서이다.

- `/Odometry`와 `/localization`이 정상적인 약 10 Hz에서 약 0.7~0.8 Hz로 간헐적으로 저하됨
- 메시지 지연이 약 14~20 ms에서 약 1.3~1.5 s로 증가함
- Nav2에서 `Extrapolation Error`, `Transform data too old`, `Unable to transform robot pose`가 연속 발생함
- `Control loop missed its desired rate`가 반복되고 navigation goal이 abort됨
- MID360 앞의 장애물이 local/global costmap에 안정적으로 반영되지 않음
- 거꾸로 장착한 MID360 좌표계, FAST-LIO의 `body`, 로봇의 `BODY`/`base_link` 사이 TF 책임이 불명확했음
- 메인 제어기 조이스틱 토픽과 Jetson의 LCM bridge 입력 토픽이 맞지 않았음

기준 커밋은 다음과 같다.

| 항목 | 값 |
|---|---|
| 브랜치 | `jetson/260904` |
| 변경 커밋 | `2b118b42abe3f0f3660207787f24a855494d318e` |
| 비교 기준 | `2b118b4^` (`cb2cdf3`) |
| 커밋 통계 | 24 files changed, 744 insertions, 207 deletions |
| 실험 환경 | ROS 2 Humble, `rmw_fastrtps_cpp`, Jetson Orin Nano, MAXN_SUPER |

> 이 문서에서 “변경 전/후”는 주로 `2b118b4^`와 `2b118b4`의 코드 차이를 뜻한다. MID360의 `roll: 180.0` 및 일부 RUBI TF/Rotation Shim 설정은 해당 커밋 이전부터 존재했으므로, 이 커밋이 새로 만든 변경이 아니라 **현재 구성에서 유지·검증한 전제**로 따로 표시한다.

---

## 2. 결론 요약

최종적으로 확인한 핵심은 다음과 같다.

1. `/livox/lidar`에서 `ros2 topic delay`가 약 0.7 s로 보였던 값은 실제 Livox driver 내부 지연을 그대로 나타내지 않았다. `CustomMsg.header.stamp`가 스캔 시작 시각이고 메시지는 약 100 ms 분량의 포인트를 모은 뒤 발행되며, Python ROS CLI가 대형 CustomMsg를 역직렬화하면서 자체적으로 밀릴 수도 있다. Driver 내부 계측에서 정상 구간의 최신 포인트 기준 실제 발행 지연은 약 1~4 ms였다.

2. 반면 `/Odometry`와 `/localization`의 1.3~1.5 s 지연 및 약 0.78 Hz 저하는 실제 문제였다. FAST-LIO의 센서 callback과 처리 timer가 single-threaded executor에 함께 있었고, 고주기 IMU/LiDAR callback 및 Nav2 실행 부하가 추가되면 처리 timer가 장시간 schedule되지 않는 현상이 관측되었다.

3. ROS 1 스타일의 매우 큰 LiDAR subscriber queue(`20000`)와 무제한 내부 deque는 한 번 밀린 데이터를 계속 처리하게 만들 수 있었다. ROS 2에서는 실시간 localization에 맞게 LiDAR DDS queue를 `BEST_EFFORT + KEEP_LAST(2)`로 바꾸고, FAST-LIO 내부 LiDAR deque도 최대 2 frame으로 제한했다.

4. FAST-LIO를 sensor callback group과 processing callback group으로 나누고 2-thread `MultiThreadedExecutor`로 실행한 뒤, Navigation 실행 전후 모두 `/Odometry`와 `/localization`이 약 10 Hz, 약 14~19 ms 지연으로 유지되었다.

5. Global ICP는 0.5 Hz로 낮췄지만 `/localization`은 `/Odometry` callback마다 최신 global correction을 합성하여 약 10 Hz로 발행한다. 즉 `/map_to_odom` 0.5 Hz와 `/localization` 10 Hz는 서로 모순되지 않는다.

6. Nav2 장애물 입력은 Livox `CustomMsg`를 직접 사용할 수 없다. Driver가 동일한 스캔에서 FAST-LIO용 `/livox/lidar` CustomMsg 10 Hz와 costmap용 `/livox/lidar_pointcloud` PointCloud2 5 Hz를 동시에 내도록 변경했다.

7. DWB 계산량과 Jetson scheduling 여유를 위해 controller frequency를 10 Hz에서 5 Hz로 낮추고 trajectory sample 수를 줄였다. 5 Hz에서 `Control loop missed its desired rate`가 사라지는 것을 확인했다.

최종 관측 결과는 대략 다음과 같다.

| 항목 | 문제 구간 | 최종 안정 구간 | 판단 |
|---|---:|---:|---|
| `/Odometry` rate | 약 0.74~0.78 Hz 또는 10 Hz로 오래된 데이터 발행 | 약 9.95~10.05 Hz | 정상화 |
| `/Odometry` delay | 약 1.3~1.6 s | 약 0.014~0.018 s | 약 99% 감소 |
| `/localization` rate | 약 0.72~0.79 Hz | 약 10 Hz | 정상화 |
| `/localization` delay | 약 1.3~1.5 s | 약 0.016~0.019 s, 간헐 max 약 0.055 s | 정상화 |
| `/map_to_odom` rate | 약 0.5 Hz | 약 0.5 Hz | 의도한 global ICP 주기 |
| `/livox/lidar` | 10 Hz | 10 Hz | FAST-LIO 입력 유지 |
| `/livox/lidar_pointcloud` | 미발행 | 약 5 Hz | Nav2 obstacle 입력 추가 |
| Nav2 controller | 10 Hz에서 missed 경고 반복 | 5 Hz에서 경고 소멸 | Jetson 실험값 채택 |

### 2.1 결론의 확실성 구분

| 구분 | 내용 |
|---|---|
| 코드와 내부 계측으로 확인 | Livox steady-state `publish_end_age` 수 ms, FAST-LIO timer starvation 구간, callback group/2-thread executor 적용 후 Odometry 10 Hz 회복, dual topic rate |
| 실행 결과로 확인 | Navigation 전후 `/Odometry`·`/localization` 약 10 Hz 유지, 5 Hz controller에서 missed 경고 소멸 |
| 정황상 가장 타당한 해석 | Python ROS CLI의 대형 CustomMsg 역직렬화/backlog가 `/livox/lidar`의 약 0.7 s 표시에 포함됨 |
| 아직 실차 A/B 검증 필요 | zero-stamp obstacle cloud의 이동 중 공간 오차, global layer 순서, point filter 4의 localization 정밀도 |

---

## 3. 전체 데이터 및 TF 구조

### 3.1 데이터 흐름

```text
MID360 UDP packets
  |
  v
livox_ros_driver2
  |-- /livox/imu                  sensor_msgs/Imu, 고주기
  |-- /livox/lidar                livox_ros_driver2/CustomMsg, 10 Hz
  |      |
  |      v
  |   fastlio_mapping
  |      |-- /Odometry            camera_init -> body local odometry, 약 10 Hz
  |      |-- /tf                  camera_init -> body
  |      `-- /cloud_registered    global localization용 scan
  |
  `-- /livox/lidar_pointcloud     sensor_msgs/PointCloud2, 5 Hz
         |
         `--> Nav2 local/global obstacle layer

/cloud_registered + /Odometry + PCD map + /initialpose
  |
  v
global_localization.py (Open3D ICP, 0.5 Hz)
  `-- /map_to_odom                최신 global correction

/Odometry + /map_to_odom
  |
  v
transform_fusion_cpp
  |-- /localization               map 기준 body pose, 약 10 Hz
  `-- /tf                         map -> camera_init, 약 10 Hz

Nav2
  |-- /map + global/local costmaps
  `-- /cmd_vel, controller 5 Hz
         |
         v
joy_to_lcm -> LCM CMD_VEL -> main controller
```

### 3.2 현재 TF chain과 각 publisher의 책임

현재 forward 방향 TF chain은 다음과 같다.

```text
world
  `--(static, identity)--> map
       `--(dynamic)------> camera_init
            `--(dynamic)-> body
                 |--(static, translation)--> BODY
                 |                            `--(static, identity)--> base_link
                 `--(static, translation)--> livox_frame
```

| TF | 종류 | 발행 주체 | 의미 |
|---|---|---|---|
| `world -> map` | static | `localization.launch.py`의 `world_to_map` | 현재 identity 보조 frame |
| `map -> camera_init` | dynamic | `transform_fusion_cpp` | Global ICP correction을 각 최신 Odometry timestamp에 재발행 |
| `camera_init -> body` | dynamic | `fastlio_mapping` | FAST-LIO local odometry; `body`는 FAST-LIO IMU 중심 |
| `body -> BODY` | static | `localization_nav_rubi.launch.py` | `(-0.112, 0.02329, 0.07278)`, FAST-LIO body에서 로봇 BODY로 이동 |
| `BODY -> base_link` | static | `localization_nav_rubi.launch.py` | identity; 현재 Navigation 기준점과 로봇 BODY를 같은 pose로 취급 |
| `body -> livox_frame` | static | `localization_nav_rubi.launch.py` | `(-0.011, 0.02329, -0.04412)`, FAST-LIO IMU 중심과 Livox frame 간 translation |

`tf2_monitor`는 chain을 반대 방향 표기로 다음처럼 출력했다.

```text
base_link -> BODY -> body -> camera_init -> map
```

이는 연결이 반대라는 뜻이 아니라 `map`에서 `base_link`를 조회할 때 나열된 chain 표현이다.

### 3.3 거꾸로 장착한 MID360의 180도 보정

현재 [`livox_ros_driver2/config/MID360_config.json`](livox_ros_driver2/config/MID360_config.json)에 다음 값이 들어 있다.

```json
"extrinsic_parameter": {
  "roll": 180.0,
  "pitch": 0.0,
  "yaw": 0.0,
  "x": 0,
  "y": 0,
  "z": 0
}
```

Driver의 `PubHandler`는 이 extrinsic rotation을 다음 두 데이터에 적용한다.

- LiDAR point의 `x/y/z`
- IMU의 angular velocity와 linear acceleration

따라서 현재 convention에서는 Driver가 거꾸로 장착된 센서의 orientation을 이미 180도 보정하고, FAST-LIO의 `mapping.extrinsic_R`과 `body -> livox_frame` TF 회전은 identity를 유지한다. 다른 launch에 다시 `roll=pi`를 추가하면 **이중 회전**이 되므로 주의해야 한다.

이 `roll: 180.0` 및 IMU/point 회전 코드는 `2b118b4` 이전부터 존재했다. 이번 커밋의 dual-publish 경로도 이미 회전된 동일 `StoragePacket`을 이용하므로 `/livox/lidar`와 `/livox/lidar_pointcloud`의 좌표 방향은 서로 동일하다.

---

## 4. 증상과 원인 분리

### 4.1 Nav2에서 관측된 증상

문제 구간에서 다음 유형의 로그가 반복되었다.

```text
Extrapolation Error looking up target frame:
Lookup would require extrapolation into the future

Transform data too old when converting from camera_init to map
Unable to transform robot pose into global plan's frame
Control loop missed its desired rate of 10.0000Hz
Controller patience exceeded
[FollowPath] Aborting handle
```

Planner는 A* path를 찾았지만 controller가 path timestamp에 대응하는 최신 `map -> base_link` TF를 얻지 못했다. 이것은 planner 자체의 path search 실패가 아니라 localization/TF 시간축이 controller 요청 시간보다 뒤처진 상태였다.

### 4.2 `ros2 topic hz`와 `ros2 topic delay`의 의미

`ros2 topic hz`는 해당 CLI subscriber가 받은 메시지 사이의 wall-clock 간격을 측정한다. Publisher 내부의 timer 주기와 항상 같지 않으며, DDS 전달과 CLI 역직렬화가 밀리면 낮게 보일 수 있다.

`ros2 topic delay`는 대체로 다음 값을 계산한다.

```text
subscriber가 메시지를 받은 현재 ROS time - message.header.stamp
```

따라서 delay는 YAML에 설정된 단일 상수가 아니며, publisher가 넣은 timestamp와 시스템의 실제 전달/처리 상태를 합친 측정값이다. Sliding window 평균이므로 출력될 때마다 값을 누적해서 더하는 구조도 아니다.

### 4.3 `/livox/lidar`의 약 0.7 s delay를 그대로 믿으면 안 된 이유

Livox `CustomMsg`의 `header.stamp/timebase`는 약 100 ms scan의 **시작 시각**이다. 마지막 point는 `offset_time`만큼 뒤에 있다. 따라서 정확한 최신 point 지연은 다음처럼 봐야 한다.

```text
scan_start_age = now - timebase
scan_end_age   = now - (timebase + last_point.offset_time)
```

Driver 내부 진단 결과 정상 구간은 다음과 같았다.

| 계측 | 관측값 | 해석 |
|---|---:|---|
| SDK ingress age | 약 0.002~0.086 ms | NIC/SDK 수신 직후 timestamp는 사실상 최신 |
| raw queue | 대부분 1 | Driver raw queue 적체 없음 |
| publish start age | 약 99.7~106 ms | 약 100 ms scan을 모으는 정상 시간 |
| publish end age | 약 0.84~3.74 ms | 최신 point 기준 실제 driver 지연은 매우 작음 |
| points per CustomMsg | 약 19,968~20,352 | 10 Hz packet 정상 구성 |

Driver 시작 직후 최초 한 frame에서 약 1.5 s가 나온 것은 장치 mode 전환/초기화 중 모인 startup frame이었다. 정상 steady state 판단에서는 제외했다.

반면 Python 기반 `ros2 topic delay /livox/lidar`는 약 0.63~0.83 s를 보였다. 대형 custom message 역직렬화 및 CLI subscriber 자체 backlog가 포함된 결과로 판단된다. 이 값 하나만으로 센서나 driver가 0.7 s 늦다고 결론 내리면 안 된다.

또한 다음 오류는 데이터 발행 실패가 아니라 현재 shell에서 custom interface를 찾지 못한 환경 문제이다.

```text
The message type 'livox_ros_driver2/msg/CustomMsg' is invalid
```

`source ~/ros2_ws/install/setup.bash` 후 `ros2 interface show livox_ros_driver2/msg/CustomMsg`가 성공하는지 먼저 확인해야 한다.

### 4.4 `/Odometry`와 `/localization`의 약 1.4 s는 실제 지연이었다

다음 증거 때문에 `/Odometry`와 `/localization` 지연은 단순 timestamp 해석 문제가 아니었다.

- `/Odometry`와 `/localization`의 rate 자체가 약 0.72~0.79 Hz로 떨어짐
- 센서를 손으로 회전했을 때 RViz arrow 반영도 실제로 느렸음
- `transform_fusion` 진단에서 `input_interval`과 `input_age`가 약 1.2~1.4 s로 증가함
- Nav2의 최신 TF lookup이 반복해서 실패함
- 개별 FAST-LIO ICP 처리 시간은 대체로 수 ms인데도 Odometry callback 간격이 1.28 s로 증가함

즉 계산 한 번이 1.3 s 걸린 것이 아니라, 처리 callback/timer가 executor에서 제때 실행되지 못한 scheduling starvation과 이전 frame 적체가 핵심이었다.

### 4.5 Global localization은 transport delay를 보정하지 않는다

Open3D global ICP는 local FAST-LIO odometry drift를 map 기준으로 보정한다. 그러나 1.4 s 전 pose가 들어오는 transport/scheduling delay 자체를 미래 pose로 복원하지는 않는다. 오래된 pose에 global correction을 곱해도 여전히 오래된 pose이다.

따라서 global localization이 있다는 이유로 `/Odometry` 또는 TF 지연을 허용할 수 없다. 먼저 local odometry의 freshness를 보장하고, 그 위에서 낮은 주기의 global correction을 적용해야 한다.

---

## 5. `livox_ros_driver2` 변경

관련 파일:

- [`livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py`](livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py)
- [`livox_ros_driver2/src/comm/pub_handler.cpp`](livox_ros_driver2/src/comm/pub_handler.cpp)
- [`livox_ros_driver2/src/comm/pub_handler.h`](livox_ros_driver2/src/comm/pub_handler.h)
- [`livox_ros_driver2/src/lddc.cpp`](livox_ros_driver2/src/lddc.cpp)
- [`livox_ros_driver2/src/lddc.h`](livox_ros_driver2/src/lddc.h)

### 5.1 전송 형식 변경 및 dual publish

변경 전:

```python
xfer_format = 0  # PointCloud2
```

변경 후:

```python
xfer_format = 1  # CustomMsg + secondary PointCloud2
```

원래 driver는 한 transfer format만 선택한다. 이번 변경에서는 ROS 2에서 custom format을 선택하면 동일한 완료 scan으로 두 topic을 발행하도록 `PublishCustomAndPointcloud2()`를 추가했다.

| Topic | Type | Rate | 소비자 | Timestamp 정책 |
|---|---|---:|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 10 Hz | FAST-LIO | 원본 timebase와 per-point offset 유지 |
| `/livox/lidar_pointcloud` | `sensor_msgs/msg/PointCloud2` | 5 Hz | Nav2 obstacle layers | 현재 실험에서는 zero stamp로 latest TF 요청 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 장치 주기, 관측 약 200 Hz | FAST-LIO | 장치 timestamp 유지 |

CustomMsg를 먼저 publish하고, 매 두 번째 frame에만 PointCloud2 변환을 수행한다. 따라서 Nav2용 변환 비용이 FAST-LIO 10 Hz 입력을 5 Hz로 낮추지 않는다.

### 5.2 완료 scan queue는 최신 frame 우선

변경 전에는 consumer가 늦어져도 완료된 scan을 FIFO로 전부 처리했다. Producer와 consumer가 다시 같은 10 Hz가 되면 이미 생긴 backlog가 영구적으로 줄지 않아 모든 후속 메시지가 계속 늦을 수 있다.

변경 후 `PublishCustomAndPointcloud2()` 진입 시 완료 frame이 둘 이상 쌓였으면 마지막 최신 frame만 남기고 이전 완료 frame을 버린다.

```text
before: old1 -> old2 -> old3 -> newest 를 모두 발행
after : old1, old2, old3 폐기 -> newest만 발행
```

FAST-LIO는 누락된 LiDAR frame 사이의 시간 구간을 IMU로 연결하므로, localization에서는 오래된 LiDAR frame을 모두 재생하는 것보다 최신 상태를 유지하는 편이 낫다. 단, 이는 lossless mapping/bag 재생용 정책이 아니라 **실시간 localization용 정책**이다.

### 5.3 raw packet queue는 무조건 최신 우선이 아님

Raw Livox packet은 완성된 scan이 아니라 scan 일부이다. 여기서 오래된 packet을 무조건 하나씩 버리면 서로 다른 시간대의 point가 섞인 불완전 scan을 만들 수 있다.

따라서 다음 2단계 정책을 사용한다.

1. queue의 timestamp span이 publish period 두 배 이내이면 기존 FIFO 순서를 유지한다.
2. span이 두 publish period를 넘으면 newest raw packet만 남기고 queue와 현재 누적 point cloud를 함께 비운 뒤 새 scan을 구성한다.

이를 위해 `ClearLidarPointClouds()`를 추가했다. 과부하 복구 시 stale raw points와 최신 packet이 섞이지 않도록 하기 위함이다.

원본 driver가 최신 packet을 쉽게 버리는 방향처럼 보였던 이유는 일반 driver가 packet 순서와 scan 완전성을 우선하기 때문이다. 실시간 localization에서는 제한 조건 아래 명시적으로 stale packet/scan을 폐기하도록 동작을 바꿨다.

### 5.4 point 변환 mutex 최적화

변경 전에는 point 하나마다 mutex lock/unlock을 수행했다. 약 2만 point/frame에서 불필요한 동기화 비용이 반복되었다.

변경 후에는 다음 세 변환 함수에서 packet당 한 번만 lock하고 필요한 vector capacity를 먼저 reserve한다.

- `ProcessCartesianHighPoint()`
- `ProcessCartesianLowPoint()`
- `ProcessSphericalPoint()`

point 순서, 값, timestamp는 유지하면서 driver CPU/lock contention만 줄였다.

### 5.5 PointCloud2 QoS와 zero timestamp의 의도

보조 `/livox/lidar_pointcloud` publisher는 `SensorDataQoS().keep_last(1)`을 사용한다. costmap은 누적된 과거 cloud보다 최신 장애물 snapshot이 중요하기 때문이다.

현재 보조 PointCloud2에는 다음 코드가 적용되어 있다.

```cpp
cloud.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
```

TF2에서 zero stamp는 latest available transform을 요청하는 의미로 사용되어 costmap callback이 조금 늦어도 old-TF rejection을 피한다. 그러나 로봇이 빠르게 움직이는 동안 “측정 시점의 센서 pose”가 아니라 “처리 시점의 최신 pose”로 cloud를 변환하므로 장애물 위치에 운동량에 비례한 공간 오차가 생길 수 있다.

따라서 이 선택은 당일 실차 안정화를 위한 의도적인 trade-off이다. 장기적으로는 원본 scan timestamp를 유지하면서 TF가 같은 timestamp로 충분히 빠르게 제공되도록 구성하는 것이 더 정확하다.

### 5.6 Driver 진단 환경변수

`LIVOX_DIAG_TIMING=1`일 때 초당 한 번 다음 정보를 출력한다.

- `ingress_age`
- `raw_queue`
- `publish_start_age`
- `publish_end_age`
- `points`

정상 운용에서는 로그 부하를 피하기 위해 환경변수를 제거해도 된다. 진단은 동작 변경이 아니라 계측용이다.

### 5.7 확인된 종료 시 문제

한 진단 실행에서 Ctrl-C 이후 driver가 SDK deinit을 마친 뒤 exit code `-11`로 종료되었다. 실시간 데이터 문제와는 별개지만 shutdown 경로의 use-after-free/double cleanup 가능성을 후속 조사해야 한다.

---

## 6. FAST-LIO 변경

관련 파일:

- [`FAST_LIO_LOCALIZATION2/src/laserMapping.cpp`](FAST_LIO_LOCALIZATION2/src/laserMapping.cpp)
- [`FAST_LIO_LOCALIZATION2/config/mid360.yaml`](FAST_LIO_LOCALIZATION2/config/mid360.yaml)
- [`FAST_LIO_LOCALIZATION2/CMakeLists.txt`](FAST_LIO_LOCALIZATION2/CMakeLists.txt)

### 6.1 Livox CustomMsg 모드 사용

`preprocess.lidar_type`을 4에서 1로 변경했다.

| 설정 | 변경 전 | 변경 후 |
|---|---:|---:|
| `preprocess.lidar_type` | 4, generic PointCloud2 | 1, Livox CustomMsg |
| 입력 topic | `/livox/lidar` | `/livox/lidar` |
| 실제 type | PointCloud2 가정 | `livox_ros_driver2/msg/CustomMsg` |

Livox CustomMsg는 각 point의 `offset_time`, `line`, `tag`를 보존한다. FAST-LIO의 deskew와 IMU 동기화에는 Nav2용으로 단순 변환한 PointCloud2보다 이 형식이 적합하다.

### 6.2 ROS 1 스타일 queue 20000 제거

변경 전:

```cpp
create_subscription<CustomMsg>(lid_topic, 20000, livox_pcl_cbk)
```

변경 후:

```cpp
auto lidar_qos = rclcpp::SensorDataQoS().keep_last(2);
create_subscription<CustomMsg>(lid_topic, lidar_qos, livox_pcl_cbk, ...)
```

`20000`은 ROS 2에서 단순히 “안전한 버퍼”가 아니다. 처리량이 입력량보다 잠시 낮을 때 오래된 대형 scan을 대량 보존하여 freshness를 파괴할 수 있다.

현재 CustomMsg publisher는 RELIABLE이고 FAST-LIO reader는 BEST_EFFORT/KEEP_LAST(2)이다. RELIABLE writer와 BEST_EFFORT reader는 호환되며, reader는 실시간 처리를 위해 오래된 sample을 유지하지 않는다.

IMU는 LiDAR와 달리 integration 연속성이 중요하므로 이번 변경에서 동일하게 depth 1~2로 줄이지 않고 기존 depth 10을 유지했다.

### 6.3 FAST-LIO 내부 LiDAR deque 상한

DDS queue를 줄여도 callback 이후의 `lidar_buffer`가 무제한이면 다시 backlog가 생길 수 있다. 내부 deque를 최대 2 frame으로 제한했다.

- 아직 마지막 IMU sample을 기다리는 front scan(`lidar_pushed == true`)은 보존
- 그 뒤의 더 오래된 pending scan을 최신 scan으로 교체
- front가 동기화 대기 중이 아니면 oldest pending scan 폐기

센서 callback과 processing callback이 병렬 실행되므로 `sync_packages()`의 queue snapshot/pop 구간에는 mutex를 추가했고, 비싼 IMU/ICP 계산 전에는 lock이 해제되도록 범위를 제한했다.

### 6.4 processing timer 주기 수정

변경 전 코드:

```cpp
5000.0 / 100.0 = 50 ms
```

변경 후:

```cpp
1000.0 / 100.0 = 10 ms
```

100 Hz로 동기화 가능한 LiDAR/IMU package가 있는지 polling한다. 이것이 Odometry를 100 Hz로 발행한다는 뜻은 아니다. LiDAR 완료 scan이 약 10 Hz이므로 `/Odometry`도 scan당 한 번, 약 10 Hz로 유지된다.

### 6.5 callback group과 MultiThreadedExecutor

가장 중요한 최종 변경이다.

변경 전:

```text
SingleThreadedExecutor
  - 200 Hz IMU callback
  - 10 Hz LiDAR callback
  - FAST-LIO processing timer
```

변경 후:

```text
MultiThreadedExecutor(2 threads)
  - MutuallyExclusive sensor callback group
      - IMU callback
      - LiDAR callback
  - MutuallyExclusive processing callback group
      - sync + IMU propagation + ICP + publish timer
```

센서 수신과 실제 FAST-LIO 처리가 서로 다른 executor thread에서 진행될 수 있고, 두 경로의 shared queue는 mutex로 보호한다.

문제 시점의 내부 mapping 계산은 대체로 10 ms 미만인데 fusion의 `input_interval`이 약 1280 ms가 되었다. 이는 ICP가 1.28 s 걸린 것이 아니라 timer가 실행 기회를 얻지 못했다는 강한 증거이다. callback group 분리와 2-thread executor 적용 후 `input_interval`이 약 100 ms로 안정되었다.

### 6.6 Jetson aarch64에서 OpenMP 활성화

기존 CMake는 x86/amd64일 때만 `MP_EN`을 정의했다. 따라서 Jetson aarch64에서는 여러 core를 `taskset`으로 허용해도 nearest-neighbor matching의 OpenMP 경로가 꺼져 있었다.

변경 후 architecture 조건을 제거하고 processor 수에 따라 다음처럼 정의한다.

- core 수가 4보다 크면 `MP_PROC_NUM=3`
- core 수가 3보다 크면 `MP_PROC_NUM=2`
- 그 이하는 1

`taskset`은 process가 실행 가능한 CPU 집합만 제한한다. core를 독점 예약하거나 DDS thread, kernel network processing, memory bandwidth를 격리하지는 않는다. 그러므로 서로 다른 affinity를 줘도 Navigation이 FAST-LIO에 간접 영향을 줄 수 있으며, executor 구조 수정이 별도로 필요했다.

### 6.7 불필요한 출력/계산 축소

`mid360.yaml` 변경:

| 설정 | 변경 전 | 변경 후 | 영향 |
|---|---:|---:|---|
| `point_filter_num` | 3 | 4 | 입력 point sampling 감소, 계산량 감소 |
| `publish.path_en` | true | false | FAST-LIO debug Path 비활성화 |
| `publish.scan_bodyframe_pub_en` | true | false | `/cloud_registered_body` 비활성화 |
| `publish.scan_publish_en` | true | true | `/cloud_registered`는 global ICP 때문에 유지 |
| `publish.dense_publish_en` | false | false | downsampled world cloud 유지 |

`/path`와 `/cloud_registered_body`는 이 시스템에서 localization state 생성에 사용되지 않는 시각화/debug 출력이므로 비활성화해도 `/Odometry`, `/cloud_registered`, global ICP, Nav2 plan에는 문제가 없다.

다만 `point_filter_num=4`는 계산량과 point density 사이 trade-off이므로 좁은 구조물/저반사 환경에서 ICP fitness와 drift를 실차로 확인해야 한다.

### 6.8 Path timestamp 문제 제거

기존 fork에서 `publish.path_en` parameter 읽기가 주석 처리되어 있었는데 이를 복구하고 실제 `publish_path()` 호출을 조건부로 변경했다. 오래전에 만들어진 Path header를 반복 발행하여 RViz가 다음 로그를 내는 현상을 방지한다.

```text
Message Filter dropping message ... timestamp ... earlier than all the data in the transform cache
```

Nav2 global plan `/plan`은 FAST-LIO debug `/path`와 별개이다.

### 6.9 FAST-LIO 진단 환경변수

`FASTLIO_DIAG_TIMING=1`일 때 다음을 출력한다.

- callback: `callback_start_age`, `callback_end_age`, `callback_work`, `lidar_buffer`, point 수
- mapping: `imu_fov_filter`, `icp`, `odom_publish`, `map_update`, `cloud_publish`, `total`, point 수

관측된 안정 구간 예시는 다음과 같다.

| 지표 | 값 |
|---|---:|
| callback start age | 약 99~106 ms |
| callback end age | 약 1.8~6.1 ms |
| callback work | 약 0.28~0.50 ms |
| lidar buffer | 1~2 |
| ICP | 대체로 약 2~6 ms |
| mapping total | 대체로 약 2.9~12 ms |
| raw points/message | 약 19,968~20,064 |
| mapping input/downsampled | 약 2,700~3,100 / 약 300~420 |

`callback_start_age` 약 100 ms는 driver 지연이 아니라 scan 시작 timestamp 기준이므로 정상이다. 판단 시 `callback_end_age`를 봐야 한다.

---

## 7. Global localization 및 transform fusion 변경

관련 파일:

- [`FAST_LIO_LOCALIZATION2/fast_lio_localization/global_localization.py`](FAST_LIO_LOCALIZATION2/fast_lio_localization/global_localization.py)
- [`FAST_LIO_LOCALIZATION2/fast_lio_localization/transform_fusion.py`](FAST_LIO_LOCALIZATION2/fast_lio_localization/transform_fusion.py)
- [`FAST_LIO_LOCALIZATION2/src/transformFusion.cpp`](FAST_LIO_LOCALIZATION2/src/transformFusion.cpp)
- [`FAST_LIO_LOCALIZATION2/launch/localization.launch.py`](FAST_LIO_LOCALIZATION2/launch/localization.launch.py)
- [`FAST_LIO_LOCALIZATION2/package.xml`](FAST_LIO_LOCALIZATION2/package.xml)

### 7.1 Global ICP 주기 1.0 Hz에서 0.5 Hz로 변경

Open3D ICP는 PCD map과 현재 scan을 coarse/fine 두 단계로 registration한다. Python single-threaded executor에서 가장 무거운 작업이므로 `freq_localization`을 1.0에서 0.5 Hz로 낮췄다.

```text
/map_to_odom: 2초마다 최신 global correction 갱신
/localization: 매 /Odometry마다 correction 합성, 약 10 Hz
```

따라서 `/map_to_odom`을 `ros2 topic hz`로 측정했을 때 약 0.49~0.52 Hz가 나오는 것은 정상이다. `/localization`까지 0.5 Hz여야 하는 것은 아니다.

### 7.2 최신 snapshot QoS

`global_localization.py`의 `/cloud_registered`와 `/Odometry` subscription을 `BEST_EFFORT + KEEP_LAST(1)`로 변경했다. Global ICP가 실행되는 동안 Python executor가 새 callback을 처리하지 못하더라도 완료 후 오래된 scan/odom을 순서대로 재생하지 않고 최신 snapshot으로 넘어가기 위한 것이다.

`/cur_scan_in_map`, `/submap` publisher도 depth 1을 사용한다. 이 topic들은 상태 snapshot/RViz용이며 오래된 대형 cloud를 보관할 필요가 없다.

### 7.3 RGB/intensity warning 제거

기존 `ros2_numpy.msgify()`는 입력 dictionary에 `rgb`와 `intensity`가 없을 때 반복 warning을 출력했다. Livox localization cloud에서 필요한 field는 XYZ뿐이므로 PointCloud2를 다음 layout으로 직접 구성한다.

```text
x: FLOAT32 offset 0
y: FLOAT32 offset 4
z: FLOAT32 offset 8
point_step: 12 bytes
```

이 변경은 point geometry를 바꾸지 않으며 불필요한 field 생성과 console spam을 없앤다. `ros2_numpy`는 들어오는 `/cloud_registered`를 numpy XYZ로 읽는 경로에는 계속 사용된다.

### 7.4 `/submap` timestamp 수정

기존에는 `self.cur_odom.header`를 재사용했다. ICP callback이 오래 걸리면 이미 TF cache 밖으로 나간 timestamp의 동일 submap이 반복 발행되어 RViz message filter가 계속 폐기했다.

변경 후 submap이 생성되는 현재 시각과 `frame_id=map`을 새 Header에 넣는다. Submap은 그 시점에 map frame으로 새로 만든 visualization snapshot이므로 이 정책이 맞다.

### 7.5 PCD map 반복 publish 주기 완화

`pcd_to_pointcloud`의 `period_ms_`를 500 ms에서 5000 ms로 늘렸다. Global localization node는 `pcd_map_path`에서 map을 직접 읽으므로 ICP에는 영향이 없고, 대형 map cloud의 DDS/RViz 반복 부하만 줄인다.

### 7.6 Python timer fusion에서 event-driven fusion으로 변경

초기 Python 구현은 `/Odometry`를 저장한 뒤 50 Hz timer에서 현재 시각으로 `/localization`과 TF를 발행했다. 같은 pose를 불필요하게 반복 발행하고, sensor timestamp와 TF timestamp를 분리하며, Python executor scheduling에 영향을 받았다.

Python fallback도 다음처럼 변경했다.

- 50 Hz timer 제거
- `/Odometry` callback이 들어올 때 즉시 fusion
- `/Odometry` 원본 timestamp를 `/localization`과 `map -> camera_init`에 동일하게 사용
- 최신 state depth 1 QoS 사용

### 7.7 실제 launch는 C++ `transform_fusion_cpp` 사용

최종 launch는 Python fallback 대신 새 C++ node를 실행한다.

```python
executable="transform_fusion_cpp"
```

CMake/package에는 `tf2_ros` dependency와 executable install을 추가했다.

C++ node의 QoS와 동작은 다음과 같다.

| I/O | QoS | 동작 |
|---|---|---|
| `/Odometry` subscribe | RELIABLE, depth 1 | 실제 FAST-LIO publisher와 신뢰성 일치, stale queue 방지 |
| `/map_to_odom` subscribe | SensorDataQoS, depth 1 | 가장 최신 correction만 유지 |
| `/localization` publish | depth 1 | 각 Odometry마다 합성 pose 발행 |
| `/tf` publish | DynamicBroadcasterQoS depth 1 | 같은 Odometry stamp로 `map -> camera_init` 발행 |

Fusion 수식은 개념적으로 다음과 같다.

```text
T_map_body = T_map_camera_init * T_camera_init_body
```

`/map_to_odom`이라는 topic 이름은 기존 이름을 유지했지만 현재 TF frame 의미로는 global ICP가 계산한 `map -> camera_init` correction이다.

Global ICP 결과가 아직 없으면 identity correction으로 시작한다. `/initialpose`를 RViz에서 받은 후 fitness가 threshold보다 높을 때 correction이 갱신된다.

### 7.8 `/initialpose`와 RViz

RUBI는 시작 map 좌표를 사전에 정확히 알 수 없으므로 RViz의 `2D Pose Estimate`로 `/initialpose`를 보내야 한다. Outer launch인 `localization_nav_rubi.launch.py`는 내부 RViz를 끄고 별도의 configured RViz를 `use_rviz=true` 기본값으로 실행하므로 RViz 중복 없이 initial pose 입력이 가능하다.

`use_rviz=false`는 headless 운용 옵션일 뿐, 별도 `/initialpose` publisher가 없으면 현재 실험 절차에는 적합하지 않다.

---

## 8. Nav2 및 A* 변경

관련 파일:

- [`pongbot_navigation/config/nav2_rubi_dwb.yaml`](pongbot_navigation/config/nav2_rubi_dwb.yaml)
- [`pongbot_navigation/behavior_trees/navigate_to_pose_rubi.xml`](pongbot_navigation/behavior_trees/navigate_to_pose_rubi.xml)
- [`pongbot_global_planner/src/astar_planner.cpp`](pongbot_global_planner/src/astar_planner.cpp)
- [`FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz`](FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz)

### 8.1 Nav2 odometry source 통일

`controller_server`와 `bt_navigator`의 `odom_topic`을 `/genz/odometry`에서 `/Odometry`로 변경했다. 현재 navigation은 FAST-LIO local odometry를 직접 사용하고, global pose는 TF `map -> camera_init -> ... -> base_link`로 얻는다.

### 8.2 DWB 계산량 축소

| 설정 | 변경 전 | 변경 후 |
|---|---:|---:|
| `controller_frequency` | 10 Hz | 5 Hz |
| `vx_samples` | 20 | 12 |
| `vtheta_samples` | 41 | 21 |
| 후보 조합 수(단순 곱) | 820 | 252 |
| `sim_time` | 2.0 s | 1.5 s |
| `debug_trajectory_details` | true | false |
| `publish_evaluation` | true | false |
| `publish_trajectories` | true | false |

후보 조합은 약 69% 줄었다. 실제 계산량은 critic 수와 collision checking에도 좌우되지만 Jetson에서 충분한 여유를 확보했다.

10 Hz에서는 `Control loop missed its desired rate`가 반복되었고, 5 Hz로 낮춘 뒤 경고가 사라졌다. 현재 local costmap update도 5 Hz이므로 controller가 같은 주기로 최신 local costmap을 이용한다. 최대 command update 간격은 약 200 ms이므로 초기 실차 속도는 보수적으로 유지해야 한다.

Rotation Shim 설정은 이번 커밋에서 새로 변경하지 않았고 기존 구성을 유지했다. `RotationShimController`가 `dwb_core::DWBLocalPlanner`를 primary controller로 감싼다.

### 8.3 Local costmap obstacle source 수정

변경 전에는 data type을 PointCloud2로 선언하면서 topic은 CustomMsg인 `/livox/lidar`를 사용했다. Type이 맞지 않으므로 Nav2 obstacle layer가 정상적으로 cloud를 받을 수 없다.

변경 후:

```yaml
topic: /livox/lidar_pointcloud
data_type: PointCloud2
marking: true
clearing: true
```

Local rolling costmap에서는 static layer를 제거하고 obstacle + inflation만 사용한다. Local frame은 `camera_init`, robot frame은 `base_link`이다.

로봇 자체 몸체/팔 point를 제외하기 위해 local `obstacle_min_range`를 0.1 m에서 0.5 m로 높였다. 이는 self-filter의 간단한 대체이므로 실제 팔 동작 범위와 근접 장애물 검출 손실을 함께 확인해야 한다.

### 8.4 Global costmap dynamic obstacle 및 replanning

Global costmap에도 `/livox/lidar_pointcloud` obstacle layer가 활성화되어 있다.

- update 2 Hz
- publish 1 Hz
- BT global replanning 1 Hz
- marking/clearing true
- obstacle range 0.3~4.0 m
- raytrace max 5.0 m

Custom A* planner는 호출될 때 현재 costmap의 lethal/inscribed/unknown cost를 확인하므로, obstacle layer가 global costmap을 갱신하면 다음 1 Hz replanning에서 새 장애물을 우회할 수 있다.

현재 plugin 순서는 다음과 같다.

```yaml
plugins: [obstacle_layer, static_layer, inflation_layer]
```

Static layer의 기본 combination이 max이면 obstacle cost가 보통 유지되지만, 장기적으로는 관례적인 `[static_layer, obstacle_layer, inflation_layer]` 순서로 바꾸고 동적 장애물이 실제 master costmap에 남는지 검증하는 편이 명확하다.

`always_send_full_costmap: true`는 RViz 검증에는 편하지만 DDS 부하가 크다. obstacle 동작을 확인한 후 false로 전환하는 것을 권장한다.

### 8.5 A* Path timestamp를 zero로 변경

변경 전에는 `start.header.stamp`를 global path와 각 pose에 복사했다. Path 전달 시점에 그 timestamp가 최신 TF보다 미래가 되면 DWB가 extrapolation error를 냈다.

변경 후 path 및 모든 pose header를 다음처럼 통일한다.

```cpp
path.header.stamp = builtin_interfaces::msg::Time{};
```

Map 위의 geometric path는 특정 센서 측정 시각의 관측값이 아니므로 controller가 latest available TF로 변환하도록 한다. 이것은 localization 지연의 근본 해결을 대체하지 않지만 수 ms 수준의 publisher/controller 순서 차이로 생기는 불필요한 future extrapolation을 방지한다.

### 8.6 수동 이동 후 navigation을 이어가기 위한 BT 변경

실험 절차는 goal을 먼저 찍고, main controller 조이스틱으로 안전 지역까지 이동한 후 navigation mode를 켜 기존 goal로 이동하는 방식이다.

이를 위해 다음 값을 크게 늘렸다.

- `SimpleProgressChecker.movement_time_allowance: 1000.0`
- top-level `RecoveryNode.number_of_retries: 1000`

또한 humanoid에서 위험할 수 있는 자동 `Spin`/`BackUp` recovery를 제거하고 다음 safe recovery만 사용한다.

```text
clear local costmap -> clear global costmap -> wait 1 s -> retry
```

중요하게도 이것은 Nav2 action의 공식 pause/resume 기능이 아니다. Goal이 active인 동안 Nav2는 계속 `/cmd_vel`을 계산할 수 있으며, main controller의 navigation mode switch가 실제 actuator command authority를 안전하게 선택해야 한다. 1000 s/1000회 설정은 실험 중 조기 abort를 막지만 진짜 stuck 상태도 오래 숨길 수 있으므로 운영 설정으로 그대로 사용하기 전에 timeout 정책을 다시 설계해야 한다.

### 8.7 RViz 변경

TF tree에서 사용하지 않는 leg/link 표시를 제거하고 `/local_costmap/costmap` Map display를 추가했다. 이는 localization 알고리즘 변경이 아니라 장애물 layer와 TF를 실시간으로 검증하기 위한 UI 변경이다.

---

## 9. 조이스틱/LCM 변경

관련 파일:

- [`pongbot_lcm/launch/joy_lcm.launch.py`](pongbot_lcm/launch/joy_lcm.launch.py)
- [`pongbot_lcm/src/joy_lcm_node.cpp`](pongbot_lcm/src/joy_lcm_node.cpp)

기본 조이스틱 topic을 `/joy`에서 `/joy_wireless`로 변경했다.

```text
before: /joy
after : /joy_wireless
```

Jetson LCM bridge가 main NUC와 연결된 wireless joystick의 실제 ROS topic을 구독하도록 맞춘 것이다. `/cmd_vel` 입력과 `CMD_VEL` LCM channel, multicast URL은 변경하지 않았다.

LCM message의 velocity field에는 Nav2 `/cmd_vel`의 최신 값이 들어가고, joystick axes/buttons는 별도 field로 전달된다. Main controller는 navigation mode button에 따라 어느 명령을 actuator에 적용할지 결정해야 한다.

---

## 10. 문제 전후 계측 상세

### 10.1 초기 문제 상태

여러 실행에서 다음 범위가 반복되었다.

| 측정 | 문제 상태 |
|---|---:|
| `/Odometry` delay | 평균 약 1.35~1.50 s, 일부 1.6~2.0 s |
| `/Odometry` rate | 약 10 Hz이지만 오래된 queue를 발행하거나, 심할 때 약 0.74~0.78 Hz |
| `/localization` delay | 약 1.3~1.5 s |
| `/localization` rate | 약 0.72~0.79 Hz |
| fusion `input_interval` | 약 1,210~1,290 ms |
| fusion `input_age` | 약 1,300~1,380 ms |
| Nav2 | future extrapolation, old TF, controller missed, abort 반복 |

한 실행에서는 `/localization` delay가 처음 약 0.017 s였다가 window 안에서 0.056, 0.1, 0.5, 1.0 s 순으로 증가한 뒤 약 1.36 s로 고정되었다. 이는 `tf2_monitor` 평균이 단순히 시간을 누적해서 커진 것이 아니라 실행 도중 pipeline이 정상 상태에서 starvation 상태로 전환되었음을 보여준다.

### 10.2 최종 안정 상태: Navigation 실행 전

| 측정 | 최종 결과 |
|---|---:|
| `/Odometry` rate | 약 9.99~10.01 Hz |
| `/Odometry` interval | 주로 약 0.096~0.105 s |
| `/Odometry` delay | 주로 약 0.014~0.017 s |
| `/localization` rate | 약 10 Hz |
| `/localization` delay | 주로 약 0.016~0.017 s |
| `/map_to_odom` rate | 약 0.5 Hz |

### 10.3 최종 안정 상태: Navigation 실행 후

| 측정 | 최종 결과 |
|---|---:|
| `/Odometry` rate | 장시간 약 9.98~10.01 Hz |
| `/Odometry` interval | 주로 약 0.088~0.113 s |
| `/Odometry` delay | 주로 약 0.015~0.016 s |
| `/localization` rate | 약 9.99~10.01 Hz |
| `/localization` delay | 주로 약 0.015~0.018 s |
| fusion `input_interval` | 약 98.8~103.4 ms |
| fusion `input_age` | 약 9.5~23 ms |
| fusion callback total | 약 0.1~0.2 ms |

짧은 0.2~0.4 s interval spike가 초기 window에 섞인 적은 있으나, 이후 window가 다시 10 Hz로 회복했고 과거처럼 1.3 s 상태에 고정되지 않았다.

### 10.4 `tf2_monitor` 해석 주의

`tf2_monitor map base_link`의 `Net delay avg`는 실행 중 받은 dynamic transform sample의 통계이며 출력될 때마다 0.01을 단순 누적하는 counter가 아니다. 문제가 생긴 실행에서는 실제 dynamic frame `body`/`camera_init` delay가 약 1.35 s였으므로 Net delay도 점차 그 분포를 반영했다.

반면 `BODY`, `base_link`, `map` 등에 수백 초 delay가 표시된 것은 `/tf_static` transform이 시작 시 한 번 발행된 시각과 현재 시각을 비교한 통계라서 dynamic localization freshness 판단에 사용하면 안 된다.

판정할 때는 다음을 우선한다.

1. `/Odometry` rate/delay
2. `/localization` rate/delay
3. `transform_fusion`의 `input_interval`/`input_age`
4. FAST-LIO의 `callback_end_age`
5. Nav2 extrapolation/old TF 로그 유무

---

## 11. 빌드 및 실행 방법

### 11.1 환경 통일

모든 terminal에서 같은 ROS domain과 RMW를 사용해야 한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

과거 build에서 shell에 `RMW_IMPLEMENTATION=rmw_zenoh_cpp`가 남아 있어 설치되지 않은 middleware를 찾는 CMake 오류가 발생했다. Build 전에도 `rmw_fastrtps_cpp`를 명시해야 한다.

### 11.2 빌드

```bash
cd ~/ros2_ws
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

colcon build --symlink-install \
  --packages-select \
    livox_ros_driver2 \
    fast_lio_localization \
    pongbot_global_planner \
    pongbot_navigation \
    pongbot_lcm \
  --allow-overriding livox_ros_driver2 \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble \
    -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

`--allow-overriding`은 같은 workspace install을 underlay로 source한 상태에서 동일 package를 다시 build할 때 나오는 override 경고를 명시적으로 허용한다. 가능하면 clean terminal에서 `/opt/ros/humble`만 source하고 workspace를 build하는 것이 더 명확하다.

### 11.3 실험 실행 예

Terminal 1, Livox:

```bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
taskset -c 0,1 ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Terminal 2, FAST-LIO localization:

```bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
taskset -c 2,3,4,5 ros2 launch \
  fast_lio_localization localization_nav_rubi.launch.py
```

Terminal 3, Navigation:

```bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
taskset -c 6,7 ros2 launch \
  pongbot_navigation rubi_navigation_dwb.launch.py
```

Terminal 4, LCM:

```bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch pongbot_lcm joy_lcm.launch.py
```

현재 affinity 배치는 실험에서 안정적이었던 값이지 hard real-time isolation은 아니다. `taskset`으로 지정하지 않은 rosbag/CLI/RViz/DDS thread도 같은 CPU와 memory bandwidth를 사용할 수 있다. `ros2 bag record -a`는 point cloud와 map까지 모두 기록하면 절대 가벼운 작업이 아니므로 장시간 실험에서는 필요한 topic만 기록하는 것이 안전하다.

### 11.4 선택적 진단 실행

```bash
LIVOX_DIAG_TIMING=1 \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
taskset -c 0,1 ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

```bash
FASTLIO_DIAG_TIMING=1 \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
taskset -c 2,3,4,5 ros2 launch \
  fast_lio_localization localization_nav_rubi.launch.py
```

진단 로그가 필요하지 않은 실차 운용에서는 두 환경변수를 제거한다.

---

## 12. 재현 및 검증 체크리스트

### 12.1 Driver만 실행

```bash
ros2 topic hz /livox/lidar --window 30
ros2 topic hz /livox/lidar_pointcloud --window 30
ros2 topic hz /livox/imu --window 100
ros2 topic info /livox/lidar --verbose
ros2 topic info /livox/lidar_pointcloud --verbose
```

기대값:

- CustomMsg 약 10 Hz
- PointCloud2 약 5 Hz
- Driver 진단 `raw_queue` 대부분 1
- Driver 진단 `publish_end_age` 대체로 수 ms

### 12.2 Localization만 실행

RViz에서 2D Pose Estimate를 입력한 뒤 확인한다.

```bash
ros2 topic hz /Odometry --window 30
ros2 topic delay /Odometry --window 30
ros2 topic hz /localization --window 30
ros2 topic delay /localization --window 30
ros2 topic hz /map_to_odom --window 10
ros2 run tf2_ros tf2_monitor map base_link
```

기대값:

- `/Odometry`, `/localization` 약 10 Hz
- 두 topic delay 수십 ms 이내, 장시간 1.3 s 상태로 전환되지 않음
- `/map_to_odom` 약 0.5 Hz
- TF chain 연결 유지

### 12.3 Navigation 추가 실행

위 명령을 새로운 CLI process로 다시 측정한다. 기존 `hz` window에 startup gap을 섞지 않는 것이 좋다.

기대값:

- Navigation 이후에도 `/Odometry`, `/localization` 약 10 Hz 유지
- `Extrapolation Error`/`Transform data too old` 연속 발생 없음
- 5 Hz controller에서 `Control loop missed its desired rate` 연속 발생 없음
- local costmap에 0.5~4.0 m 범위 장애물 marking/clearing 확인
- global costmap에 동적 장애물 반영 후 약 1~1.5 s 이내 다음 replan에서 path 변경 확인

### 12.4 조이스틱/LCM 확인

```bash
ros2 topic hz /joy_wireless
ros2 topic info /joy_wireless --verbose
ros2 topic hz /cmd_vel
```

Goal active 중 `/cmd_vel`은 현재 controller 설정상 약 5 Hz가 기대된다. Main controller의 navigation mode가 꺼진 동안에도 Nav2 topic 자체는 발행될 수 있으므로, actuator에서 어느 source가 선택되는지를 별도로 확인한다.

---

## 13. 현재 남아 있는 위험 및 후속 작업

### 13.1 실차 전에 반드시 확인할 항목

1. `/livox/lidar_pointcloud`의 zero timestamp가 이동 중 obstacle 위치를 허용 가능한 수준으로 유지하는지 확인한다.
2. 정지 장애물 앞에서 local costmap marking뿐 아니라 뒤로 물러났을 때 clearing도 확인한다.
3. Global costmap layer 순서를 `[static_layer, obstacle_layer, inflation_layer]`로 바꾼 A/B 결과와 현재 순서를 비교한다.
4. `point_filter_num=4`에서 좁은 기둥/난간과 빠른 회전 시 FAST-LIO drift 및 ICP fitness를 확인한다.
5. `movement_time_allowance=1000`, retry 1000이 진짜 고장 상태를 무한 재시도하지 않도록 운영용 timeout을 별도로 정한다.
6. `max_vel_x=0.7`, `max_vel_theta=1.5`, 큰 가감속 값은 5 Hz control의 200 ms 간격을 고려해 저속부터 단계적으로 검증한다.
7. `body`, `BODY`, `base_link`, `livox_frame`의 translation을 CAD/실측값과 대조한다. 대소문자가 다른 `body`와 `BODY`는 ROS TF에서 완전히 다른 frame이다.

### 13.2 코드 유지보수상 주의

- Livox upstream driver 내부를 직접 수정했으므로 driver update/rebase 시 `pub_handler` 및 `lddc` 충돌 가능성이 높다.
- Real-time latest-frame 정책은 localization에 맞춘 것이다. 원본 packet을 모두 보존해야 하는 mapping/recording 용도에는 별도 launch 또는 upstream driver를 사용하는 편이 안전하다.
- `MID360_config.json`의 180도 회전과 TF에 동시에 180도 회전을 넣지 않는다.
- `FAST_LIO_LOCALIZATION2/config/mid360.yaml`과 launch에 절대 경로가 포함되어 있어 다른 PC/사용자 경로에서 바로 실행되지 않는다.
- `map_file_path`는 `FAST_LIO_ROS2/PCD/...`를 가리키지만 global localization launch는 별도 `pcd_map_path`를 사용한다. 실제 사용 여부를 정리하고 package share/launch argument로 통일하는 것이 좋다.
- 진단용 `std::cerr`/INFO throttle은 원인 분석 후 기본 off 상태로 유지한다.

### 13.3 Git hygiene 문제

현재 커밋에는 다음 생성물이 실수로 포함되었다.

```text
FAST_LIO_LOCALIZATION2/fast_lio_localization/__pycache__/__init__.cpython-310.pyc
FAST_LIO_LOCALIZATION2/fast_lio_localization/__pycache__/global_localization.cpython-310.pyc
FAST_LIO_LOCALIZATION2/fast_lio_localization/__pycache__/transform_fusion.cpython-310.pyc
FAST_LIO_LOCALIZATION2/launch/__pycache__/localization.launch.cpython-310.pyc
```

Python bytecode는 source가 아니며 Python version/환경에 종속되므로 다음 후속 커밋에서 repository tracking에서 제거하고 `.gitignore`에 추가하는 것이 좋다.

```gitignore
__pycache__/
*.py[cod]
```

또한 현재 `.gitignore`의 `.zip`은 이름이 정확히 `.zip`인 파일만 대상으로 하므로 zip archive 전체를 무시하려는 의도라면 `*.zip`이어야 한다.

### 13.4 아직 해결 완료로 간주하면 안 되는 항목

- 장시간 주행 중 thermal/power mode 변화가 10 Hz 유지에 영향을 주는지
- rosbag 동시 기록 시 disk I/O와 serialization 부하
- 빠른 보행/회전 중 zero-stamp obstacle cloud 오차
- Livox driver Ctrl-C shutdown `-11`
- Global ICP fitness가 낮을 때 correction이 갱신되지 않는 구간의 drift
- 통신이 여러 컴퓨터를 통과할 경우 clock synchronization(NTP/PTP)

---

## 14. 파일별 변경 인덱스

| 파일 | 핵심 변경 |
|---|---|
| `.gitignore` | `Livox-SDK2`, PCD, zip 관련 ignore 수정; pycache는 아직 누락 |
| `FAST_LIO_LOCALIZATION2/CMakeLists.txt` | aarch64 OpenMP 활성화, C++ fusion build/install, tf2 dependency |
| `FAST_LIO_LOCALIZATION2/package.xml` | `tf2_ros` dependency 추가 |
| `FAST_LIO_LOCALIZATION2/config/mid360.yaml` | Livox mode, point filter 4, 불필요 출력 off, map 경로 변경 |
| `FAST_LIO_LOCALIZATION2/src/laserMapping.cpp` | QoS depth 2, 내부 queue 제한, mutex, 10 ms timer, callback groups, 2-thread executor, 진단 |
| `FAST_LIO_LOCALIZATION2/src/transformFusion.cpp` | 최신 correction을 매 Odometry에 합성하는 C++ fusion 추가 |
| `FAST_LIO_LOCALIZATION2/fast_lio_localization/transform_fusion.py` | Python fallback을 timer 방식에서 Odometry event 방식으로 변경 |
| `FAST_LIO_LOCALIZATION2/fast_lio_localization/global_localization.py` | latest snapshot QoS, XYZ PointCloud2 직접 생성, submap timestamp 수정 |
| `FAST_LIO_LOCALIZATION2/launch/localization.launch.py` | ICP 0.5 Hz, C++ fusion 선택, PCD publish 5 s |
| `FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz` | TF 표시 정리, local costmap display 추가 |
| `livox_ros_driver2/launch_ROS2/rviz_MID360_launch.py` | CustomMsg 기반 dual output 모드 선택 |
| `livox_ros_driver2/src/comm/pub_handler.*` | raw backlog 복구, 누적 cloud clear, packet 단위 lock, timing 진단 |
| `livox_ros_driver2/src/lddc.*` | CustomMsg 10 Hz + PointCloud2 5 Hz, 완료 scan 최신 우선, depth 1 PCL publisher |
| `pongbot_global_planner/src/astar_planner.cpp` | Path/pose timestamp zero로 latest TF 사용 |
| `pongbot_navigation/config/nav2_rubi_dwb.yaml` | 5 Hz controller, DWB sample 축소, Odometry topic, costmap PointCloud2 source, global obstacle layer |
| `pongbot_navigation/behavior_trees/navigate_to_pose_rubi.xml` | retry 확대, Spin/BackUp 제거, clear/wait recovery |
| `pongbot_lcm/launch/joy_lcm.launch.py` | 기본 joy topic `/joy_wireless` |
| `pongbot_lcm/src/joy_lcm_node.cpp` | C++ node 기본 joy topic `/joy_wireless` |

---

## 15. 최종 판단

현재 변경은 다음 목표에는 부합한다.

- FAST-LIO는 per-point timing이 있는 Livox CustomMsg 10 Hz를 계속 사용한다.
- Nav2는 별도 PointCloud2 5 Hz로 장애물을 받는다.
- 오래된 LiDAR/Odometry state가 무한히 쌓이는 경로를 제한한다.
- Global correction은 낮은 주기로 계산하되 local pose/TF는 약 10 Hz freshness를 유지한다.
- Jetson에서 Navigation을 추가해도 최종 측정상 `/Odometry`와 `/localization`이 약 10 Hz, 수십 ms 이내 지연을 유지한다.
- DWB controller를 5 Hz로 낮춰 deadline misss를 없앴다.

따라서 저속·안전요원·물리적 비상정지 조건의 단계적 실차 검증으로 넘어갈 수 있는 상태이다. 다만 zero-stamp obstacle cloud, 1000회 BT retry, 0.7 m/s 최대 속도, global layer 순서는 임시 실험 정책의 성격이 있으므로 최종 운영 설정으로 확정하기 전에 위 체크리스트를 반드시 검증해야 한다.
