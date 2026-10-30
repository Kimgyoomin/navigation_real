# DDS SHM·UDP 설정 및 bag 지연 기준값 — 2026-10-29

## 목적과 범위

실험용 송신 공유메모리 용량을 늘리고, 이미 기록된 bag에서 비교 기준값을 구했다.
기존 bag을 재생해 로봇 명령을 보내지 않았다. SQLite를 읽기 전용으로 열어 분석했다.
새 설정은 다음에 시작하는 프로세스에 적용된다. 실행 중인 사용자 노드는 재시작하지 않았다.
시스템 `/dev/shm`, swap, 커널, CUDA, apt 설정은 변경하지 않는다.

용량 확장은 전송 버퍼 여유를 위한 조치다. 과거 16 MiB에서도 재발했으므로
SHM 부족이 모든 지연의 원인이라고 확정하지 않는다. LCM이 이전 명령을 계속 재전송하는
문제, 메인제어기의 수신/명령 선택, 가속도와 경로 추종은 이번 변경과 별개다.

## 설정 구조

| 프로세스 | SHM segment | 설정 위치 |
| --- | ---: | --- |
| Livox driver | 32 MiB | `livox_ros_driver2/config/fastdds_livox_shm.xml` |
| FAST-LIO, global localization, transform fusion, TF, localization RViz | 각각 32 MiB | `navigation_dds/config/fastdds_common.xml` |
| `pcl_ros/pcd_to_pointcloud` (`map_publisher`) | 128 MiB | `navigation_dds/config/fastdds_large.xml` |
| RUBI Nav2 launch의 각 프로세스 | 각각 32 MiB | 공통 프로필 |
| `joy_lcm.launch.py`의 ROS/DDS participant | 32 MiB | 공통 프로필; LCM UDP 자체에는 영향 없음 |
| 별도 rosbag record / RViz | 각각 32 MiB | `with_profile common` 실행 시 |

각 프로필은 `SHM + UDPv4`, `useBuiltinTransports=false`다. 중복 builtin SHM을 만들지
않으며 endpoint QoS, history, timestamp, 센서 데이터, 주기를 재정의하지 않는다.
Fast DDS 2.6/Humble에서 인식하는 `FASTRTPS_DEFAULT_PROFILES_FILE`과
`FASTDDS_DEFAULT_PROFILES_FILE`을 해당 launch 그룹의 자식 환경에만 설정한다.
128 MiB PCD 그룹 종료 뒤에는 32 MiB 환경으로 돌아오며 사용자 셸에는 남지 않는다.

SHM은 터미널당 공용 풀이 아니라 DDS participant의 송신 segment다. 보통 이 실행 구성은
driver 1 + localization 9 + navigation 9 + LCM 1 + recorder 1 프로세스다.
각 프로세스가 participant 하나를 만들 경우 `20 × 32 + 128 = 768 MiB`의 segment 설정 예산이다.
실제 RAM 사용량과 같지는 않으며 DDS history, 메시지 복사본, ICP 지도, RViz, recorder cache 등은 별도다.
추가 CLI/RViz 및 다중 context에 따라 participant 수가 늘 수 있다.
확인 당시 RAM available 약 9.8 GiB, `/dev/shm` 한도 약 7.7 GiB였지만 전체 스택 최대 부하 측정은 아니다.

`map_publisher`는 기존 PCD를 PointCloud2로 시각화하는 노드다. Global localization은
동일 PCD 파일을 직접 읽어 ICP에 사용한다. 새 SLAM 지도를 만드는 노드가 아니다.
수신자는 송신자의 segment를 읽으므로 recorder/RViz도 반드시 128 MiB일 필요는 없다.

## DDS UDP 소켓 버퍼 추가 조정

2026-10-29 21:20경 Goal 없는 실행을 약 1분 관찰했을 때, 안정 구간의 IMU는 200 Hz,
Odometry/localization은 10 Hz, 지연 중앙값은 약 17~18 ms였다. 그러나 DDS discovery
unicast 포트에서 수신 버퍼 폐기가 증가했다. 진단 ROS 구독자를 종료한 뒤에도 10초간
`Udp:RcvbufErrors`가 7,589 증가했으므로 진단 노드 추가 때만 생기는 현상은 아니었다.

당시 `net.core.rmem_max/wmem_max=16777216`이었지만 실제 DDS 소켓의 `SO_RCVBUF`는
212,992 bytes였다. 프로필에서 UDP 크기를 지정하지 않으면 시스템 기본값을 사용한다.
Livox SDK의 센서 수신 소켓은 이와 별개로 16 MiB 요청 상한이 적용되어 있었고,
측정 전후 센서용 소켓의 drops 증가는 0이었다. 이 DDS 폐기를 센서 패킷 손실이나
과거 로봇 명령 freeze의 확정 원인으로 해석하지 않는다.

`fastdds_common.xml`, `fastdds_large.xml`, `fastdds_livox_shm.xml`의 UDPv4 transport에
다음을 추가했다. SHM 크기는 위 표와 동일하다.

```xml
<sendBufferSize>4194304</sendBufferSize>
<receiveBufferSize>4194304</receiveBufferSize>
```

4 MiB는 큰 기본값을 무조건 적용하기보다 현재 208 KiB에서 여유를 늘려 비교하기 위한
초기 시험값이다. 각 UDP 소켓에 대한 요청이며 프로세스 전체의 공용 버퍼가 아니다.
Linux의 `getsockopt(SO_RCVBUF/SO_SNDBUF)` 값은 내부 관리 공간을 포함해 통상 요청의
두 배로 보인다. 실제 큐 메모리 사용량은 소켓 수와 트래픽에 따라 달라진다.
이 XML 변경은 센서 펌웨어·Livox SDK 프로토콜·LCM 라이브러리 자체의 UDP 버퍼를 바꾸지 않는다.

커널 상한은 다음처럼 확인한다. 이 저장소에서는 영구 sysctl 설정을 설치하지 않는다.

```bash
sysctl net.core.rmem_max net.core.wmem_max
# 재부팅 후 기존 실험의 16 MiB 상한을 복원할 경우:
sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216
```

적용은 프로세스 재시작 시점이다. 시작 로그는 요청 크기이며, 실제 적용은 소켓 조회로
확인해야 한다. 재시작 후에는 센서 소켓과 DDS 소켓을 구분해 폐기 증가량을 확인하고
IMU/Odometry의 지연·수신 공백도 함께 비교한다. 실행 중 노드는 이 수정 작업에서 재시작하지 않았다.
큰 버퍼가 계산 성능이나 지속적인 과부하를 해결하는 것은 아니다.

검증: `navigation_dds`와 `livox_ros_driver2` 빌드 성공, 기존 프로필/launch 검사 9개 통과.
실차 domain 13과 별개인 domain 215에서 세 설치 XML을 각각 실제 Fast DDS에 로드했다.
DDS 수신 소켓의 `SO_RCVBUF`, 송신 소켓의 `SO_SNDBUF`가 각각 8,388,608 bytes로
조회되어 4 MiB 요청 적용을 확인했다. 수신 소켓의 송신 버퍼나 송신 소켓의 수신 버퍼,
인터페이스 조회용 미바인딩 소켓은 이 확인 대상이 아니다. 시험 노드는 주행 토픽을 발행하지 않았다.
이는 프로필 로딩·소켓 크기 검증이며, 실차 전체 스택의 폐기 감소와 지연 개선은 재시작 후 확인해야 한다.
변경 전 파일·변경 patch·빌드/시험 로그:
`~/ros2_ws/log/dds_udp_buffers_20261029_5qaat324/`.

## 빌드와 실행

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  navigation_dds livox_ros_driver2 fast_lio_localization pongbot_navigation pongbot_lcm \
  --executor sequential
source install/local_setup.bash
```

새 터미널마다:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

기존 실행 명령을 사용한다. 자동으로 셸의 기존 DDS XML 설정 대신 위 프로필이 선택된다.

```bash
# 터미널 1
LIVOX_DIAG_TIMING=1 taskset -c 0,1 \
  ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 터미널 2 — RViz에서 기존처럼 2D Pose Estimate 사용
FASTLIO_DIAG_TIMING=1 taskset -c 2,3,4,5 \
  ros2 launch fast_lio_localization localization_nav_rubi.launch.py

# 터미널 3
taskset -c 6,7 \
  ros2 launch pongbot_navigation rubi_navigation_dwb.launch.py

# 터미널 4 — 실제 구동 명령을 전달하는 단계이므로 시험 준비 후 사용
cd ~/ros2_ws/src/navigation_real
./run_lcm_start.sh && ros2 launch pongbot_lcm joy_lcm.launch.py
```

시작 로그:

```text
Livox DDS profile: 32 MiB SHM + UDP (driver process only)
FAST-LIO localization DDS profile: 32 MiB SHM + UDP (per DDS participant)
PCD map publisher DDS profile: 128 MiB SHM + UDP (per DDS participant)
RUBI navigation DDS profile: 32 MiB SHM + UDP (per DDS participant)
LCM ROS bridge DDS profile: 32 MiB SHM + UDP (per DDS participant)
```

각 줄 뒤에 `DDS UDP buffer requests: TX/RX 4 MiB each (kernel caps apply)`가 표시된다.

별도 recorder는 wrapper를 사용해야 32 MiB 설정을 받는다. 아래는 지연 비교에 필요한
명시적 토픽 목록이다. 동일 부하의 전후 비교라면 기존 실험과 같은 recorder 토픽 구성도 유지한다.
`/cloud_pcd`와 global costmap은 큰 트래픽을 재현하기 위해 포함했다. 기록 공간을 고려한다.

```bash
ros2 run navigation_dds with_profile common ros2 bag record \
  -o ~/rosbag_record/shm32_128_trial_01 \
  /livox/lidar /livox/imu /livox/lidar_pointcloud /livox/lidar_costmap \
  /Odometry /localization /fastlio/odometry_velocity /cloud_registered \
  /cloud_pcd /tf /tf_static /rosout /cmd_vel \
  /plan /local_plan /local_costmap/costmap /global_costmap/costmap \
  /global_costmap/costmap_raw \
  /local_costmap/published_footprint /global_costmap/published_footprint
```

`with_profile`는 그 뒤에 지정한 명령만 실행하는 환경 wrapper다. 예를 들어 별도 RViz는
`ros2 run navigation_dds with_profile common rviz2`다. 큰 지도를 재발행하는 bag player에는
`large`가 맞을 수 있지만, 실험 bag에는 `/cmd_vel`이 있으므로 실차 domain에 통째로 재생하지 않는다.

비교용으로 각 launch에 `dds_profile:=default`를 붙이면 해당 launch는 상속 환경을 유지한다.
라이브러리 기본 프로필과 비교하려면 셸의 두 XML 환경변수와 작업 디렉터리의 기본 XML 존재도
확인해야 한다. `default`는 16 MiB 복원 스위치가 아니다. `ROS_LOCALHOST_ONLY=1`이면
Humble이 추가 기본 SHM transport를 만들기 때문에 자동 프로필 모드는 명시적으로 거부한다.

## 기존 실험 bag 측정

입력: `~/rosbag_record/rosbag2_2026_10_29-01_50_03`, 약 292.87초.
분석 결과: `~/ros2_ws/log/dds_baseline_20261029/`의 `timing.json`, `timestamps.npz`, `per_second.csv`.
label은 사용자가 지정한 비교 이름이며, bag만으로 모든 프로세스의 당시 DDS XML을 입증할 수는 없다.

지연은 **bag 수신 timestamp − 메시지 관측 timestamp**다. 센서·컴퓨터 시계 정렬을 전제로 하며,
센서 수집, driver, DDS, recorder 스케줄링을 함께 포함한다. 순수 네트워크 시간이나 저장장치
쓰기 소요 시간을 직접 측정한 값은 아니다. `p95`는 관측값의 95%가 이 값 이하임을 뜻한다.

| 토픽/기준 | 수신 Hz | 지연 중앙값 | p95 | 최대 | 최대 수신 간격 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `/livox/imu`, IMU stamp | 199.97 | 0.343 ms | 1.491 ms | 48.694 ms | 65.516 ms |
| `/livox/lidar`, scan 시작 header | 10.000 | 101.846 ms | 104.860 ms | 158.687 ms | 155.681 ms |
| `/livox/lidar`, 마지막 점 시각 | 10.000 | 1.983 ms | 4.842 ms | 58.437 ms | 155.681 ms |
| `/Odometry`, scan 종료 stamp | 10.000 | 16.901 ms | 25.188 ms | 71.112 ms | 156.526 ms |
| `/localization`, scan 종료 stamp | 10.000 | 16.887 ms | 24.783 ms | 71.058 ms | 156.714 ms |

CustomMsg 2,926개의 scan 길이 중앙값은 100.151 ms다. header와 timebase 불일치 0,
마지막 점보다 뒤인 offset을 가진 점 0이었다. 따라서 헤더 기준 102 ms를 그대로
전송 적체라고 해석하면 안 된다. LiDAR/IMU/Odometry/localization stamp 역행·중복은 없었다.
`/livox/lidar_pointcloud`와 `/livox/lidar_costmap`은 약 5 Hz이며 동일 scan-start header를
전달하므로 이들의 약 103~104 ms header age도 scan 수집 시간을 포함한다.

Odometry와 LiDAR scan-end stamp를 1 ms 이내로 대응시킨 2,706쌍의 recorder 수신 시각 차이는
중앙값 14.204 ms, p95 21.667 ms, 최대 66.428 ms였다. 전체 2,926개 중 일부만 매칭되므로
이 값은 대응된 샘플의 통계이며 순수 FAST-LIO 계산 시간이라고 부르지 않는다.

`/rosout` 내부 계측도 확인했다:

- Livox IMU 진단 292개 창: queue depth 최대 1, queue 대기 최대 11.857 ms,
  publish 호출 최대 6.312 ms. queue가 계속 쌓이는 증거는 없었다.
- FAST-LIO IMU 진단 292개 창: 약 1초당 198~204개 수신, callback 간격 최대 51.453 ms.
- FAST-LIO 처리 진단: 약 1초당 sync 성공 9~11개, `lidar_replaced=0`, timer 처리 최대 52.241 ms.
- Fusion 진단: 기록된 창의 callback 총 시간 중앙값 0.140 ms, 최대 4.351 ms.

이 bag에서는 과거의 0.78 Hz / 1.3초 누적 지연은 재현되지 않았다. 기록되지 않은 시기의
문제나 다른 subscriber의 손실까지 없었다는 뜻은 아니다.

### 큰 지도 전송과 짧은 지연의 연관

`/cloud_pcd`는 약 34.53 MiB씩 3초마다 98회 발행되어 serialized payload만 약 3.55 GB다.
발행 header 시각 전후 150 ms에 해당하는 IMU 샘플 5,842개의 age p95는 17.765 ms,
나머지 52,669개는 1.153 ms였다. 10 ms 초과는 각각 459개 / 102개였다.
큰 지도 발행 주변에 지연이 집중되는 **상관관계**는 있지만 SHM 부족, CPU/memory 복사,
recorder 처리 중 어디가 원인인지는 이 통계만으로 구분할 수 없다.

현재 PCD launch에는 `period_ms_`가 있으나 설치된 pcl_ros 2.4.5의 실제 파라미터는
`publishing_period_ms`라 기본 3,000 ms로 동작한다. 이번에는 SHM 효과 비교를 위해 발행
주기·다운샘플링을 바꾸지 않았다. 이후 별도 실험으로 시각화용 지도 부하를 줄일 수 있다.

## 이번 검증과 한계

- 관련 5개 패키지 빌드 성공. 기존 PCL CMP0074 개발자 경고는 남아 있다.
- 새 프로필/launch 회귀 검사 9개 통과: 9개 localization, 9개 navigation, 1개 LCM 노드의
  환경을 실제 launch 트리에서 확인했다. 노드를 실행하는 대신 실행 지점에서 환경만 캡처했다.
  PCD만 128 MiB, 나머지는 32 MiB이며 상위 환경으로 유출되지 않는다. 상속 모드도 검사했다.
- 별도 domain 214, 시험용 UDP loopback 제한으로 transport 확인. 128 MiB 송신자에서
  36,209,289-byte payload 3개를 32 MiB 수신자에게 모두 전달했고 번호/크기를 확인했다.
  송신 segment 매핑 134,246,400 bytes, 수신자 자체 매핑 33,583,104 bytes를 확인했다.
  관리 영역 때문에 설정값보다 조금 크며, 수신자는 원격 128 MiB segment도 매핑한다.
  이는 설정 로딩·대용량 전달 확인이지, 전체 Navigation 동시 부하 성능 시험은 아니다.
  결과는 `~/ros2_ws/log/dds_profile_validation_20261029/`에 있다.
- 기존 navigation 설정 검사에는 `local footprint_padding` 기대값 0.01과 현재 사용자 설정
  0.02의 불일치 1건이 있다. 실제 설정은 이번 SHM 작업으로 바꾸지 않았다. Global은 0.01이다.
- 기존 driver C++ 변경의 CRLF 및 Nav2 YAML 공백 때문에 전체 `git diff --check` 경고가 남는다.
  이번 추가 파일/launch 변경은 별도로 검사한다.
- LCM timeout/중복 송신/메인제어기 적용 명령은 이번 SHM 변경으로 해결되거나 검증된 것이 아니다.

## 다음 bag 비교

```bash
cd ~/ros2_ws/src/navigation_real
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
OPENBLAS_NUM_THREADS=1 python3 tools/analyze_bag_timing.py \
  ~/rosbag_record/shm32_128_trial_01 \
  --output ~/ros2_ws/log/shm32_128_trial_01_analysis \
  --label common32_pcd128
```

출력 디렉터리는 새 경로여야 한다. 분석기는 SQLite bag 분할 파일들을 읽고 ROS graph에
접속하지 않는다. CustomMsg CDR의 offset 추출은 첫 메시지를 ROS 표준 역직렬화와 대조한다.
MCAP은 이 도구에서 지원하지 않는다. 관측 시각의 지연과 수신 간격을 별도로 출력한다.

새 실행에서 같은 recorder 토픽, RViz 표시, CPU affinity를 유지하고 몇 분간 비교한다.
IMU 200 Hz / LiDAR·Odometry·localization 10 Hz 유지 여부, p95/p99/최대 지연,
연속 수신 공백, driver queue 대기와 FAST-LIO IMU callback 공백을 비교한다.
`free -h`, `df -h /dev/shm`, `vmstat 1`로 RAM/SHM와 실제 swap-in/out도 함께 본다.
기존 bag과 새 bag의 장소·움직임·부하가 다르면 차이를 전부 SHM 효과로 해석하지 않는다.

## 참고

- [Fast DDS 2.6 UDP XML 버퍼 설정](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html)
- [Fast DDS 2.6 SHM transport](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)
- [pcl_ros 2.4.5 pcd_to_pointcloud](https://github.com/ros-perception/perception_pcl/blob/2.4.5/pcl_ros/tools/pcd_to_pointcloud.cpp)
