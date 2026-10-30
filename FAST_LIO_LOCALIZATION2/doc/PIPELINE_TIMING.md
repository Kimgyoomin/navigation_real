# FAST-LIO 간헐적 1.3초 지연 계측 (2026-10-28)

> 최신 설정: 일반 DDS participant 32 MiB, PCD publisher만 128 MiB로 확장했다.
> 기존 bag 기준값과 새 설정 검증은 [DDS_SHM_VALIDATION.md](../../docs/DDS_SHM_VALIDATION.md)를 참고한다.
> 아래 16 MiB 기록은 이전 실험 이력이다.
>
> 2026-10-29 추가 확인: **driver SHM 16 MiB 적용 후에도 재발했다.** 아래의
> 정상 관측은 해당 실행 구간의 결과이며 완치 판정이 아니다. 문서 끝의
> 「16 MiB 적용 후 재발 및 메인제어기 Ethernet 링크 이상」을 함께 참고한다.

대상은 `fast_lio_localization`의 `fastlio_mapping`이다. 단독 `fast_lio`에는
이 추가 계측을 넣지 않았다. 추정식, IMU/LiDAR 동기화 조건, 타임스탬프, 큐 폐기
정책, DDS QoS, executor 구성, Navigation 설정은 변경하지 않았다.
기존 LiDAR 진단 출력은 센서 버퍼 mutex를 해제한 뒤 출력하도록 옮겼다.

## 계측 이유

첫 실행 로그 `ros2_ws/log/fastlio_diag_ydUhn7.log`에서 Navigation 시작 전부터
fusion 입력 간격 약 1,278 ms, 입력 age 약 1,300 ms가 관측됐다. 같은 구간에서
LiDAR 콜백의 scan-end age는 약 3–6 ms, mapping 처리시간은 약 8–11 ms였다.
기존 mapping 시간은 `sync_packages()` 이후부터 측정하므로, 이 값만으로 처리
타이머의 실행 지연과 IMU 동기화 대기를 구별할 수 없다. 두 번째 실행은 정상으로
회복됐지만 재발 원인이 해결됐다는 근거는 아니다.

## 실행

빌드가 끝나도 이미 실행 중인 프로세스에는 적용되지 않는다. 사용자가 기존
Navigation과 Localization을 종료한 다음 Localization부터 다시 실행한다.
Livox driver는 유지할 수 있다. 이 계측에는 goal이나 LCM 실행이 필요하지 않다.

```bash
cd /home/rclab/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

fastlio_diag_log=$(mktemp /home/rclab/ros2_ws/log/fastlio_diag_XXXXXX.log)
echo "$fastlio_diag_log"
FASTLIO_DIAG_TIMING=1 taskset -c 2,3,4,5 \
  ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  2>&1 | tee "$fastlio_diag_log"
```

시작 시 `FAST-LIO pipeline diagnostics v1 enabled`를 확인한다. 초기 위치를 지정한
뒤 약 30초 기다리고, 같은 ROS 환경을 설정한 다른 터미널에서 Navigation을 실행한다.

```bash
taskset -c 6,7 ros2 launch pongbot_navigation rubi_navigation_dwb.launch.py
```

첫 비교에서는 goal과 추가 `ros2 topic hz/delay/echo` 구독 없이 로그를 남긴다.
정상/비정상 상태가 바뀌면 바로 재시작하지 말고 해당 로그를 보존한다. 주행 중의
controller 계산량은 goal 없는 이 시험으로 검증되지 않는다.

## 로그 해석

기존 `FAST-LIO timing`, `mapping timing`, `fusion timing`에 다음 두 줄이 추가된다.
진단을 활성화했을 때만 처리 타이머에서 약 1초에 한 번 출력하며, 타이머가 멈추면
다음 실행 때 실제 `window_s` 길이로 보고한다. 별도 ROS 구독자는 만들지 않는다.
센서당 누적 카운터와 최대값/합계만 저장하므로 메시지 저장량은 늘어나지 않는다.

| 필드 | 의미 |
| --- | --- |
| `timer_n / window_s` | 처리 타이머 호출 빈도. 설정은 100 Hz이며 Odometry 목표 10 Hz와 다름 |
| `timer_gap_max_ms` | 타이머 진입 사이 최대 간격. 보통 약 10 ms; 실제 계산시간/스케줄링 등에 따라 증가 |
| `timer_work_max_ms` | 완료한 타이머 콜백 전체 최대 시간. 동기화·진단 출력·조기 반환도 포함 |
| `sync_ok` | 동기화 성공 횟수. 초기화/빈 점군으로 Odometry 발행이 생략될 수 있음 |
| `empty_lidar`, `empty_imu` | 해당 버퍼 부족으로 동기화가 실패한 호출 수. 둘 다 비면 `empty_lidar`로 집계 |
| `imu_behind` | 마지막 수신 IMU 시각이 선택된 LiDAR scan-end보다 이전이라 기다린 호출 수 |
| `imu_shortfall_max_ms` | 그 대기 판단 시 필요한 IMU 시간 차이의 최대값. 대기가 없으면 nan |
| `sync_lock_max_ms` | 동기화 함수가 센서 버퍼 mutex를 얻기까지 걸린 최대 시간 |
| `sync_work_max_ms` | mutex 대기까지 포함한 동기화 함수 전체 최대 시간 |
| `ready_age_max_ms` | 동기화 성공 시점의 scan-end age 최대값. 계산에 들어가기 전 이미 오래됐는지 확인 |
| `lidar_n`, `lidar_replaced` | LiDAR 콜백 수, 기존 2프레임 제한으로 교체한 프레임 수 |
| `lidar_buffer`, `imu_buffer`, `pinned` | 보고 시점의 버퍼 길이와 선택된 대기 스캔 유무 |
| `pending_end_age_ms` | 현재 대기 스캔의 scan-end age. 대기 스캔이 없으면 nan |
| `latest_imu_age_ms`, `imu_shortfall_ms` | 보고 시점 마지막 IMU의 age, 대기 스캔 끝과 IMU의 시간 차이(양수면 부족) |
| `since_lidar_cb_ms`, `since_imu_cb_ms` | 마지막 센서 콜백 진입 이후의 경과시간 |
| `imu_n / window_s` | FAST-LIO 내부 IMU 콜백 수신 빈도. 센서 설정이 200 Hz라면 약 200 기대 |
| `gap_max_ms`, `stamp_step_max_ms` | IMU 콜백 사이 최대 간격, 입력 stamp 사이 최대 간격 |
| `raw_age_mean_ms / max_ms` | IMU 콜백 진입 시 원본 stamp 기준 age의 평균/최대 |
| `corrected_age_max_ms` | 기존 IMU 시간 보정 후 stamp 기준 최대 age |
| `lock_max_ms` | IMU 콜백의 센서 버퍼 mutex 획득 대기 최대 시간 |

예를 들어 IMU 콜백 gap은 약 1.3초인데 stamp는 5 ms씩 진행하고 raw age도 크면
FAST-LIO가 오래된 IMU를 받는 구간이 있다. 타이머 호출은 유지되면서 `imu_behind`와
시간 부족량이 커지면 동기화 대기다. 반대로 IMU는 신선한데 타이머 gap만 크면
타이머 실행/스케줄링 쪽을 확인한다. `timer_work_max_ms`도 크면 콜백 내부 대기나
작업 지연부터 봐야 한다. 어느 경우든 이 수치만으로 특정 DDS 버그라고 단정하지 않는다.

새 age 값은 노드의 ROS clock, 간격/소요시간은 steady clock을 사용한다.
ROS 시간 점프·잘못된 장비 시각은 age에 영향을 주며, 기존 LiDAR 콜백 로그는 여전히
system clock 기준이므로 bag 재생에서는 새 age와 직접 비교하지 않는다.
음수 age는 숨기지 않고 유지한다. 표본이 없는 통계는 0 대신 `nan`이다.
보고 시점 스냅샷과 기간 최대값은 구분해서 읽어야 한다. 완료 콜백 시간은 다음
보고에 반영되므로 현재 콜백의 시간은 다음 줄에서 확인한다. 계측 자체가 타이밍에
영향을 줄 수 있으므로, 정상으로 돌아왔다는 사실만으로 수정 효과를 판정하지 않는다.

## 검증 명령

```bash
cd /home/rclab/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
MAKEFLAGS="-j2 -l2" CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
  --packages-select fast_lio_localization --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select fast_lio_localization --executor sequential
colcon test-result --test-result-base build/fast_lio_localization --verbose
```

추가 테스트는 보고 경계를 가로지르는 1.28초 수신 정체, 정체 후 신선한 표본이
들어와도 최대 지연값이 남는지, 시간 보정과 수신 지연의 구분, 음수 age 보존을 확인한다.
실제 재발 위치와 주행 성능은 재시작 후 로그로 별도 확인한다.

## Livox IMU 발행 경로 계측 추가 (2026-10-28 밤)

실행 중 수동 패킷 관측에서 센서 `192.168.1.145` → Jetson `56401/UDP`는
5초간 999개(약 199.8 Hz)였고, 해당 UDP 수신 소켓의 누적 드롭은 0이었다.
같은 지연 구간에 FAST-LIO는 IMU를 수십 Hz, 원본 stamp age 약 1,275 ms로
받았다. 23:09:38경 프로세스 재시작 없이 약 200 Hz, 평균 age 약 0.45 ms로
회복했다. 이는 원인 해결을 뜻하지 않는다. 드라이버 큐 메모리 스냅샷은 회복
시점과 가까워 지연 구간의 큐 적체를 배제하는 증거로 사용하지 않는다.

`LIVOX_DIAG_TIMING=1`이면 드라이버의 별도 ROS 타이머가 초당 한 번
`Livox IMU diag`를 출력한다. SDK 수신·발행 스레드 안에서 로그를 출력하지
않으며, 발행 함수가 아직 반환하지 않았더라도 타이머가 실행되면 그 경과시간을
볼 수 있다. ROS 타이머 자체가 스케줄되지 않는 경우에는 로그도 늦어질 수 있다.
누적 통계는 크기가 고정되어 있고 IMU 큐의 기존 mutex로 보호한다.
큐 순서·보관 정책·DDS QoS·축 회전·센서 stamp·ROS 메시지 필드는 그대로다.

Navigation을 종료하고 Localization, Driver를 종료한 뒤, 각각 새 터미널에서
공통 환경을 설정한다. 이 명령들은 자동 주행/LCM을 실행하지 않는다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Driver 터미널:

```bash
LIVOX_DIAG_TIMING=1 taskset -c 0,1 \
  ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Localization 터미널:

```bash
FASTLIO_DIAG_TIMING=1 taskset -c 2,3,4,5 \
  ros2 launch fast_lio_localization localization_nav_rubi.launch.py
```

정지 상태에서 초기 자세를 지정한 후 Navigation을 실행하고 두 로그를 함께
비교한다. 먼저 Goal 없이 확인한다. 짧은 정상 구간만으로 해결을 판정하지 않는다.
이전 재발은 Localization 시작 후 약 12분 뒤에도 나타났다.

| Driver 값 | 의미 |
|---|---|
| `received / window_s` | SDK 콜백에서 드라이버 IMU 큐까지 도달한 빈도; 센서 설정 200 Hz와 비교 |
| `ingress_gap_max_ms`, `stamp_step_max_ms` | 큐 진입 사이 간격 / IMU 원본 stamp 간격의 최대값 |
| `ingress_age_mean_ms`, `ingress_age_max_ms` | 드라이버 큐 진입 시 원본 stamp의 age; system clock 기준 |
| `popped`, `published` | 큐에서 꺼낸 수 / ROS publish 호출이 정상 반환한 수; subscriber 수신 확인은 아님 |
| `queue_depth`, `queue_peak` | 보고 순간 큐 크기 / 보고 구간 최대 크기 |
| `queue_wait_max_ms`, `oldest_wait_ms` | 꺼낸 IMU의 최대 대기 / 현재 큐의 가장 오래된 IMU가 기다린 시간 |
| `publish_age_max_ms` | ROS publish 직전 원본 stamp age의 구간 최대값 |
| `publish_work_max_ms` | 반환 완료된 publish 호출의 최대 소요시간 |
| `publish_inflight_ms` | 아직 반환하지 않은 publish 호출의 현재 경과시간; 없으면 `nan` |
| `since_ingress_ms`, `since_publish_ms` | 마지막 큐 진입 / publish 반환 이후 경과시간 |

- `received`부터 낮으면 센서·SDK 수신 구간을 확인한다. `time_type=0`에서는
  SDK 콜백에서 호스트 시각을 부여하므로 낮은 ingress age만으로 수신 이전 지연을
  배제할 수 없다. 필요하면 실제 UDP 패킷 수신과 대조한다.
- `received`는 200 Hz인데 큐 크기·대기와 `publish_inflight_ms`가 증가하면
  드라이버의 publish/통신 계층 호출이 밀리는지 확인한다.
- Driver `published`가 200 Hz이고 `publish_age_max_ms`가 작지만 FAST-LIO의
  `raw_age`만 약 1.3초라면 publish 이후 DDS 전달·수신/콜백 스케줄링을 확인한다.
- 내부 IMU FIFO는 상한 없는 list다. ROS publisher history 깊이 256과는 별개다.
  256 / 200 = 1.28초라는 수치 일치는 단서이며, 깊이 자체가 지연의 증거는 아니다.

신선한 단일 표본으로 정상 판정하지 말고 구간 최대값과 양쪽 수신·발행 수를 함께
본다. 이 계측 추가는 원인 구분을 위한 것이며 지연 해결이나 주행 승인으로 간주하지 않는다.

23:28:05~23:28:10 재발 중에도 기존 드라이버 프로세스를 중단하지 않고 IMU FIFO를
읽었다. 일관성 검사를 통과한 969개 스냅샷 중 깊이 0이 965개, 1이 3개, 2가
1개였고, 관측된 가장 오래된 큐 데이터의 age 최대는 4.86 ms였다. 같은 구간
FAST-LIO IMU age는 약 1,275 ms였다. 따라서 그 구간의 1.3초 지연은 드라이버
FIFO 적체로 설명되지 않는다. 스냅샷은 모든 순간을 포착하지 못하며 DDS 버그와
수신 executor 문제를 구분하지는 못한다.

### DDS UDP 우회 후보 검증 결과 — 실행 설정에 채택하지 않음

실제 센서와 분리한 ROS domain 167에서 UDP 전송 publisher와 기본 설정 subscriber를
비교했다. RELIABLE IMU 200 Hz, 480 kB 점군 10 Hz와 5 Hz를 동시에 보냈다.
워밍업 후 약 20초 수신 결과 IMU 3,995개(199.7 Hz), age 중앙값 0.94 ms,
95백분위 1.92 ms였으나 점군은 각각 185개와 85개로 목표 수신량에 미달했다.
검사한 메시지의 payload 오류는 없었지만, 이 결과로 전체 센서 경로의 정상 동작을
승인할 수 없다. 따라서 시험용 UDP launch 옵션과 XML은 최종 변경에서 제거했고,
이 단계에서는 기존 DDS 전송 설정을 유지했다. 이후의 SHM 크기 비교는 아래와 같다.

참고로 `ROS_LOCALHOST_ONLY=1`은 Humble `rmw_fastrtps`에서 SHM transport를
추가하므로 UDP-only 시험과 충돌한다. 해당 조건으로 실행한 최초 시험은
UDP-only 검증으로 사용할 수 없어 제외하고, 위 결과는 이 변수를 0으로 한 시험이다.
시스템 socket buffer/sysctl 설정과 실제 로봇의 ROS domain은 변경하지 않았다.

참고 구현:
- https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/udp/udp.html
- https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/participant.cpp

## 재발 확인과 SHM 크기 수정 (2026-10-29)

사용자가 진단 버전을 다시 실행한 뒤 Navigation을 Goal 없이 유지했다.
약 4분 후 지연이 재발했다. 첫 재발 구간 `1793199236`~`1793199291` 초에서
드라이버는 매초 200개를 수신하고 발행했으나 FAST-LIO IMU 평균 age는 대부분
약 1,274 ms였다. `1793199236`~`1793199300` 구간 드라이버 publish 직전 age 최대는
15.452 ms, publish 호출 소요시간 최대는 13.316 ms, 보고 순간 내부 큐 깊이는 0이었다.
FAST-LIO 센서 버퍼 mutex 대기도 대부분 0.001 ms였다. 따라서 이 재발은 센서나
드라이버 FIFO에 1.3초치가 쌓이는 현상으로 설명되지 않는다.

이후 재발 구간 `1793200044`~`1793200060` 초에는 독립 IMU 전용 구독자를 비교했다.
RELIABLE(depth 10)과 BEST_EFFORT(depth 10) 구독자는 모두 약 200 Hz, 평균 age
약 1~2 ms로 받는 동안, FAST-LIO는 여전히 대체로 1.27초 오래된 IMU를 받았다.
새 구독자의 추가가 시스템에 영향을 줄 수 있으므로 이것만으로 executor 문제를
확정하지 않는다. 특히 아래 합성 시험은 FAST-LIO 또는 MultiThreadedExecutor 없이도
동일한 약 1.28초 지연을 재현했다.

### 크기 근거와 분리 시험

`LivoxPointXyzrtlt`는 packed 26 bytes다. 20,064점의 PointCloud2 data 배열만
521,664 bytes이며 ROS/CDR/RTPS 부가 데이터는 별도다. Fast DDS 기본 SHM segment는
524,288 bytes(512 KiB)다. 같은 참가자의 점군 두 스트림과 IMU가 이를 공유한다.
실행 중 프로세스의 SHM 매핑도 기본 크기와 일치했다. 공식 문서는 segment 크기가
송신 데이터 크기에 가까운 경우 데이터 손실 위험을 명시한다.

센서와 분리된 ROS domain 167에서 RELIABLE IMU 200 Hz와 521,664-byte PointCloud2
두 스트림(10 Hz, 5 Hz)을 송신했다. IMU reader는 RELIABLE depth 10,
점군 reader는 BEST_EFFORT였다. 합성 점군 writer는 BEST_EFFORT이므로 실제
CustomMsg writer와 모든 설정이 같은 시험은 아니다. subscriber는 단일 스레드
Python executor를 사용했다. 첫 5초를 제외한 관측 결과:

| 송신 SHM 설정 | 관측 길이 | IMU 수신율 | IMU 최대 age | 점군 10/5 Hz 수신 개수 |
|---|---:|---:|---:|---:|
| 16 MiB, 첫 비교 | 약 20초 | 199.85 Hz | 6.07 ms | 200 / 100 |
| 기본 512 KiB | 약 20초 | 160.26 Hz | 1,282.27 ms | 197 / 100 |
| 최종 16 MiB 프로필 | 약 55초 | 199.67 Hz | 16.10 ms | 550 / 275 |

최종 프로필에서는 IMU age 중앙값 0.995 ms, 95백분위 2.547 ms였으며 검사한
payload 오류는 0이었다. 60초 전체 실행에서 실제 송신 프로세스의 약 16 MiB SHM
매핑도 확인했다. 이는 설정의 유효성과 재현 부하에서의 개선 근거이며, 실물 장기
운전이나 모든 드롭의 부재를 보장하는 시험은 아니다. 1.28초와 IMU writer history
256/200초의 수치 일치가 내부 재전송 원리를 증명하는 것은 아니다.

원본 로그:
- `~/.ros/log/livox_ros_driver2_node_94912_1793198901873.log`
- `~/.ros/log/fastlio_mapping_95228_1793198939077.log`
- `~/.ros/log/transform_fusion_cpp_95232_1793198938949.log`

### 최종 적용 범위와 다음 검증

- `livox_ros_driver2/config/fastdds_livox_shm.xml`: driver SHM 16 MiB + UDP.
- `rviz_MID360_launch.py`: 기본 `dds_profile:=livox`; 해당 driver 자식 프로세스에만
  XML 환경변수를 적용한다. 기본 SHM 중복 생성 방지를 위해 builtin transports를
  끄고 SHM과 UDP를 명시한다. IMU와 점군 QoS, timestamp와 축 회전은 유지한다.
- SHM 설정 크기 증가는 driver 참가자 기준 약 15.5 MiB다. 점 수, 알고리즘 계산량,
  다른 노드의 DDS 설정과 시스템 sysctl은 바꾸지 않는다. 실제 RSS 증가는 메모리
  접근/공유 상태에 따라 달라지므로 이 수치를 실제 RSS 측정값으로 해석하지 않는다.
- `ROS_LOCALHOST_ONLY=1`이면 Humble이 별도 기본 SHM을 추가하므로 launch에서
  중단한다. 현재 실행 환경은 0이다. 별도 XML 환경설정을 유지하거나 비교 복원하려면
  `dds_profile:=default`를 지정한다.

빌드와 설치된 launch/XML 검증을 완료했다. 실제 센서는 사용자가 Navigation,
Localization, Driver를 종료한 뒤 기존 순서로 재실행해야 한다. Driver 시작에
`Livox DDS profile: 16 MiB SHM + UDP (driver process only)`와 기존 IMU 진단
marker를 확인한다. 먼저 Goal 없이 양쪽 IMU 200 Hz, fusion 입력 약 100 ms,
age 수십 ms를 유지하는지 비교한다. 이후 Goal 상태에서 별도로 검증한다.
현재 실행 중인 노드의 종료·goal 지정·LCM 명령 전송은 이 작업에서 수행하지 않았다.

근거:
[Fast DDS 2.6.x Shared Memory Transport](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html).

### 실물 센서 재시작 후 Goal 없는 Navigation 검증 (2026-10-29)

사용자가 Livox, Localization, Navigation을 재실행했다. driver PID 116361의
환경변수에서 위 XML 적용을 확인했고, `/proc/116361/maps`에서 16,805,888-byte
SHM 매핑을 확인했다(설정 용량에 관리 영역이 추가된 매핑). 다른 참가자의 작은
SHM도 함께 매핑되어 있으므로 작은 매핑의 존재만으로 설정 실패를 판단하지 않는다.

Navigation 활성화 시각 `1793201074.2664164`부터 약 360초 동안 기존 진단 로그를
읽어 관측했다. 추가 ROS 구독자, rosbag recorder, Goal, 주행 명령은 사용하지 않았다.
드라이버 355개, FAST-LIO 356개 진단 구간과 fusion 174개 로그 표본의 결과:

| 측정 항목 | 결과 |
|---|---:|
| Driver IMU 발행율(구간 개수/구간 시간 합계) | 200.005 Hz |
| FAST-LIO IMU 콜백 수신율 | 200.004 Hz |
| IMU 구간별 평균 raw age의 중앙값 / 최대 | 0.469 / 1.027 ms |
| IMU 개별 raw age의 구간 최대값 중 최대 | 30.194 ms |
| Driver IMU 큐 대기 최대 / publish 호출 최대 | 10.927 / 3.283 ms |
| FAST-LIO 동기화 성공률 (`sync_ok`) | 10.000 Hz |
| FAST-LIO LiDAR 버퍼 교체 (`lidar_replaced`) | 관측 구간 모두 0 |
| fusion 입력 간격 표본 중앙값 / 최대 | 100.75 / 114.5 ms |
| fusion 입력 Odometry age 표본 중앙값 / 최대 | 16.3 / 31.6 ms |

`sync_ok`는 처리율이고, fusion 수치는 약 2초마다 출력되는 진단 표본이다.
독립 `/localization` 구독자의 모든 메시지 지연을 측정한 결과와 구별한다.
로그 flush 시점 때문에 360초의 마지막 수 초는 집계 시 아직 출력되지 않았을 수 있다.
활성화 후 약 8분 시점에도 최신 IMU 진단은 200 Hz, fusion 입력 간격 100.2 ms,
입력 age 13.7 ms였다. 이때까지 확인한 driver, FAST-LIO, fusion, sector filter,
planner/controller server 및 Navigation lifecycle manager 로그에 새 WARN/ERROR,
TF Message Filter drop, Goal 수신 기록은 없었다.

별도 30초 `/proc` 표본에서 driver CPU는 약 11.4%, FAST-LIO는 약 19.3%였다.
여기서 100%는 CPU 코어 하나이며, Goal 없는 상태의 측정이다. 전체 시스템 부하나
Goal 처리 중의 controller 연산 여유를 이 수치만으로 판단하지 않는다.

이전에는 Goal 없이도 약 4분 후 1.27초 IMU 지연이 재발했지만, 이번 관측에서는
재발하지 않았다. 이는 SHM 크기 변경의 실물 개선 근거다. 장시간 실행, 실제 Goal
처리, rosbag 기록 부하까지 해결됐다는 결론은 아직 내리지 않는다. 다음 비교에서는
동일한 설정을 유지하고 사용자가 Goal과 기록 부하를 추가했을 때의 지연을 확인한다.

원본 로그:
- `~/.ros/log/livox_ros_driver2_node_116361_1793200984760.log`
- `~/.ros/log/fastlio_mapping_116819_1793201041639.log`
- `~/.ros/log/transform_fusion_cpp_116823_1793201041542.log`
- `~/.ros/log/planner_server_117206_1793201071040.log`
- `~/.ros/log/controller_server_117216_1793201071100.log`

이번 관측에서 실행 중인 노드나 코드/설정을 변경하지 않았다.

### Goal 처리 중 추가 검증 (2026-10-29)

같은 프로세스와 DDS 설정을 유지한 채 사용자가 Goal을 지정했다.
BT는 `1793201798.814512488`, controller는 `1793201799.220219514`에 Goal 처리를
시작했다. A*는 경로를 생성했고 DWB가 명령을 발행했다. Goal 이후 약 276초의
진단 로그에서 IMU 발행/수신은 모두 200.004 Hz, `sync_ok`는 10.003 Hz였다.
IMU 개별 raw age의 관측 최대는 27.632 ms, fusion 입력 age 표본 최대는 28.4 ms,
LiDAR 버퍼 교체는 0이었다. TF 오류와 `Control loop missed`는 관측되지 않았다.
Goal 시작 시 유일한 goal_checker를 자동 선택한다는 일회성 경고는 있었다.

이 구간 중 읽기 전용 BEST_EFFORT depth 5 구독자를 별도로 실행해 네 토픽을
측정했다. 전체 70초 중 최초 5초를 제외한 약 65초의 결과다. 이 수치는 수신자
기준이며, 새로운 구독자가 없는 앞선 관측과 측정 조건을 구별한다. rosbag은 없다.

| 토픽 | 수신율 | 수신 간격 최대 | header age 중앙값 / 최대 |
|---|---:|---:|---:|
| `/Odometry` | 10.000 Hz | 109.4 ms | 17.4 / 35.6 ms |
| `/localization` | 9.999 Hz | 109.1 ms | 18.1 / 33.2 ms |
| `/cmd_vel` | 5.000 Hz | 220.4 ms | header 없는 Twist |
| `/local_costmap/costmap` | 3.210 Hz | 403.2 ms | 1.33 / 5.13 ms |

`/cmd_vel` 325개 중 0 명령은 없었다. 출력 선속도 x 범위는 0.477~0.750 m/s였으며
이는 명령값이지 실제 로봇 이동속도 검증 결과가 아니다. 이번 측정은 Goal 계산과
전송 부하를 확인하며, 실제 주행 성능·속도 추종 검증을 대신하지 않는다.

local costmap 토픽이 설정 5 Hz보다 낮아 추가로 약 20초를 비교했다.
`/local_costmap/published_footprint`는 5.000 Hz(최대 간격 202.5 ms),
`/local_costmap/costmap`은 3.214 Hz(최대 간격 402.3 ms)였다.
설치된 nav2_costmap_2d 버전은 1.1.20이다. 해당 upstream의 `updateMap()`은
`layered_costmap_->updateMap()` 이후 footprint를 발행하므로 footprint 수신은
갱신 루프 완료의 간접 지표다. 지도 갱신 루프가 약 5 Hz로 완료되는 동안 표시용
costmap 토픽 발행만 일부 주기를 건너뛰는 해석과 일치한다. 개별 센서 관측이
매번 적용되었다는 보장은 이 지표만으로 할 수 없다.

같은 버전의 `mapUpdateLoop()`는 갱신 루프 안에서
`last_publish_ + publish_cycle_ < current_time`을 검사한 후에만 costmap을 발행하고
`last_publish_ = current_time`으로 기록한다. 갱신/발행 모두 5 Hz일 때 미세한
시간 차이로 다음 주기까지 발행이 미뤄질 수 있다. 이번 200/400 ms 수신 간격과
부합하며, 이를 IMU 1.3초 지연의 재발이나 costmap 내부 갱신 정지로 판단하지 않는다.
관측 중 주파수나 설정은 변경하지 않았다.

근거 코드:
[Nav2 1.1.20 Costmap2DROS](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_costmap_2d/src/costmap_2d_ros.cpp).

70초 CPU 표본(100%=코어 하나)은 driver 11.4%, FAST-LIO 18.3%, planner server
29.5%, controller server 15.1%였다. 이는 해당 프로세스 평균이며 전체 CPU/GPU
부하, 순간 정체, rosbag 기록 부하를 모두 검증한 것은 아니다. 진단 결과 및
수신 표본은 `/tmp/probe_nav_goal_261029_result.json`,
`/tmp/probe_nav_goal_261029_samples.json`, `/tmp/probe_costmap_cycle_261029_result.json`,
`/tmp/shm16_goal_diag_261029.json`에 보관했다(임시 파일이므로 영구 보존 경로는 아님).

## 16 MiB 적용 후 재발 및 메인제어기 Ethernet 링크 이상 (2026-10-29)

사용자가 화살표/위치 갱신 지연을 보고한 실행에서 driver PID 35224의 XML
환경변수와 16,805,888-byte SHM 매핑을 확인했다. 따라서 이 재발은 단순히
이전 512 KiB 설정으로 실행한 결과가 아니다. 기존 진단 로그를 집계한 결과:

| 항목 | 지연 실행: epoch 1793204940~1793205190 | 재시작 후 Goal 실행: 1793205655~1793205856 |
|---|---:|---:|
| Driver IMU 발행 | 200.002 Hz | 200.005 Hz |
| FAST-LIO IMU 콜백 수신 | 25.095 Hz | 200.002 Hz |
| IMU 구간 평균 age 중앙값 | 1273.909 ms | 0.437 ms |
| 동기화 성공 (`sync_ok`) | 0.750 Hz | 9.997 Hz |
| LiDAR 버퍼 교체 합계 | 2286 | 0 |
| fusion 입력 간격 표본 중앙값 | 1298.3 ms | 100.7 ms |
| fusion 입력 age 표본 중앙값 | 1368.8 ms | 15.9 ms |
| Driver publish 진입 age 최대 | 24.470 ms | 7.819 ms |

드라이버 발행 이후 FAST-LIO IMU 콜백까지 전달되는 구간에서 지연이 나타났다.
이로 인해 FAST-LIO가 스캔 끝 시각까지의 IMU를 기다리고 LiDAR 프레임을
건너뛰는 현상이 확인됐다. DDS 전송/재전송과 수신 executor 중 어느 내부
기전이 원인인지는 이 계측만으로 확정하지 않는다. timestamp를 현재 시각으로
덮어쓰거나 동기화 대기 조건을 없애는 수정은 하지 않았다.

사용자가 동일 명령으로 재시작했다고 확인했다. 조사 중에도 프로세스가 다시
바뀌었으므로 서로 다른 PID의 결과를 동일 실행으로 합치지 않았다. 23초짜리
독립 ROS 구독 시험은 재시작 후 정상 실행에서 수행됐으며, 장애 실행 중 다른
구독자가 정상이었다는 증거로 사용할 수 없다.

별도로 메인제어기 Ethernet `enP7p1s0`에 아래 이상이 있었다.

- 커널 `lan743x` 로그에서 물리 `Link is Down` / `Link is Up - 1Gbps/Full` 반복.
- 01:45 KST 표본: `NO-CARRIER`, carrier 변경 누계 266회(Up 133 / Down 133),
  RX FCS 오류 누계 951개. 누적값이며 전부 이번 관측 구간에 발생한 것은 아니다.
- 사용자는 케이블/메인제어기 전원을 직접 조작하지 않았다고 답했다.
- 링크가 내려간 표본에서 `ip route get 239.255.76.67`이 `wlP1p1s0`와
  Wi-Fi 주소를 반환했다. 이전의 `multicast route ready` 출력은 이후의
  링크 유지나 상대 수신까지 보장하지 않는다.
- Livox 포트 `enP8p1s0`는 별개다. 읽은 NIC 통계에서 rx_errors는 0이었다.

같은 Jetson의 DDS endpoint GID 첫 4바이트도 driver/FAST-LIO는 `010f5939`,
Nav2는 `010f0106`으로 달랐다. 각 그룹의 SHM 매핑도 분리되어 있었다.
설치된 Fast DDS 2.6.11은 IPv4 주소 목록으로 host ID를 만들고 GUID 첫 4바이트로
동일 호스트를 판별한다. 시작 시점의 인터페이스 변화가 이 차이를 만든다는
설명과 부합하지만, 링크 이상이 IMU 1.27초 지연의 단독 원인임을 입증한 것은 아니다.
실제로 링크가 흔들려도 IMU/FAST-LIO가 정상인 구간도 있었다.

근거 코드:
[Host.hpp](https://github.com/eProsima/Fast-DDS/blob/v2.6.11/src/cpp/utils/Host.hpp),
[SystemInfo.hpp](https://github.com/eProsima/Fast-DDS/blob/v2.6.11/src/cpp/utils/SystemInfo.hpp),
[GuidUtils.hpp](https://github.com/eProsima/Fast-DDS/blob/v2.6.11/src/cpp/rtps/common/GuidUtils.hpp).

LCM 송신 프로세스도 한때 두 개였다. 이전 PID 3354는 `/tmp/fastdds_profile.xml`
(UDP 및 192.168.100.5 인터페이스 제한)을 상속했고 새 프로세스는 기본 DDS
설정이었다. ROS graph에 하나만 보여도 OS 프로세스가 하나라는 보장은 없다.
이것 역시 IMU 지연의 원인으로 확정하지 않으며, 실험 전 중복 송신을 정리할
별도 항목이다. 조사에서 사용자 노드 종료나 주행 명령 전송은 하지 않았다.

`run_lcm_start.sh`의 단순 `grep UP` 검사는 `NO-CARRIER,...,UP`도 통과했다.
이를 정확한 admin-UP 플래그와 sysfs carrier 검사로 바꾸고, route 설정 후에도
선택된 장치와 carrier를 다시 검사하도록 보강했다. 준비 문구는 링크/경로
검사 결과일 뿐 상대 수신 확인이 아님을 명시했다. 시스템 라우트는 이 작업에서
변경하지 않았다. bash 문법 검사, 모의 no-carrier/admin-down 거부 확인을 통과했다.

다음 비교는 케이블/상대 포트의 링크가 안정된 상태에서 시작하고, 모든 ROS
프로세스의 환경을 일치시킨 뒤 Goal 없는 IMU 수신율부터 확인한다. 현재 증거만으로
추정식·IMU 시간 보정·QoS를 더 바꾸거나 SHM 크기만 계속 늘리는 것은 근거가 부족하다.

원본 로그 사본, 링크 로그, NIC 표본, 집계 JSON은 다음 로컬 폴더에 보존했다:
`/home/rclab/ros2_ws/log/imu_recurrence_20261029_d2lwcuym/`.
이 폴더는 Git 저장소 밖에 있으며 위의 수치는 `summary.json`에서 확인할 수 있다.
