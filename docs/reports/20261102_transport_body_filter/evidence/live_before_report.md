# Goal 없이 실행 중인 Navigation 점검 — 2026-10-29

## 범위와 측정 방법

Livox, FAST-LIO localization, Nav2가 실행 중인 상태를 읽기 전용으로 점검했다. Goal·속도 명령·LCM 패킷을 보내거나 노드를 재시작하지 않았다. 코드 및 시스템 설정 변경 없음. 로그 폴더에 진단 결과만 저장했다.

ROS 토픽 측정: 21:20:29~21:21:29 KST 약 60초. BEST_EFFORT 구독자의 수신 시각−header.stamp를 사용하므로 구독자 스케줄링도 포함한다. 첫 CustomMsg의 전체 역직렬화 검증이 진단 프로세스를 약 200ms 지연시켰다. 모든 토픽의 최대 지연이 21:20:30.24에 동시에 발생했고, 해당 값을 실제 생산 노드의 병목으로 해석하지 않는다. 아래 통계는 각 토픽 측정 시작 2초 이후 약 58초에 대한 값이다. 원본 전체 측정은 timing.json/timestamps.npz에 보존했다.

| 토픽 | 수신 Hz | 지연 중앙값 ms | p95 ms | 최대 ms |
|---|---:|---:|---:|---:|
| /livox/imu | 200.002 | 0.576 | 1.839 | 29.630 |
| /livox/lidar | 10.000 | 102.044 | 104.461 | 126.155 |
| /livox/lidar_pointcloud | 5.000 | 104.586 | 108.697 | 117.910 |
| /livox/lidar_costmap | 5.000 | 106.063 | 110.165 | 118.807 |
| /Odometry | 10.000 | 17.408 | 22.417 | 26.489 |
| /localization | 10.000 | 17.818 | 22.349 | 27.541 |
| /fastlio/odometry_velocity | 10.000 | 17.894 | 23.322 | 28.148 |
| TF camera_init->body | 10.000 | 18.877 | 24.428 | 30.280 |
| TF map->camera_init | 10.000 | 19.428 | 24.446 | 29.167 |

CustomMsg는 스캔 시작 기준 stamp다. 마지막 점의 offset_time을 반영한 스캔 종료 기준 지연 중앙값 2.53ms, p95 4.88ms, 최대24.76ms. 약100ms의 스캔 수집 시간을 그대로 계산 지연으로 해석하면 안 된다.

## 현재 실행 상태

- controller/planner/local/global costmap lifecycle: 모두 active.
- /cmd_vel 수신 0건: Goal 없는 상태와 일치한다. 기존 LCM 노드가 이전에 저장한 명령을 송신하지 않는다는 증거는 아니다.
- local costmap footprint 약5Hz, global footprint 약1Hz. local /costmap 발행은 약3.31Hz로, 설정 publish_frequency=5보다 낮다. 내부 업데이트 주기의 정확한 계측은 아니며 이 값만으로 obstacle 갱신 중단을 단정하지 않는다.
- Nav2 현재 odom_topic=/Odometry. /fastlio/odometry_velocity는10Hz로 나오지만 Nav2 속도 입력으로 선택되어 있지 않다.
- 현재 설정 controller5Hz, max_vel_x1.0, max_vel_theta1.5, acc_lim_x/acc_lim_theta1.0, decel_lim_x/decel_lim_theta−7.0.
- RAM약5.2/15.7GB, swap0. Goal 없는 조건이라 A*/DWB 주행 중 부하를 검증한 것은 아니다.

## 버퍼 적용과 남은 문제

- net.core.rmem_max/wmem_max=16,777,216: 적용됨. rmem_default/wmem_default=212,992는 그대로다.
- 프로세스별 SHM: 일반32MiB, PCD 송신128MiB. 공유 메모리와 UDP 소켓 버퍼는 별개다.
- Livox SDK 센서용 포트56000/56101/56201/56301/56401: getsockopt(SO_RCVBUF)=33,554,432. Linux 내부 회계 배수까지 포함한 값으로, 요청 상한16MiB가 적용되어 있다. 측정 전후 각 소켓 drops=0.
- DDS용 UDP 소켓은 실제 SO_RCVBUF=212,992로 유지된다. XML에 receiveBufferSize/sendBufferSize가 없어서 시스템 기본값을 사용한다. rmem_max만 올려도 애플리케이션이 큰 값을 요청하지 않으면 자동 확대되지 않는다.
- 60초 진단 구간에서 UDP RcvbufErrors +47,132, SndbufErrors +259. 소켓별 drops 증가는 domain13의 DDS discovery/metatraffic unicast 범위(10660+2×participant_id)에 집중됐다. Livox 센서 패킷47,132개 손실이라는 뜻이 아니다. 진단 노드의 discovery 추가 영향도 포함되어 있어 별도 ROS 구독 없이10초 추가 측정에서도 RcvbufErrors +7,589, SndbufErrors +27이 발생했다. 따라서 진단 구독자 추가 때만 생기는 현상은 아니다.
- 아직 모든 통신 병목이 해소됐다고 결론낼 수 없다. 다음 검토 대상은 프로필 UDP transport의 명시적인 receiveBufferSize/sendBufferSize와 재시작 전후 오류 증가량 비교다. 이번 점검 중에는 변경하지 않았다.

## 시작 시 일시 지연과 기존 제한

- Livox 실행 직후 21:15:58, IMU queue_wait_max약3초/600건 대기가 해소됐다. 아직 localization 시작 전 구간이다.
- Nav2 활성화 중 21:16:21, IMU publish_work_max약225ms, queue_wait_max약221ms가 있었다. 안정화 이후 측정의 지속적인1.3초 지연과 구분해야 한다.
- 현재 안정화 구간에서는 과거0.78Hz/1.3초 지연이 재현되지 않았다. 단일 안정 구간이 간헐적 문제의 완전 해결을 입증하지는 않는다.
- 기존 LCM 코드의 /cmd_vel 갱신 중단 시 마지막 명령 유지 동작은 이 점검으로 수정되거나 검증되지 않았다. Goal 없는 상태만으로 실제 명령 전달·주행 안전을 확인할 수 없다.

## 근거

- timing.json, timing_after_warmup.json, timestamps.npz
- before.json/after.json, udp_passive_followup.json, livox_socket_buffers.json
- process_load.json, tegrastats.log, 복사한 개별 노드 로그
- Fast DDS2.6 UDP transport: https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html
- RTPS metatraffic port: 설치된 /opt/ros/humble/include/fastrtps/fastdds/rtps/common/PortParameters.h
