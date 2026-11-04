# DDS UDP 4 MiB 적용 후 점검 — 2026-10-29

## 범위와 측정 방법

실행 중인 사용자 스택을 읽기 전용으로 점검했다. Goal이나 주행 명령을 보내거나 노드를 재시작하지 않았다. 토픽 측정은 21:41:31~21:42:31 KST의 약 60초다. 측정 전 준비 시간 5초 동안 CustomMsg 역직렬화 검증을 마쳤다. 이전 측정에서 발생한 진단 도구 초기화 지연을 측정 구간에 포함하지 않기 위한 조치다.

토픽 지연은 구독자의 수신 시각에서 header.stamp를 뺀 값으로, 구독자 스케줄링도 포함한다. UDP before/after 카운터에는 준비 시간도 포함된다. 별도로 진단 ROS 구독자를 종료한 뒤 10초간 UDP 카운터를 측정했다.

## 실제 적용 상태

- 재시작된 Livox, FAST-LIO, global localization, transform fusion, PCD 발행, RViz, Nav2 주요 노드에서 DDS 수신 소켓 SO_RCVBUF=8,388,608 bytes를 확인했다. Linux 내부 관리 공간을 포함한 조회값으로, 4 MiB 요청이 적용된 결과다.
- DDS 송신용 소켓 SO_SNDBUF도 8,388,608 bytes다. 수신 소켓의 송신 버퍼와 송신 소켓의 수신 버퍼는 확인 대상과 구분해야 한다. 전체 소켓 목록은 processes.json에 있다.
- LCM PID 3867은 이전 실행이 유지되어 DDS 수신 소켓이 212,992 bytes다. 새 설정은 아직 적용되지 않았다.
- 커널 rmem_max/wmem_max는 각각 16 MiB다.

## 토픽 측정

| 토픽 | 수신 Hz | 지연 중앙값 ms | p95 ms | 최대 ms |
|---|---:|---:|---:|---:|
| /livox/imu | 199.997 | 0.588 | 2.050 | 36.575 |
| /livox/lidar | 10.000 | 101.930 | 104.944 | 127.698 |
| /livox/lidar_pointcloud | 5.000 | 104.126 | 107.795 | 123.308 |
| /livox/lidar_costmap | 5.000 | 105.130 | 108.756 | 123.579 |
| /Odometry | 10.001 | 13.535 | 20.146 | 42.915 |
| /localization | 10.001 | 14.277 | 20.458 | 43.146 |
| /fastlio/odometry_velocity | 10.001 | 14.050 | 20.790 | 43.293 |
| /local_costmap/costmap | 3.300 | 1.161 | 2.869 | 6.592 |

LiDAR header는 스캔 시작 기준이다. 마지막 점 기준 지연은 중앙값 2.49 ms, p95 5.28 ms, 최대 28.14 ms다.

- local costmap footprint는 5 Hz, global footprint는 1 Hz다. local costmap 시각화 토픽은 3.30 Hz로 변경 전 3.31 Hz와 비슷하다. 이 수치만으로 내부 갱신 실패를 단정하지 않는다.
- map→camera_init 및 camera_init→body TF는 각각 10 Hz이며 지연 p95는 약 21~22 ms다.
- controller/planner/local/global costmap lifecycle은 모두 active다. /cmd_vel 수신은 0건으로 Goal 없는 상태와 일치한다.
- 측정 구간의 드라이버 IMU queue_wait_max는 6.03 ms다. FAST-LIO 내부 IMU raw_age_max는 17.211 ms, gap_max는 22.117 ms다. Fusion 진단 표본의 input_interval 최대는 104.2 ms, input_age 최대는 19.4 ms다.

## UDP 카운터 비교

각각 진단 ROS 구독자 없이 10초간 관찰한 시스템 전체 카운터다. 동시에 수행한 A/B 시험은 아니며, 프로세스 재시작에 따른 변화도 포함한다.

| 카운터 증가량 / 10초 | 변경 전 | 변경 후 |
|---|---:|---:|
| RcvbufErrors | 7,589 | 0 |
| SndbufErrors | 27 | 59 |

토픽 측정 전후의 UDP 카운터(준비 시간 포함)는 RcvbufErrors 0, SndbufErrors 392 증가였다. 소켓별 drops 증가는 없었다.

수신 버퍼 오류는 이번 관찰에서 재현되지 않았다. 송신 버퍼 오류는 남아 있고, 이전보다 감소했다고 볼 수 없다. 시스템 전체 카운터만으로 DDS·LCM·다른 프로세스 중 어느 송신자인지 지정할 수는 없다.

센서 및 메인제어기 Ethernet은 확인 시점에 UP/LOWER_UP이며 인터페이스 RX/TX errors와 drops는 0이었다.

## 해석과 남은 확인

새 UDP 수신 버퍼 설정은 실행 중인 프로세스에서 확인되었다. 이번 구간에는 수신 폐기 0건, IMU 200 Hz, Odometry와 localization 10 Hz를 유지했다. 과거 0.78 Hz / 1.3초 지연은 재현되지 않았다.

RAM available은 약 9 GB, swap 사용량은 0이다. Goal 없는 조건이므로 실제 경로 계획 및 제어 중의 부하를 검증한 것은 아니다.

LCM에 새 설정을 적용하려면 기존 LCM 프로세스를 종료한 뒤 기존 명령으로 다시 시작해야 한다. 중복 실행하지 않는다. 남은 SndbufErrors의 발생 프로세스와 인터페이스는 별도 추적 대상이다.

이번 결과만으로 과거 명령 freeze나 주행 중 급회전의 원인 또는 해결을 확정하지 않는다. 기존 LCM의 /cmd_vel 입력 중단 시 마지막 명령 유지 동작은 변경되지 않았다. /cmd_vel 수신 0건은 실제 LCM 송신 명령이 0이라는 증거가 아니다.

## 원시 자료

- timing.json, timestamps.npz: 토픽 시각과 통계
- processes.json: 실행 인자, 프로필 환경, 실제 소켓 크기
- before.json / after.json, udp_passive_followup.json: UDP 카운터
- socket_drop_attribution.json: 소켓별 폐기 증가량
- measurement_window_source_logs.json 및 window_*.log: 측정 구간의 생산 노드 진단 로그
