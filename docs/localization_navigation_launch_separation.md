# ROS1 Navigation: Localization / Navigation Launch Separation Guide

이 문서는 `navigation_ros1` 브랜치에서 FAST-LIO localization, GenZ-ICP localization, 그리고 `move_base` navigation launch를 분리 운용하기 위한 기준 문서다.

핵심 목표는 다음과 같다.

```text
1. localization backend는 별도 launch에서 실행한다.
2. navigation launch는 localization을 include하지 않는다.
3. FAST-LIO와 GenZ-ICP를 바꿀 때 navigation launch의 frame / odom topic argument만 바꾼다.
4. TF extrapolation error는 중복 localization, time source mismatch, frame chain mismatch를 우선 확인한다.
```

---

## 1. 왜 launch를 분리해야 하는가?

기존 `nav_real_sparse.launch`는 navigation launch 내부에서 FAST-LIO localization을 include하고 있었다.

```xml
<include file="$(find fast_lio_localization)/launch/localization_velodyne_nav.launch">
    <arg name="rviz" value="false"/>
</include>
```

이 구조에서 별도 터미널로 FAST-LIO localization 또는 GenZ-ICP localization을 다시 실행하면 TF publisher가 중복될 수 있다.

대표적인 문제는 다음과 같다.

```text
- map -> camera_init TF 중복 publish
- camera_init -> body 또는 camera_init -> velodyne TF 중복 publish
- /Odometry와 /genz/odometry 혼용
- move_base가 잘못된 odom topic을 사용
- TF extrapolation into the past / future
- TF_OLD_DATA warning
```

따라서 앞으로는 다음 구조를 사용한다.

```text
Terminal 1: localization only
Terminal 2: navigation only
Terminal 3: optional LCM bridge / command monitor
```

---

## 2. 목표 launch 구조

추천 파일 구성은 다음과 같다.

```text
pongbot_navigation/launch/
├── navigation.launch              # 기본 AStarPlannerROS
├── navigation_sparse.launch       # AStarSparsePlannerROS
├── nav_real_fastlio.launch        # 선택: FAST-LIO + navigation 통합 실행용
└── nav_real_genz.launch           # 선택: GenZ-ICP + navigation 통합 실행용
```

단, 디버깅과 실험 재현성 측면에서는 localization과 navigation을 따로 실행하는 것을 기본으로 한다.

---

## 3. FAST-LIO localization 기준 frame contract

FAST-LIO 기반 navigation의 기준 TF 구조는 다음과 같다.

```text
map
 └── camera_init
      └── body
           └── velodyne
```

FAST-LIO odometry topic은 다음을 기대한다.

```text
/Odometry.header.frame_id = camera_init
/Odometry.child_frame_id  = body
```

따라서 navigation launch에 들어갈 핵심 파라미터는 다음과 같다.

```text
nav_map_frame      := map_2d
local_odom_frame   := camera_init
robot_base_frame   := body
odom_topic         := /Odometry
use_body_to_velodyne_tf := true
```

---

## 4. GenZ-ICP localization 기준 frame contract

GenZ-ICP 기반 navigation의 기준 TF 구조는 다음과 같다.

```text
map
 └── camera_init
      └── velodyne
```

GenZ odometry topic은 다음을 기대한다.

```text
/genz/odometry.header.frame_id = camera_init
/genz/odometry.child_frame_id  = velodyne
```

따라서 navigation launch에 들어갈 핵심 파라미터는 다음과 같다.

```text
nav_map_frame      := map_2d
local_odom_frame   := camera_init
robot_base_frame   := velodyne
odom_topic         := /genz/odometry
use_body_to_velodyne_tf := false
```

GenZ 모드에서는 `velodyne` 자체를 robot base처럼 사용하므로 `body -> velodyne` static TF를 publish하면 안 된다.

---

## 5. `navigation_sparse.launch` 작성 기준

기존 `nav_real_sparse.launch`를 복사해서 `navigation_sparse.launch`를 만들고, localization include를 제거한다.

```bash
cp pongbot_navigation/launch/nav_real_sparse.launch \
   pongbot_navigation/launch/navigation_sparse.launch
```

그리고 아래 항목을 반영한다.

### 5.1 반드시 제거할 항목

navigation-only launch에서는 아래 FAST-LIO include를 제거한다.

```xml
<include file="$(find fast_lio_localization)/launch/localization_velodyne_nav.launch">
    <arg name="rviz" value="false"/>
</include>
```

### 5.2 추가 / argument화할 항목

```xml
<arg name="use_rviz" default="true"/>
<arg name="use_lcm_bridge" default="false"/>
<arg name="use_sim_time" default="false"/>

<arg name="map_yaml"
     default="/home/rclab/catkin_nav_ws/src/navigation_real/FAST_LIO_LOCALIZATION/map/building_1f_map.yaml"/>

<arg name="pcd_map" default=""/>

<arg name="map_frame" default="map"/>
<arg name="nav_map_frame" default="map_2d"/>
<arg name="local_odom_frame" default="camera_init"/>
<arg name="robot_base_frame" default="body"/>
<arg name="odom_topic" default="/Odometry"/>

<arg name="use_body_to_velodyne_tf" default="true"/>
<arg name="pointcloud_topic" default="/velodyne_points"/>
```

### 5.3 map -> map_2d alignment 유지

기존 실험에서 사용하던 alignment 값은 그대로 유지한다.

```xml
<arg name="align_x" default="0.15"/>
<arg name="align_y" default="-0.2"/>
<arg name="align_z" default="-0.10"/>
<arg name="align_roll" default="0.0"/>
<arg name="align_pitch" default="0.0"/>
<arg name="align_yaw" default="-0.02"/>

<node pkg="tf2_ros" type="static_transform_publisher" name="map_to_map2d" output="screen"
      args="$(arg align_x) $(arg align_y) $(arg align_z) $(arg align_roll) $(arg align_pitch) $(arg align_yaw) $(arg map_frame) $(arg nav_map_frame)" />
```

### 5.4 body -> velodyne static TF는 조건부로 실행

```xml
<group if="$(arg use_body_to_velodyne_tf)">
    <node pkg="tf2_ros" type="static_transform_publisher" name="body_to_velodyne"
          args="0.297 0 0.1536 0 0 0 $(arg robot_base_frame) velodyne" />
</group>
```

FAST-LIO mode에서는 `true`, GenZ-ICP mode에서는 `false`로 둔다.

### 5.5 DWA odometry topic은 argument로 받기

기존 하드코딩:

```xml
<param name="DWAPlannerROS/odom_topic" value="/Odometry"/>
```

수정:

```xml
<param name="DWAPlannerROS/odom_topic" value="$(arg odom_topic)"/>
<param name="DWAPlannerROS/odom_frame" value="$(arg local_odom_frame)"/>
```

---

## 6. 기본 A* `navigation.launch` 작성 기준

`navigation_sparse.launch`와 동일하되 global planner plugin만 바꾼다.

```xml
<param name="base_global_planner" value="pongbot_global_planner/AStarPlannerROS"/>

<rosparam file="$(find pongbot_global_planner)/params/astar_params.yaml"
          command="load"
          ns="AStarPlannerROS"/>
```

---

## 7. Sparse A* `navigation_sparse.launch` 작성 기준

Sparse planner에서는 다음을 사용한다.

```xml
<param name="base_global_planner" value="pongbot_global_planner/AStarSparsePlannerROS"/>

<rosparam file="$(find pongbot_global_planner)/params/astar_sparse_params.yaml"
          command="load"
          ns="AStarSparsePlannerROS"/>
```

---

## 8. FAST-LIO 기반 실행 예시

### Terminal 1: FAST-LIO localization only

```bash
roslaunch fast_lio_localization localization_velodyne_nav.launch
```

### Terminal 2: 기본 A*

```bash
roslaunch pongbot_navigation navigation.launch \
  use_sim_time:=false \
  nav_map_frame:=map_2d \
  local_odom_frame:=camera_init \
  robot_base_frame:=body \
  odom_topic:=/Odometry \
  use_body_to_velodyne_tf:=true \
  use_lcm_bridge:=false
```

### Terminal 2: Sparse A*

```bash
roslaunch pongbot_navigation navigation_sparse.launch \
  use_sim_time:=false \
  nav_map_frame:=map_2d \
  local_odom_frame:=camera_init \
  robot_base_frame:=body \
  odom_topic:=/Odometry \
  use_body_to_velodyne_tf:=true \
  use_lcm_bridge:=false
```

---

## 9. GenZ-ICP 기반 실행 예시

### Terminal 1: GenZ-ICP localization only

```bash
roslaunch genz_icp genz_velodyne_localization.launch \
  use_sim_time:=false \
  visualize:=true
```

### Terminal 2: Sparse A*

```bash
roslaunch pongbot_navigation navigation_sparse.launch \
  use_sim_time:=false \
  nav_map_frame:=map_2d \
  local_odom_frame:=camera_init \
  robot_base_frame:=velodyne \
  odom_topic:=/genz/odometry \
  use_body_to_velodyne_tf:=false \
  use_lcm_bridge:=false
```

---

## 10. rosbag / live time policy

### rosbag replay

```text
/use_sim_time = true
rosbag play <bag>.bag --clock
localization launch use_sim_time:=true
navigation launch use_sim_time:=true
```

확인:

```bash
rosparam get /use_sim_time
rostopic hz /clock
rostopic hz /velodyne_points
```

### live sensor

```text
/use_sim_time = false
localization launch use_sim_time:=false
navigation launch use_sim_time:=false
```

확인:

```bash
rosparam get /use_sim_time
rostopic hz /velodyne_points
```

GenZ live sensor에서는 sensor timestamp가 wall time과 맞지 않으면 `genz_velodyne_odometry.yaml`, `genz_velodyne_localization.yaml`에서 다음처럼 설정한다.

```yaml
use_sensor_stamp: false
```

---

## 11. TF extrapolation error 디버깅 순서

TF extrapolation이 계속 발생하면 아래 순서로 확인한다.

### 11.1 localization 중복 실행 확인

```bash
rosnode list | grep -E "fast|lio|genz|localization"
rostopic info /tf
```

원칙:

```text
FAST-LIO mode:
  FAST-LIO localization만 map/camera_init/body TF를 publish

GenZ mode:
  GenZ localization만 map/camera_init/velodyne TF를 publish
```

### 11.2 time source 확인

```bash
rosparam get /use_sim_time
rostopic hz /clock
```

rosbag인데 `/clock`이 없으면 `rosbag play --clock`이 빠진 것이다. live sensor인데 `/use_sim_time=true`면 잘못된 설정이다.

### 11.3 TF chain 확인

FAST-LIO:

```bash
rosrun tf tf_echo map camera_init
rosrun tf tf_echo camera_init body
rosrun tf tf_echo body velodyne
```

GenZ-ICP:

```bash
rosrun tf tf_echo map camera_init
rosrun tf tf_echo camera_init velodyne
```

### 11.4 move_base 실제 parameter 확인

```bash
rosparam get /move_base/global_costmap/global_frame
rosparam get /move_base/local_costmap/global_frame
rosparam get /move_base/global_costmap/robot_base_frame
rosparam get /move_base/local_costmap/robot_base_frame
rosparam get /move_base/DWAPlannerROS/odom_topic
```

FAST-LIO 기대값:

```text
global_costmap/global_frame: map_2d
local_costmap/global_frame: camera_init
robot_base_frame: body
DWAPlannerROS/odom_topic: /Odometry
```

GenZ 기대값:

```text
global_costmap/global_frame: map_2d
local_costmap/global_frame: camera_init
robot_base_frame: velodyne
DWAPlannerROS/odom_topic: /genz/odometry
```

### 11.5 transform tolerance 임시 완화

수 ms ~ 수십 ms 수준의 extrapolation이면 costmap tolerance를 1.0으로 늘려볼 수 있다.

```yaml
global_costmap:
  transform_tolerance: 1.0

local_costmap:
  transform_tolerance: 1.0
```

단, 수 초 이상 차이가 나면 tolerance 문제가 아니라 time source mismatch이다.

---

## 12. 멈칫하는 motion의 주요 원인

중간중간 멈칫하는 motion은 보통 다음 원인이 많다.

```text
1. TF extrapolation으로 local plan 생성 실패
2. localization 중복 또는 time source mismatch
3. global replanning path jitter
4. obstacle layer가 순간적으로 robot 앞을 막음
5. cmd_vel bridge deadman timeout
6. sparse path segment가 너무 길어 DWA가 코너에서 급격히 반응
```

우선 TF extrapolation을 먼저 해결한다. 그다음 아래를 확인한다.

```bash
rostopic echo /cmd_vel
rostopic echo /move_base/recovery_status
rostopic echo /move_base/DWAPlannerROS/local_plan
rostopic hz /cmd_vel
```

LCM bridge deadman이 원인이면 timeout을 늘린다.

```text
deadman_timeout_s: 0.3 -> 0.7 or 1.0
```

---

## 13. 대각선 이동 / holonomic motion 방지

현재 DWA config에 `max_vel_y`, `min_vel_y`, `vy_samples`, `holonomic_robot` 설정이 없으면 `linear.y`가 들어갈 수 있다.

비홀로노믹처럼 사용하려면 `pongbot_navigation/config/base_local_planner_params.yaml`에 아래를 추가한다.

```yaml
DWAPlannerROS:
  max_vel_y: 0.0
  min_vel_y: 0.0
  acc_lim_y: 0.0
  vy_samples: 1
  holonomic_robot: false
```

또한 legged robot 실차 초기 테스트에서는 yaw 속도도 낮게 시작하는 것이 좋다.

```yaml
DWAPlannerROS:
  max_vel_theta: 0.8
  min_vel_theta: -0.8
  min_in_place_vel_theta: 0.3
  acc_lim_th: 1.0
```

반드시 `/cmd_vel`에서 확인한다.

```bash
rostopic echo /cmd_vel
```

기대값:

```text
linear.y = 0.0
```

LCM bridge에서도 안전장치로 `linear_y=0`을 강제하는 것이 좋다.

---

## 14. 최종 원칙

```text
1. localization launch와 navigation launch는 분리한다.
2. navigation launch는 localization을 include하지 않는다.
3. FAST-LIO와 GenZ-ICP 전환은 robot_base_frame, odom_topic, use_body_to_velodyne_tf만 바꾼다.
4. TF extrapolation은 tolerance부터 만지지 말고 중복 TF publisher와 time source부터 확인한다.
5. live sensor에서는 /use_sim_time=false가 기본이다.
6. rosbag에서는 /use_sim_time=true와 rosbag play --clock이 반드시 같이 필요하다.
7. DWA는 y velocity를 막아서 non-holonomic 형태로 시작한다.
8. LCM bridge는 localization/navigation 검증이 끝난 뒤에 켠다.
```
