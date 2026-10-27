> ROS2 Fork repo maintainer: [Ericsiii](https://github.com/Ericsii)

## Related Works and Extended Application

**SLAM:**

1. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): A state-of-art dynamic KD-Tree for 3D kNN search.
2. [R2LIVE](https://github.com/hku-mars/r2live): A high-precision LiDAR-inertial-Vision fusion work using FAST-LIO as LiDAR-inertial front-end.
3. [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init): A robust, real-time LiDAR-IMU extrinsic initialization and synchronization package..
4. [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): The integration of FAST-LIO with **Re-localization** function module.

**Control and Plan:**

1. [IKFOM](https://github.com/hku-mars/IKFoM): A Toolbox for fast and high-precision on-manifold Kalman filter.
2. [UAV Avoiding Dynamic Obstacles](https://github.com/hku-mars/dyn_small_obs_avoidance): One of the implementation of FAST-LIO in robot's planning.
3. [UGV Demo](https://www.youtube.com/watch?v=wikgrQbE6Cs): Model Predictive Control for Trajectory Tracking on Differentiable Manifolds.
4. [Bubble Planner](https://arxiv.org/abs/2202.12177): Planning High-speed Smooth Quadrotor Trajectories using Receding Corridors.

<!-- 10. [**FAST-LIVO**](https://github.com/hku-mars/FAST-LIVO): Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry. -->

## FAST-LIO
**FAST-LIO** (Fast LiDAR-Inertial Odometry) is a computationally efficient and robust LiDAR-inertial odometry package. It fuses LiDAR feature points with IMU data using a tightly-coupled iterated extended Kalman filter to allow robust navigation in fast-motion, noisy or cluttered environments where degeneration occurs. Our package address many key issues:
1. Fast iterated Kalman filter for odometry optimization;
2. Automaticaly initialized at most steady environments;
3. Parallel KD-Tree Search to decrease the computation;

## FAST-LIO 2.0 (2021-07-05 Update)
<!-- ![image](doc/real_experiment2.gif) -->
<!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->
<div align="left">
<img src="doc/real_experiment2.gif" width=49.6% />
<img src="doc/ulhkwh_fastlio.gif" width = 49.6% >
</div>

**Related video:**  [FAST-LIO2](https://youtu.be/2OvjGnxszf8),  [FAST-LIO1](https://youtu.be/iYCY6T79oNU)

**Pipeline:**
<div align="center">
<img src="doc/overview_fastlio2.svg" width=99% />
</div>

**New Features:**
1. Incremental mapping using [ikd-Tree](https://github.com/hku-mars/ikd-Tree), achieve faster speed and over 100Hz LiDAR rate.
2. Direct odometry (scan to map) on Raw LiDAR points (feature extraction can be disabled), achieving better accuracy.
3. Since no requirements for feature extraction, FAST-LIO2 can support many types of LiDAR including spinning (Velodyne, Ouster) and solid-state (Livox Avia, Horizon, MID-70) LiDARs, and can be easily extended to support more LiDARs.
4. Support external IMU.
5. Support ARM-based platforms including Khadas VIM3, Nivida TX2, Raspberry Pi 4B(8G RAM).

**Related papers**: 

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

**Contributors**

[Wei Xu 徐威](https://github.com/XW-HKU)，[Yixi Cai 蔡逸熙](https://github.com/Ecstasy-EC)，[Dongjiao He 贺东娇](https://github.com/Joanna-HE)，[Fangcheng Zhu 朱方程](https://github.com/zfc-zfc)，[Jiarong Lin 林家荣](https://github.com/ziv-lin)，[Zheng Liu 刘政](https://github.com/Zale-Liu), [Borong Yuan](https://github.com/borongyuan)

<!-- <div align="center">
    <img src="doc/results/HKU_HW.png" width = 49% >
    <img src="doc/results/HKU_MB_001.png" width = 49% >
</div> -->

## Jetson MID360S: timestamp / native velocity check (2026-10-27)

This workspace's `mid360.yaml` now matches the current dual-output driver:
`/livox/lidar` is Livox `CustomMsg` (`lidar_type: 1`), while
`/livox/lidar_pointcloud` is `PointCloud2` for costmaps. The legacy launch name
`rviz_MID360_launch.py` selects `MID360s_config.json`. Do not mix overlays from
`legged_robot_ws` into this test.

### Changes and limits

- The secondary PointCloud2 header keeps the packet's acquisition time
  (`pkg.base_time`), matching the corresponding CustomMsg header. It is no
  longer overwritten with zero. Point offsets, IMU timestamps, driver rotation,
  QoS and the 10 Hz CustomMsg / 5 Hz secondary cloud schedule are unchanged.
- Zero means latest available TF, not synchronization. Nonzero stamps require
  TF history at the observation time. Real TF delay, missing transforms or mixed
  clocks may therefore appear as drops/waits; do not hide them by restamping
  with zero or `now()`. Nav2 live costmap verification remains a separate test.
- Both `fast_lio` and `fast_lio_localization` expose an additional
  `nav_msgs/msg/Odometry` topic, **`/fastlio/odometry_velocity`**. It copies the
  original pose/header at scan end (`camera_init` parent, `body` child).
  Legacy `/Odometry` twist, TF and Nav2 configuration are unchanged. The new
  diagnostic publisher uses BEST_EFFORT, VOLATILE, KEEP_LAST(1) to favor fresh
  readings over an accumulated diagnostic backlog; subscribers must accept
  BEST_EFFORT.
- Linear velocity is the existing posterior filter state transformed into body
  coordinates: `v_body = R_world_body.transpose() * v_world` [m/s]. Angular
  velocity is the last averaged gyro used in propagation minus the posterior
  gyro bias [rad/s], already in body coordinates. There is no pose differencing,
  additional estimator, command-velocity feedback or additional ICP pass.
- Twist covariance propagates attitude/velocity/bias uncertainty and adds the
  configured gyro noise. This is an approximation (gyro/state cross-correlation
  is not modeled), not an accuracy certificate. Invalid/nonfinite estimates
  are not published on the new topic. There is no new tracking-quality flag.
- This is **IMU-origin velocity, not robot-base velocity**. The sensor-only test
  does not assume a `base_link` transform. A later Nav2 connection needs an
  explicit frame/lever-arm check and measured validation. No automatic fallback
  to `/cmd_vel` is provided.
- The default standalone config fixes the internal LiDAR-to-IMU extrinsic to
  the existing localization config: `[-0.011, 0.02329, -0.04412]` m. This is
  consistent with this driver's 180-degree X rotation of BOTH points and IMU;
  it is not a newly measured MID360S calibration or a robot mounting offset.
  Use a different config if the driver rotation changes. Automatic PCD saving
  is disabled for this velocity test.

### Run the sensor-only check

In **each fresh terminal**, use the same workspace, domain and middleware:

```bash
source /opt/ros/humble/setup.bash
source /home/rclab/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Terminal 1 (user starts the connected sensor):

```bash
taskset -c 0,1 ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

Terminal 2 (no PCD map / initial pose / global localization needed):

```bash
taskset -c 2,3,4,5 ros2 launch fast_lio mapping.launch.py
```

Defaults are `use_sim_time:=false`, `rviz:=true`, `config_file:=mid360.yaml`.
Keep the sensor still during initialization. Keep Nav2/LCM off for this test.
**Never run `fast_lio` and `fast_lio_localization` together**: they own the same
Odometry topics and `camera_init -> body` TF. The driver launch itself does not
start RViz; the mapping launch does.

Terminal 3 (run measurements individually; Ctrl-C between them):

```bash
ros2 topic echo /livox/lidar_pointcloud --field header --once --qos-reliability best_effort
ros2 topic hz /fastlio/odometry_velocity --window 30
ros2 topic delay /fastlio/odometry_velocity --window 30
ros2 topic echo /fastlio/odometry_velocity --field twist.twist --qos-reliability best_effort
```

Check that the cloud stamp is nonzero and advances. Its stamp is scan start;
Odometry is scan end, approximately 0.1 s later for a 10 Hz scan. That difference
is not, by itself, queue latency. In the new topic, examine `linear.x/y/z` and
`angular.x/y/z`; the legacy `/Odometry` twist is intentionally not populated.

Observe 30-60 seconds stationary, then translate gently forward/backward and
rotate gently. Look for near-zero stationary output, consistent signs/units,
finite values and a rate matching `/Odometry` after initialization (typically
10 Hz here). Holding and rotating the sensor around the hand can legitimately
produce translational velocity at the IMU origin. This qualitative check does
not establish absolute speed accuracy or validate robot-base feedback.

### Rebuild / regression tests

```bash
cd /home/rclab/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --symlink-install \
  --packages-select livox_ros_driver2 fast_lio fast_lio_localization \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DRMW_IMPLEMENTATION=rmw_fastrtps_cpp
colcon test --packages-select fast_lio fast_lio_localization --executor sequential
colcon test-result --test-result-base build/fast_lio --verbose
colcon test-result --test-result-base build/fast_lio_localization --verbose
```

This reuses the already configured driver SDK; it does not install or upgrade
system packages. The tests cover body-frame transformation, gyro bias,
covariance, unchanged source messages, copied Odometry timestamps and nonfinite
rejection. They are not a live sensor/TF/costmap performance test.

Verification on this Jetson (2026-10-27): all three selected packages built;
five GTest cases passed in each FAST-LIO package (ten cases total), with no
failures. Launch argument discovery confirmed the standalone defaults, and the
driver component resolves its SDK from the existing workspace-private install.
The old `fast_lio` CMake cache selected unavailable `rmw_zenoh_cpp`; the explicit
Fast DDS build option above corrected that cache without installing middleware.
Sensor publication, tracking accuracy and live Nav2 costmap updates were not
run as part of this build/test verification.

Pre-change backup: [backup/jetson_261027](https://github.com/Kimgyoomin/navigation_real/tree/backup/jetson_261027),
commit `45b280787c609c315d59d99be230f615d195ef02`. This backs up tracked source/config,
including the pre-existing global obstacle minimum range of 0.6 m, not ignored
maps, recordings or installed SDK binaries. Implementation branch:
`jetson/261027-stamp-velocity`.

The velocity conversion follows the approach reviewed in
[the handoff branch](https://github.com/Kimgyoomin/legged_robot_ws/tree/codex/navigation-handoff-20261027),
without importing its Nav2 adapter/controller changes.
ROS defines twist in the [Odometry child frame](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg);
see also the [tf2 time semantics](https://github.com/ros2/ros2_documentation/blob/humble/source/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.rst).

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
**Ubuntu >= 20.04**

The **default from apt** PCL and Eigen is enough for FAST-LIO to work normally.

ROS >= Foxy (Recommend to use ROS-Humble). [ROS Installation](https://docs.ros.org/en/humble/Installation.html)

### 1.2. **PCL && Eigen**
PCL    >= 1.8,   Follow [PCL Installation](https://pointclouds.org/downloads/#linux).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### <span id="1.3">1.3. **livox_ros_driver2**</span>
Follow [livox_ros_driver2 Installation](https://github.com/Livox-SDK/livox_ros_driver2).

You can also use the one I modified [livox_ros_driver2](https://github.com/Ericsii/livox_ros_driver2/tree/feature/use-standard-unit)

*Remarks:*
- Since the FAST-LIO must support Livox serials LiDAR firstly, so the **livox_ros_driver** must be installed and **sourced** before run any FAST-LIO launch file.
- How to source? The easiest way is add the line ``` source $Livox_ros_driver_dir$/devel/setup.bash ``` to the end of file ``` ~/.bashrc ```, where ``` $Livox_ros_driver_dir$ ``` is the directory of the livox ros driver workspace (should be the ``` ws_livox ``` directory if you completely followed the livox official document).


## 2. Build
Clone the repository and colcon build:

```bash
    cd <ros2_ws>/src # cd into a ros2 workspace folder
    git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
    cd ..
    rosdep install --from-paths src --ignore-src -y
    colcon build --symlink-install
    . ./install/setup.bash # use setup.zsh if use zsh
```
- **Remember to source the livox_ros_driver before build (follow [1.3 livox_ros_driver](#1.3))**
- If you want to use a custom build of PCL, add the following line to ~/.bashrc
```export PCL_ROOT={CUSTOM_PCL_PATH}```
## 3. Directly run
Noted:

A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.

B. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important for the forward propagation and backwark propagation.

C. We recommend to set the **extrinsic_est_en** to false if the extrinsic is give. As for the extrinsic initiallization, please refer to our recent work: [**Robust Real-time LiDAR-inertial Initialization**](https://github.com/hku-mars/LiDAR_IMU_Init).

### 3.1 Run use ros launch
Connect to your PC to Livox LiDAR by following  [Livox-ros-driver2 installation](https://github.com/Livox-SDK/livox_ros_driver2), then
```bash
cd <ros2_ws>
. install/setup.bash # use setup.zsh if use zsh
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml
```

Change `config_file` parameter to other yaml file under config directory as you need.

Launch livox ros driver. Use MID360 as an example.

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

- For livox serials, FAST-LIO only support the data collected by the ``` livox_lidar_msg.launch ``` since only its ``` livox_ros_driver2/CustomMsg ``` data structure produces the timestamp of each LiDAR point which is very important for the motion undistortion. ``` livox_lidar.launch ``` can not produce it right now.
- If you want to change the frame rate, please modify the **publish_freq** parameter in the [livox_lidar_msg.launch](https://github.com/Livox-SDK/livox_ros_driver/blob/master/livox_ros_driver2/launch/livox_lidar_msg.launch) of [Livox-ros-driver](https://github.com/Livox-SDK/livox_ros_driver2) before make the livox_ros_driver pakage.

### 3.2 For Livox serials with external IMU

mapping_avia.launch theratically supports mid-70, mid-40 or other livox serial LiDAR, but need to setup some parameters befor run:

Edit ``` config/avia.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ```
3. Translational extrinsic: ``` extrinsic_T ```
4. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame). They can be found in the official manual.
- FAST-LIO produces a very simple software time sync for livox LiDAR, set parameter ```time_sync_en``` to ture to turn on. But turn on **ONLY IF external time synchronization is really not possible**, since the software time sync cannot make sure accuracy.

### 3.4 PCD file save

1. Enable `pcd_save.pcd_save_en` in the config file and set the `map_file_path` to the path where the map will be saved.
2. Launch the fastlio2 according to README.
3. Open RQt and switch to `Plugins->Services->Service Caller`. Trigger the service `/map_save`, then the pcd map file will be generated

```pcl_viewer scans.pcd``` can visualize the point clouds.

*Tips for pcl_viewer:*
- change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 
```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```

## 4. Rosbag Example
### 4.1 Livox Avia Rosbag
<div align="left">
<img src="doc/results/HKU_LG_Indoor.png" width=47% />
<img src="doc/results/HKU_MB_002.png" width = 51% >

Files: Can be downloaded from [google drive](https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP?usp=sharing)**!!!This ros1 bag should be convert to ros2!!!**

Run:
```bash
ros2 launch fast_lio mapping.launch.py config_path:=<path_to_your_config_file>
ros2 bag play <your_bag_dir>

```

### 4.2 Velodyne HDL-32E Rosbag

**NCLT Dataset**: Original bin file can be found [here](http://robots.engin.umich.edu/nclt/).

We produce [Rosbag Files](https://drive.google.com/drive/folders/1VBK5idI1oyW0GC_I_Hxh63aqam3nocNK?usp=sharing) and [a python script](https://drive.google.com/file/d/1leh7DxbHx29DyS1NJkvEfeNJoccxH7XM/view) to generate Rosbag files: ```python3 sensordata_to_rosbag_fastlio.py bin_file_dir bag_name.bag```**!!!This ros1 bag should be convert to ros2!!!** To convert ros1 bag to ros2 bag, please follow the documentation [Convert rosbag versions](https://ternaris.gitlab.io/rosbags/topics/convert.html)
    
Run:
```
roslaunch fast_lio mapping_velodyne.launch
rosbag play YOUR_DOWNLOADED.bag
```

## 5.Implementation on UAV
In order to validate the robustness and computational efficiency of FAST-LIO in actual mobile robots, we build a small-scale quadrotor which can carry a Livox Avia LiDAR with 70 degree FoV and a DJI Manifold 2-C onboard computer with a 1.8 GHz Intel i7-8550U CPU and 8 G RAM, as shown in below.

The main structure of this UAV is 3d printed (Aluminum or PLA), the .stl file will be open-sourced in the future.

<div align="center">
    <img src="doc/uav01.jpg" width=40.5% >
    <img src="doc/uav_system.png" width=57% >
</div>

## 6.Acknowledgments

Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [Livox_Mapping](https://github.com/Livox-SDK/livox_mapping), [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) and [Loam_Livox](https://github.com/hku-mars/loam_livox).
