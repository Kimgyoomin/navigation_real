# GenZ-ICP ROS wrappers

## RUBI PCD localization (ROS 2 Humble)

This workspace adds fixed-PCD localization alongside GenZ odometry. These
launches run both estimators and RViz; they do not start Nav2 or the LCM bridge.
The registration algorithm is unchanged by the launch setup below.

Defaults shared by live and bag operation:

- PointCloud2 input: `/livox/lidar_pointcloud`. `/livox/lidar` is Livox
  `CustomMsg` in this workspace and cannot be passed directly to these nodes.
- PCD: `FAST_LIO_LOCALIZATION2/PCD/1f_fast_lio_260731_segmented.pcd` under
  `/home/rclab/ros2_ws/src/navigation_real`. Override with `map_path:=...` on
  another machine.
- TF: `map -> camera_init -> base_link -> livox_frame`.
- `base_link -> livox_frame`: translation `[0.0995, 0, -0.1375]` metres,
  identity rotation. The RUBI Livox driver already rotates the inverted sensor
  data around X; do not apply that rotation a second time in TF.
- Sensor timestamps are preserved; `transform_publish_tolerance=0.0` adds no
  future offset to `map -> camera_init`.
- `localization_corridor.yaml` currently disables deskew. These GenZ nodes do
  not subscribe to IMU. This is different from FAST-LIO's CustomMsg + IMU path.
- `dds_profile:=navigation`: common processes use 32 MiB SHM, and the PCD
  localization process uses 128 MiB. Both retain UDP transport. These are the
  existing `navigation_dds` profiles, scoped to child processes, not system
  settings. `dds_profile:=default` keeps the inherited DDS environment.

### Bag comparison

Use a separate ROS domain for replay so existing live TF and commands do not
mix with the comparison. Run this setup in **each replay terminal**:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

Terminal 1:

```bash
taskset -c 2,3,4,5 ros2 launch genz_icp localization_rosbag.launch.py \
  bagfile:=/home/rclab/rosbag_record/rosbag2_2026_10_30-20_50_26
```

This launches exactly one player, paused initially, with `/clock` and only the
selected PointCloud2 topic. Recorded `/tf`, `/tf_static`, `/Odometry`,
`/localization`, `/initialpose` and `/cmd_vel` are not replayed. All estimator,
static-TF and RViz nodes use simulation time. The player's `--clock 40` is the
clock publication frequency; playback speed is `rate:=1.0` independently.

Once the PCD is visible, use **2D Pose Estimate** to set the robot's approximate
starting position and heading in the bag. Then in terminal 2:

```bash
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
```

`start_paused:=false` enables immediate playback. `rate:=0.5` can be used to
separate replay compute limits from registration behavior. Restart the launch
for a fresh replay; the estimators are not automatically reset by seeking back.

If the bag is already being played separately, omit `bagfile`. The wrapper
starts no player in that mode. The separate player must supply the same input
and clock, for example:

```bash
ros2 bag play /home/rclab/rosbag_record/rosbag2_2026_10_30-20_50_26 \
  --clock 40 --topics /livox/lidar_pointcloud
```

The `20_50_26` recording contains 2,085 PointCloud2 messages versus 4,400
CustomMsg messages. This launch uses the recorded PointCloud2 stream as-is;
it neither recovers missing scans nor converts CustomMsg. Thus this is a
pipeline/visual comparison, not an equal-input accuracy benchmark against
FAST-LIO. Such a benchmark needs a separate, timestamp-preserving conversion
of the better-recorded CustomMsg stream.

### Live sensor check

In a live terminal, source the same workspace, use the driver's ROS domain
(normally `ROS_DOMAIN_ID=13`) and Fast DDS. Stop any bag player or FAST-LIO TF
broadcasters in that domain before switching estimators. Keep the existing
Livox driver publishing PointCloud2, then run:

```bash
taskset -c 2,3,4,5 ros2 launch genz_icp localization.launch.py
```

Live operation defaults to `use_sim_time:=false`, RViz enabled, and waits for
**2D Pose Estimate**. It contains no bag player. Override mount values with
`base_to_livox_x/y/z/roll/pitch/yaw`, and set `publish_livox_tf:=false` if a
robot description already owns this static transform. When testing at the
sensor origin with `base_frame:=livox_frame`, the mount broadcaster is omitted.

Inspect `/genz/odometry` (local frame) and `/genz/localization_odom` (PCD map
frame), including header time, pose Z and orientation. Neither current GenZ
output populates `twist`; do not switch the Nav2 velocity-feedback topic to
these messages and assume measured velocity is available. Nav2 integration,
timing under load and height-filter behavior require separate verification.

## :gear: How to build & Run

### ROS1

#### How to build

You should not need any extra dependency, just clone and build:
    
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/cocel-postech/genz-icp.git
cd ..
catkin build genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

#### How to run

#### Option 1

If you want to use a pre-tuned parameter set, you need to provide the **config file** with the **topic name** as arguments:

```sh
roslaunch genz_icp odometry.launch topic:=<topic_name> config_file:=<config_file_name>.yaml
```
```sh
rosbag play <rosbag_file_name>.bag
```

For example,

1. **Long_Corridor** sequence of SubT-MRS dataset

```sh
roslaunch genz_icp odometry.launch topic:=/velodyne_points config_file:=long_corridor.yaml
```
```sh
rosbag play subt_mrs_long_corridor.bag
```

The original bagfile for the **Long_Corridor** sequence of SubT-MRS dataset can be downloaded from [here][long_corridor_original_link]

`subt_mrs_long_corridor.bag` includes only the `/velodyne_points` topic and can be downloaded from [here][long_corridor_link]

[long_corridor_original_link]: https://superodometry.com/iccv23_challenge_Mul
[long_corridor_link]: https://postechackr-my.sharepoint.com/:u:/g/personal/daehanlee_postech_ac_kr/EduGeuaT5ypBvSQY539XFEgBRSsQSUopeNjalk9jJqyq5Q?e=yFaLEN

Alternatively, you can download it using the following command:

```sh
wget https://postechackr-my.sharepoint.com/:u:/g/personal/daehanlee_postech_ac_kr/EduGeuaT5ypBvSQY539XFEgBRSsQSUopeNjalk9jJqyq5Q?e=2IM8ed&download=1 -O subt_mrs_long_corridor.zip
```

2. **Exp07** Long Corridor sequence of HILTI-Oxford dataset

```sh
roslaunch genz_icp odometry.launch topic:=/hesai/pandar config_file:=exp07.yaml
```
```sh
rosbag play exp07_long_corridor.bag
```

The bagfile for the **Exp07** Long Corridor sequence of HILTI-Oxford dataset can be downloaded from [here][exp07_link]

[exp07_link]: https://hilti-challenge.com/dataset-2022.html

3. **Corridor1** and **Corridor2** sequences of Ground-Challenge dataset

```sh
roslaunch genz_icp odometry.launch topic:=/velodyne_points config_file:=corridor.yaml
```
```sh
rosbag play corridor1.bag && rosbag play corridor2.bag
```

The bagfile for the **Corridor1** and **Corridor2** sequences of Ground-Challenge dataset can be downloaded from [here][ground_challenge_link]

[ground_challenge_link]: https://github.com/sjtuyinjie/Ground-Challenge

4. **short experiment** sequence of Newer-College dataset

```sh
roslaunch genz_icp odometry.launch topic:=/os1_cloud_node/points config_file:=newer_college.yaml
```
```sh
rosbag play rooster_2020-03-10-10-36-30_0.bag && rosbag play rooster_2020-03-10-10-39-18_1.bag && rosbag play rooster_2020-03-10-10-42-05_2.bag && rosbag play rooster_2020-03-10-10-44-52_3.bag && rosbag play rooster_2020-03-10-10-47-39_4.bag && rosbag play rooster_2020-03-10-10-50-26_5.bag && rosbag play rooster_2020-03-10-10-53-13_6.bag && rosbag play rooster_2020-03-10-10-56-00_7.bag && rosbag play rooster_2020-03-10-10-58-47_8.bag && rosbag play rooster_2020-03-10-11-01-34_9.bag
```

The bagfile for the **short experiment** sequence of Newer-College dataset can be downloaded from [here][newer_college_link]

[newer_college_link]: https://ori-drs.github.io/newer-college-dataset/

5. **Seq. 00** of KITTI odometry dataset

```sh
roslaunch genz_icp odometry.launch topic:=/velodyne_points config_file:=kitti.yaml
```
```sh
rosbag play 00.bag
```

The bagfile for the **Seq. 00** of KITTI odometry dataset can be downloaded from [here][00_link]

[00_link]: https://postechackr-my.sharepoint.com/:u:/g/personal/daehanlee_postech_ac_kr/EVxChaGoOiJIr5vTy44DNUIBlAs4Mbld5wj94qJYyWwAKg?e=QjhpLE

Alternatively, you can download it using the following command:

```sh
wget https://postechackr-my.sharepoint.com/:u:/g/personal/daehanlee_postech_ac_kr/EVxChaGoOiJIr5vTy44DNUIBlAs4Mbld5wj94qJYyWwAKg?e=ZPE6JT&download=1 -O 00.zip
```

If you need bag files for other sequences, you can download the original files from [here][kitti_link] and convert them using [lidar2rosbag_kitti][lidar2rosbag_kitti_link]

[kitti_link]: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
[lidar2rosbag_kitti_link]: https://github.com/AbnerCSZ/lidar2rosbag_KITTI

#### Option 2

Otherwise, the only required argument to provide is the **topic name**:

```sh
roslaunch genz_icp odometry.launch topic:=<topic_name>
```
```sh
rosbag play <rosbag_file_name>.bag
```

Check out the tuning guide for the parameters of GenZ-ICP at this [link][tuning_guide_link]

[tuning_guide_link]: https://github.com/cocel-postech/genz-icp/blob/master/ros/config/parameter_tuning_guide.md

### ROS2

#### How to build

You should not need any extra dependency, just clone and build:
    
```sh
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/cocel-postech/genz-icp.git
cd ..
colcon build --packages-select genz_icp --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source ~/colcon_ws/install/setup.bash
```

#### How to convert ROS1 bag files to ROS2 bag files (.db3)

To use above demo datasets with ROS2, they must be converted to either `.db3` (SQLite) or `.mcap` format.

1. Install the rosbags conversion tool:
```sh
pip3 install rosbags
```

2. Convert the ROS1 bag file to ROS2 format:
```sh
rosbags-convert --src <rosbag_file_name>.bag --dst <rosbag_file_name>
```

The generated <rosbag_file_name> folder will contain a `.db3` (SQLite) file and a corresponding metadata `.yaml` file.

#### How to run

#### Option 1

If you want to use a pre-tuned parameter set, you need to provide the **config file** with the **topic name** as arguments:

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name> config_file:=<config_file_name>.yaml
```
```sh
ros2 bag play <rosbag_file_name>.db3
```

#### Option 2

Otherwise, the only required argument to provide is the **topic name**:

```sh
ros2 launch genz_icp odometry.launch.py topic:=<topic_name>
```
```sh
ros2 bag play <rosbag_file_name>.db3
```

Check out the tuning guide for the parameters of GenZ-ICP at this [link][tuning_guide_link]
