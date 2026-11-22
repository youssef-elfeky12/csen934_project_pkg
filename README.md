# CSEN934 Project – Milestone 01

## One-time setup (already done on this machine)

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-cartographer-ros ros-humble-pointcloud-to-laserscan \
  ros-humble-gtsam ros-humble-cv-bridge ros-humble-pcl-msgs ros-humble-tf2-sensor-msgs ros-humble-xacro
cd ~/AS/ros2_ws/src
ln -s ~/AS/csen934_project_pkg .
git clone -b ros2 https://github.com/TixiaoShan/LIO-SAM.git
cd ~/AS/ros2_ws && colcon build
```

In **every** terminal first run:

```bash
source /opt/ros/humble/setup.bash && source ~/AS/ros2_ws/install/setup.bash
```

## 1. Webots, control, noise, EKF

| Terminal | Command |
|---|---|
| 1 | `webots ~/AS/csen934_project_pkg/Webots_Worlds_Controller/Worlds/city.wbt` then press ▶ |
| 2 | `ros2 run csen934_project_pkg control.py` (10 m/s, 0 rad) |
| 3 | `ros2 run csen934_project_pkg localization_add_noise.py` |
| 4 | `ros2 run csen934_project_pkg localization_ekf_node.py` |

## 2. Plots, graph, bag

| Terminal | Command | Shows |
|---|---|---|
| 5 | `ros2 launch csen934_project_pkg plots.launch.py` | 3 rqt_plot windows: x, y, yaw — GT vs Noisy vs Filtered |
| 6 | `rqt_graph` | all nodes and topics |
| 7 | `ros2 bag record -o csen934_bag /Car_Odom_GT /Car_Odom_Noisy /Car_Odom_Filtered /Car_Ackrm /Car_Imu /Car_Scan /tf /tf_static /clock` | ROS bag (Ctrl+C to stop) |

`control.py` drives straight, so the car stops at the barrier of the first curve after ~10 s.

## 3. SLAM trials (one at a time)

Keep Webots, the noise node and the EKF running. Stop `control.py` (Ctrl+C), then for each trial:

1. Reset the car: `ros2 service call /Car_Reset std_srvs/srv/Trigger`
2. Start the trial (opens RViz with the live map and trajectories):

   | Trial | Launch |
   |---|---|
   | 2D #1 – slam_toolbox (3D lidar → pointcloud_to_laserscan) | `ros2 launch csen934_project_pkg slam_toolbox.launch.py` |
   | 2D #2 – Cartographer (3D lidar → pointcloud_to_laserscan) | `ros2 launch csen934_project_pkg cartographer.launch.py` |
   | 3D – LIO-SAM (3D lidar + IMU) | `ros2 launch csen934_project_pkg lio_sam.launch.py` |

3. Drive one lap (a ~1 km figure-8 through both loops of the city): `ros2 run csen934_project_pkg autopilot --ros-args -p laps:=1` (wait for `Lap 1 done`)
4. Save the map:

   | Trial | Command |
   |---|---|
   | slam_toolbox | `ros2 run csen934_project_pkg save_map --ros-args -p name:=slam_toolbox` |
   | Cartographer | `ros2 run csen934_project_pkg save_map --ros-args -p name:=cartographer` |
   | LIO-SAM | `ros2 run csen934_project_pkg save_map --ros-args -p name:=lio_sam -p topic:=/lio_sam/mapping/map_global` |

5. Ctrl+C the launch (and the autopilot).

RViz colours: green = ground truth path, blue = EKF path, red = SLAM path.

## 4. Compare the SLAM outputs

```bash
ros2 launch csen934_project_pkg compare.launch.py
```

Shows the three saved maps overlaid in one RViz view.
