# csen934_project_pkg

CSEN934 Autonomous Systems project: drive a Webots BMW X5 from ROS 2 (Humble).

The Webots controller `CSEN934_Test_OL_Sensors` subscribes to `AckermannDrive` commands and publishes the car's ground-truth odometry:

| Topic | Type | Direction |
|---|---|---|
| `/Car_Ackrm` | `ackermann_msgs/AckermannDrive` | ROS → Webots (speed in m/s, steering in rad, + = left) |
| `/Car_Odom_GT` | `nav_msgs/Odometry` | Webots → ROS (GPS position, IMU orientation, gyro rates) |
| `/Car_Reset` | `std_srvs/Trigger` (service) | Puts the car back at its start pose, at rest |

## How to run it

Put the package in a workspace and build it:

```bash
ln -s ~/AS/csen934_project_pkg ~/AS/ros2_ws/src/
cd ~/AS/ros2_ws && colcon build --packages-select csen934_project_pkg
```

**Terminal 1:** start Webots from a terminal where ROS is sourced, so its Python controller can import `rclpy`. Then press play.

```bash
source /opt/ros/humble/setup.bash
webots ~/AS/csen934_project_pkg/Webots_Worlds_Controller/Worlds/city.wbt
```

**Terminal 2:** source both setups, then run **one** of the two:

```bash
source /opt/ros/humble/setup.bash && source ~/AS/ros2_ws/install/setup.bash

# Test 1: teleop (graphical window, see controls below)
ros2 run csen934_project_pkg teleop

# Test 2: constant velocity (defaults: 10 m/s, straight)
ros2 run csen934_project_pkg control
ros2 run csen934_project_pkg control --ros-args -p speed:=5.0 -p steering_angle:=0.1
```

Don't run teleop and control at the same time: both publish on `/Car_Ackrm`.

To check that the car is alive: `ros2 topic echo /Car_Odom_GT` should print a changing position.

## Teleop controls

The `teleop` window shows a speedometer (target and actual speed), the steering wheel, a live trajectory map and telemetry.

| Key | Action |
|---|---|
| `W` / `↑` (hold) | Speed up. Release to keep cruising at that speed |
| `S` / `↓` (hold) | Brake to 0. Release and press again to reverse |
| `A` `D` / `←` `→` (hold) | Steer. Re-centers on release (can be switched off) |
| `Space` | Stop |
| `X` | Center steering |
| `R` | Reset the car to its start pose |

The sliders and the 0 / 5 / 10 / 15 / 20 m/s buttons set speed and steering with the mouse. Closing the window or pressing Ctrl+C in the terminal stops the car.

Optional parameters: `max_speed` (20.0), `max_reverse` (5.0), `max_steer` (0.5), `accel` (4.0 m/s per second), `steer_rate` (0.8 rad/s). Example:

```bash
ros2 run csen934_project_pkg teleop --ros-args -p max_speed:=15.0 -p accel:=6.0
```

The original keyboard-only terminal version is still available as `ros2 run csen934_project_pkg teleop_cli` (w/s speed, a/d steer, x center, space stop, r reset, q quit).

## Resetting the car

The car can be put back at its start pose (the pose in `city.wbt`) without restarting Webots, in any of these ways:

- **Teleop window:** press `R` or click **Reset**. It stops the car first.
- **Terminal teleop:** press `r`.
- **Any terminal:**
  ```bash
  ros2 service call /Car_Reset std_srvs/srv/Trigger
  ```

If `control` is running, the car starts driving again right after the reset, since that node keeps publishing its constant command.

The service moves the car through the Webots Supervisor API, so the `BmwX5` node in `city.wbt` has `supervisor TRUE`. To reset the whole simulation instead (all objects, sim time back to 0), use *File → Reset Simulation* (Ctrl+Shift+T) in Webots.

## Troubleshooting

- **No car in Webots / `/Car_Odom_GT` missing.** Webots R2025a downloads the car and city models from GitHub the first time. If the console shows `Error downloading EXTERNPROTO ...`, use *File → Reload World* (don't save) until the errors are gone. Downloads are cached, so each reload only retries what is missing.
- **`ModuleNotFoundError: No module named 'rclpy'` in the Webots console.** Webots was started without sourcing ROS. Close it and start it again as in Terminal 1.
