"""Webots controller of the BMW X5, running the ROS 2 node "Tesla_Node".

Publishes (stamped with simulation time):
  /Car_Odom_GT     nav_msgs/Odometry        ground truth, every simulation step:
                                             pose = GPS position + IMU orientation,
                                             twist = GPS velocity + gyro rates (in base_link)
  /Car_Imu         sensor_msgs/Imu          IMU (orientation, gyro, accelerometer)
  /Car_Scan        sensor_msgs/LaserScan    SICK LMS 291 front 2D lidar, 10 Hz
  /Car_PointCloud  sensor_msgs/PointCloud2  16-layer 360 deg roof lidar, 10 Hz
  /clock           rosgraph_msgs/Clock      simulation time (for use_sim_time)
  /tf_static       base_link -> laser_front, lidar_3d, imu_link
Subscribes:
  /Car_Ackrm       ackermann_msgs/AckermannDrive  speed (m/s), steering angle (rad, + = left)
Service:
  /Car_Reset       std_srvs/Trigger         put the car back at its start pose
"""
import ctypes
import math

import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from controller.wb import wb          # raw Webots C API, for fast point-cloud access
from vehicle import Driver

from Define_Sensors import define_sensors
from Helping_Functions import euler_to_quaternion

# Sensor positions relative to base_link (the car origin = rear-axle centre, where the GPS
# is), from the BmwX5 sensor slots plus the translations used in city.wbt.
SENSOR_FRAMES = {
    "laser_front": (3.85 + 0.06, 0.0, 0.45),   # sensorsSlotFront + SickLms291
    "lidar_3d": (1.0, 0.0, 1.45 + 0.25),       # sensorsSlotTop + Lidar
    "imu_link": (0.0, 0.0, 0.0),               # sensorsSlotCenter
}

# Webots point record: float x, y, z; int layer; float time (20 bytes)
WB_POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                     ("layer", "<i4"), ("time", "<f4")])
# Velodyne-style point (x y z intensity ring time), the layout LIO-SAM expects
CLOUD_POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                        ("ring", "<u2"), ("time", "<f4")], align=True)
CLOUD_FIELDS = [
    PointField(name="x", offset=CLOUD_POINT.fields["x"][1], datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=CLOUD_POINT.fields["y"][1], datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=CLOUD_POINT.fields["z"][1], datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=CLOUD_POINT.fields["intensity"][1],
               datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=CLOUD_POINT.fields["ring"][1],
               datatype=PointField.UINT16, count=1),
    PointField(name="time", offset=CLOUD_POINT.fields["time"][1],
               datatype=PointField.FLOAT32, count=1),
]


def rotation_matrix(roll, pitch, yaw):
    """Body -> world rotation for Z-Y-X Euler angles."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class TeslaNode(Node):
    def __init__(self, driver, sensors):
        super().__init__("Tesla_Node")
        self.driver = driver
        self.s = sensors
        self.speed = 0.0   # m/s
        self.steer = 0.0   # rad

        self.odom_pub = self.create_publisher(Odometry, "/Car_Odom_GT", 10)
        self.imu_pub = self.create_publisher(Imu, "/Car_Imu", 50)
        self.scan_pub = self.create_publisher(LaserScan, "/Car_Scan", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/Car_PointCloud", 5)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.create_subscription(AckermannDrive, "/Car_Ackrm", self.cmd_callback, 10)
        self.create_service(Trigger, "/Car_Reset", self.reset_callback)

        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        # Remember the start pose so /Car_Reset can put the car back there.
        # Moving the car needs "supervisor TRUE" on the BmwX5 node in the world file.
        self.car = driver.getSelf() if driver.getSupervisor() else None
        if self.car is None:
            self.get_logger().warn("Car is not a supervisor: /Car_Reset is disabled")
        else:
            self.start_translation = self.car.getField("translation").getSFVec3f()
            self.start_rotation = self.car.getField("rotation").getSFRotation()

    # ------------------------------------------------------------------ inputs
    def cmd_callback(self, msg):
        self.speed = msg.speed
        self.steer = msg.steering_angle

    def reset_callback(self, request, response):
        """Put the car back at its start pose, at rest."""
        if self.car is None:
            response.success = False
            response.message = "Set 'supervisor TRUE' on the BmwX5 node to enable reset"
            return response
        self.speed = 0.0
        self.steer = 0.0
        self.driver.setCruisingSpeed(0.0)
        self.driver.setSteeringAngle(0.0)
        self.car.getField("translation").setSFVec3f(list(self.start_translation))
        self.car.getField("rotation").setSFRotation(list(self.start_rotation))
        self.car.resetPhysics()   # clear all velocities of the car and its wheels
        response.success = True
        response.message = "Car reset to its start pose"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------ outputs
    def publish_static_tf(self):
        transforms = []
        for frame, (x, y, z) in SENSOR_FRAMES.items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.static_tf.sendTransform(transforms)

    def publish_step(self, sim_time):
        """Called once per simulation step."""
        sec = int(sim_time)
        stamp = Clock().clock
        stamp.sec = sec
        stamp.nanosec = int(round((sim_time - sec) * 1e9)) % 1000000000

        clock = Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)

        roll, pitch, yaw = self.s["imu"].getRollPitchYaw()
        quat = euler_to_quaternion(roll, pitch, yaw)
        self.publish_odom(stamp, quat, rotation_matrix(roll, pitch, yaw))
        self.publish_imu(stamp, quat)

        # Lidars only produce a new scan every lidar period
        if int(round(sim_time * 1000)) % self.s["lidar_period_ms"] == 0:
            if self.s["scan"] is not None:
                self.publish_scan(stamp)
            if self.s["lidar_3d"] is not None:
                self.publish_cloud(stamp)

    def publish_odom(self, stamp, quat, rot):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Pose: position from GPS, orientation from IMU
        x, y, z = self.s["gps"].getValues()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = quat

        # Twist (expressed in base_link, as nav_msgs/Odometry requires):
        # linear velocity from GPS (world frame -> rotated into the car frame),
        # angular velocity from the gyro (already in the car frame)
        vx, vy, vz = rot.T @ np.array(self.s["gps"].getSpeedVector())
        wx, wy, wz = self.s["gyro"].getValues()
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.linear.z = float(vz)
        msg.twist.twist.angular.x = wx
        msg.twist.twist.angular.y = wy
        msg.twist.twist.angular.z = wz

        self.odom_pub.publish(msg)  # covariances stay zero (ground truth)

    def publish_imu(self, stamp, quat):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_link"
        (msg.orientation.x, msg.orientation.y,
         msg.orientation.z, msg.orientation.w) = quat
        (msg.angular_velocity.x, msg.angular_velocity.y,
         msg.angular_velocity.z) = self.s["gyro"].getValues()
        if self.s["accelerometer"] is not None:
            (msg.linear_acceleration.x, msg.linear_acceleration.y,
             msg.linear_acceleration.z) = self.s["accelerometer"].getValues()
        msg.orientation_covariance[0::4] = [1e-6] * 3
        msg.angular_velocity_covariance[0::4] = [1e-6] * 3
        msg.linear_acceleration_covariance[0::4] = [1e-4] * 3
        self.imu_pub.publish(msg)

    def publish_scan(self, stamp):
        lidar = self.s["scan"]
        n = lidar.getHorizontalResolution()
        fov = lidar.getFov()
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "laser_front"
        msg.angle_min = -fov / 2.0
        msg.angle_max = fov / 2.0
        msg.angle_increment = fov / (n - 1) if fov < 2 * math.pi - 1e-3 else fov / n
        msg.scan_time = self.s["lidar_period_ms"] / 1000.0
        msg.range_min = lidar.getMinRange()
        msg.range_max = lidar.getMaxRange()
        # Webots orders the range image left -> right; ROS scans go right -> left
        msg.ranges = [float(r) for r in reversed(lidar.getRangeImage())]
        self.scan_pub.publish(msg)

    def publish_cloud(self, stamp):
        lidar = self.s["lidar_3d"]
        n = lidar.getNumberOfPoints()
        raw = np.frombuffer(
            ctypes.string_at(wb.wb_lidar_get_point_cloud(lidar._tag), n * WB_POINT.itemsize),
            dtype=WB_POINT)
        raw = raw[np.isfinite(raw["x"]) & np.isfinite(raw["y"]) & np.isfinite(raw["z"])]

        cloud = np.zeros(raw.shape[0], dtype=CLOUD_POINT)
        cloud["x"], cloud["y"], cloud["z"] = raw["x"], raw["y"], raw["z"]
        cloud["ring"] = raw["layer"].astype(np.uint16)
        # A "fixed" Webots lidar captures the whole cloud at one instant (no rotation
        # distortion), so every point's time offset from the header stamp is 0.
        # (Webots fills its own time field with absolute sim time, which LIO-SAM would read
        # as a scan lasting hundreds of seconds.)
        cloud["time"] = 0.0

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "lidar_3d"
        msg.height = 1
        msg.width = cloud.shape[0]
        msg.fields = CLOUD_FIELDS
        msg.is_bigendian = False
        msg.point_step = CLOUD_POINT.itemsize
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = cloud.tobytes()
        self.cloud_pub.publish(msg)


def main():
    driver = Driver()
    timestep = int(driver.getBasicTimeStep())
    sensors = define_sensors(driver, timestep)

    rclpy.init()
    node = TeslaNode(driver, sensors)

    while driver.step() != -1:
        rclpy.spin_once(node, timeout_sec=0)       # process incoming commands
        driver.setCruisingSpeed(node.speed * 3.6)  # m/s -> km/h
        driver.setSteeringAngle(-node.steer)       # ROS: +left, Webots: +right
        node.publish_step(driver.getTime())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
