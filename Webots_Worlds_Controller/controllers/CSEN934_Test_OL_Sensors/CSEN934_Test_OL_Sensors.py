import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from vehicle import Driver

from Define_Sensors import define_sensors
from Helping_Functions import euler_to_quaternion


class TeslaNode(Node):
    def __init__(self):
        super().__init__("Tesla_Node")
        self.odom_pub = self.create_publisher(Odometry, "/Car_Odom_GT", 10)
        self.create_subscription(AckermannDrive, "/Car_Ackrm", self.cmd_callback, 10)
        self.speed = 0.0   # m/s
        self.steer = 0.0   # rad

    def cmd_callback(self, msg):
        self.speed = msg.speed
        self.steer = msg.steering_angle

    def publish_odom(self, sim_time, gps, gyro, imu):
        msg = Odometry()
        sec = int(sim_time)
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = int((sim_time - sec) * 1e9)
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Pose: position from GPS, orientation from IMU
        x, y, z = gps.getValues()
        roll, pitch, yaw = imu.getRollPitchYaw()
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Twist: linear velocity from GPS, angular velocity from gyro
        vx, vy, vz = gps.getSpeedVector()
        wx, wy, wz = gyro.getValues()
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        msg.twist.twist.angular.x = wx
        msg.twist.twist.angular.y = wy
        msg.twist.twist.angular.z = wz

        self.odom_pub.publish(msg)  # covariances stay zero (ground truth)


def main():
    driver = Driver()
    timestep = int(driver.getBasicTimeStep())
    gps, gyro, imu = define_sensors(driver, timestep)

    rclpy.init()
    node = TeslaNode()

    while driver.step() != -1:
        rclpy.spin_once(node, timeout_sec=0)       # process incoming commands
        driver.setCruisingSpeed(node.speed * 3.6)  # m/s -> km/h
        driver.setSteeringAngle(node.steer)
        node.publish_odom(driver.getTime(), gps, gyro, imu)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
