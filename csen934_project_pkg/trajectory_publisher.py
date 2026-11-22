"""Vehicle trajectories for RViz, as nav_msgs/Path.

  /Path_GT        from /Car_Odom_GT        (ground truth)
  /Path_Filtered  from /Car_Odom_Filtered  (EKF)
  /Path_SLAM      pose of `slam_base_frame` in `slam_map_frame`, read from TF
                  (the SLAM estimate; disabled if slam_map_frame is empty)
"""
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from csen934_project_pkg.node_runner import run

MIN_STEP = 0.3   # m between stored poses


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_publisher")
        self.map_frame = self.declare_parameter("slam_map_frame", "").value
        self.base_frame = self.declare_parameter("slam_base_frame", "base_link").value
        self.paths = {}
        self.pubs = {}
        for name, topic in (("GT", "/Car_Odom_GT"), ("Filtered", "/Car_Odom_Filtered")):
            self.pubs[name] = self.create_publisher(Path, f"/Path_{name}", 1)
            self.create_subscription(Odometry, topic,
                                     lambda msg, n=name: self.on_odom(n, msg), 10)
        if self.map_frame:
            self.pubs["SLAM"] = self.create_publisher(Path, "/Path_SLAM", 1)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(0.2, self.on_tf)
        self.create_timer(0.5, self.publish_all)

    def add(self, name, frame, stamp, x, y, z, q):
        path = self.paths.setdefault(name, Path())
        if path.poses:
            last = path.poses[-1].pose.position
            if math.hypot(x - last.x, y - last.y) < MIN_STEP:
                return
            if math.hypot(x - last.x, y - last.y) > 20.0:   # car was reset: start over
                path.poses.clear()
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = stamp
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
        ps.pose.orientation = q
        path.header.frame_id = frame
        path.header.stamp = stamp
        path.poses.append(ps)

    def on_odom(self, name, msg):
        p = msg.pose.pose.position
        self.add(name, msg.header.frame_id, msg.header.stamp, p.x, p.y, p.z,
                 msg.pose.pose.orientation)

    def on_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:      # SLAM not publishing yet
            return
        tr = t.transform.translation
        self.add("SLAM", self.map_frame, t.header.stamp, tr.x, tr.y, tr.z, t.transform.rotation)

    def publish_all(self):
        for name, path in self.paths.items():
            self.pubs[name].publish(path)


def main():
    run(TrajectoryPublisher)


if __name__ == "__main__":
    main()
