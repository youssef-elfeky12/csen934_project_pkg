"""Save the current SLAM map (in the odom / world frame) for the comparison view.

  2D: ros2 run csen934_project_pkg save_map --ros-args -p name:=slam_toolbox
      /map (OccupancyGrid) -> ~/csen934_maps/slam_toolbox.pgm + .yaml
  3D: ros2 run csen934_project_pkg save_map --ros-args -p name:=lio_sam \
          -p topic:=/lio_sam/mapping/map_global
      PointCloud2 -> ~/csen934_maps/lio_sam.pcd
"""
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of_matrix(m):
    return math.atan2(m[1, 0], m[0, 0])


class SaveMap(Node):
    def __init__(self):
        super().__init__("save_map")
        self.name = self.declare_parameter("name", "slam_toolbox").value
        self.topic = self.declare_parameter("topic", "/map").value
        self.frame = self.declare_parameter("frame", "odom").value
        self.dir = os.path.expanduser(self.declare_parameter("dir", "~/csen934_maps").value)
        self.msg = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for_message(self, timeout=20.0):
        end = time.monotonic() + timeout
        msg_type = None
        while time.monotonic() < end and msg_type is None:
            for name, types in self.get_topic_names_and_types():
                if name == self.topic:
                    msg_type = (OccupancyGrid if "OccupancyGrid" in types[0] else PointCloud2)
            rclpy.spin_once(self, timeout_sec=0.2)
        if msg_type is None:
            raise RuntimeError(f"Topic {self.topic} not found. Is the SLAM node running?")
        # Grids: reliable (large messages); clouds: best effort (LIO-SAM publishes best effort)
        qos = 1 if msg_type is OccupancyGrid else qos_profile_sensor_data
        self.create_subscription(msg_type, self.topic, lambda m: setattr(self, "msg", m), qos)
        while time.monotonic() < end and self.msg is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.msg is None:
            raise RuntimeError(f"No message on {self.topic} within {timeout:.0f} s")
        return self.msg

    def transform_to_frame(self, source_frame):
        end = time.monotonic() + 5.0
        while True:
            try:
                t = self.tf_buffer.lookup_transform(self.frame, source_frame, Time())
                tr = t.transform.translation
                return np.array([tr.x, tr.y, tr.z]), quat_to_matrix(t.transform.rotation)
            except Exception:
                if time.monotonic() > end:
                    raise
                rclpy.spin_once(self, timeout_sec=0.1)

    def save(self):
        os.makedirs(self.dir, exist_ok=True)
        msg = self.wait_for_message()
        p, R = self.transform_to_frame(msg.header.frame_id)
        if isinstance(msg, OccupancyGrid):
            return self.save_grid(msg, p, R)
        return self.save_cloud(msg, p, R)

    def save_grid(self, msg, p, R):
        info = msg.info
        o = info.origin
        R_o = quat_to_matrix(o.orientation)
        origin = p + R @ np.array([o.position.x, o.position.y, o.position.z])
        yaw = yaw_of_matrix(R @ R_o)
        data = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        img = np.full(data.shape, 205, dtype=np.uint8)     # unknown
        img[(data >= 0) & (data <= 25)] = 254               # free
        img[data >= 65] = 0                                 # occupied
        img = np.flipud(img)                                # image row 0 = top (max y)
        pgm = os.path.join(self.dir, f"{self.name}.pgm")
        with open(pgm, "wb") as f:
            f.write(f"P5\n{info.width} {info.height}\n255\n".encode())
            f.write(img.tobytes())
        with open(os.path.join(self.dir, f"{self.name}.yaml"), "w") as f:
            f.write(f"image: {self.name}.pgm\nresolution: {info.resolution}\n"
                    f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, {yaw:.6f}]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        return pgm

    def save_cloud(self, msg, p, R):
        offsets = {f.name: f.offset for f in msg.fields}
        dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                          "offsets": [offsets["x"], offsets["y"], offsets["z"]],
                          "itemsize": msg.point_step})
        raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
        pts = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)] @ R.T + p
        pcd = os.path.join(self.dir, f"{self.name}.pcd")
        with open(pcd, "w") as f:
            f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                    f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\n"
                    "DATA ascii\n")
            np.savetxt(f, pts, fmt="%.3f")
        return pcd


def main():
    rclpy.init()
    node = SaveMap()
    try:
        path = node.save()
        node.get_logger().info(f"Saved {path}")
    except Exception as e:     # noqa: BLE001  (report any failure to the user)
        node.get_logger().error(str(e))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
