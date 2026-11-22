"""Publish every saved map (see save_map) at once, in the odom frame, for RViz comparison.

  ~/csen934_maps/<name>.yaml -> /compare/<name>  nav_msgs/OccupancyGrid
  ~/csen934_maps/<name>.pcd  -> /compare/<name>  sensor_msgs/PointCloud2
"""
import glob
import math
import os

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField

from csen934_project_pkg.node_runner import run

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
GROUND_CUT = 0.5   # m above the ground (world z = 0)


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:                       # magic, width, height, maxval
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    width, height = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[pos + 1:pos + 1 + width * height], np.uint8).reshape(height, width)


def read_yaml(path):
    out = {}
    for line in open(path):
        if ":" in line:
            key, value = line.split(":", 1)
            out[key.strip()] = value.strip()
    return out


class MapCompare(Node):
    def __init__(self):
        super().__init__("map_compare")
        folder = os.path.expanduser(self.declare_parameter("dir", "~/csen934_maps").value)
        self.keep = []
        for yaml_path in sorted(glob.glob(os.path.join(folder, "*.yaml"))):
            self.publish_grid(yaml_path)
        for pcd_path in sorted(glob.glob(os.path.join(folder, "*.pcd"))):
            self.publish_cloud(pcd_path)
        if not self.keep:
            self.get_logger().warn(f"No saved maps in {folder} (use save_map first)")

    def publisher(self, msg_type, path):
        name = os.path.splitext(os.path.basename(path))[0]
        pub = self.create_publisher(msg_type, f"/compare/{name}", LATCHED)
        self.keep.append(pub)
        self.get_logger().info(f"/compare/{name} <- {path}")
        return pub

    def publish_grid(self, yaml_path):
        meta = read_yaml(yaml_path)
        img = np.flipud(read_pgm(os.path.join(os.path.dirname(yaml_path), meta["image"])))
        ox, oy, oyaw = (float(v) for v in meta["origin"].strip("[]").split(","))
        grid = np.full(img.shape, -1, dtype=np.int8)
        grid[img >= 250] = 0
        grid[img <= 50] = 100
        msg = OccupancyGrid()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(meta["resolution"])
        msg.info.height, msg.info.width = img.shape
        msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
        msg.info.origin.orientation.z = math.sin(oyaw / 2)
        msg.info.origin.orientation.w = math.cos(oyaw / 2)
        msg.data = grid.flatten().tolist()
        self.publisher(OccupancyGrid, yaml_path).publish(msg)

    def publish_cloud(self, pcd_path):
        with open(pcd_path) as f:
            for line in f:
                if line.startswith("DATA"):
                    break
            pts = np.loadtxt(f, dtype=np.float32, ndmin=2)
        pts = pts[pts[:, 2] > GROUND_CUT]     # drop ground returns: show buildings, trees, poles
        msg = PointCloud2()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate("xyz")]
        msg.point_step, msg.row_step = 12, 12 * len(pts)
        msg.is_dense = True
        msg.data = pts[:, :3].astype(np.float32).tobytes()
        self.publisher(PointCloud2, pcd_path).publish(msg)


def main():
    run(MapCompare)


if __name__ == "__main__":
    main()
