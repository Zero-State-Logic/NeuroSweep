#!/usr/bin/env python3
"""
NeuroSweep - Bayesian Dirt Inference Node (v2, robust)
======================================================
Fixes vs v1:
  * Survives the SLAM map changing size mid-run (v1 crashed on this).
  * More robust robot-position lookup; warns clearly if the pose is FROZEN
    so you know why nothing is cooling.
  * Faster-visible dirt so colours show up quickly in RViz.

Concept (unchanged):
  - Grid mirrors the real SLAM /map (resolution, size, origin).
  - Free cells accumulate dirt:   P <- P + (P_max - P)*(1 - exp(-lambda*dt))
  - Cells under the robot clean:   P <- P * exp(-mu*dt)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

import tf2_ros


class DirtInferenceNode(Node):
    def __init__(self):
        super().__init__('dirt_inference_node')

        self.declare_parameter('accumulation_rate', 0.02)
        self.declare_parameter('cleaning_rate', 2.0)
        self.declare_parameter('p_max', 1.0)
        self.declare_parameter('robot_radius_m', 0.35)
        self.declare_parameter('tick_period_s', 0.5)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        self.lam = self.get_parameter('accumulation_rate').value
        self.mu = self.get_parameter('cleaning_rate').value
        self.p_max = self.get_parameter('p_max').value
        self.robot_radius = self.get_parameter('robot_radius_m').value
        self.dt = self.get_parameter('tick_period_s').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        self.grid_ready = False
        self.resolution = None
        self.width = None
        self.height = None
        self.origin_x = None
        self.origin_y = None
        self.dirt = None
        self.free_mask = None

        self.last_robot_cell = None
        self.stale_count = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        heat_qos = QoSProfile(depth=1)
        heat_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.heat_pub = self.create_publisher(
            OccupancyGrid, '/neuro_sweep/heatmap', heat_qos)

        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info('Dirt Inference Node v2 started. Waiting for /map...')

    def map_callback(self, msg: OccupancyGrid):
        new_res = msg.info.resolution
        new_w = msg.info.width
        new_h = msg.info.height
        new_ox = msg.info.origin.position.x
        new_oy = msg.info.origin.position.y

        occ = np.array(msg.data, dtype=np.int8).reshape(new_h, new_w)
        new_free = (occ == 0)

        if self.dirt is None:
            self.dirt = np.zeros((new_h, new_w), dtype=np.float32)
            self.dirt[new_free] = np.random.rand(np.count_nonzero(new_free)) * 0.2
        elif (new_h, new_w) != self.dirt.shape or new_ox != self.origin_x or new_oy != self.origin_y:
            new_dirt = np.zeros((new_h, new_w), dtype=np.float32)
            new_dirt[new_free] = np.random.rand(np.count_nonzero(new_free)) * 0.2
            if self.resolution is not None and abs(new_res - self.resolution) < 1e-9:
                off_c = int(round((self.origin_x - new_ox) / new_res))
                off_r = int(round((self.origin_y - new_oy) / new_res))
                oh, ow = self.dirt.shape
                dr0 = max(0, off_r); dr1 = min(new_h, off_r + oh)
                dc0 = max(0, off_c); dc1 = min(new_w, off_c + ow)
                sr0 = dr0 - off_r; sr1 = sr0 + (dr1 - dr0)
                sc0 = dc0 - off_c; sc1 = sc0 + (dc1 - dc0)
                if dr1 > dr0 and dc1 > dc0:
                    new_dirt[dr0:dr1, dc0:dc1] = self.dirt[sr0:sr1, sc0:sc1]
            self.dirt = new_dirt

        self.resolution = new_res
        self.width = new_w
        self.height = new_h
        self.origin_x = new_ox
        self.origin_y = new_oy
        self.free_mask = new_free
        self.grid_ready = True

    def get_robot_cell(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except Exception:
            return None
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        col = int((rx - self.origin_x) / self.resolution)
        row = int((ry - self.origin_y) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return (row, col)
        return None

    def update(self):
        if not self.grid_ready:
            return

        gain = (1.0 - math.exp(-self.lam * self.dt))
        self.dirt[self.free_mask] += (self.p_max - self.dirt[self.free_mask]) * gain

        robot = self.get_robot_cell()
        clean_str = " | NO ROBOT POSE"
        if robot is not None:
            row, col = robot
            if robot == self.last_robot_cell:
                self.stale_count += 1
            else:
                self.stale_count = 0
            self.last_robot_cell = robot

            r_cells = max(1, int(self.robot_radius / self.resolution))
            decay = math.exp(-self.mu * self.dt)
            r0, r1 = max(0, row - r_cells), min(self.height, row + r_cells + 1)
            c0, c1 = max(0, col - r_cells), min(self.width, col + r_cells + 1)
            for rr in range(r0, r1):
                for cc in range(c0, c1):
                    if (rr - row) ** 2 + (cc - col) ** 2 <= r_cells ** 2:
                        self.dirt[rr, cc] *= decay

            if self.stale_count > 6:
                clean_str = f" | cleaning ({row},{col}) [POSE FROZEN {self.stale_count}x - is SLAM updating?]"
            else:
                clean_str = f" | cleaning ({row},{col})"

        self.publish_heatmap()

        masked = np.where(self.free_mask, self.dirt, -1.0)
        idx = int(np.argmax(masked))
        max_val = float(masked.flat[idx])
        mr, mc = np.unravel_index(idx, masked.shape)
        self.get_logger().info(f'Max dirt P={max_val:.2f} at ({mr},{mc}){clean_str}')

    def publish_heatmap(self):
        grid = OccupancyGrid()
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0

        data = np.full((self.height, self.width), -1, dtype=np.int8)
        vals = np.clip(self.dirt[self.free_mask] * 100.0, 0, 100).astype(np.int8)
        data[self.free_mask] = vals
        grid.data = data.flatten().tolist()
        self.heat_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = DirtInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
