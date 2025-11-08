#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose

from .occupancy_grid_manager import OccupancyGridManager


class SensorManager:
    """Samler ROS-sensorer og eksponerer enkle hjelpere."""

    def __init__(self, node_ref):
        self.node = node_ref

        self.latest_scan = None
        self.latest_odom = None
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0

        self.current_marker_id = None
        self.aruco_callback = None

        self.occupancy_grid_manager = OccupancyGridManager(node_ref)

        self._setup_subscribers()

    # --- ROS subscriptions -------------------------------------------------

    def _setup_subscribers(self):
        self.node.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.node.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.node.create_subscription(Int64, 'marker_id', self.marker_id_callback, 10)
        self.node.create_subscription(Pose, 'marker_map_pose', self.marker_pose_callback, 10)

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_orientation = math.atan2(siny_cosp, cosy_cosp)

    def marker_id_callback(self, msg: Int64):
        self.current_marker_id = msg.data

    def marker_pose_callback(self, msg: Pose):
        if self.current_marker_id is None or self.aruco_callback is None:
            return

        try:
            marker_id = int(self.current_marker_id)
            position = (msg.position.x, msg.position.y)
            self.aruco_callback(marker_id, position)
        finally:
            self.current_marker_id = None

    # --- Access helpers ----------------------------------------------------

    def get_latest_scan(self):
        return self.latest_scan

    def is_scan_valid(self) -> bool:
        return self.latest_scan is not None and bool(self.latest_scan.ranges)

    def get_range_at_angle(self, angle_deg: float, default: float = 100.0) -> float:
        scan = self.latest_scan
        if scan is None or not scan.ranges:
            return default

        rad = math.radians(angle_deg)
        idx = int(round((rad - scan.angle_min) / scan.angle_increment))
        if idx < 0 or idx >= len(scan.ranges):
            return default

        distance = scan.ranges[idx]
        if distance is None or math.isinf(distance) or math.isnan(distance) or distance == 0.0:
            return default
        return float(distance)

    def get_robot_position(self) -> tuple:
        return self.robot_position

    def get_robot_orientation(self) -> float:
        return self.robot_orientation

    # --- Map helpers -------------------------------------------------------

    def get_occupancy_grid_manager(self):
        return self.occupancy_grid_manager

    def is_map_available(self) -> bool:
        return self.occupancy_grid_manager.is_map_available()

    def get_map_info(self) -> dict:
        return self.occupancy_grid_manager.get_map_info()

