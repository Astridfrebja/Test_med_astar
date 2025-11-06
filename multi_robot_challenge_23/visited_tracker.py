#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class VisitedTracker:
    """
    Single Responsibility: Spore og visualisere besøkte celler.
    Publiserer MarkerArray på 'astar_visited'.
    """

    def __init__(self, node: Node, robot_id: str, occupancy_grid_manager):
        self.node = node
        self.robot_id = robot_id
        self.ogm = occupancy_grid_manager
        self.pub = self.node.create_publisher(MarkerArray, 'astar_visited', 10)
        self.visited = {}  # (mx,my) -> count
        # Periodisk publisering slik at RViz alltid kan se visited, uansett state
        self.timer = self.node.create_timer(0.5, self._on_timer)

    def mark_current(self, wx: float, wy: float):
        if not self.ogm or not self.ogm.is_map_available():
            return
        mx, my = self.ogm.world_to_map(wx, wy)
        key = (mx, my)
        self.visited[key] = self.visited.get(key, 0) + 1

    def publish(self):
        if not self.visited:
            return
        marker_array = MarkerArray()
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = f'astar_visited_{self.robot_id}'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.08
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=0.9)

        for (mx, my), _ in list(self.visited.items()):
            wx, wy = self.ogm.map_to_world(mx, my) if self.ogm else (0.0, 0.0)
            p = Point()
            p.x = wx + (self.ogm.map_msg.info.resolution if self.ogm and self.ogm.is_map_available() else 0.05) * 0.5
            p.y = wy + (self.ogm.map_msg.info.resolution if self.ogm and self.ogm.is_map_available() else 0.05) * 0.5
            p.z = 0.05
            m.points.append(p)
        marker_array.markers.append(m)
        self.pub.publish(marker_array)

    # --- intern timer ---
    def _on_timer(self):
        try:
            self.publish()
        except Exception:
            pass


