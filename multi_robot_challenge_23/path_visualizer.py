#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class PathVisualizer:
    """
    Single Responsibility: Visualisering av planlagt path i RViz.
    Publiserer MarkerArray på topic 'astar_path' i robotens namespace.
    """

    def __init__(self, node: Node, robot_id: str):
        self.node = node
        self.robot_id = robot_id
        self.pub = self.node.create_publisher(MarkerArray, 'astar_path', 10)

    def publish(self, path_world: list):
        marker_array = MarkerArray()

        path_ns = f"astar_path_{self.robot_id}"
        waypoint_ns = f"astar_waypoints_{self.robot_id}"

        if path_world:
            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.node.get_clock().now().to_msg()
            line_marker.ns = path_ns
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05
            if '0' in self.robot_id or '2' in self.robot_id or '4' in self.robot_id:
                line_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
            else:
                line_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)

            for wx, wy in path_world:
                p = Point()
                p.x = wx
                p.y = wy
                p.z = 0.1
                line_marker.points.append(p)
            marker_array.markers.append(line_marker)

            for i, (wx, wy) in enumerate(path_world):
                wp = Marker()
                wp.header.frame_id = 'map'
                wp.header.stamp = self.node.get_clock().now().to_msg()
                wp.ns = waypoint_ns
                wp.id = i + 1
                wp.type = Marker.SPHERE
                wp.action = Marker.ADD
                wp.pose.position.x = wx
                wp.pose.position.y = wy
                wp.pose.position.z = 0.1
                wp.scale.x = 0.2
                wp.scale.y = 0.2
                wp.scale.z = 0.2
                wp.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                marker_array.markers.append(wp)
        else:
            delete_marker = Marker()
            delete_marker.header.frame_id = 'map'
            delete_marker.header.stamp = self.node.get_clock().now().to_msg()
            delete_marker.ns = path_ns
            delete_marker.id = 0
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

            delete_wp = Marker()
            delete_wp.header.frame_id = 'map'
            delete_wp.header.stamp = self.node.get_clock().now().to_msg()
            delete_wp.ns = waypoint_ns
            delete_wp.id = 0
            delete_wp.action = Marker.DELETEALL
            marker_array.markers.append(delete_wp)

        self.pub.publish(marker_array)


