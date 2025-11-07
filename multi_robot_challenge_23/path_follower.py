#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PathFollower - EN ansvar: Path following og obstacle avoidance

Single Responsibility: Kun path following logikk
"""

import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class PathFollower:
    """
    Path Following Logic - Navigerer langs en planlagt rute
    
    Single Responsibility: Kun path following
    """
    
    # --- INNSTILLINGER ---
    FORWARD_SPEED = 0.3
    TURN_SPEED = 0.5
    WAYPOINT_THRESHOLD = 0.3  # meters til waypoint er nådd
    GOAL_THRESHOLD = 0.5  # meters til endelig mål er nådd
    P_GAIN = 1.5  # P-kontroll for heading
    OBSTACLE_THRESHOLD = 0.25  # meter til hindring
    BLOCKED_LIMIT = 60  # antall påfølgende sykluser med blokkert front før fallback
    
    def __init__(self, node: Node, sensor_manager=None, cmd_vel_publisher=None):
        """
        Args:
            node: ROS2 node for logging
            sensor_manager: SensorManager instance (optional)
            cmd_vel_publisher: Publisher for cmd_vel (optional, creates own if None)
        """
        self.node = node
        self.sensor_manager = sensor_manager
        
        if cmd_vel_publisher:
            self.cmd_vel_publisher = cmd_vel_publisher
        else:
            self.cmd_vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        self.blocked_counter = 0
    
    def navigate_to_waypoint(self, waypoint: tuple, robot_position: tuple, 
                            robot_orientation: float, msg: LaserScan = None) -> tuple:
        """
        Naviger mot et spesifikt waypoint med obstacle avoidance
        
        Args:
            waypoint: (x, y) world koordinater
            robot_position: (x, y) robot posisjon
            robot_orientation: Robot orientering (radians)
            msg: LaserScan data (optional)
        
        Returns:
            tuple: (linear_x, angular_z, should_fallback)
        """
        # Beregn retning til waypoint
        dx = waypoint[0] - robot_position[0]
        dy = waypoint[1] - robot_position[1]
        distance_to_waypoint = math.sqrt(dx*dx + dy*dy)
        
        # Hent sensor data for obstacle avoidance
        if self.sensor_manager and self.sensor_manager.is_scan_valid():
            dF = self.sensor_manager.get_range_at_angle(0.0)
            dL = self.sensor_manager.get_range_at_angle(+15.0)
            dR = self.sensor_manager.get_range_at_angle(-15.0)
        elif msg:
            dF = self._range_at_deg(msg, 0.0)
            dL = self._range_at_deg(msg, +15.0)
            dR = self._range_at_deg(msg, -15.0)
        else:
            # Ingen sensor data - stopp
            return (0.0, 0.0, False)
        
        # Beregn kommandoer
        linear_x, angular_z = self.calculate_commands(
            dx, dy, distance_to_waypoint, dL, dR, dF, robot_orientation
        )
        
        # Telle blokkert front for fallback
        should_fallback = False
        if dF < self.OBSTACLE_THRESHOLD:
            self.blocked_counter += 1
        else:
            self.blocked_counter = 0
        
        if self.blocked_counter >= self.BLOCKED_LIMIT:
            should_fallback = True
        
        return (linear_x, angular_z, should_fallback)
    
    def calculate_commands(self, dx: float, dy: float, distance: float,
                          dL: float, dR: float, dF: float, 
                          robot_orientation: float) -> tuple:
        """
        Beregn lineær og vinkelhastighet med obstacle avoidance
        
        Args:
            dx, dy: Delta til waypoint
            distance: Avstand til waypoint
            dL, dR, dF: Sensor readings (left, right, front)
            robot_orientation: Robot orientering (radians)
        
        Returns:
            tuple: (linear_x, angular_z)
        """
        linear_x = 0.0
        angular_z = 0.0
        
        # Obstacle avoidance har høyest prioritet
        if dF < self.OBSTACLE_THRESHOLD:
            # Hindring foran - sving til side med minst hinder
            if dL > dR:
                angular_z = self.TURN_SPEED  # Sving venstre
            else:
                angular_z = -self.TURN_SPEED  # Sving høyre
            linear_x = 0.1  # Sakte fremover
        else:
            # Ingen hinder - naviger mot waypoint
            desired_heading = math.atan2(dy, dx)
            heading_error = desired_heading - robot_orientation
            
            # Normaliser vinkel til [-π, π]
            heading_error = self.normalize_angle(heading_error)
            
            # P-kontroll for heading
            angular_z = self.P_GAIN * heading_error
            
            # Begrens angular hastighet
            angular_z = max(min(angular_z, self.TURN_SPEED), -self.TURN_SPEED)
            
            # Adaptiv lineær hastighet basert på heading error
            if abs(heading_error) > math.radians(30):
                # Stor heading error - sving mer, kjør saktere
                linear_x = self.FORWARD_SPEED * 0.3
            elif abs(heading_error) > math.radians(15):
                linear_x = self.FORWARD_SPEED * 0.6
            else:
                # God alignment - full fart
                linear_x = self.FORWARD_SPEED
            
            # Reduser hastighet når vi nærmer oss waypoint
            if distance < 1.0:
                linear_x *= 0.5
        
        return linear_x, angular_z
    
    def normalize_angle(self, angle: float) -> float:
        """Normaliser vinkel til [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _range_at_deg(self, scan: LaserScan, deg: float, default: float = 100.0) -> float:
        """Hent avstand ved vinkel (grader) fra LIDAR"""
        if scan is None or not scan.ranges:
            return default
        
        rad = math.radians(deg)
        idx = int(round((rad - scan.angle_min) / scan.angle_increment))
        
        if idx < 0 or idx >= len(scan.ranges):
            return default
        
        d = scan.ranges[idx]
        if d is None or math.isnan(d) or math.isinf(d) or d == 0.0:
            return default
            
        return float(d)
    
    def publish_twist(self, linear_x: float, angular_z: float):
        """Publiser Twist kommando"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)
    
    def stop_robot(self):
        """Stopp roboten"""
        self.publish_twist(0.0, 0.0)
    
    def reset_blocked_counter(self):
        """Reset blocked counter"""
        self.blocked_counter = 0

