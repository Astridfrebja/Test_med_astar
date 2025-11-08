#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

from rclpy.node import Node
from scoring_interfaces.srv import SetMarkerPosition
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Importer alle komponenter
from .wall_follower import WallFollower
from .astar_navigator import AStarNavigator
from .big_fire_coordinator import BigFireCoordinator
from .aruco_detector import ArUcoDetector
from .robot_memory import RobotMemory
from .sensor_manager import SensorManager


class SearchRescueCoordinator:

    """

    Search & Rescue koordinator - koordinerer alle komponenter

    """

    

    def __init__(self, node_ref: Node):

        self.node = node_ref

        self.robot_id = self.node.get_namespace().strip('/')

        

        self.robot_position = (0.0, 0.0)

        self.robot_orientation = 0.0

        

        self.sensor_manager = SensorManager(node_ref)
        self.robot_memory = RobotMemory()
        self.big_fire_coordinator = BigFireCoordinator(node_ref, self.robot_memory)
        self.aruco_detector = ArUcoDetector(node_ref, self.handle_aruco_detection)
        
        # Koble sensorens ArUco callback direkte til koordinatorens handler
        # slik at roboten stopper uansett hvilket ArUco-merke som oppdages
        self.sensor_manager.aruco_callback = self.handle_aruco_detection
        
        # Navigasjonskomponenter
        self.wall_follower = WallFollower(node_ref, self.sensor_manager)
        self.simple_goal = None
        self.goal_threshold = 0.5
        self.goal_turn_speed = 0.5
        self.goal_forward_speed = 0.3
        self.goal_obstacle_threshold = 0.8
        
        # A* Navigator (brukes for Big Fire navigasjon når map er tilgjengelig)
        # Send callback for å hente lederens posisjon (hvis supporter)
        def get_leader_position():
            """Hent lederens posisjon hvis den er ved brannen
            
            Når supporteren skal gå til Big Fire og lederen allerede er ved brannen,
            returnerer vi Big Fire posisjonen som referanse. Siden lederen er ved brannen
            (innenfor 2m av Big Fire), er lederens faktiske posisjon nær Big Fire posisjonen,
            men ikke på veggen (siden lederen kan stå der). A* vil finne et traversable
            punkt nær denne posisjonen.
            """
            if self.robot_memory.my_role == self.robot_memory.SUPPORTER and \
               self.robot_memory.other_robot_at_fire and \
               self.robot_memory.big_fire_position:
                self.node.get_logger().info(
                    f'Supporter bruker lederposisjon {self.robot_memory.big_fire_position}'
                )
                return self.robot_memory.big_fire_position
            return None
        
        self.astar_navigator = AStarNavigator(
            node_ref, 
            self.sensor_manager, 
            self.sensor_manager.get_occupancy_grid_manager(),
            leader_position_callback=get_leader_position
        )

        # Scoring service client
        self.scoring_client = self.node.create_client(SetMarkerPosition, '/set_marker_position')
        self._waited_for_scoring = False
        
        # Navigation mode
        self.use_astar_for_big_fire = False  # Vil bli satt til True når map er tilgjengelig
        
        self.node.get_logger().info(f'🤖 SearchRescueCoordinator ({self.robot_id}) initialisert')
        
    def process_scan(self, msg: LaserScan):
        """
        Hovedfunksjon - koordinerer navigasjon. Kaller KUN ÉN navigasjonskontroller per syklus.
        """
        map_available = self.sensor_manager.is_map_available()

        if map_available and not self.use_astar_for_big_fire:
            self.use_astar_for_big_fire = True
            map_info = self.sensor_manager.get_map_info()
            self.node.get_logger().info(
                f'⭐ Map tilgjengelig! A* aktivert ({map_info.get("width", "?")}x{map_info.get("height", "?")}, res={map_info.get("resolution", "?")})'
            )

        self.big_fire_coordinator.update_state(self.robot_position, self.robot_orientation)
        big_fire_active = self.big_fire_coordinator.should_handle_big_fire()

        if big_fire_active:
            target = self.big_fire_coordinator.get_target_position()

            if self.robot_memory.big_fire_state == self.robot_memory.LEDER_WAITING:
                self._stop_all_navigation()
                self.handle_big_fire_state_logic()
                return

            if not target or not self.robot_memory.is_moving_to_fire():
                self._stop_all_navigation()
                self.handle_big_fire_state_logic()
                return

            if map_available and self.use_astar_for_big_fire:
                self._stop_simple_nav()
                if not self.astar_navigator.navigation_active or self.astar_navigator.target_position != target:
                    self.node.get_logger().info(f'A*: mål {target}')
                    self.astar_navigator.set_goal(target)
                goal_reached = self.astar_navigator.navigate_to_goal(msg)
            else:
                if getattr(self.astar_navigator, 'navigation_active', False):
                    self.astar_navigator.clear_goal()
                goal_reached = self._navigate_simple_goal(target)

            if goal_reached:
                self._on_big_fire_goal_reached()
            return

        # Ingen Big Fire: normal utforskning
        if getattr(self.astar_navigator, 'navigation_active', False):
            self.astar_navigator.clear_goal()
        self._stop_simple_nav()
        self.wall_follower.follow_wall(msg)


    def process_odom(self, msg: Odometry):
        """Oppdater robot posisjon og orientering"""
        
        self.robot_position = self.sensor_manager.get_robot_position()
        self.robot_orientation = self.sensor_manager.get_robot_orientation()
        
        # Oppdater alle navigatorer med robot posisjon (viktig for path planning)
        self.robot_memory.update_robot_pose(self.robot_position, self.robot_orientation)
        self.astar_navigator.update_robot_pose(self.robot_position, self.robot_orientation)
        if getattr(self.astar_navigator, 'navigation_active', False):
            try:
                self.wall_follower.stop_robot()
            except Exception:
                pass
            self._stop_simple_nav()
            try:
                self.astar_navigator.navigate_to_goal(None)
            except Exception:
                pass

    def _stop_all_navigation(self):
        try:
            self.astar_navigator.stop_robot()
        except Exception:
            pass
        try:
            self.wall_follower.stop_robot()
        except Exception:
            pass
        self._stop_simple_nav()

    def _stop_simple_nav(self):
        if self.simple_goal is not None:
            self.node.publish_twist(0.0, 0.0)
            self.simple_goal = None

    def _navigate_simple_goal(self, target: tuple) -> bool:
        if target is None:
            self._stop_simple_nav()
            return False

        dx = target[0] - self.robot_position[0]
        dy = target[1] - self.robot_position[1]
        distance = math.hypot(dx, dy)

        if distance <= self.goal_threshold:
            self._stop_simple_nav()
            return True

        if not self.sensor_manager.is_scan_valid():
            self._stop_simple_nav()
            return False

        dF = self.sensor_manager.get_range_at_angle(0.0, default=10.0)
        dL = self.sensor_manager.get_range_at_angle(15.0, default=10.0)
        dR = self.sensor_manager.get_range_at_angle(-15.0, default=10.0)

        if dF < self.goal_obstacle_threshold:
            angular = self.goal_turn_speed if dL > dR else -self.goal_turn_speed
            linear = 0.1
        else:
            desired_heading = math.atan2(dy, dx)
            heading_error = math.atan2(math.sin(desired_heading - self.robot_orientation),
                                       math.cos(desired_heading - self.robot_orientation))
            angular = max(min(heading_error, self.goal_turn_speed), -self.goal_turn_speed)

            if abs(heading_error) > math.radians(30):
                linear = self.goal_forward_speed * 0.3
            elif abs(heading_error) > math.radians(15):
                linear = self.goal_forward_speed * 0.6
            else:
                linear = self.goal_forward_speed

            if distance < 1.0:
                linear *= 0.5

        self.simple_goal = target
        self.node.publish_twist(linear, angular)
        return False

    def _on_big_fire_goal_reached(self):
        self._stop_all_navigation()
        if self.robot_memory.my_role == self.robot_memory.LEDER:
            self.robot_memory.transition_to_leder_waiting()
        else:
            try:
                self.big_fire_coordinator.publish_robot_at_fire(self.robot_position)
            except Exception:
                pass
            self.robot_memory.transition_to_extinguishing()
        self.handle_big_fire_state_logic()

    def handle_big_fire_state_logic(self):

        """Håndterer KUN tilstandsoverganger og publisering."""

        memory = self.big_fire_coordinator.memory
        state = memory.big_fire_state

        if state == memory.LEDER_WAITING:
            bf_pos = memory.big_fire_position
            if bf_pos and not memory.i_am_at_fire:
                distance = math.hypot(bf_pos[0] - self.robot_position[0],
                                       bf_pos[1] - self.robot_position[1])
                if distance <= 2.0:
                    self.big_fire_coordinator.publish_robot_at_fire(self.robot_position)

            if memory.i_am_at_fire and memory.other_robot_at_fire:
                memory.transition_to_extinguishing()

        elif state == memory.EXTINGUISHING:
            if not memory.fire_extinguished:
                self.big_fire_coordinator.publish_fire_extinguished()

        elif state == memory.NORMAL:
            if memory.big_fire_detected_by_me:
                memory.transition_to_leder_going_to_fire()
            elif memory.big_fire_detected_by_other and memory.big_fire_position:
                pos = memory.big_fire_position
                self.node.get_logger().info(
                    f'Supporter {self.robot_id} navigerer mot ({pos[0]:.2f}, {pos[1]:.2f})'
                )
                self._stop_all_navigation()
                if self.use_astar_for_big_fire:
                    self.astar_navigator.set_goal(pos)
                memory.transition_to_supporter_going_to_fire()


    def handle_aruco_detection(self, marker_id: int, position: tuple):

        """Håndterer ArUco marker detection"""

        # Only log once per marker to avoid spam
        if not hasattr(self, '_processed_aruco_markers'):
            self._processed_aruco_markers = set()
        
        marker_key = f"{marker_id}_{position[0]:.1f}_{position[1]:.1f}"
        if marker_key in self._processed_aruco_markers:
            return  # Already processed this marker at this position
        
        self._processed_aruco_markers.add(marker_key)

        # Stop all navigators
        self._stop_all_navigation()
        
        self.node.get_logger().info(f'🛑 ROBOT STOPPED! ArUco ID {marker_id} oppdaget på {position}')
        
        # Report to scoring service for all markers (0-4)
        try:
            self._report_marker_to_scoring(marker_id, position)
        except Exception as e:
            self.node.get_logger().warn(f'Scoring service call failed: {e}')

        if marker_id == 4:  # Big Fire
            self.node.get_logger().info(f'🔥 BIG FIRE DETECTED! Calling detect_big_fire({position})')
            # Viktig: Bruk robotens faktiske posisjon (ikke ArUco merket sin posisjon på veggen)
            # siden alle ArUco merkene er på veggen, men roboten står ved siden av veggen
            self.big_fire_coordinator.detect_big_fire(self.robot_position)
            # Kaller update_state umiddelbart for å sette i gang navigasjonen i neste process_scan
            self.big_fire_coordinator.update_state(self.robot_position, self.robot_orientation)
            # Big Fire håndteres av Big Fire logikk - ikke start wall follower her
        else:
            self.node.get_logger().info(f'📊 ArUco ID {marker_id} på {position} - Roboten stopper for scoring!')
            # For andre markers: stopp kort, rapporter, så fortsett med wall following
            # Wall follower vil starte automatisk i neste process_scan() siden Big Fire ikke er aktiv 

    def _report_marker_to_scoring(self, marker_id: int, position: tuple):
        """Call scoring service to report a found marker."""
        if marker_id < 0 or marker_id > 4:
            return
        # Wait for service only once (non-blocking loop with logs)
        if not self._waited_for_scoring:
            tries = 0
            while not self.scoring_client.wait_for_service(timeout_sec=0.1) and tries < 20:
                tries += 1
            self._waited_for_scoring = True

        request = SetMarkerPosition.Request()
        request.marker_id = int(marker_id)
        pt = Point()
        pt.x = float(position[0])
        pt.y = float(position[1])
        pt.z = 0.0 if len(position) < 3 else float(position[2])
        request.marker_position = pt

        future = self.scoring_client.call_async(request)
        # Add done callback to log result without blocking
        def _done_cb(f):
            try:
                resp = f.result()
                if resp and resp.accepted:
                    self.node.get_logger().info(f'✅ Scoring accepted marker {marker_id} at ({pt.x:.2f}, {pt.y:.2f})')
                else:
                    self.node.get_logger().warn(f'⚠️ Scoring rejected marker {marker_id} (accuracy or state)')
            except Exception as ex:
                self.node.get_logger().warn(f'⚠️ Scoring service error: {ex}')
        future.add_done_callback(_done_cb)