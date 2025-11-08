#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AStarNavigator - Refaktorert versjon som bruker separate klasser

Single Responsibility: Koordinerer A* pathfinding og path following
"""

import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .path_visualizer import PathVisualizer
from .astar_pathfinder import AStarPathfinder
from .path_planner import PathPlanner
from .path_follower import PathFollower
from .recovery_handler import RecoveryHandler


class AStarNavigator:
    """
    A* Pathfinding Navigator - Planlegger optimal rute gjennom kartet
    
    Single Responsibility: Koordinerer A* pathfinding og path following
    """
    
    # --- INNSTILLINGER ---
    REPLAN_INTERVAL_SEC = 1.0  # hvor ofte vi periodisk replanlegger
    
    def __init__(self, node_ref: Node, sensor_manager=None, occupancy_grid_manager=None, leader_position_callback=None):
        self.node = node_ref
        self.sensor_manager = sensor_manager
        self.occupancy_grid_manager = occupancy_grid_manager
        self.leader_position_callback = leader_position_callback
        
        # Hent robot ID for visualisering
        self.robot_id = self.node.get_namespace().strip('/')
        
        # Path planning state
        self.path = []  # List of (x, y) waypoints i world koordinater
        self.current_waypoint_idx = 0
        self.target_position = None
        self.navigation_active = False
        
        # Robot state
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0
        
        # Visualization counter for periodic updates
        self.viz_counter = 0
        self.last_plan_time = 0.0
        self.planning_in_progress = False  # Flag for å unngå samtidig planlegging
        self.initial_plan_done = False  # Flag for å unngå replan umiddelbart etter første planlegging
        
        # Setup publisher
        self.cmd_vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        # Visuals for planned paths
        self.path_visualizer = PathVisualizer(self.node, self.robot_id)

        # Opprett komponenter
        self.path_planner = PathPlanner(self.node, self.occupancy_grid_manager, leader_position_callback)
        self.path_follower = PathFollower(self.node, self.sensor_manager, self.cmd_vel_publisher)
        
        # Setup A* pathfinder med callbacks
        self.pathfinder = AStarPathfinder(
            self.node,
            is_traversable_callback=self.path_planner.is_traversable,
            get_neighbors_callback=self.path_planner.get_neighbors
        )
        
        # Setup recovery handler
        self.recovery_handler = RecoveryHandler(
            self.node,
            publish_twist_callback=self.path_follower.publish_twist,
            plan_path_callback=self.plan_path
        )
        
        self.node.get_logger().info(f'⭐ AStarNavigator ({self.robot_id}) initialisert')
    
    def set_goal(self, position: tuple):
        """Sett nytt mål og planlegg rute med A*"""
        self.target_position = position
        self.navigation_active = True
        
        # Planlegg rute med A* (blokkerer ikke - navigasjon kan starte mens planlegging pågår)
        if not self.planning_in_progress:
            self.planning_in_progress = True
            success = self.plan_path()
            self.planning_in_progress = False
            
            # Oppdater last_plan_time for å unngå umiddelbar replan
            now_sec = self.node.get_clock().now().nanoseconds / 1e9
            self.last_plan_time = now_sec
            
            if success:
                self.initial_plan_done = True  # Marker at første planlegging er ferdig
                self.node.get_logger().info(f'⭐ A* ({self.robot_id}): Rute planlagt til {position} med {len(self.path)} waypoints')
                self.viz_counter = 0  # Reset counter for umiddelbar visualisering
                self.path_visualizer.publish(self.path)
            else:
                self.node.get_logger().warn(f'⭐ A* ({self.robot_id}): Kunne ikke finne rute til {position}')
                # Publiser tom path for å fjerne gamle markers
                self.path_visualizer.publish([])
                # Ikke deaktiver navigasjon - prøv igjen ved neste replan
        else:
            self.node.get_logger().debug('⭐ A*: Planlegging pågår allerede, hopper over')
    
    def clear_goal(self):
        """Fjern aktivt mål og rute"""
        self.target_position = None
        self.path = []
        self.current_waypoint_idx = 0
        self.navigation_active = False
        self.initial_plan_done = False  # Reset flag
        self.path_visualizer.publish([])
        self.stop_robot()
    
    def update_robot_pose(self, position: tuple, orientation: float):
        """Oppdater robot posisjon og orientering"""
        self.robot_position = position
        self.robot_orientation = orientation
    
    def plan_path(self) -> bool:
        """
        Planlegg rute fra nåværende posisjon til mål med A* algoritmen
        
        Returns:
            bool: True hvis rute ble funnet
        """
        if not self.occupancy_grid_manager:
            self.node.get_logger().error('⭐ A*: OccupancyGridManager er None!')
            return False
        
        if not self.occupancy_grid_manager.is_map_available():
            self.node.get_logger().warn('⭐ A*: Ingen map tilgjengelig for pathfinding.')
            return False
        
        map_info = self.occupancy_grid_manager.get_map_info() or {}
        map_height = map_info.get('height', 0)
        map_width = map_info.get('width', 0)
        
        if not self.target_position:
            self.node.get_logger().warn('⭐ A*: Ingen target_position satt!')
            return False
        
        # Sjekk at robot_position er satt
        if not self.robot_position or self.robot_position == (0.0, 0.0):
            self.node.get_logger().warn(f'⭐ A*: Robot position er ikke satt eller er (0,0): {self.robot_position}')
        
        # Konverter start og mål til map koordinater
        self.node.get_logger().info(f'⭐ A*: Planlegger rute. Robot pos: {self.robot_position}, Target: {self.target_position}')
        
        start_map = self.occupancy_grid_manager.world_to_map(
            self.robot_position[0], self.robot_position[1]
        )
        goal_map = self.occupancy_grid_manager.world_to_map(
            self.target_position[0], self.target_position[1]
        )
        
        self.node.get_logger().info(
            f'⭐ A*: start_world={self.robot_position}, goal_world={self.target_position}, '
            f'start_map={start_map}, goal_map={goal_map}'
        )
        
        # Debug: Sjekk traversability for start og goal
        start_traversable = self.path_planner.is_traversable(start_map[0], start_map[1])
        goal_traversable = self.path_planner.is_traversable(goal_map[0], goal_map[1])
        start_occupancy = self.occupancy_grid_manager.get_occupancy_value(start_map[0], start_map[1])
        goal_occupancy = self.occupancy_grid_manager.get_occupancy_value(goal_map[0], goal_map[1])
        
        start_in_bounds = 0 <= start_map[0] < map_height and 0 <= start_map[1] < map_width
        goal_in_bounds = 0 <= goal_map[0] < map_height and 0 <= goal_map[1] < map_width
        
        self.node.get_logger().info(
            f'⭐ A* traversability: start_occ={start_occupancy}, goal_occ={goal_occupancy}, '
            f'start_in_bounds={start_in_bounds}, goal_in_bounds={goal_in_bounds}'
        )
        
        # Viktig: Sjekk at start er traversable
        if not start_traversable:
            self.node.get_logger().warn(
                f'⭐ A*: START er ikke traversable! occ={start_occupancy}, '
                f'start_map={start_map}, start_world={self.robot_position}. '
                f'Dette kan skje hvis roboten er manuelt flyttet til en hindring. '
                f'Prøver å finne nærmeste traversable punkt...'
            )
            # Prøv å finne nærmeste traversable punkt til start
            adjusted_start = self.path_planner.find_nearest_traversable(start_map, max_search_radius=10)
            if adjusted_start:
                start_map = adjusted_start
                self.node.get_logger().warn(f'⭐ A*: Justert start til {start_map}')
            else:
                self.node.get_logger().error('⭐ A*: Kunne ikke finne traversable start!')
                return False
        
        # Alltid prøv å bruke lederens posisjon hvis tilgjengelig (for Big Fire scenario)
        # Dette sikrer at supporteren navigerer til riktig side av veggen
        leader_pos = None
        if self.leader_position_callback:
            try:
                leader_pos = self.leader_position_callback()
            except Exception as e:
                self.node.get_logger().debug(f'⭐ A*: Kunne ikke hente lederens posisjon: {e}')
        
        # Hvis lederens posisjon er tilgjengelig, bruk den i stedet for ArUco merket
        if leader_pos:
            leader_map = self.occupancy_grid_manager.world_to_map(leader_pos[0], leader_pos[1])
            leader_traversable = self.path_planner.is_traversable(leader_map[0], leader_map[1])
            
            if leader_traversable:
                # Lederens posisjon er traversable - bruk den direkte
                goal_map = leader_map
                self.target_position = leader_pos
                self.node.get_logger().info(f'⭐ A*: Bruker lederens posisjon direkte (ikke ArUco merket): map={goal_map}, world={self.target_position}')
            else:
                # Lederens posisjon er ikke traversable - finn nærmeste traversable punkt
                adjusted_goal = self.path_planner.find_nearest_traversable_towards_goal(start_map, leader_map, max_search_radius=20)
                if adjusted_goal:
                    goal_map = adjusted_goal
                    self.target_position = self.occupancy_grid_manager.map_to_world(goal_map[0], goal_map[1])
                    self.node.get_logger().info(f'⭐ A*: Bruker punkt nær lederen (riktig side av veggen): map={goal_map}, world={self.target_position}')
                else:
                    # Fallback: prøv vanlig søk
                    adjusted_goal = self.path_planner.find_nearest_traversable(leader_map, max_search_radius=15)
                    if adjusted_goal:
                        goal_map = adjusted_goal
                        self.target_position = self.occupancy_grid_manager.map_to_world(goal_map[0], goal_map[1])
                        self.node.get_logger().warn(f'⭐ A*: Fallback - bruker punkt nær lederen: map={goal_map}, world={self.target_position}')
                    else:
                        self.node.get_logger().error('⭐ A*: Kunne ikke finne traversable punkt nær lederen!')
                        return False
        
        # Hvis målet er på en hindring (og vi ikke har lederens posisjon), finn nærmeste traversable punkt
        elif not goal_traversable:
            self.node.get_logger().warn(f'⭐ A*: Målet er på en hindring! Søker etter nærmeste traversable punkt...')
            # Ingen leder-posisjon tilgjengelig: finn nærmeste traversable punkt i retning fra roboten til målet
            adjusted_goal = self.path_planner.find_nearest_traversable_towards_goal(start_map, goal_map, max_search_radius=20)
            if adjusted_goal:
                goal_map = adjusted_goal
                self.target_position = self.occupancy_grid_manager.map_to_world(goal_map[0], goal_map[1])
                self.node.get_logger().info(f'⭐ A*: Justert mål til traversable punkt i retning: map={goal_map}, world={self.target_position}')
            else:
                self.node.get_logger().error('⭐ A*: Kunne ikke finne traversable punkt!')
                return False
        
        # Kjør A* algoritmen
        self.node.get_logger().info(f'⭐ A*: Starter søk fra {start_map} til {goal_map}')
        map_info = self.occupancy_grid_manager.get_map_info()
        path_map = self.pathfinder.search(start_map, goal_map, map_info['height'], map_info['width'])
        
        if not path_map:
            self.node.get_logger().warn(f'⭐ A* fant ingen rute fra {start_map} til {goal_map}')
            return False
        
        self.node.get_logger().info(f'⭐ A*: Søk fullført, fikk path med {len(path_map)} punkter')
        
        # Konverter path til world koordinater og reduser antall waypoints
        self.path = self.path_planner.smooth_path(path_map)
        self.current_waypoint_idx = 0
        
        # DEBUG: Verifiser at path koordinater er riktige
        if self.path:
            # Sjekk første og siste waypoint
            first_wp = self.path[0]
            last_wp = self.path[-1]
            
            # Konverter tilbake til map for å verifisere
            first_map_check = self.occupancy_grid_manager.world_to_map(first_wp[0], first_wp[1])
            last_map_check = self.occupancy_grid_manager.world_to_map(last_wp[0], last_wp[1])
            
            # Sjekk om første waypoint er nær start
            dist_to_start = math.sqrt(
                (first_wp[0] - self.robot_position[0])**2 + 
                (first_wp[1] - self.robot_position[1])**2
            )
            
            # Sjekk om siste waypoint er nær mål
            dist_to_goal = math.sqrt(
                (last_wp[0] - self.target_position[0])**2 + 
                (last_wp[1] - self.target_position[1])**2
            )
            
            self.node.get_logger().info(
                f'⭐ A*: Path satt! path_len={len(self.path)}, '
                f'first_waypoint_world={first_wp}, first_waypoint_map={first_map_check}, '
                f'last_waypoint_world={last_wp}, last_waypoint_map={last_map_check}, '
                f'dist_to_start={dist_to_start:.2f}m, dist_to_goal={dist_to_goal:.2f}m, '
                f'robot_pos={self.robot_position}, target={self.target_position}'
            )
            
            # Sjekk om noen waypoints er på hindringer
            obstacles_in_path = 0
            for wp in self.path[::max(1, len(self.path)//10)]:  # Sjekk hver 10. waypoint
                wp_map = self.occupancy_grid_manager.world_to_map(wp[0], wp[1])
                if not self.path_planner.is_traversable(wp_map[0], wp_map[1]):
                    obstacles_in_path += 1
            
            if obstacles_in_path > 0:
                self.node.get_logger().warn(
                    f'⭐ A*: ADVARSEL! {obstacles_in_path} waypoints i path er på hindringer! '
                    f'Dette kan tyde på koordinatfeil.'
                )
        else:
            self.node.get_logger().warn('⭐ A*: Path er tom etter smoothing!')
        
        return True
    
    def navigate_to_goal(self, msg: LaserScan = None) -> bool:
        """
        Naviger langs planlagt rute
        
        Args:
            msg: LaserScan data (optional, for obstacle avoidance)
        
        Returns:
            bool: True hvis endelig mål er nådd
        """
        if not self.navigation_active:
            if not hasattr(self, '_last_inactive_log') or \
               (self.node.get_clock().now().nanoseconds / 1e9 - getattr(self, '_last_inactive_log', 0.0)) > 2.0:
                self.node.get_logger().warn('⭐ A*: navigation_active er False - venter på aktivering')
                self._last_inactive_log = self.node.get_clock().now().nanoseconds / 1e9
            return False
        
        # Publiser visualisering periodisk (hver 10. gang ≈ 1Hz hvis scan er 10Hz)
        self.viz_counter += 1
        if self.viz_counter % 10 == 0:
            self.path_visualizer.publish(self.path)
        
        # Periodisk replan (kun hvis ikke allerede planlegger og vi har en path)
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        
        # Beregn avstand til mål for å avgjøre om vi skal replan
        distance_to_goal = math.sqrt(
            (self.target_position[0] - self.robot_position[0])**2 +
            (self.target_position[1] - self.robot_position[1])**2
        ) if self.target_position else float('inf')
        
        # Sjekk om vi har nådd endelig mål FØR replan-sjekk
        if distance_to_goal <= PathFollower.GOAL_THRESHOLD:
            self.node.get_logger().info('⭐ ENDELIG MÅL NÅDD!')
            self.stop_robot()
            return True
        
        # Kun replan hvis:
        # 1. Første planlegging er ferdig (initial_plan_done)
        # 2. Det har gått nok tid siden siste planlegging (REPLAN_INTERVAL_SEC)
        # 3. Vi har en path
        # 4. Vi ikke allerede planlegger
        # 5. Vi har et mål
        # 6. Roboten er fortsatt langt unna målet (ikke replan hvis vi er nærme eller har nådd målet)
        time_since_last_plan = now_sec - self.last_plan_time
        should_replan = (
            self.initial_plan_done and
            time_since_last_plan >= self.REPLAN_INTERVAL_SEC and 
            self.target_position is not None and 
            not self.planning_in_progress and 
            self.path and
            distance_to_goal > PathFollower.GOAL_THRESHOLD * 2  # Ikke replan hvis vi er nærme målet
        )
        
        if should_replan:
            self.planning_in_progress = True
            planned = self.plan_path()
            self.planning_in_progress = False
            self.last_plan_time = now_sec
            if not planned:
                # Ingen path: la koordinatoren velge alternativ strategi
                self.path = []
        
        if not self.path:
            # Logg sjeldent for å indikere at path er tom (mulig fallback)
            if not hasattr(self, '_last_empty_path_log'):
                self._last_empty_path_log = 0.0
            if (now_sec - getattr(self, '_last_empty_path_log', 0.0)) > 1.5:
                self.node.get_logger().warn(
                    f'⭐ A*: Ingen aktiv path – venter/replanlegger. '
                    f'Robot pos: {self.robot_position}, Target: {self.target_position}, '
                    f'Active: {self.navigation_active}'
                )
                self._last_empty_path_log = now_sec
            return False
        
        # Avstand til mål er allerede beregnet over (i replan-seksjonen)
        # Sjekk om vi har nådd endelig mål
        if distance_to_goal <= PathFollower.GOAL_THRESHOLD:
            self.node.get_logger().info('⭐ ENDELIG MÅL NÅDD!')
            self.stop_robot()
            return True
        
        # Finn neste waypoint
        if self.current_waypoint_idx >= len(self.path):
            # Alle waypoints nådd, gå direkte til mål
            current_waypoint = self.target_position
        else:
            current_waypoint = self.path[self.current_waypoint_idx]
            
            # Sjekk om waypoint er nådd
            distance_to_waypoint = math.sqrt(
                (current_waypoint[0] - self.robot_position[0])**2 +
                (current_waypoint[1] - self.robot_position[1])**2
            )
            
            if distance_to_waypoint <= PathFollower.WAYPOINT_THRESHOLD:
                self.current_waypoint_idx += 1
                self.node.get_logger().info(
                    f'⭐ Waypoint {self.current_waypoint_idx}/{len(self.path)} nådd'
                )
                
                # Hvis det var siste waypoint, gå til mål
                if self.current_waypoint_idx >= len(self.path):
                    current_waypoint = self.target_position
                else:
                    current_waypoint = self.path[self.current_waypoint_idx]
        
        # STUCK detection og recovery før navigasjon
        if self.recovery_handler.should_recover(distance_to_goal, now_sec):
            recovery_done = self.recovery_handler.do_recovery(now_sec)
            if recovery_done:
                # Recovery ferdig - replan path
                planned = self.plan_path()
                self.last_plan_time = now_sec
                if not planned:
                    self.path = []
                    self.navigation_active = False
            return False
        
        # Debug: Log path status første gang eller ved endringer
        if not hasattr(self, '_last_path_status_log') or \
           (now_sec - getattr(self, '_last_path_status_log', 0.0)) > 2.0:
            self.node.get_logger().info(
                f'⭐ A* NAVIGATING: path_len={len(self.path)}, '
                f'waypoint_idx={self.current_waypoint_idx}, '
                f'robot_pos={self.robot_position}, '
                f'waypoint={current_waypoint}, '
                f'dist_to_goal={distance_to_goal:.2f}m'
            )
            self._last_path_status_log = now_sec
        
        # Naviger mot nåværende waypoint
        linear_x, angular_z, should_fallback = self.path_follower.navigate_to_waypoint(
            current_waypoint, self.robot_position, self.robot_orientation, msg
        )
        
        if should_fallback:
            # Tving koordinatoren til å velge alternativ strategi
            self.node.get_logger().warn('⭐ A*: Front blokkert lenge – tvinger fallback (tømmer path)')
            self.path = []
            self.navigation_active = False
            self.path_follower.reset_blocked_counter()
            return False
        
        # Publiser kommandoer
        self.path_follower.publish_twist(linear_x, angular_z)
        
        # Oppdater fremdrift
        self.recovery_handler.update_progress(distance_to_goal, now_sec)
        
        # Marker nåværende celle som besøkt
        return False
    
    def publish_twist(self, linear_x: float, angular_z: float):
        """Publiser Twist kommando (for recovery handler)"""
        self.path_follower.publish_twist(linear_x, angular_z)
    
    def stop_robot(self):
        """Stopp roboten"""
        self.path_follower.stop_robot()