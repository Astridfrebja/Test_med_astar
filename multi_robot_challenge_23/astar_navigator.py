#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import heapq
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from .path_visualizer import PathVisualizer
from .visited_tracker import VisitedTracker
from .path_visualizer import PathVisualizer
from .visited_tracker import VisitedTracker

class AStarNavigator:
    """
    A* Pathfinding Navigator - Planlegger optimal rute gjennom kartet
    
    Single Responsibility: A* pathfinding og path following
    """
    
    # --- INNSTILLINGER ---
    FORWARD_SPEED = 0.3
    TURN_SPEED = 0.5
    WAYPOINT_THRESHOLD = 0.3  # meters til waypoint er nådd
    GOAL_THRESHOLD = 0.5  # meters til endelig mål er nådd
    P_GAIN = 1.5  # P-kontroll for heading
    OBSTACLE_THRESHOLD = 0.4  # meter til hindring (mer konservativt)
    INFLATION_RADIUS = 1  # Grid cells å inflere rundt hindringer (redusert fra 2 for mindre streng traversability)
    REPLAN_INTERVAL_SEC = 1.0  # hvor ofte vi periodisk replanlegger
    STUCK_TIME_SEC = 2.0       # hvor lenge uten fremdrift før recovery
    STUCK_PROGRESS_EPS = 0.10  # meter forbedring som teller som fremdrift
    BLOCKED_LIMIT = 15         # antall påfølgende sykluser med blokkert front før fallback
    
    def __init__(self, node_ref: Node, sensor_manager=None, occupancy_grid_manager=None):
        self.node = node_ref
        self.sensor_manager = sensor_manager
        self.occupancy_grid_manager = occupancy_grid_manager
        
        # Hent robot ID for visualisering
        self.robot_id = self.node.get_namespace().strip('/')
        
        # Path planning
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
        self.last_progress_time = 0.0
        self.last_progress_dist = float('inf')
        self.blocked_counter = 0
        
        # Recovery state
        self.recovering = False
        self.recovery_phase = None  # 'back' eller 'turn'
        self.recovery_phase_end = 0.0
        
        # Setup publisher
        self.cmd_vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        # Visuals & tracking (SRP)
        self.path_visualizer = PathVisualizer(self.node, self.robot_id)
        self.visited_tracker = VisitedTracker(self.node, self.robot_id, self.occupancy_grid_manager)
        
        # Visualisering og besøks-sporing (SRP)
        self.path_visualizer = PathVisualizer(self.node, self.robot_id)
        self.visited_tracker = VisitedTracker(self.node, self.robot_id, self.occupancy_grid_manager)
        
        self.node.get_logger().info(f'⭐ AStarNavigator ({self.robot_id}) initialisert')
    
    def set_goal(self, position: tuple):
        """Sett nytt mål og planlegg rute med A*"""
        self.target_position = position
        self.navigation_active = True
        
        # Planlegg rute med A*
        success = self.plan_path()
        
        if success:
            self.node.get_logger().info(f'⭐ A* ({self.robot_id}): Rute planlagt til {position} med {len(self.path)} waypoints')
            self.viz_counter = 0  # Reset counter for umiddelbar visualisering
            self.path_visualizer.publish(self.path)
        else:
            self.node.get_logger().warn(f'⭐ A* ({self.robot_id}): Kunne ikke finne rute til {position}')
            # Publiser tom path for å fjerne gamle markers
            self.path_visualizer.publish([])
            self.navigation_active = False
    
    def clear_goal(self):
        """Fjern aktivt mål og rute"""
        self.target_position = None
        self.navigation_active = False
        self.path = []
        self.current_waypoint_idx = 0
        # Slett visualisering ved å publisere DELETE markers
        self.path_visualizer.publish([])
        self.node.get_logger().info(f'⭐ ({self.robot_id}): Mål og rute fjernet')
    
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
            self.node.get_logger().warn('⭐ A*: Ingen map tilgjengelig for pathfinding. Sjekk om map_server kjører og publiserer på /map topic.')
            return False
        
        if not self.target_position:
            self.node.get_logger().warn('⭐ A*: Ingen target_position satt!')
            return False
        
        # Sjekk at robot_position er satt
        if not self.robot_position or self.robot_position == (0.0, 0.0):
            self.node.get_logger().warn(f'⭐ A*: Robot position er ikke satt eller er (0,0): {self.robot_position}')
            # Ikke return False - kan være at vi faktisk starter på (0,0)
        
        # Konverter start og mål til map koordinater
        self.node.get_logger().info(f'⭐ A*: Planlegger rute. Robot pos: {self.robot_position}, Target: {self.target_position}')
        
        start_map = self.occupancy_grid_manager.world_to_map(
            self.robot_position[0], self.robot_position[1]
        )
        goal_map = self.occupancy_grid_manager.world_to_map(
            self.target_position[0], self.target_position[1]
        )
        
        self.node.get_logger().info(f'⭐ A*: Map koordinater - start={start_map}, goal={goal_map}')
        
        # Debug: Sjekk traversability for start og goal
        start_traversable = self.is_traversable(start_map[0], start_map[1])
        goal_traversable = self.is_traversable(goal_map[0], goal_map[1])
        start_occupancy = self.occupancy_grid_manager.get_occupancy_value(start_map[0], start_map[1])
        goal_occupancy = self.occupancy_grid_manager.get_occupancy_value(goal_map[0], goal_map[1])
        
        self.node.get_logger().info(
            f'⭐ A* planning: start={start_map}, goal={goal_map}, '
            f'start_world={self.robot_position}, goal_world={self.target_position}'
        )
        self.node.get_logger().info(
            f'⭐ A* traversability: start_traversable={start_traversable} (occ={start_occupancy}), '
            f'goal_traversable={goal_traversable} (occ={goal_occupancy})'
        )
        
        # Kjør A* algoritmen
        path_map = self.astar_search(start_map, goal_map)
        
        if not path_map:
            self.node.get_logger().warn('⭐ A* fant ingen rute')
            return False
        
        # Konverter path til world koordinater og reduser antall waypoints
        self.path = self.smooth_path(path_map)
        self.current_waypoint_idx = 0
        
        return True
    
    def astar_search(self, start: tuple, goal: tuple) -> list:
        """
        A* søkealgoritme
        
        Args:
            start: (map_x, map_y) startposisjon
            goal: (map_x, map_y) målposisjon
        
        Returns:
            list: Path som liste av (map_x, map_y) koordinater, eller []
        """
        # A* datastrukturer
        open_set = []  # Priority queue (f_score, counter, node)
        heapq.heappush(open_set, (0, 0, start))
        
        came_from = {}  # node -> parent node
        g_score = {start: 0}  # node -> cost from start
        f_score = {start: self.heuristic(start, goal)}  # node -> estimated total cost
        
        counter = 0  # For å bryte likhet i priority queue
        closed_set = set()
        
        map_info = self.occupancy_grid_manager.get_map_info()
        max_x = map_info['height']
        max_y = map_info['width']
        
        while open_set:
            _, _, current = heapq.heappop(open_set)
            
            # Sjekk om vi har nådd målet - mer fleksibel sjekk
            dist_to_goal = self.distance(current, goal)
            if dist_to_goal < 3.0:  # Økt fra 2.0 for å tillate nærliggende celler nær vegger
                # Rekonstruer path
                path = self.reconstruct_path(came_from, current)
                # Legg til goal hvis ikke allerede inkludert
                if len(path) == 0 or path[-1] != goal:
                    path.append(goal)
                self.node.get_logger().info(f'⭐ A* fant rute med {len(path)} steg (distance til goal: {dist_to_goal:.2f})')
                return path
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Utforsk naboer (8-retnings bevegelse)
            for neighbor in self.get_neighbors(current, max_x, max_y):
                if neighbor in closed_set:
                    continue
                
                # Sjekk om nabo er traverserbar
                if not self.is_traversable(neighbor[0], neighbor[1]):
                    continue
                
                # Beregn tentativ g_score
                tentative_g = g_score[current] + self.distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # Denne ruten til neighbor er bedre
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    
                    counter += 1
                    heapq.heappush(open_set, (f, counter, neighbor))
        
        # Ingen rute funnet - debug informasjon
        self.node.get_logger().warn(f'⭐ A* søk fullført uten å finne rute. Utforsket {len(closed_set)} noder.')
        # Prøv å finne nærmeste traversable celle til goal
        best_node = None
        best_dist = float('inf')
        for node in closed_set:
            if self.is_traversable(node[0], node[1]):
                dist = self.distance(node, goal)
                if dist < best_dist:
                    best_dist = dist
                    best_node = node
        
        if best_node:
            self.node.get_logger().info(f'⭐ Nærmeste traversable node til goal: {best_node}, avstand: {best_dist:.2f}')
            # Returner path til nærmeste traversable node
            path = self.reconstruct_path(came_from, best_node)
            path.append(goal)  # Legg til goal selv om ikke traversable
            return path
        
        return []
    
    def get_neighbors(self, node: tuple, max_x: int, max_y: int) -> list:
        """
        Få naboer til en node (8-retninger)
        
        Args:
            node: (map_x, map_y)
            max_x, max_y: Map dimensjoner
        
        Returns:
            list: Liste av nabo-koordinater
        """
        x, y = node
        neighbors = []
        
        # 8-retnings bevegelse
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),  # Cardinal
            (-1, -1), (-1, 1), (1, -1), (1, 1)  # Diagonal
        ]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            # Sjekk bounds
            if 0 <= nx < max_x and 0 <= ny < max_y:
                neighbors.append((nx, ny))
        
        return neighbors
    
    def is_traversable(self, map_x: int, map_y: int) -> bool:
        """
        Sjekk om en map celle er traverserbar (ikke hindring)
        
        Args:
            map_x, map_y: Map koordinater
        
        Returns:
            bool: True hvis traverserbar
        """
        # Sjekk bounds
        map_info = self.occupancy_grid_manager.get_map_info()
        if map_x < 0 or map_x >= map_info['height'] or map_y < 0 or map_y >= map_info['width']:
            return False
        
        # Sjekk om selve cellen er fri eller unknown (unknown behandles som traversable)
        occupancy = self.occupancy_grid_manager.get_occupancy_value(map_x, map_y)
        if occupancy == -1:
            # Unknown område - behandles som traversable
            return True
        if occupancy > 50:  # Obstacle threshold
            return False
        
        # Sjekk inflated område rundt (for robot safety margin) - kun sjekker nærmeste naboer
        if self.INFLATION_RADIUS > 0:
            for dx in range(-self.INFLATION_RADIUS, self.INFLATION_RADIUS + 1):
                for dy in range(-self.INFLATION_RADIUS, self.INFLATION_RADIUS + 1):
                    if dx == 0 and dy == 0:
                        continue
                    
                    check_x = map_x + dx
                    check_y = map_y + dy
                    
                    # Sjekk bounds først
                    if check_x < 0 or check_x >= map_info['height'] or check_y < 0 or check_y >= map_info['width']:
                        continue
                    
                    # Hvis det er en hindring i nærheten, marker som ikke traverserbar
                    check_occupancy = self.occupancy_grid_manager.get_occupancy_value(check_x, check_y)
                    if check_occupancy > 50:  # Obstacle threshold
                        return False
        
        return True
    
    def heuristic(self, a: tuple, b: tuple) -> float:
        """
        Heuristisk funksjon for A* (Euclidean distance)
        
        Args:
            a, b: (map_x, map_y) koordinater
        
        Returns:
            float: Estimert avstand
        """
        return self.distance(a, b)
    
    def distance(self, a: tuple, b: tuple) -> float:
        """
        Beregn Euclidean distance mellom to punkter
        
        Args:
            a, b: (x, y) koordinater
        
        Returns:
            float: Avstand
        """
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def reconstruct_path(self, came_from: dict, current: tuple) -> list:
        """
        Rekonstruer path fra came_from map
        
        Args:
            came_from: Dictionary mapping node -> parent node
            current: Sluttnode
        
        Returns:
            list: Path som liste av (map_x, map_y) koordinater
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def smooth_path(self, path_map: list) -> list:
        """
        Glatt ut path og konverter til world koordinater
        Reduserer antall waypoints ved å kun beholde hjørnepunkter
        
        Args:
            path_map: Path i map koordinater
        
        Returns:
            list: Smoothed path i world koordinater
        """
        if len(path_map) <= 2:
            # For korte paths, konverter bare direkte
            return [
                self.occupancy_grid_manager.map_to_world(x, y) 
                for x, y in path_map
            ]
        
        # Douglas-Peucker-lignende forenkling
        # Beholder kun waypoints der retningen endrer seg betydelig
        smoothed = [path_map[0]]  # Behold start
        
        for i in range(1, len(path_map) - 1):
            prev = path_map[i - 1]
            current = path_map[i]
            next_point = path_map[i + 1]
            
            # Beregn retningsendring
            dir1 = (current[0] - prev[0], current[1] - prev[1])
            dir2 = (next_point[0] - current[0], next_point[1] - current[1])
            
            # Hvis retningen endrer seg, behold waypoint
            if dir1 != dir2:
                # Kun ta med hvert 3. punkt for å redusere støy
                if i % 3 == 0 or self.distance(prev, current) > 5:
                    smoothed.append(current)
        
        smoothed.append(path_map[-1])  # Behold mål
        
        # Konverter til world koordinater
        world_path = [
            self.occupancy_grid_manager.map_to_world(x, y) 
            for x, y in smoothed
        ]
        
        self.node.get_logger().info(f'⭐ Path smoothed fra {len(path_map)} til {len(world_path)} waypoints')
        
        return world_path
    
    def navigate_to_goal(self, msg: LaserScan = None) -> bool:
        """
        Naviger langs planlagt rute
        
        Args:
            msg: LaserScan data (optional, for obstacle avoidance)
        
        Returns:
            bool: True hvis endelig mål er nådd
        """
        if not self.navigation_active:
            return False
        
        # Publiser visualisering periodisk (hver 10. gang ≈ 1Hz hvis scan er 10Hz)
        self.viz_counter += 1
        if self.viz_counter % 10 == 0:
            self.path_visualizer.publish(self.path)
            self.visited_tracker.publish()
        
        # Periodisk replan
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        if (now_sec - self.last_plan_time) >= self.REPLAN_INTERVAL_SEC:
            if self.target_position is not None:
                planned = self.plan_path()
                self.last_plan_time = now_sec
                if not planned:
                    # Ingen path: la koordinatoren falle tilbake til Bug2
                    self.path = []
        
        if not self.path:
            return False
        
        # Sjekk om vi har nådd endelig mål
        distance_to_goal = math.sqrt(
            (self.target_position[0] - self.robot_position[0])**2 +
            (self.target_position[1] - self.robot_position[1])**2
        )
        
        if distance_to_goal <= self.GOAL_THRESHOLD:
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
            
            if distance_to_waypoint <= self.WAYPOINT_THRESHOLD:
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
        if self._should_recover(distance_to_goal):
            self._do_recovery()
            return False
        
        # Naviger mot nåværende waypoint
        self.navigate_to_waypoint(current_waypoint, msg)
        
        # Oppdater fremdrift
        if (self.last_progress_dist - distance_to_goal) >= self.STUCK_PROGRESS_EPS:
            self.last_progress_dist = distance_to_goal
            self.last_progress_time = now_sec
        
        # Marker nåværende celle som besøkt
        self.visited_tracker.mark_current(self.robot_position[0], self.robot_position[1])
        
        return False
    
    def navigate_to_waypoint(self, waypoint: tuple, msg: LaserScan = None):
        """
        Naviger mot et spesifikt waypoint med obstacle avoidance
        
        Args:
            waypoint: (x, y) world koordinater
            msg: LaserScan data (optional)
        """
        # Beregn retning til waypoint
        dx = waypoint[0] - self.robot_position[0]
        dy = waypoint[1] - self.robot_position[1]
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
            # Ingen sensor data
            self.stop_robot()
            return
        
        # Beregn kommandoer
        linear_x, angular_z = self.calculate_commands(dx, dy, distance_to_waypoint, dL, dR, dF)
        
        # Telle blokkert front for fallback
        if dF < self.OBSTACLE_THRESHOLD:
            self.blocked_counter += 1
        else:
            self.blocked_counter = 0
        
        if self.blocked_counter >= self.BLOCKED_LIMIT:
            # Tving fallback til Bug2 i koordinator
            self.node.get_logger().warn('⭐ A*: Front blokkert lenge – tvinger fallback (tømmer path)')
            self.path = []
            self.navigation_active = False
            self.blocked_counter = 0
            return
        
        # Publiser kommandoer
        self.publish_twist(linear_x, angular_z)
    
    def calculate_commands(self, dx: float, dy: float, distance: float,
                          dL: float, dR: float, dF: float) -> tuple:
        """
        Beregn lineær og vinkelhastighet med obstacle avoidance
        
        Args:
            dx, dy: Delta til waypoint
            distance: Avstand til waypoint
            dL, dR, dF: Sensor readings (left, right, front)
        
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
            
            # Logg obstacle avoidance
            if not hasattr(self, '_obstacle_log_counter'):
                self._obstacle_log_counter = 0
            self._obstacle_log_counter += 1
            if self._obstacle_log_counter % 20 == 0:
                self.node.get_logger().warn(f'⭐ Obstacle avoidance: dF={dF:.2f}m')
        else:
            # Ingen hinder - naviger mot waypoint
            desired_heading = math.atan2(dy, dx)
            heading_error = desired_heading - self.robot_orientation
            
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
    
    def visualize_path(self):
        """
        Visualiser planlagt path i RViz med robot ID i namespace for å støtte flere roboter
        
        Publiserer alltid (selv med tom path) for å fjerne gamle markers ved clear_goal()
        """
        marker_array = MarkerArray()
        
        # Unik namespace per robot basert på robot_id
        path_ns = f"astar_path_{self.robot_id}"
        waypoint_ns = f"astar_waypoints_{self.robot_id}"
        
        if self.path:
            # Path line marker med robot-spesifikk namespace
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.node.get_clock().now().to_msg()
            line_marker.ns = path_ns
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05  # Line width
            # Forskjellig farge basert på robot ID (roterende mellom cyan og magenta)
            if '0' in self.robot_id or '2' in self.robot_id or '4' in self.robot_id:
                line_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # Cyan for partall
            else:
                line_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)  # Magenta for oddetall
            
            for wx, wy in self.path:
                p = Point()
                p.x = wx
                p.y = wy
                p.z = 0.1
                line_marker.points.append(p)
            
            marker_array.markers.append(line_marker)
            
            # Waypoint markers med robot-spesifikk namespace
            for i, (wx, wy) in enumerate(self.path):
                wp_marker = Marker()
                wp_marker.header.frame_id = "map"
                wp_marker.header.stamp = self.node.get_clock().now().to_msg()
                wp_marker.ns = waypoint_ns
                wp_marker.id = i + 1
                wp_marker.type = Marker.SPHERE
                wp_marker.action = Marker.ADD
                wp_marker.pose.position.x = wx
                wp_marker.pose.position.y = wy
                wp_marker.pose.position.z = 0.1
                wp_marker.scale.x = 0.2
                wp_marker.scale.y = 0.2
                wp_marker.scale.z = 0.2
                wp_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
                marker_array.markers.append(wp_marker)
        else:
            # Slett gamle markers ved å publisere DELETE action markers
            delete_marker = Marker()
            delete_marker.header.frame_id = "map"
            delete_marker.header.stamp = self.node.get_clock().now().to_msg()
            delete_marker.ns = path_ns
            delete_marker.id = 0
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)
            
            delete_waypoint = Marker()
            delete_waypoint.header.frame_id = "map"
            delete_waypoint.header.stamp = self.node.get_clock().now().to_msg()
            delete_waypoint.ns = waypoint_ns
            delete_waypoint.id = 0
            delete_waypoint.action = Marker.DELETEALL
            marker_array.markers.append(delete_waypoint)
        
        self.path_viz_pub.publish(marker_array)
        
        # Reduser logging for å unngå spam
        if self.path and self.viz_counter % 50 == 0:
            self.node.get_logger().debug(f'⭐ Path visualisert for {self.robot_id}: {len(self.path)} waypoints')
    
    def visualize_visited(self):
        """Visualiser besøkte celler i RViz som små gule prikker."""
        if not self.visited_cells:
            return
        marker_array = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = f"astar_visited_{self.robot_id}"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.08
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=0.9)
        
        # Konverter map-celler til world-senter
        res = self.occupancy_grid_manager.map_msg.info.resolution if self.occupancy_grid_manager and self.occupancy_grid_manager.is_map_available() else 0.05
        for (mx, my), count in list(self.visited_cells.items()):
            wx, wy = self.occupancy_grid_manager.map_to_world(mx, my) if self.occupancy_grid_manager else (0.0, 0.0)
            p = Point()
            p.x = wx + res * 0.5
            p.y = wy + res * 0.5
            p.z = 0.05
            m.points.append(p)
        marker_array.markers.append(m)
        self.visited_viz_pub.publish(marker_array)
    
    def _mark_current_cell_visited(self):
        if not self.occupancy_grid_manager or not self.occupancy_grid_manager.is_map_available():
            return
        mx, my = self.occupancy_grid_manager.world_to_map(self.robot_position[0], self.robot_position[1])
        key = (mx, my)
        self.visited_cells[key] = self.visited_cells.get(key, 0) + 1
    
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
        """Publiserer bevegelseskommandoer"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)
    
    def stop_robot(self):
        """Stopper robot bevegelse"""
        self.publish_twist(0.0, 0.0)

    # --- STUCK / RECOVERY ---
    def _should_recover(self, distance_to_goal: float) -> bool:
        if not hasattr(self, 'last_progress_time'):
            return False
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        if self.last_progress_time == 0.0:
            self.last_progress_time = now_sec
            self.last_progress_dist = distance_to_goal
            return False
        # Hvis vi ikke har gjort nok fremdrift over tid
        if (now_sec - self.last_progress_time) > self.STUCK_TIME_SEC and \
           (self.last_progress_dist - distance_to_goal) < self.STUCK_PROGRESS_EPS:
            return True
        return self.recovering

    def _do_recovery(self):
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        if not self.recovering:
            # Start recovery: rygg 0.4s
            self.recovering = True
            self.recovery_phase = 'back'
            self.recovery_phase_end = now_sec + 0.4
            self.node.get_logger().warn('⭐ A*: STUCK – utfører recovery (rygg)')
            self.publish_twist(-0.1, 0.0)
            return
        
        if self.recovery_phase == 'back':
            if now_sec >= self.recovery_phase_end:
                # Start sving i 0.6s (≈35°)
                self.recovery_phase = 'turn'
                self.recovery_phase_end = now_sec + 0.6
                self.node.get_logger().warn('⭐ A*: Recovery – roterer')
                self.publish_twist(0.0, self.TURN_SPEED * 0.6)
            else:
                self.publish_twist(-0.1, 0.0)
            return
        
        if self.recovery_phase == 'turn':
            if now_sec >= self.recovery_phase_end:
                # Recovery ferdig – be om replan og nullstill fremdrift
                self.recovering = False
                self.recovery_phase = None
                self.last_progress_time = now_sec
                self.last_progress_dist = float('inf')
                # Tving ny plan
                planned = self.plan_path()
                self.last_plan_time = now_sec
                if not planned:
                    # Ingen plan – tøm path for å trigge fallback
                    self.path = []
                    self.navigation_active = False
                self.publish_twist(0.0, 0.0)
            else:
                self.publish_twist(0.0, self.TURN_SPEED * 0.6)


