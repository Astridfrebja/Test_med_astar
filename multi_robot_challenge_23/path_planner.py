#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PathPlanner - EN ansvar: Path planning logikk

Single Responsibility: Kun path planning (traversability, goal adjustment, path smoothing)
"""

import math
from rclpy.node import Node


class PathPlanner:
    """
    Path Planning Logic - Håndterer traversability, goal adjustment, og path smoothing
    
    Single Responsibility: Kun path planning logikk
    """
    
    INFLATION_RADIUS = 0  # Grid cells å inflere rundt hindringer (0 = kun sjekk selve cellen)
    
    def __init__(self, node: Node, occupancy_grid_manager, leader_position_callback=None):
        """
        Args:
            node: ROS2 node for logging
            occupancy_grid_manager: OccupancyGridManager instance
            leader_position_callback: Optional callback for å hente lederens posisjon
        """
        self.node = node
        self.occupancy_grid_manager = occupancy_grid_manager
        self.leader_position_callback = leader_position_callback
    
    def is_traversable(self, map_x: int, map_y: int) -> bool:
        """
        Sjekk om en map celle er traverserbar (ikke hindring)
        Bruker en liten mask (3x3) for å fjerne støy og gjøre traversability check mer robust
        
        Args:
            map_x, map_y: Map koordinater
        
        Returns:
            bool: True hvis traverserbar
        """
        # Sjekk bounds
        map_info = self.occupancy_grid_manager.get_map_info()
        if map_x < 0 or map_x >= map_info['height'] or map_y < 0 or map_y >= map_info['width']:
            return False
        
        # Bruk en liten mask (3x3) for å fjerne støy i map data
        # Dette ligner på MapFilterClass, men vi gjør det "on-the-fly" uten å lagre et nytt map
        mask_size = 3
        mask_radius = mask_size // 2  # 1 for 3x3 mask
        
        occupancy_sum = 0
        valid_cells = 0
        
        # Gå gjennom masken
        for dx in range(-mask_radius, mask_radius + 1):
            for dy in range(-mask_radius, mask_radius + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                
                # Sjekk bounds
                if check_x < 0 or check_x >= map_info['height'] or check_y < 0 or check_y >= map_info['width']:
                    continue
                
                occupancy = self.occupancy_grid_manager.get_occupancy_value(check_x, check_y)
                
                # Ignorer unknown celler i gjennomsnittet (de teller ikke med)
                if occupancy == -1:
                    continue
                
                occupancy_sum += occupancy
                valid_cells += 1
        
        # Hvis ingen gyldige celler (alle unknown), behandles som ikke traversable
        if valid_cells == 0:
            return False
        
        # Beregn gjennomsnittlig occupancy i masken
        avg_occupancy = occupancy_sum / valid_cells
        
        # Hvis gjennomsnitt >= 50, er det en hindring
        if avg_occupancy >= 50:
            return False
        
        return True
    
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
    
    def find_nearest_traversable_towards_goal(self, start_map: tuple, goal_map: tuple, max_search_radius: int = 15) -> tuple:
        """
        Finn nærmeste traversable punkt i retning fra start til mål
        Dette sikrer at roboten kommer til riktig side av veggen
        
        Args:
            start_map: (map_x, map_y) startposisjon
            goal_map: (map_x, map_y) målposisjon (kan være på hindring)
            max_search_radius: Maksimal søkeradius i grid cells
        
        Returns:
            tuple: (map_x, map_y) nærmeste traversable punkt, eller None
        """
        goal_x, goal_y = goal_map
        start_x, start_y = start_map
        
        # Beregn retning fra start til mål
        dx_total = goal_x - start_x
        dy_total = goal_y - start_y
        dist_total = math.sqrt(dx_total*dx_total + dy_total*dy_total)
        
        if dist_total == 0:
            # Fallback til vanlig søk
            return self.find_nearest_traversable(goal_map, max_search_radius)
        
        # Normaliser retning
        dir_x = dx_total / dist_total
        dir_y = dy_total / dist_total
        
        best_point = None
        best_score = float('inf')  # Lavere score = bedre (nærmere mål og i retning)
        
        # Søk i koncentriske sirkler rundt målet, men prioriter punkter i retning fra start
        for radius in range(1, max_search_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    check_x = goal_x + dx
                    check_y = goal_y + dy
                    
                    map_info = self.occupancy_grid_manager.get_map_info()
                    if check_x < 0 or check_x >= map_info['height'] or check_y < 0 or check_y >= map_info['width']:
                        continue
                    
                    if self.is_traversable(check_x, check_y):
                        # Beregn avstand til goal
                        dist_to_goal = math.sqrt(dx*dx + dy*dy)
                        
                        # Beregn vektor fra start til dette punktet
                        vec_to_point = (check_x - start_x, check_y - start_y)
                        vec_len = math.sqrt(vec_to_point[0]*vec_to_point[0] + vec_to_point[1]*vec_to_point[1])
                        
                        if vec_len > 0:
                            # Normaliser vektor
                            vec_to_point = (vec_to_point[0] / vec_len, vec_to_point[1] / vec_len)
                            
                            # Priorieter punkter i retning fra start til mål
                            dot_product = vec_to_point[0] * dir_x + vec_to_point[1] * dir_y
                            
                            # Score: lavere er bedre (prioriter nærmere mål og i retning)
                            score = dist_to_goal - (dot_product * 2.0)  # Bonus for å være i retning
                            
                            if score < best_score:
                                best_score = score
                                best_point = (check_x, check_y)
            
            # Hvis vi fant et godt punkt i denne radiusen, returner det
            if best_point and best_score < 5.0:  # Terskel for "godt nok"
                return best_point
        
        # Returner beste funnet punkt (selv om score ikke er perfekt)
        return best_point
    
    def find_nearest_traversable(self, goal_map: tuple, max_search_radius: int = 10) -> tuple:
        """
        Finn nærmeste traversable punkt til målet
        
        Args:
            goal_map: (map_x, map_y) målposisjon
            max_search_radius: Maksimal søkeradius i grid cells
        
        Returns:
            tuple: (map_x, map_y) nærmeste traversable punkt, eller None
        """
        goal_x, goal_y = goal_map
        best_point = None
        best_distance = float('inf')
        
        # Søk i koncentriske sirkler rundt målet (fra nærmeste til lengst unna)
        for radius in range(1, max_search_radius + 1):
            # Sjekk alle punkter i denne radiusen
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    # Sjekk bounds først
                    check_x = goal_x + dx
                    check_y = goal_y + dy
                    
                    map_info = self.occupancy_grid_manager.get_map_info()
                    if check_x < 0 or check_x >= map_info['height'] or check_y < 0 or check_y >= map_info['width']:
                        continue
                    
                    if self.is_traversable(check_x, check_y):
                        distance = self.distance(goal_map, (check_x, check_y))
                        if distance < best_distance:
                            best_distance = distance
                            best_point = (check_x, check_y)
            
            # Hvis vi fant et punkt i denne radiusen, returner det (nærmeste er funnet)
            if best_point:
                return best_point
        
        return None
    
    def distance(self, a: tuple, b: tuple) -> float:
        """Beregn Euclidean distance mellom to punkter"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def smooth_path(self, path_map: list) -> list:
        """
        Glatt ut path og konverter til world koordinater
        Reduserer antall waypoints ved å kun beholde hjørnepunkter
        
        Args:
            path_map: Path i map koordinater
        
        Returns:
            list: Path i world koordinater med færre waypoints
        """
        if not path_map or len(path_map) < 2:
            return []
        
        # Forenkling: behold kun hjørnepunkter (hvor retningen endrer seg)
        smoothed = [path_map[0]]  # Behold start
        
        for i in range(1, len(path_map) - 1):
            prev = path_map[i - 1]
            current = path_map[i]
            next_point = path_map[i + 1]
            
            # Beregn retninger
            dir1 = (current[0] - prev[0], current[1] - prev[1])
            dir2 = (next_point[0] - current[0], next_point[1] - current[1])
            
            # Normaliser retninger
            len1 = math.sqrt(dir1[0]*dir1[0] + dir1[1]*dir1[1])
            len2 = math.sqrt(dir2[0]*dir2[0] + dir2[1]*dir2[1])
            
            if len1 > 0:
                dir1 = (dir1[0] / len1, dir1[1] / len1)
            if len2 > 0:
                dir2 = (dir2[0] / len2, dir2[1] / len2)
            
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

