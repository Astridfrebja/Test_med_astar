#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AStarPathfinder - EN ansvar: Kun A* søkealgoritme

Single Responsibility: Kun A* søkealgoritme implementasjon
"""

import math
import heapq
from rclpy.node import Node


class AStarPathfinder:
    """
    A* Pathfinding Algorithm - Kun søkealgoritmen
    
    Single Responsibility: Kun A* søkealgoritme
    """
    
    def __init__(self, node: Node, is_traversable_callback, get_neighbors_callback):
        """
        Args:
            node: ROS2 node for logging
            is_traversable_callback: Callback funksjon (map_x, map_y) -> bool
            get_neighbors_callback: Callback funksjon (node, max_x, max_y) -> list
        """
        self.node = node
        self.is_traversable = is_traversable_callback
        self.get_neighbors = get_neighbors_callback
    
    def search(self, start: tuple, goal: tuple, max_x: int, max_y: int) -> list:
        """
        A* søkealgoritme
        
        Args:
            start: (map_x, map_y) startposisjon
            goal: (map_x, map_y) målposisjon
            max_x, max_y: Map dimensjoner
            
        Returns:
            list: Path som liste av (map_x, map_y) koordinater, eller []
        """
        # A* datastrukturer
        came_from = {}  # node -> parent node
        g_score = {start: 0}  # node -> cost from start
        f_score_start = self.heuristic(start, goal)
        f_score = {start: f_score_start}  # node -> estimated total cost
        
        open_set = []  # Priority queue (f_score, counter, node)
        heapq.heappush(open_set, (f_score_start, 0, start))
        
        counter = 0  # For å bryte likhet in priority queue
        closed_set = set()
        open_set_dict = {start}  # Track nodes in open_set for O(1) lookup
        
        # Debug: Log start av søk
        self.node.get_logger().debug(f'⭐ A* søk: start={start}, goal={goal}, map_size={max_x}x{max_y}')
        
        iterations = 0
        # Beregn maks iterations basert på avstand (sikkerhetsmargin)
        estimated_distance = self.distance(start, goal)
        max_iterations = int(estimated_distance * 50)  # 50x avstand som sikkerhetsmargin
        max_iterations = max(50000, min(max_iterations, 200000))  # Min 50k, max 200k
        
        while open_set:
            iterations += 1
            if iterations > max_iterations:
                self.node.get_logger().warn(f'⭐ A*: Max iterations ({max_iterations}) nådd, avbryter søk')
                break
            
            # Sjekk om open_set er tom (skal ikke skje siden vi sjekker i while)
            if len(open_set) == 0:
                self.node.get_logger().warn('⭐ A*: Open set er tom - ingen rute funnet')
                break
                
            _, _, current = heapq.heappop(open_set)
            
            # Fjern fra open_set tracking
            if current in open_set_dict:
                open_set_dict.remove(current)
            
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
            neighbors = self.get_neighbors(current, max_x, max_y)
            traversable_neighbors = [n for n in neighbors if self.is_traversable(n[0], n[1])]
            
            # Log periodisk for å se fremgang
            if iterations % 5000 == 0:
                # Beregn fremgang (hvor nærmere målet vi har kommet)
                initial_dist = self.distance(start, goal)
                current_dist = self.distance(current, goal)
                progress_pct = ((initial_dist - current_dist) / initial_dist * 100) if initial_dist > 0 else 0
                
                # Sjekk om vi har noen traversable neighbors i det hele tatt
                if len(traversable_neighbors) == 0:
                    self.node.get_logger().warn(
                        f'⭐ A*: Iteration {iterations}, INGEN TRAVERSABLE NEIGHBORS! '
                        f'current={current}, open_set={len(open_set)}, closed_set={len(closed_set)}, '
                        f'dist_to_goal={current_dist:.1f}, progress={progress_pct:.1f}%'
                    )
                else:
                    self.node.get_logger().info(
                        f'⭐ A*: Iteration {iterations}, open_set={len(open_set)}, '
                        f'closed_set={len(closed_set)}, current={current}, '
                        f'dist_to_goal={current_dist:.1f} (start_dist={initial_dist:.1f}, progress={progress_pct:.1f}%), '
                        f'traversable_neighbors={len(traversable_neighbors)}/{len(neighbors)}'
                    )
            
            for neighbor in traversable_neighbors:
                if neighbor in closed_set:
                    continue
                
                # Beregn tentativ g_score
                tentative_g = g_score[current] + self.distance(current, neighbor)
                
                # Beregn f_score for neighbor
                f_new = tentative_g + self.heuristic(neighbor, goal)
                
                # Hvis neighbor ikke er i open_set eller vi har funnet en bedre rute
                if neighbor not in open_set_dict:
                    # Ny node - legg til
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = f_new
                    open_set_dict.add(neighbor)
                    counter += 1
                    heapq.heappush(open_set, (f_new, counter, neighbor))
                elif tentative_g < g_score.get(neighbor, float('inf')):
                    # Bedre rute funnet - oppdater
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = f_new
                    # Legg til igjen med ny score (heapq vil håndtere dette)
                    counter += 1
                    heapq.heappush(open_set, (f_new, counter, neighbor))
        
        # Ingen rute funnet - debug informasjon
        self.node.get_logger().warn(
            f'⭐ A* søk fullført uten å finne rute. Utforsket {len(closed_set)} noder, '
            f'iterations={iterations}, start={start}, goal={goal}'
        )
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
            # Hvis nærmeste node er for langt fra goal (>10 celler), anse søket som feilet
            if best_dist > 10.0:
                self.node.get_logger().warn(
                    f'⭐ A*: Nærmeste traversable node er for langt fra goal ({best_dist:.1f} > 10), '
                    f'returnerer tom path for fallback til Bug2'
                )
                return []
            # Returner path til nærmeste traversable node
            path = self.reconstruct_path(came_from, best_node)
            path.append(goal)  # Legg til goal selv om ikke traversable
            return path
        
        return []
    
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

