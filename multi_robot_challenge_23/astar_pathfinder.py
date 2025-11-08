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
        # Beregn maks iterations basert på avstand
        # Redusert fra 500x til 150x for å unngå unødvendig lange søk
        estimated_distance = self.distance(start, goal)
        max_iterations = max(10000, min(int(estimated_distance * 150), 150000))  # Min 10k, max 150k
        self.node.get_logger().info(f'⭐ A*: Starter søk fra {start} til {goal}. Max iterations: {max_iterations}')
        
        while open_set:
            iterations += 1
            if iterations > max_iterations:
                self.node.get_logger().warn(f'⭐ A*: Max iterations ({max_iterations}) nådd, avbryter søk')
                break
            
            # Pop node fra heap - hopp over duplikater som allerede er prosessert
            current = None
            while open_set:
                _, _, candidate = heapq.heappop(open_set)
                # Hvis noden ikke er i closed_set og er i open_set_dict, prosesser den
                if candidate not in closed_set and candidate in open_set_dict:
                    current = candidate
                    break
                # Hvis noden allerede er prosessert (duplikat i heap), hopp over
                # og fortsett til neste node
            
            if current is None:
                # open_set er tom eller alle noder er allerede prosessert - ingen rute funnet
                self.node.get_logger().warn('⭐ A*: Open set er tom eller alle noder prosessert - ingen rute funnet')
                break
            
            # Fjern fra open_set tracking
            open_set_dict.remove(current)
            
            # Legg til i closed_set FØR vi prosesserer
            closed_set.add(current)
            
            # Sjekk om vi har nådd målet - mer presis sjekk
            dist_to_goal = self.distance(current, goal)
            if dist_to_goal <= 1.5:  # Tillat nærliggende celler (1.5 grid cells)
                # Rekonstruer path
                path = self.reconstruct_path(came_from, current)
                # Legg til goal hvis ikke allerede inkludert
                if len(path) == 0 or path[-1] != goal:
                    path.append(goal)
                self.node.get_logger().info(f'⭐ A* fant rute med {len(path)} steg (distance til goal: {dist_to_goal:.2f})')
                return path
            
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
                
                # Corner detection - forhindre å kutte hjørner gjennom vegger
                # Hvis vi beveger oss diagonalt, sjekk at begge nabo-celler er traversable
                dx = neighbor[0] - current[0]
                dy = neighbor[1] - current[1]
                
                # Hvis diagonal bevegelse, sjekk hjørner
                if abs(dx) == 1 and abs(dy) == 1:
                    # Sjekk at begge nabo-celler er traversable (forhindrer å kutte hjørner)
                    corner1 = (current[0] + dx, current[1])  # Kardinal nabo 1
                    corner2 = (current[0], current[1] + dy)    # Kardinal nabo 2
                    
                    # Sjekk bounds først
                    if (0 <= corner1[0] < max_x and 0 <= corner1[1] < max_y and
                        0 <= corner2[0] < max_x and 0 <= corner2[1] < max_y):
                        # Hvis en av nabo-cellene er en hindring, kan vi ikke gå diagonalt
                        if not self.is_traversable(corner1[0], corner1[1]) or \
                           not self.is_traversable(corner2[0], corner2[1]):
                            continue  # Hopp over denne diagonal bevegelsen
                
                # Beregn kostnad (g_score) - bruk diagonal distance for diagonal bevegelse
                if abs(dx) == 1 and abs(dy) == 1:
                    # Diagonal bevegelse - kostnad sqrt(2) ≈ 1.414
                    move_cost = 1.414
                else:
                    # Kardinal bevegelse - kostnad 1.0
                    move_cost = 1.0
                
                tentative_g = g_score.get(current, float('inf')) + move_cost
                
                # Sjekk om vi har funnet en bedre rute til denne noden
                if neighbor in open_set_dict:
                    # Node er allerede i open_set - sjekk om ny rute er bedre
                    if tentative_g >= g_score.get(neighbor, float('inf')):
                        continue  # Ny rute er ikke bedre, hopp over
                    # Ny rute er bedre - oppdater (gamle entry i heap vil bli ignorert når den poppes)
                
                # Oppdater came_from, g_score
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                
                # Heuristikk (må være admissible - aldri overestimere)
                # Bruker Euclidean distance (admissible for 8-retnings bevegelse)
                h = self.heuristic(neighbor, goal)
                
                # Tie-breaking: Legg til en liten verdi basert på avstand til mål
                # Dette favoriserer noder nærmere målet når f_score er lik
                # Bruker en liten epsilon (0.001) for å unngå å bryte admissibility
                # Men hjelper med å unngå sirkulær utforskning
                tie_breaker = h * 0.001
                f_new = tentative_g + h + tie_breaker
                
                # Oppdater f_score
                f_score[neighbor] = f_new
                
                if neighbor not in open_set_dict:
                    # Ny node - legg til i open_set
                    open_set_dict.add(neighbor)
                    counter += 1
                    heapq.heappush(open_set, (f_new, counter, neighbor))
                else:
                    # Bedre rute funnet - legg til ny entry i heap
                    # Den gamle, dårligere verdien vil bli ignorert når den poppes (sjekket i while loop)
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
                    'returnerer tom path for alternativ strategi'
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

