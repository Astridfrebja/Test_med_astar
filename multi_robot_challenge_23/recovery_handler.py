#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RecoveryHandler - EN ansvar: Stuck detection og recovery

Single Responsibility: Kun stuck detection og recovery logikk
"""

from rclpy.node import Node


class RecoveryHandler:
    """
    Stuck Detection and Recovery - Håndterer når roboten er fast
    
    Single Responsibility: Kun stuck detection og recovery
    """
    
    STUCK_TIME_SEC = 2.0  # hvor lenge uten fremdrift før recovery
    STUCK_PROGRESS_EPS = 0.10  # meter forbedring som teller som fremdrift
    TURN_SPEED = 0.5
    
    def __init__(self, node: Node, publish_twist_callback, plan_path_callback):
        """
        Args:
            node: ROS2 node for logging
            publish_twist_callback: Callback for å publisere twist (linear_x, angular_z)
            plan_path_callback: Callback for å replan path
        """
        self.node = node
        self.publish_twist = publish_twist_callback
        self.plan_path = plan_path_callback
        
        self.recovering = False
        self.recovery_phase = None  # 'back' eller 'turn'
        self.recovery_phase_end = 0.0
        self.last_progress_time = 0.0
        self.last_progress_dist = float('inf')
    
    def should_recover(self, distance_to_goal: float, current_time: float) -> bool:
        """
        Sjekk om roboten skal utføre recovery
        
        Args:
            distance_to_goal: Nåværende avstand til mål
            current_time: Nåværende tid (seconds)
        
        Returns:
            bool: True hvis recovery skal utføres
        """
        if self.last_progress_time == 0.0:
            self.last_progress_time = current_time
            self.last_progress_dist = distance_to_goal
            return False
        
        # Hvis vi ikke har gjort nok fremdrift over tid
        if (current_time - self.last_progress_time) > self.STUCK_TIME_SEC and \
           (self.last_progress_dist - distance_to_goal) < self.STUCK_PROGRESS_EPS:
            return True
        
        return self.recovering
    
    def do_recovery(self, current_time: float) -> bool:
        """
        Utfør recovery handling
        
        Args:
            current_time: Nåværende tid (seconds)
        
        Returns:
            bool: True hvis recovery er ferdig og path skal replannes
        """
        if not self.recovering:
            # Start recovery: rygg 0.4s
            self.recovering = True
            self.recovery_phase = 'back'
            self.recovery_phase_end = current_time + 0.4
            self.node.get_logger().warn('⭐ A*: STUCK – utfører recovery (rygg)')
            self.publish_twist(-0.1, 0.0)
            return False
        
        if self.recovery_phase == 'back':
            if current_time >= self.recovery_phase_end:
                # Start sving i 0.6s (≈35°)
                self.recovery_phase = 'turn'
                self.recovery_phase_end = current_time + 0.6
                self.node.get_logger().warn('⭐ A*: Recovery – roterer')
                self.publish_twist(0.0, self.TURN_SPEED * 0.6)
            else:
                self.publish_twist(-0.1, 0.0)
            return False
        
        if self.recovery_phase == 'turn':
            if current_time >= self.recovery_phase_end:
                # Recovery ferdig – be om replan og nullstill fremdrift
                self.recovering = False
                self.recovery_phase = None
                self.last_progress_time = current_time
                self.last_progress_dist = float('inf')
                self.publish_twist(0.0, 0.0)
                return True  # Signaliser at path skal replannes
            else:
                self.publish_twist(0.0, self.TURN_SPEED * 0.6)
            return False
        
        return False
    
    def update_progress(self, distance_to_goal: float, current_time: float):
        """Oppdater fremdrift tracking"""
        if (self.last_progress_dist - distance_to_goal) >= self.STUCK_PROGRESS_EPS:
            self.last_progress_dist = distance_to_goal
            self.last_progress_time = current_time
    
    def reset(self):
        """Reset recovery state"""
        self.recovering = False
        self.recovery_phase = None
        self.last_progress_time = 0.0
        self.last_progress_dist = float('inf')

