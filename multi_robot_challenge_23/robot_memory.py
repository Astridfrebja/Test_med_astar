#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class RobotMemory:
    """
    Robot Memory - EN ansvar: Kun robotens minne og tilstand
    
    Single Responsibility: Kun å huske robotens tilstand og fase
    """
    
    # Big Fire States
    NORMAL = "NORMAL"
    LEDER_GOING_TO_FIRE = "LEDER_GOING_TO_FIRE"
    LEDER_WAITING = "LEDER_WAITING"
    SUPPORTER_GOING_TO_FIRE = "SUPPORTER_GOING_TO_FIRE"
    EXTINGUISHING = "EXTINGUISHING"
    
    # Roles
    LEDER = "LEDER"
    SUPPORTER = "SUPPORTER"
    
    def __init__(self):
        # Robot position tracking
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0
        
        # Big Fire state
        self.big_fire_position = None
        self.big_fire_detected_by_me = False
        self.big_fire_detected_by_other = False
        self.other_robot_at_fire = False
        self.i_am_at_fire = False
        self.fire_extinguished = False
        
        # Leder & Supporter roles
        self.my_role = None
        self.big_fire_state = self.NORMAL

        # ArUco processing bookkeeping (avoid duplicate handling)
        self._processed_aruco_ids = set()

    def set_big_fire_detected_by_me(self, position: tuple):
        """Sett Big Fire oppdaget av denne roboten"""
        # Note: Kan ikke bruke self.node.get_logger() her siden RobotMemory ikke har node reference
        self.big_fire_detected_by_me = True
        self.big_fire_position = position
        self.my_role = self.LEDER
        # LEDER skal stoppe og vente umiddelbart, ikke navigere
        self.big_fire_state = self.LEDER_WAITING

    def set_big_fire_detected_by_other(self, position: tuple):
        """Sett Big Fire oppdaget av annen robot"""
        self.big_fire_detected_by_other = True
        self.big_fire_position = position
        self.my_role = self.SUPPORTER
        self.big_fire_state = self.SUPPORTER_GOING_TO_FIRE

    def transition_to_leder_waiting(self):
        """Transition til Leder venting"""
        self.big_fire_state = self.LEDER_WAITING

    def transition_to_extinguishing(self):
        """Transition til slukking"""
        self.big_fire_state = self.EXTINGUISHING

    def transition_to_normal(self):
        """Transition til normal tilstand"""
        self.big_fire_state = self.NORMAL
        self.big_fire_detected_by_me = False
        self.big_fire_detected_by_other = False
        self.big_fire_position = None
        self.other_robot_at_fire = False
        self.i_am_at_fire = False
        self.fire_extinguished = False

    def set_other_robot_at_fire(self, value: bool):
        """Sett om annen robot er ved brannen"""
        self.other_robot_at_fire = value

    def set_i_am_at_fire(self, value: bool):
        """Sett om jeg er ved brannen"""
        self.i_am_at_fire = value

    def set_fire_extinguished(self, value: bool):
        """Sett om brannen er slukket"""
        self.fire_extinguished = value

    def should_handle_big_fire(self) -> bool:
        """Sjekk om vi skal håndtere Big Fire koordinering"""
        return (
            self.big_fire_detected_by_me
            or self.big_fire_detected_by_other
            or self.big_fire_state != self.NORMAL
        )

    # --- Convenience helpers used by coordinator ---
    def is_moving_to_fire(self) -> bool:
        """Sjekk om vi er i en bevegelsesfase mot brannen."""
        return self.big_fire_state in [self.LEDER_GOING_TO_FIRE, self.SUPPORTER_GOING_TO_FIRE]

    def transition_to_leder_going_to_fire(self):
        """Sett state når leder skal gå mot brannen."""
        self.big_fire_state = self.LEDER_GOING_TO_FIRE

    def transition_to_supporter_going_to_fire(self):
        """Sett state når supporter skal gå mot brannen."""
        self.big_fire_state = self.SUPPORTER_GOING_TO_FIRE

    def is_leder_waiting(self) -> bool:
        """Sjekk om Leder venter"""
        return self.big_fire_state == self.LEDER_WAITING

    def is_extinguishing(self) -> bool:
        """Sjekk om vi slukker brannen"""
        return self.big_fire_state == self.EXTINGUISHING

    def is_leder_going_to_fire(self) -> bool:
        """Sjekk om Leder går til brannen"""
        return self.big_fire_state == self.LEDER_GOING_TO_FIRE

    def is_supporter_going_to_fire(self) -> bool:
        """Sjekk om Supporter går til brannen"""
        return self.big_fire_state == self.SUPPORTER_GOING_TO_FIRE

    def update_robot_pose(self, position: tuple, orientation: float):
        """Oppdater robot posisjon og orientering"""
        self.robot_position = position
        self.robot_orientation = orientation

    def reset_big_fire_state(self):
        """Reset Big Fire state"""
        self.big_fire_position = None
        self.big_fire_detected_by_me = False
        self.big_fire_detected_by_other = False
        self.other_robot_at_fire = False
        self.i_am_at_fire = False
        self.fire_extinguished = False
        self.my_role = None
        self.big_fire_state = self.NORMAL

    # --- ArUco processing helpers ---
    def is_aruco_processed(self, marker_id: int) -> bool:
        """Returner om gitt ArUco-ID allerede er behandlet."""
        return marker_id in self._processed_aruco_ids

    def mark_aruco_processed(self, marker_id: int) -> None:
        """Merk en ArUco-ID som behandlet."""
        self._processed_aruco_ids.add(marker_id)
