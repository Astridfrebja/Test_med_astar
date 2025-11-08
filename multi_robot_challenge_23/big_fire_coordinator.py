#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from rclpy.node import Node
from std_msgs.msg import String
from .robot_memory import RobotMemory

class BigFireCoordinator:
    """
    Big Fire koordinering - EN ansvar: Leder & Supporter logikk
    
    Single Responsibility: Kun Big Fire koordinering
    (Nye endringer: Bruker RobotMemory flagg for å redusere repeterende logging)
    """
    
    # States
    NORMAL = "NORMAL"
    LEDER_GOING_TO_FIRE = "LEDER_GOING_TO_FIRE"
    LEDER_WAITING = "LEDER_WAITING"
    SUPPORTER_GOING_TO_FIRE = "SUPPORTER_GOING_TO_FIRE"
    EXTINGUISHING = "EXTINGUISHING"
    
    # Roles
    LEDER = "LEDER"
    SUPPORTER = "SUPPORTER"
    
    def __init__(self, node_ref: Node, robot_memory: RobotMemory):
        self.node = node_ref
        self.robot_id = self.node.get_namespace().strip('/')
        
        # Use shared RobotMemory for state management
        self.memory = robot_memory
        
        # Communication
        self.setup_communication()
        
        self.node.get_logger().info(f'🔥 BigFireCoordinator ({self.robot_id}) initialisert')
        self.node.get_logger().info(f'📡 Topics: /big_fire_detected, /robot_at_fire, /fire_extinguished')

    def setup_communication(self):
        """Sett opp kommunikasjon for Big Fire koordinering"""
        # Publisher for Big Fire detection (global topic for cross-namespace communication)
        self.big_fire_pub = self.node.create_publisher(
            String, '/big_fire_detected', 10
        )
        
        # Subscriber for Big Fire detection (global topic for cross-namespace communication)
        self.big_fire_sub = self.node.create_subscription(
            String, '/big_fire_detected', self.big_fire_callback, 10
        )
        
        # Publisher for robot position at fire (global topic for cross-namespace communication)
        self.fire_position_pub = self.node.create_publisher(
            String, '/robot_at_fire', 10
        )
        
        # Subscriber for robot position at fire (global topic for cross-namespace communication)
        self.fire_position_sub = self.node.create_subscription(
            String, '/robot_at_fire', self.robot_at_fire_callback, 10
        )
        
        # Publisher for fire extinguished (global topic for cross-namespace communication)
        self.fire_extinguished_pub = self.node.create_publisher(
            String, '/fire_extinguished', 10
        )
        
        # Subscriber for fire extinguished (global topic for cross-namespace communication)
        self.fire_extinguished_sub = self.node.create_subscription(
            String, '/fire_extinguished', self.fire_extinguished_callback, 10
        )

    def detect_big_fire(self, position: tuple):
        """Leder oppdager Big Fire"""
        if self.memory.big_fire_state != self.memory.NORMAL:
            return

        self.memory.set_big_fire_detected_by_me(position)
        self.publish_big_fire_detection(position)
        self.node.get_logger().info(f'Leder registrerte big fire ved ({position[0]:.2f}, {position[1]:.2f})')

    def big_fire_callback(self, msg: String):
        """Supporter mottar Big Fire melding fra Leder. Logger KUN første gangen per hendelse."""
        if "BIG_FIRE_DETECTED" in msg.data:
            # Parse position and scout_id
            parts = msg.data.split(':')
            if len(parts) < 3: return # Feil format
            
            position = (float(parts[1]), float(parts[2]))
            scout_id = parts[3] if len(parts) > 3 else "unknown"

            if self.memory.big_fire_state != self.memory.NORMAL:
                return

            self.memory.set_big_fire_detected_by_other(position)
            self.node.get_logger().info(
                f'Supporter {self.robot_id} mottok big fire fra {scout_id} ved ({position[0]:.2f}, {position[1]:.2f})'
            )

    def robot_at_fire_callback(self, msg: String):
        """Håndterer meldinger om at annen robot er ved brannen. Logger KUN ved tilstandsskifte.

        Viktig fiks: Ignorer egen melding. Tidligere sammenlignet vi hele strengen med self.robot_id,
        noe som førte til at lederen kunne tolke sin egen "tb3_0:AT_FIRE" som "annen robot".
        """
        if "AT_FIRE" not in msg.data:
            return
        # Parse avsender-id før sammenligning
        try:
            sender_id = msg.data.split(":")[0]
        except Exception:
            return
        # Bare sett flagg hvis det er en ANNEN robot
        if sender_id != self.robot_id:
            parts = msg.data.split(":")
            if len(parts) >= 4:
                try:
                    x = float(parts[2])
                    y = float(parts[3])
                    self.memory.big_fire_position = (x, y)
                    self.node.get_logger().info(
                        f'Robot {sender_id} er ved brannen, pos=({x:.2f}, {y:.2f})'
                    )
                except ValueError:
                    self.node.get_logger().debug(f'Kunne ikke tolke AT_FIRE posisjon fra "{msg.data}"')
            if not self.memory.other_robot_at_fire:
                self.memory.set_other_robot_at_fire(True)

    def fire_extinguished_callback(self, msg: String):
        """Håndterer meldinger om at brannen er slukket. Logger KUN ved tilstandsskifte."""
        if "FIRE_EXTINGUISHED" in msg.data:
            if not self.memory.fire_extinguished:
                self.memory.set_fire_extinguished(True)
                self.node.get_logger().info('Brannmelding mottatt: slukket')

    def update_state(self, robot_position: tuple, robot_orientation: float):
        """Oppdater Big Fire tilstand basert på posisjon"""
        # Denne er uendret og er ment å være tom eller for fremtidig bruk i en SearchRescueCoordinator
        pass

    def get_target_position(self) -> tuple:
        """Hent målposisjon for navigasjon
        
        Hvis lederen allerede er ved brannen, bruk lederens posisjon i stedet for ArUco merket sin posisjon.
        Dette sikrer at supporteren navigerer til riktig side av veggen.
        """
        # Hvis supporteren skal navigere og lederen er ved brannen, bruk lederens posisjon
        if self.memory.my_role == self.memory.SUPPORTER and \
           self.memory.other_robot_at_fire and \
           self.memory.big_fire_state == self.memory.SUPPORTER_GOING_TO_FIRE:
            # Lederen er ved brannen - bruk lederens posisjon i stedet for ArUco merket
            # Vi bruker Big Fire posisjonen som referanse, men A* vil justere den til lederens faktiske posisjon
            # via leader_position_callback
            return self.memory.big_fire_position
        
        # Ellers bruk ArUco merket sin posisjon (eller Big Fire posisjon)
        return self.memory.big_fire_position

    def should_handle_big_fire(self) -> bool:
        """Sjekk om vi skal håndtere Big Fire koordinering"""
        return self.memory.should_handle_big_fire()

    def is_leder_waiting(self) -> bool:
        """Sjekk om Leder venter på Supporter"""
        return self.memory.is_leder_waiting()

    def is_extinguishing(self) -> bool:
        """Sjekk om vi slukker brannen"""
        return self.memory.is_extinguishing()

    def publish_big_fire_detection(self, position: tuple):
        """Leder publiserer Big Fire detection."""
        msg = String()
        msg.data = f"BIG_FIRE_DETECTED:{position[0]}:{position[1]}:{self.robot_id}"
        self.big_fire_pub.publish(msg)


    def publish_robot_at_fire(self, position: tuple = None):
        """Leder publiserer at den er ved brannen."""
        msg = String()
        if position and len(position) >= 2:
            msg.data = f"{self.robot_id}:AT_FIRE:{float(position[0])}:{float(position[1])}"
            self.memory.big_fire_position = (float(position[0]), float(position[1]))
        else:
            msg.data = f"{self.robot_id}:AT_FIRE"
        self.fire_position_pub.publish(msg)
        if not self.memory.i_am_at_fire:
            self.memory.set_i_am_at_fire(True)
        if position and len(position) >= 2:
            self.node.get_logger().info(f'Sendte AT_FIRE @ ({position[0]:.2f}, {position[1]:.2f})')
        else:
            self.node.get_logger().info('Sendte AT_FIRE')

    def publish_fire_extinguished(self):
        """Publiserer at brannen er slukket. Logges alltid ved utløsning."""
        msg = String()
        msg.data = "FIRE_EXTINGUISHED"
        self.fire_extinguished_pub.publish(msg)
        
        self.memory.set_fire_extinguished(True)
        self.memory.transition_to_normal()
        
        self.node.get_logger().info('Brann slukket – tilbake til utforskning')

    def reset(self):
        """Reset Big Fire koordinering"""
        self.memory.reset_big_fire_state()