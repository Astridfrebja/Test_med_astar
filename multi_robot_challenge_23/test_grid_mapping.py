#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script for OccupancyGridManager mapping functions
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from .occupancy_grid_manager import OccupancyGridManager


def main():
    rclpy.init()
    
    # Create a test node
    node = Node('grid_test_node')
    
    # Create OccupancyGridManager with same QoS as in the actual code
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        depth=5,
    )
    
    manager = OccupancyGridManager(node)
    
    # Wait for map to be available (spin until we get it)
    print("⏳ Venter på kart fra /map topic...")
    timeout_sec = 10.0
    start_time = node.get_clock().now().nanoseconds / 1e9
    map_received = False
    
    while (node.get_clock().now().nanoseconds / 1e9 - start_time) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        if manager.is_map_available():
            map_received = True
            break
    
    if not map_received:
        print("❌ FEIL: Fikk ikke kart innen 10 sekunder!")
        print("   Sjekk at map_server kjører og publiserer på /map topic")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    print("✅ Kart mottatt!")
    map_info = manager.get_map_info()
    print(f"   Kartstørrelse: {map_info.get('width', '?')}x{map_info.get('height', '?')}")
    print(f"   Oppløsning: {map_info.get('resolution', '?')} m/celle")
    print(f"   Origin: ({map_info.get('origin_x', '?')}, {map_info.get('origin_y', '?')})")
    print()
    
    # Test mapping functions
    print("=== YAML / Kart mapping-test ===")
    
    test_points = [
        (0.0, 0.0),
        (1.0, 1.0),
        (5.0, 5.0),
        (-10.0, -10.0),
        (2.24, -1.26),
        (1.07, 4.20),
    ]
    
    for world_x, world_y in test_points:
        # World to map
        map_coords = manager.world_to_map(world_x, world_y)
        occupancy = manager.get_occupancy_value(map_coords[0], map_coords[1])
        
        # Map back to world
        world_back = manager.map_to_world(map_coords[0], map_coords[1])
        
        print(f"- World ({world_x:.2f}, {world_y:.2f}) -> Map cell {map_coords} – occupancy: {occupancy}")
        print(f"    Map cell {map_coords} -> World ({world_back[0]:.2f}, {world_back[1]:.2f})")
    
    print("=== Slutt på mapping-test ===")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

