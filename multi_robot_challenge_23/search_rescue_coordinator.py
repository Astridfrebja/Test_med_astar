#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from rclpy.node import Node
from scoring_interfaces.srv import SetMarkerPosition
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Importer alle komponenter
from .wall_follower import WallFollower
from .goal_navigator import GoalNavigator
from .bug2_navigator import Bug2Navigator
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
        self.goal_navigator = GoalNavigator(node_ref, self.sensor_manager)
        self.bug2_navigator = Bug2Navigator(node_ref, self.wall_follower, self.goal_navigator)

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
                # Lederen er ved brannen - returner Big Fire posisjon som referanse
                # Dette er ikke ArUco merket sin posisjon, men posisjonen lederen publiserte
                # når den oppdaget Big Fire. A* vil finne et traversable punkt nær denne.
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

        # Plan pause: støtte for å pause supporteren kort mens rute planlegges
        self.planning_pause_until = 0.0
        self.planning_pause_initialized = False


    def process_scan(self, msg: LaserScan):
        """
        Hovedfunksjon - koordinerer navigasjon. Kaller KUN ÉN navigasjonskontroller per syklus.
        """
        # Sjekk om map er tilgjengelig for A* navigasjon
        map_available = self.sensor_manager.is_map_available()
        if not self.use_astar_for_big_fire and map_available:
            self.use_astar_for_big_fire = True
            map_info = self.sensor_manager.get_map_info()
            self.node.get_logger().info(f'⭐ Map tilgjengelig! A* navigasjon aktivert for Big Fire. Map: {map_info.get("width", "?")}x{map_info.get("height", "?")}, res={map_info.get("resolution", "?")}')

        # Log map-status hvis Big Fire er aktivt men map mangler
        if not map_available:
            # Log kun ved Big Fire-situasjoner for å unngå spam
            if self.big_fire_coordinator.should_handle_big_fire():
                if not hasattr(self, '_last_map_warn_time') or \
                   (self.node.get_clock().now().nanoseconds / 1e9 - getattr(self, '_last_map_warn_time', 0)) > 5.0:
                    self.node.get_logger().warn('⚠️ Map IKKE tilgjengelig! A* kan ikke brukes. Sjekk om map_server kjører og publiserer på /map topic.')
                    self._last_map_warn_time = self.node.get_clock().now().nanoseconds / 1e9

        # Oppdater tilstanden (viktig å gjøre FØR navigasjon sjekkes)
        self.big_fire_coordinator.update_state(self.robot_position, self.robot_orientation)

        # Sjekk om Big Fire skal håndteres (kun basert på Big Fire state, ikke A* aktivitet)
        big_fire_active = self.big_fire_coordinator.should_handle_big_fire()

        if big_fire_active:
            self.node.get_logger().debug('🔥 BIG FIRE KOORDINERING AKTIV')

            # Sjekk om vi er LEDER som venter - skal IKKE navigere
            if self.robot_memory.big_fire_state == self.robot_memory.LEDER_WAITING:
                # LEDER skal stoppe og vente - ikke bevege seg
                self.astar_navigator.stop_robot()
                self.bug2_navigator.stop_robot()
                self.wall_follower.stop_robot()
                self.handle_big_fire_state_logic()
                return

            # Hent målet for Big Fire navigasjon
            target = self.big_fire_coordinator.get_target_position()

            # KORRIGERT: Kun SUPPORTER skal navigere. LEDER venter allerede.
            if target and self.robot_memory.is_moving_to_fire():

                # Bruk A* for Big Fire navigasjon når map er tilgjengelig (ingen Bug2-fallback her)
                # Dobbeltsjekk map-tilgjengelighet (kan ha forandret seg)
                map_currently_available = self.sensor_manager.is_map_available()

                # DEBUG: Log status for diagnose
                self.node.get_logger().debug(
                    f'🔥 NAVIGASJON STATUS: target={target}, '
                    f'use_astar={self.use_astar_for_big_fire}, '
                    f'map_available={map_currently_available}, '
                    f'state={self.robot_memory.big_fire_state}, '
                    f'moving_to_fire={self.robot_memory.is_moving_to_fire()}'
                )

                if self.use_astar_for_big_fire and map_currently_available:
                    # Sørg for at kun A* publiserer cmd_vel i denne fasen
                    try:
                        self.wall_follower.stop_robot()
                        self.bug2_navigator.stop_robot()
                    except Exception:
                        pass
                    # Eventuell plan-pause etter mottatt beskjed
                    now_sec = self.node.get_clock().now().nanoseconds / 1e9
                    # Initier pause første gang vi kommer inn i SUPPORTER_GOING_TO_FIRE
                    if (not self.planning_pause_initialized) and \
                       (self.robot_memory.big_fire_state == self.robot_memory.SUPPORTER_GOING_TO_FIRE):
                        self.planning_pause_until = now_sec + 0.3
                        self.planning_pause_initialized = True
                        self.astar_navigator.stop_robot()
                        self.wall_follower.stop_robot()
                        self.bug2_navigator.stop_robot()
                        self.node.get_logger().info('⭐ SUPPORTER: Mottatt koordinater – stopper og planlegger A* rute...')
                        return

                    if now_sec < self.planning_pause_until:
                        self.astar_navigator.stop_robot()
                        self.node.get_logger().info('⭐ SUPPORTER: Planlegger A* rute... (kort pause)')
                        return

                    # ---- START KORRIGERT SUPPORTER-ADFERD ----
                    # Gjør navigasjonsmålet traverserbart på riktig side av veggen
                    grid_manager = self.sensor_manager.get_occupancy_grid_manager()
                    path_planner = self.astar_navigator.path_planner
                    robot_pos = self.robot_position

                    goal_map = grid_manager.world_to_map(target[0], target[1])
                    start_map = grid_manager.world_to_map(robot_pos[0], robot_pos[1])
                    if not path_planner.is_traversable(goal_map[0], goal_map[1]):
                        adjusted_goal_map = path_planner.find_nearest_traversable_towards_goal(start_map, goal_map, max_search_radius=10)
                        if adjusted_goal_map:
                            goal_map = adjusted_goal_map
                            target = grid_manager.map_to_world(goal_map[0], goal_map[1])
                            self.node.get_logger().info(
                                f"Supporter: Justerer mål til traversabel ({goal_map}) / world ({target})"
                            )
                        else:
                            self.node.get_logger().warn("Supporter: Ingen traversabel posisjon funnet nær lederen, avbryter!")
                            self.astar_navigator.stop_robot()
                            return
                    # ---- SLUTT KORRIGERT ----

                    # Sett målet i AStarNavigator (planlegger automatisk rute)
                    if not self.astar_navigator.navigation_active or self.astar_navigator.target_position != target:
                        self.node.get_logger().info(f'⭐ SUPPORTER: Setter A* mål til {target}')
                        self.astar_navigator.set_goal(target)
                    else:
                        self.node.get_logger().debug(f'⭐ SUPPORTER: A* allerede aktivt med mål {self.astar_navigator.target_position}')

                    # Utfør A*-navigasjon (A* håndterer replan, recovery og blokkeringer kontinuerlig)
                    goal_reached = self.astar_navigator.navigate_to_goal(msg)

                    if goal_reached:
                        self.astar_navigator.stop_robot()
                        self.node.get_logger().info('⭐ A*: Mål nådd! Oppdaterer Big Fire state.')
                        # Oppdater Big Fire-tilstanden
                        if self.robot_memory.my_role == self.robot_memory.LEDER:
                            self.robot_memory.transition_to_leder_waiting()
                        else:  # Supporter
                            # Supporter publiserer AT_FIRE slik at leder kan starte slukking
                            try:
                                self.big_fire_coordinator.publish_robot_at_fire()
                            except Exception:
                                pass
                            self.robot_memory.transition_to_extinguishing()
                else:
                    # Map ikke tilgjengelig - bruk Bug2 direkte
                    self.node.get_logger().info(f'🔥 BUG2 (map ikke tilgjengelig): Target={target}. State={self.robot_memory.big_fire_state}')
                    # Sett målet i Bug2Navigator
                    self.bug2_navigator.set_goal(target)
                    # Utfør Bug2-navigasjon
                    goal_reached = self.bug2_navigator.navigate(msg)
                    if goal_reached:
                        self.bug2_navigator.stop_robot()
                        self.node.get_logger().info('🔥 BUG2: Mål nådd! Oppdaterer Big Fire state.')
                        # Oppdater Big Fire-tilstanden
                        if self.robot_memory.my_role == self.robot_memory.LEDER:
                            self.robot_memory.transition_to_leder_waiting()
                        else:  # Supporter
                            try:
                                self.big_fire_coordinator.publish_robot_at_fire()
                            except Exception:
                                pass
                            self.robot_memory.transition_to_extinguishing()
            else:
                # Ingen bevegelse (Venter, slukker, eller nylig detektert)
                if self.use_astar_for_big_fire:
                    self.astar_navigator.stop_robot()
                else:
                    self.bug2_navigator.stop_robot()
                self.handle_big_fire_state_logic()

        else:
            # Standard utforskning (Big Fire inaktiv)
            # Rydd A* mål hvis det ikke skal brukes
            if getattr(self.astar_navigator, 'navigation_active', False):
                self.astar_navigator.clear_goal()
            self.bug2_navigator.clear_goal()
            # Wall Follower skal alltid være aktiv når Big Fire ikke er aktiv
            self.wall_follower.follow_wall(msg) 


    def process_odom(self, msg: Odometry):
        """Oppdater robot posisjon og orientering"""
        self.robot_position = self.sensor_manager.get_robot_position()
        self.robot_orientation = self.sensor_manager.get_robot_orientation()

        # Oppdater alle navigatorer med robot posisjon (viktig for path planning)
        self.robot_memory.update_robot_pose(self.robot_position, self.robot_orientation)
        self.bug2_navigator.update_robot_pose(self.robot_position, self.robot_orientation)
        self.astar_navigator.update_robot_pose(self.robot_position, self.robot_orientation)
        # Marker besøkt posisjon alltid, også under utforskning
        try:
            self.astar_navigator.visited_tracker.mark_current(self.robot_position[0], self.robot_position[1])
        except Exception:
            pass

        # Kjør A* også på odometri for robust fremdrift (i tilfelle scan-loop hopper)
        # Viktig: Sørg for at andre komponenter ikke publiserer når A* er aktiv
        big_fire_active = self.big_fire_coordinator.should_handle_big_fire() or \
                          getattr(self.astar_navigator, 'navigation_active', False)
        if big_fire_active and getattr(self.astar_navigator, 'navigation_active', False):
            # A* er aktiv - stopp andre komponenter
            try:
                self.wall_follower.stop_robot()
                self.bug2_navigator.stop_robot()
            except Exception:
                pass
            now_sec = self.node.get_clock().now().nanoseconds / 1e9
            if now_sec >= self.planning_pause_until:
                try:
                    self.astar_navigator.navigate_to_goal(None)
                except Exception:
                    pass

    def handle_big_fire_state_logic(self):
        """Håndterer KUN tilstandsoverganger og publisering."""
        coordinator = self.big_fire_coordinator
        current_state = coordinator.memory.big_fire_state

        if current_state == coordinator.memory.LEDER_WAITING:
            # Logg kun ved state entry for å redusere støy
            if not coordinator.memory.waiting_logged:
                self.node.get_logger().info('🔥 LEDER: In LEDER_WAITING state!')
                coordinator.memory.waiting_logged = True
            # Publiser AT_FIRE kun hvis lederen faktisk er ved brannen (innenfor radius)
            if not coordinator.memory.i_am_at_fire:
                bf_pos = coordinator.memory.big_fire_position
                if bf_pos is not None:
                    dx = bf_pos[0] - self.robot_position[0]
                    dy = bf_pos[1] - self.robot_position[1]
                    dist = (dx*dx + dy*dy) ** 0.5
                    # Krav: innen 2m radius for å kunne melde AT_FIRE
                    if dist <= 2.0:
                        coordinator.publish_robot_at_fire()
                        self.node.get_logger().info(f'🔥 LEDER: Ved brannen (dist={dist:.2f}m). Publiserer AT_FIRE.')
                    else:
                        self.node.get_logger().info(f'⏳ LEDER: Venter på supporter. Ikke ved brannen ennå (dist={dist:.2f}m).')
            # Begynn slukking først når BEGGE er ved brannen
            if coordinator.memory.i_am_at_fire and coordinator.memory.other_robot_at_fire:
                coordinator.memory.transition_to_extinguishing()
                self.node.get_logger().info('🔥 LEDER: Begge ved brannen - begynner slukking!')

        elif current_state == coordinator.memory.EXTINGUISHING:
            self.node.get_logger().info('🔥 SLUKKING PÅGÅR!')
            if not coordinator.memory.fire_extinguished:
                coordinator.publish_fire_extinguished()
                self.node.get_logger().info('🔥 Brannen slukket! Roboter returnerer til normal utforskning.')
                coordinator.memory.transition_to_normal()

        elif current_state == coordinator.memory.NORMAL:
            # Sjekk om den mottok melding (supporter) eller nettopp detekterte (leder)
            if coordinator.memory.big_fire_detected_by_me:
                self.node.get_logger().info('🔥 LEDER: Jeg oppdaget Big Fire - starter navigasjon!')
                coordinator.memory.transition_to_leder_going_to_fire()
            elif coordinator.memory.big_fire_detected_by_other:
                pos = coordinator.memory.big_fire_position
                self.node.get_logger().info(
                    f'🔥 SUPPORTER ({self.robot_id}): Mottok Big Fire melding - '
                    f'planlegger A* rute mot ({pos[0]:.2f}, {pos[1]:.2f})'
                )
                # Stopp bevegelse straks og sett kort plan-pause
                try:
                    self.wall_follower.stop_robot()
                    self.bug2_navigator.stop_robot()
                    self.astar_navigator.stop_robot()
                except Exception:
                    pass
                self.planning_pause_until = self.node.get_clock().now().nanoseconds / 1e9 + 0.8
                # Forhåndssett mål i A* for å vise ruten i RViz straks
                if self.use_astar_for_big_fire and pos is not None:
                    self.astar_navigator.set_goal(pos)
                coordinator.memory.transition_to_supporter_going_to_fire()

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
        self.astar_navigator.stop_robot()
        self.bug2_navigator.stop_robot()
        self.wall_follower.stop_robot()
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