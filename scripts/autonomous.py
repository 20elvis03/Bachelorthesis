#!/usr/bin/env python3
"""
Autonomes Fahren v3 – Multi-Robot Rasenmäher mit Bug2-Algorithmus
=================================================================
Features:
  1. Boustrophedon (Rasenmäher) – Bahnen hin-und-her
  2. Bug2-Hindernisumfahrung – Ameisen-artig um Hindernisse herum,
     dann zurück auf die ursprüngliche Bahn
  3. Batterie-Simulation – Entladung pro gefahrenem Meter
  4. Rückkehr-Planung – fährt rechtzeitig zur Ladestation zurück
  5. Multi-Robot-Kollisionsvermeidung – über LiDAR + Odom der anderen
  6. Emergency Stop – einzeln (/{ns}/emergency_stop) + global (/emergency_stop_all)

Starten (einzeln):
  ros2 run my_robot_gazebo autonomous

Multi-Robot (im Namespace):
  ros2 run my_robot_gazebo autonomous --ros-args \
       -r __ns:=/robot_1 \
       -p spawn_x:=22.5 -p spawn_y:=-22.5 \
       -p peer_namespaces:="robot_2,robot_3"

Emergency Stop:
  Einzeln:  ros2 topic pub /{ns}/emergency_stop std_msgs/msg/Bool "data: true" --once
  Alle:     ros2 topic pub /emergency_stop_all   std_msgs/msg/Bool "data: true" --once
  Freigabe: ... "data: false" --once
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations

# ═══════════════════════════════════════════════════════════════════
#  KONFIGURATION
# ═══════════════════════════════════════════════════════════════════

# ── Geschwindigkeiten ─────────────────────────────────────────────
DRIVE_SPEED       = 0.5       # m/s – Geradeausfahrt
TURN_FORWARD_SPD  = 0.18      # m/s – Vorwärts beim Wenden
TURN_SPEED        = 0.4       # rad/s – Drehgeschwindigkeit
WALL_FOLLOW_SPEED = 0.30      # m/s – während Bug2 Wandfolgen
REVERSE_SPEED     = -0.25     # m/s – Rückwärts

# ── Lenkung ───────────────────────────────────────────────────────
MAX_STEER         = 0.48      # rad – max. Lenkwinkel
HEADING_KP        = 1.2       # P-Regler Heading
HEADING_KD        = 0.3       # D-Regler Heading (Dämpfung)

# ── Rasenmäher-Muster ────────────────────────────────────────────
LANE_WIDTH        = 3.0       # m – Abstand zwischen Bahnen
WAYPOINT_REACHED  = 1.5       # m – Toleranz "angekommen"
LANE_TURN_PHASES  = 3         # Wenden: 90° + geradeaus + 90°

# ── Hindernis-Erkennung ──────────────────────────────────────────
OBSTACLE_DETECT   = 2.5       # m – Hindernis vorne erkannt → Bug2
OBSTACLE_STOP     = 0.8       # m – Notstopp-Distanz
OBSTACLE_BACK     = 0.6       # m – Hindernis hinten
FRONT_CONE_DEG    = 25        # ° – Frontkegel halbe Breite
SIDE_CONE_START   = 55        # ° – Seitenkegel Beginn
SIDE_CONE_END     = 125       # ° – Seitenkegel Ende
BACK_CONE_DEG     = 30        # ° – Rückkegel halbe Breite

# ── LiDAR-Konfiguration ──────────────────────────────────────────
LIDAR_HORIZ_IDX   = 55        # Vertikale Reihe (horizontal ≈ Mitte)
LIDAR_HORIZ_TOL   = 10        # ±Reihen um HORIZ_IDX

# ── Bug2-Algorithmus (Ameisen-Umfahrung) ─────────────────────────
WALL_FOLLOW_DIST  = 1.5       # m – Zielabstand zur Wand
WALL_FOLLOW_KP    = 0.6       # P-Regler Wandabstand
BUG2_MLINE_TOL    = 1.0       # m – Toleranz zur M-Linie
BUG2_MIN_TRAVEL   = 3.0       # m – Min. Strecke bevor M-Linie gecheckt wird
BUG2_TIMEOUT      = 60.0      # s – Max. Wall-Follow bevor Abbruch

# ── Stuck-Detection ──────────────────────────────────────────────
STUCK_CHECK_TIME  = 4.0       # s – Prüfintervall
STUCK_MIN_DIST    = 0.20      # m – Mindestbewegung
ESCAPE_REVERSE_T  = 2.0       # s – Rückwärts bei Stuck
ESCAPE_TURN_DEG   = 120.0     # ° – Drehung nach Rückwärts

# ── Batterie ─────────────────────────────────────────────────────
BATTERY_FULL      = 100.0     # %
BATTERY_PER_METER = 0.5       # % pro Meter (→ 200m Reichweite)
BATTERY_RESERVE   = 15.0      # % Sicherheitspuffer
BATTERY_WARN      = 30.0      # % Warnschwelle
CHARGE_TIME       = 15.0      # s – Vollladung
BASE_ARRIVE_DIST  = 1.5       # m – "an der Basis angekommen"

# ── Multi-Robot Kollisionsvermeidung ─────────────────────────────
PEER_SAFE_DIST    = 4.0       # m – Abstand zu anderem Roboter
PEER_STOP_DIST    = 2.0       # m – Stopp-Distanz zu anderem Roboter
PEER_YIELD_SLOW   = 0.15      # m/s – Schleichgeschwindigkeit beim Ausweichen

# ═══════════════════════════════════════════════════════════════════
#  ZUSTÄNDE
# ═══════════════════════════════════════════════════════════════════
ST_INIT          = 'INIT'
ST_DRIVE_LANE    = 'DRIVE_LANE'
ST_LANE_TURN     = 'LANE_TURN'
ST_BUG2_WALL     = 'BUG2_WALL_FOLLOW'
ST_ESCAPE        = 'ESCAPE'
ST_RETURN_BASE   = 'RETURN_BASE'
ST_BUG2_RETURN   = 'BUG2_RETURN'      # Bug2 während Rückkehr
ST_CHARGING      = 'CHARGING'
ST_EMERGENCY     = 'EMERGENCY'
ST_DONE          = 'DONE'


class AutonomousDrive(Node):
    """Autonomer Rasenmäher-Roboter mit Bug2-Hindernisumfahrung."""

    def __init__(self):
        super().__init__('autonomous_drive')

        # ── Parameter ─────────────────────────────────────────────
        self.declare_parameter('spawn_x', 22.5)
        self.declare_parameter('spawn_y', -22.5)
        self.declare_parameter('area_x_min', -20.0)
        self.declare_parameter('area_x_max', 20.0)
        self.declare_parameter('area_y_min', -20.0)
        self.declare_parameter('area_y_max', 14.0)
        self.declare_parameter('lane_width', LANE_WIDTH)
        self.declare_parameter('peer_namespaces', '')   # "robot_2,robot_3"
        self.declare_parameter('robot_priority', 0)     # Niedrigere Nr = höhere Priorität

        self.base_x = self.get_parameter('spawn_x').value
        self.base_y = self.get_parameter('spawn_y').value
        area_x_min  = self.get_parameter('area_x_min').value
        area_x_max  = self.get_parameter('area_x_max').value
        area_y_min  = self.get_parameter('area_y_min').value
        area_y_max  = self.get_parameter('area_y_max').value
        lane_w      = self.get_parameter('lane_width').value
        self.priority = self.get_parameter('robot_priority').value

        # ── Rasenmäher-Waypoints generieren ───────────────────────
        self.waypoints = []
        self._generate_lawnmower(area_x_min, area_x_max,
                                 area_y_min, area_y_max, lane_w)
        self.wp_index = 0

        # ── Publisher ─────────────────────────────────────────────
        self.cmd_pub   = self.create_publisher(Twist,   'cmd_vel',  10)
        self.steer_pub = self.create_publisher(Float64, 'steering', 10)

        # ── Subscriber (eigene Sensoren, relativ → Namespace) ────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos)
        self.create_subscription(Odometry,  'odom', self.odom_cb, qos)

        # ── Emergency Stop ────────────────────────────────────────
        # Einzeln: /{ns}/emergency_stop  (relativ)
        self.create_subscription(Bool, 'emergency_stop',
                                 self.emergency_cb, 10)
        # Global:  /emergency_stop_all  (absolut)
        self.create_subscription(Bool, '/emergency_stop_all',
                                 self.emergency_all_cb, 10)
        self.emergency_active = False
        self.state_before_emg = ST_INIT

        # ── Multi-Robot: Peer-Positionen ──────────────────────────
        self.peer_positions = {}   # {ns: (x, y, yaw)}
        peer_ns_str = self.get_parameter('peer_namespaces').value
        self.peer_namespaces = []
        if peer_ns_str:
            self.peer_namespaces = [s.strip() for s in peer_ns_str.split(',')
                                    if s.strip()]
        for pns in self.peer_namespaces:
            topic = f'/{pns}/odom'
            self.create_subscription(
                Odometry, topic,
                lambda msg, ns=pns: self._peer_odom_cb(ns, msg),
                qos)
            self.get_logger().info(f'Peer-Odom: {topic}')

        # ── Zustand ───────────────────────────────────────────────
        self.state          = ST_INIT
        self.pos_x          = 0.0
        self.pos_y          = 0.0
        self.yaw            = 0.0
        self.prev_yaw_err   = 0.0       # für D-Regler
        self.odom_ready     = False

        # ── LiDAR-Distanzen ───────────────────────────────────────
        self.dist_front     = 99.0
        self.dist_left      = 99.0
        self.dist_right     = 99.0
        self.dist_back      = 99.0

        # ── Batterie ──────────────────────────────────────────────
        self.battery_pct       = BATTERY_FULL
        self.total_dist        = 0.0
        self.charge_timer      = 0.0
        self._battery_warned   = False

        # ── Bug2-Zustand ──────────────────────────────────────────
        self.bug2_goal       = None     # (x, y) – Ziel vor dem Hindernis
        self.bug2_hit_point  = None     # (x, y) – wo wir das Hindernis trafen
        self.bug2_wall_side  = 1        # +1=links, -1=rechts
        self.bug2_travel     = 0.0      # Strecke seit Wall-Follow Start
        self.bug2_timer      = 0.0      # Timeout-Zähler
        self.bug2_prev_x     = 0.0
        self.bug2_prev_y     = 0.0
        self.resume_wp_index = 0        # Waypoint nach Rückkehr vom Laden

        # ── Wende-Zustand ─────────────────────────────────────────
        self.turn_phase      = 0        # 0=erste 90°, 1=geradeaus, 2=zweite 90°
        self.turn_dir        = 1        # +1=links, -1=rechts
        self.yaw_start_turn  = 0.0
        self.turn_straight_t = 0.0

        # ── Escape (Stuck) ────────────────────────────────────────
        self.stuck_timer     = 0.0
        self.stuck_ref_x     = 0.0
        self.stuck_ref_y     = 0.0
        self.escape_phase    = 'reverse'
        self.escape_timer    = 0.0
        self.escape_yaw_start = 0.0
        self.escape_turn_dir = 1

        # ── Logging ───────────────────────────────────────────────
        self._log_timer = 0.0

        # ── Haupt-Timer (20 Hz) ──────────────────────────────────
        self.dt = 0.05
        self.create_timer(self.dt, self.loop)

        self.get_logger().info(
            f'═══ Autonomes Fahren v3 ═══\n'
            f'  Basis:      ({self.base_x}, {self.base_y})\n'
            f'  Bahnen:     {len(self.waypoints)//2} Stück, Breite={lane_w}m\n'
            f'  Reichweite: {BATTERY_FULL/BATTERY_PER_METER:.0f}m\n'
            f'  Peers:      {self.peer_namespaces or "keine"}\n'
            f'  E-Stop:     /{self.get_namespace()}/emergency_stop  oder  /emergency_stop_all')

    # ═══════════════════════════════════════════════════════════════
    #  RASENMÄHER-WAYPOINTS
    # ═══════════════════════════════════════════════════════════════
    def _generate_lawnmower(self, x_min, x_max, y_min, y_max, width):
        """Erzeugt Boustrophedon-Waypoints (Rasenmäher-Muster).

        Bahnen verlaufen in X-Richtung, versetzt in Y-Richtung.
        Ungerade Bahnen: links→rechts, gerade: rechts→links.
        """
        y = y_min
        lane_idx = 0
        while y <= y_max:
            if lane_idx % 2 == 0:
                self.waypoints.append((x_min, y))
                self.waypoints.append((x_max, y))
            else:
                self.waypoints.append((x_max, y))
                self.waypoints.append((x_min, y))
            y += width
            lane_idx += 1

        self.get_logger().info(
            f'Rasenmäher: {len(self.waypoints)} Waypoints, '
            f'{lane_idx} Bahnen  '
            f'X=[{x_min},{x_max}]  Y=[{y_min},{y_max}]')

    # ═══════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ═══════════════════════════════════════════════════════════════
    def emergency_cb(self, msg: Bool):
        """Einzelner Emergency Stop für diesen Roboter."""
        self._handle_emergency(msg.data, 'EINZELN')

    def emergency_all_cb(self, msg: Bool):
        """Globaler Emergency Stop für alle Roboter."""
        self._handle_emergency(msg.data, 'GLOBAL')

    def _handle_emergency(self, active: bool, source: str):
        if active and not self.emergency_active:
            self.emergency_active = True
            self.state_before_emg = self.state
            self.state = ST_EMERGENCY
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().error(
                f'🛑 EMERGENCY STOP ({source})! Roboter gestoppt.')
        elif not active and self.emergency_active:
            self.emergency_active = False
            self.state = self.state_before_emg
            self.get_logger().warn(
                f'✅ Emergency aufgehoben ({source}) → {self.state}')

    def _peer_odom_cb(self, ns: str, msg: Odometry):
        """Position eines anderen Roboters speichern."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self.peer_positions[ns] = (x, y, yaw)

    # ── LiDAR ─────────────────────────────────────────────────────
    def scan_cb(self, msg: LaserScan):
        """LiDAR auswerten – Distanzen in 4 Richtungen."""
        n_horiz = 1200
        n_vert  = 64
        total   = len(msg.ranges)

        front_min = 99.0
        left_min  = 99.0
        right_min = 99.0
        back_min  = 99.0

        fc = math.radians(FRONT_CONE_DEG)
        ss = math.radians(SIDE_CONE_START)
        se = math.radians(SIDE_CONE_END)
        bt = math.radians(180 - BACK_CONE_DEG)

        def classify(r, angle):
            nonlocal front_min, left_min, right_min, back_min
            if abs(angle) < fc:
                front_min = min(front_min, r)
            elif ss < angle < se:
                left_min = min(left_min, r)
            elif -se < angle < -ss:
                right_min = min(right_min, r)
            elif abs(angle) > bt:
                back_min = min(back_min, r)

        if total == n_horiz * n_vert:
            v_lo = max(0, LIDAR_HORIZ_IDX - LIDAR_HORIZ_TOL)
            v_hi = min(n_vert, LIDAR_HORIZ_IDX + LIDAR_HORIZ_TOL + 1)
            for v in range(v_lo, v_hi):
                for h in range(n_horiz):
                    r = msg.ranges[v * n_horiz + h]
                    if math.isnan(r) or math.isinf(r) or r <= 0.05:
                        continue
                    angle = msg.angle_min + h * msg.angle_increment
                    classify(r, angle)
        else:
            for i, r in enumerate(msg.ranges):
                if math.isnan(r) or math.isinf(r) or r <= 0.05:
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                classify(r, angle)

        self.dist_front = front_min
        self.dist_left  = left_min
        self.dist_right = right_min
        self.dist_back  = back_min

    # ── Odometrie + Batterie ──────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        if self.odom_ready:
            delta = math.hypot(new_x - self.pos_x, new_y - self.pos_y)
            if delta < 0.5:  # Sprung-Filter
                self.total_dist += delta
                self.battery_pct -= delta * BATTERY_PER_METER
                self.battery_pct = max(0.0, self.battery_pct)

        self.pos_x = new_x
        self.pos_y = new_y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

        if not self.odom_ready:
            self.odom_ready    = True
            self.stuck_ref_x   = self.pos_x
            self.stuck_ref_y   = self.pos_y
            self.get_logger().info(
                f'Odom bereit: ({self.pos_x:.1f}, {self.pos_y:.1f})  '
                f'Batterie: {self.battery_pct:.0f}%')

    # ═══════════════════════════════════════════════════════════════
    #  HAUPT-LOOP
    # ═══════════════════════════════════════════════════════════════
    def loop(self):
        dt = self.dt
        if not self.odom_ready:
            return

        # ── Emergency: nur stoppen ────────────────────────────────
        if self.state == ST_EMERGENCY:
            self._publish(0.0, 0.0, 0.0)
            return

        # ── Logging alle 2s ───────────────────────────────────────
        self._log_timer += dt
        if self._log_timer >= 2.0:
            self._log_timer = 0.0
            wp_str = (f'WP {self.wp_index}/{len(self.waypoints)}'
                      if self.wp_index < len(self.waypoints)
                      else 'fertig')
            self.get_logger().info(
                f'[{self.state:18s}] '
                f'Pos=({self.pos_x:.1f},{self.pos_y:.1f})  '
                f'F={self.dist_front:.1f} L={self.dist_left:.1f} '
                f'R={self.dist_right:.1f}  '
                f'🔋{self.battery_pct:.1f}%  {self.total_dist:.0f}m  '
                f'{wp_str}')

        # ── Batterie leer ─────────────────────────────────────────
        if self.battery_pct <= 0.0 and self.state != ST_CHARGING:
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().error('🔋 BATTERIE LEER – Roboter gestoppt!')
            self.state = ST_DONE
            return

        # ── Batterie-Warnung ──────────────────────────────────────
        if (not self._battery_warned
                and self.battery_pct <= BATTERY_WARN
                and self.state not in (ST_RETURN_BASE, ST_BUG2_RETURN,
                                       ST_CHARGING)):
            self._battery_warned = True
            self.get_logger().warn(
                f'⚠️ Batterie: {self.battery_pct:.1f}%  '
                f'Rückkehr braucht: {self._battery_for_return():.1f}%')

        # ── Batterie kritisch → zurück zur Basis ──────────────────
        if self._should_return_to_base():
            self.resume_wp_index = self.wp_index
            self.state = ST_RETURN_BASE
            self.get_logger().warn(
                f'🔋 Rückkehr! ({self.battery_pct:.1f}% übrig, '
                f'brauche {self._battery_for_return():.1f}%)  '
                f'Dist={self._dist_to_base():.1f}m')

        # ── Multi-Robot: Peer zu nah → bremsen ────────────────────
        peer_block = self._check_peer_collision()

        # ── Notstopp vorne (nicht in bestimmten Zuständen) ────────
        if (self.dist_front < OBSTACLE_STOP
                and self.state in (ST_DRIVE_LANE, ST_RETURN_BASE)):
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().warn(
                f'⛔ Notstopp! Hindernis {self.dist_front:.2f}m')
            # In DRIVE_LANE → Bug2 starten
            if self.state == ST_DRIVE_LANE:
                self._start_bug2_lane()
            elif self.state == ST_RETURN_BASE:
                self._start_bug2_return()
            return

        # ═════════════════════════════════════════════════════════
        #  ZUSTANDS-MASCHINE
        # ═════════════════════════════════════════════════════════

        # ── INIT → Zum ersten Waypoint fahren ─────────────────────
        if self.state == ST_INIT:
            self.state = ST_DRIVE_LANE
            self.stuck_timer = 0.0
            self.stuck_ref_x = self.pos_x
            self.stuck_ref_y = self.pos_y
            self.get_logger().info('▶ Start Rasenmäher!')

        # ── DRIVE_LANE: Bahn entlang fahren ──────────────────────
        elif self.state == ST_DRIVE_LANE:
            if self.wp_index >= len(self.waypoints):
                self._publish(0.0, 0.0, 0.0)
                self.state = ST_DONE
                self.get_logger().info('✅ Alle Bahnen abgefahren!')
                return

            goal = self.waypoints[self.wp_index]
            dist = math.hypot(goal[0] - self.pos_x, goal[1] - self.pos_y)

            # Waypoint erreicht?
            if dist < WAYPOINT_REACHED:
                self.wp_index += 1
                if self.wp_index >= len(self.waypoints):
                    self._publish(0.0, 0.0, 0.0)
                    self.state = ST_DONE
                    self.get_logger().info('✅ Alle Bahnen abgefahren!')
                    return
                # Wenn Bahnende (jeder 2. WP) → Wende einleiten
                if self.wp_index % 2 == 0:
                    self._start_lane_turn()
                    return
                self.get_logger().info(
                    f'Bahn {self.wp_index//2 + 1} – '
                    f'Ziel: ({self.waypoints[self.wp_index][0]:.0f}, '
                    f'{self.waypoints[self.wp_index][1]:.0f})')
                return

            # Hindernis → Bug2
            if self.dist_front < OBSTACLE_DETECT:
                self._start_bug2_lane()
                return

            # Stuck-Detection
            if self._check_stuck(dt):
                return

            # Peer-Kollision
            if peer_block:
                self._publish(0.0, 0.0, 0.0)
                return

            # Zum Waypoint fahren
            speed = DRIVE_SPEED
            if dist < 3.0:
                speed *= max(0.3, dist / 3.0)  # Abbremsen vor Waypoint
            self._drive_toward(goal[0], goal[1], speed, dt)

        # ── LANE_TURN: Wende am Bahnende ─────────────────────────
        elif self.state == ST_LANE_TURN:
            self._do_lane_turn(dt)

        # ── BUG2_WALL: Hindernis umfahren (auf Bahn) ─────────────
        elif self.state == ST_BUG2_WALL:
            self._do_bug2_wall_follow(dt, ST_DRIVE_LANE)

        # ── RETURN_BASE: Zur Ladestation fahren ──────────────────
        elif self.state == ST_RETURN_BASE:
            dist_base = self._dist_to_base()
            if dist_base < BASE_ARRIVE_DIST:
                self._publish(0.0, 0.0, 0.0)
                self.charge_timer    = 0.0
                self._battery_warned = False
                self.state = ST_CHARGING
                self.get_logger().info(
                    f'🔌 Ladestation erreicht! '
                    f'({dist_base:.2f}m) – lade auf...')
                return

            # Hindernis → Bug2 für Rückkehr
            if self.dist_front < OBSTACLE_DETECT:
                self._start_bug2_return()
                return

            if peer_block:
                self._publish(0.0, 0.0, 0.0)
                return

            self._drive_toward(self.base_x, self.base_y,
                               DRIVE_SPEED * 0.8, dt)

        # ── BUG2_RETURN: Hindernis umfahren (Rückkehr) ───────────
        elif self.state == ST_BUG2_RETURN:
            self._do_bug2_wall_follow(dt, ST_RETURN_BASE)

        # ── CHARGING: Aufladen ────────────────────────────────────
        elif self.state == ST_CHARGING:
            self._publish(0.0, 0.0, 0.0)
            self.charge_timer += dt
            charge_rate = (BATTERY_FULL / CHARGE_TIME) * dt
            self.battery_pct = min(BATTERY_FULL, self.battery_pct + charge_rate)

            if self.charge_timer >= CHARGE_TIME:
                self.battery_pct = BATTERY_FULL
                self.wp_index    = self.resume_wp_index
                self.state       = ST_DRIVE_LANE
                self.stuck_timer = 0.0
                self.stuck_ref_x = self.pos_x
                self.stuck_ref_y = self.pos_y
                self.get_logger().info(
                    f'✅ Voll geladen! Weiter bei WP {self.wp_index}')

        # ── ESCAPE: Stuck-Befreiung ──────────────────────────────
        elif self.state == ST_ESCAPE:
            self._do_escape(dt)

        # ── DONE: Fertig ──────────────────────────────────────────
        elif self.state == ST_DONE:
            self._publish(0.0, 0.0, 0.0)

    # ═══════════════════════════════════════════════════════════════
    #  BUG2-ALGORITHMUS (Ameisen-Umfahrung)
    # ═══════════════════════════════════════════════════════════════
    def _start_bug2_lane(self):
        """Bug2 starten – Hindernis auf der Bahn umfahren."""
        if self.wp_index < len(self.waypoints):
            self.bug2_goal = self.waypoints[self.wp_index]
        else:
            return
        self.bug2_hit_point = (self.pos_x, self.pos_y)
        self.bug2_wall_side = 1 if self.dist_left >= self.dist_right else -1
        self.bug2_travel = 0.0
        self.bug2_timer  = 0.0
        self.bug2_prev_x = self.pos_x
        self.bug2_prev_y = self.pos_y
        side_str = 'links' if self.bug2_wall_side == 1 else 'rechts'
        self.get_logger().info(
            f'🐜 Bug2 START – Hindernis {self.dist_front:.1f}m, '
            f'Wand {side_str}, '
            f'Ziel ({self.bug2_goal[0]:.0f}, {self.bug2_goal[1]:.0f})')
        self.state = ST_BUG2_WALL

    def _start_bug2_return(self):
        """Bug2 starten – Hindernis auf dem Weg zur Basis umfahren."""
        self.bug2_goal = (self.base_x, self.base_y)
        self.bug2_hit_point = (self.pos_x, self.pos_y)
        self.bug2_wall_side = 1 if self.dist_left >= self.dist_right else -1
        self.bug2_travel = 0.0
        self.bug2_timer  = 0.0
        self.bug2_prev_x = self.pos_x
        self.bug2_prev_y = self.pos_y
        self.get_logger().info(
            f'🐜 Bug2 (Rückkehr) – Hindernis {self.dist_front:.1f}m')
        self.state = ST_BUG2_RETURN

    def _do_bug2_wall_follow(self, dt, resume_state):
        """Bug2 Wall-Following – Hindernis umrunden.

        Folgt der Wand, bis die M-Linie (Gerade Hit→Goal) wieder
        erreicht wird und der Weg zum Ziel frei ist.
        """
        if self.bug2_goal is None:
            self.state = resume_state
            return

        # Strecke tracken
        step = math.hypot(self.pos_x - self.bug2_prev_x,
                          self.pos_y - self.bug2_prev_y)
        self.bug2_travel += step
        self.bug2_prev_x = self.pos_x
        self.bug2_prev_y = self.pos_y
        self.bug2_timer  += dt

        # Timeout
        if self.bug2_timer > BUG2_TIMEOUT:
            self.get_logger().warn('🐜 Bug2 TIMEOUT – Abbruch')
            self.state = resume_state
            return

        # M-Linie Check: Sind wir zurück auf der Linie Hit→Goal
        # UND näher am Ziel als der Hit-Punkt UND Weg ist frei?
        if self.bug2_travel > BUG2_MIN_TRAVEL:
            dist_to_goal = math.hypot(self.bug2_goal[0] - self.pos_x,
                                      self.bug2_goal[1] - self.pos_y)
            hit_to_goal  = math.hypot(self.bug2_goal[0] - self.bug2_hit_point[0],
                                      self.bug2_goal[1] - self.bug2_hit_point[1])

            # Abstand zur M-Linie berechnen
            mline_dist = self._point_to_line_dist(
                self.pos_x, self.pos_y,
                self.bug2_hit_point[0], self.bug2_hit_point[1],
                self.bug2_goal[0], self.bug2_goal[1])

            # Winkel zum Ziel prüfen
            angle_to_goal = math.atan2(self.bug2_goal[1] - self.pos_y,
                                       self.bug2_goal[0] - self.pos_x)
            yaw_err = abs(self._angle_diff(angle_to_goal, self.yaw))

            path_clear = (self.dist_front > OBSTACLE_DETECT
                          and yaw_err < math.radians(45))

            if (mline_dist < BUG2_MLINE_TOL
                    and dist_to_goal < hit_to_goal - 0.5
                    and path_clear):
                self.get_logger().info(
                    f'🐜 Bug2 FERTIG – M-Linie erreicht, '
                    f'Dist zum Ziel: {dist_to_goal:.1f}m '
                    f'(Travel: {self.bug2_travel:.1f}m)')
                self.state = resume_state
                self.stuck_timer = 0.0
                self.stuck_ref_x = self.pos_x
                self.stuck_ref_y = self.pos_y
                return

        # Wall-Following ausführen
        wall_dist = (self.dist_left if self.bug2_wall_side == 1
                     else self.dist_right)

        # P-Regler: Abstand zur Wand halten
        dist_err = wall_dist - WALL_FOLLOW_DIST
        angular  = WALL_FOLLOW_KP * dist_err * self.bug2_wall_side
        angular  = max(-TURN_SPEED, min(TURN_SPEED, angular))
        steer    = max(-MAX_STEER, min(MAX_STEER, angular * 0.6))

        if self.dist_front < OBSTACLE_STOP:
            # Zu nah vorne → auf der Stelle drehen (weg von der Wand)
            self._publish(0.0,
                          TURN_SPEED * (-self.bug2_wall_side),
                          MAX_STEER * (-self.bug2_wall_side))
        elif self.dist_front < WALL_FOLLOW_DIST * 1.8:
            # Ecke → langsamer + stärker drehen
            self._publish(WALL_FOLLOW_SPEED * 0.4,
                          TURN_SPEED * (-self.bug2_wall_side) * 0.8,
                          MAX_STEER * (-self.bug2_wall_side))
        else:
            self._publish(WALL_FOLLOW_SPEED, angular, steer)

    # ═══════════════════════════════════════════════════════════════
    #  WENDE AM BAHNENDE
    # ═══════════════════════════════════════════════════════════════
    def _start_lane_turn(self):
        """Wende einleiten: nächste Bahn ist versetzt in Y."""
        # Bestimme Drehrichtung basierend auf nächstem Waypoint
        if self.wp_index < len(self.waypoints):
            next_wp = self.waypoints[self.wp_index]
            dy = next_wp[1] - self.pos_y
            # Wir müssen in Y-Richtung versetzt werden
            self.turn_dir = 1 if dy > 0 else -1
        else:
            self.turn_dir = 1

        self.turn_phase     = 0
        self.yaw_start_turn = self.yaw
        self.state = ST_LANE_TURN
        self.get_logger().info(
            f'↩ Wende Bahn {self.wp_index//2} → '
            f'{"links" if self.turn_dir == 1 else "rechts"}')

    def _do_lane_turn(self, dt):
        """3-Phasen Wende: 90° drehen → geradeaus → 90° drehen."""
        if self.turn_phase == 0:
            # Phase 1: 90° drehen
            turned = abs(self._angle_diff(self.yaw, self.yaw_start_turn))
            if turned < math.radians(85):
                self._publish(TURN_FORWARD_SPD,
                              TURN_SPEED * self.turn_dir,
                              MAX_STEER * self.turn_dir)
            else:
                self.turn_phase = 1
                self.turn_straight_t = 0.0
                self.get_logger().info('↩ Wende Phase 2: geradeaus')

        elif self.turn_phase == 1:
            # Phase 2: Geradeaus (eine Bahnbreite)
            self.turn_straight_t += dt
            needed_time = LANE_WIDTH / max(0.1, TURN_FORWARD_SPD * 2)
            if self.turn_straight_t < needed_time:
                if self.dist_front > OBSTACLE_STOP:
                    self._publish(TURN_FORWARD_SPD * 2, 0.0, 0.0)
                else:
                    self._publish(0.0, 0.0, 0.0)
            else:
                self.turn_phase = 2
                self.yaw_start_turn = self.yaw
                self.get_logger().info('↩ Wende Phase 3: 90° zurück')

        elif self.turn_phase == 2:
            # Phase 3: Weitere 90° drehen (gleiche Richtung)
            turned = abs(self._angle_diff(self.yaw, self.yaw_start_turn))
            if turned < math.radians(85):
                self._publish(TURN_FORWARD_SPD,
                              TURN_SPEED * self.turn_dir,
                              MAX_STEER * self.turn_dir)
            else:
                self._publish(0.0, 0.0, 0.0)
                self.state = ST_DRIVE_LANE
                self.stuck_timer = 0.0
                self.stuck_ref_x = self.pos_x
                self.stuck_ref_y = self.pos_y
                lane_num = self.wp_index // 2 + 1
                self.get_logger().info(
                    f'↩ Wende fertig → Bahn {lane_num}')

    # ═══════════════════════════════════════════════════════════════
    #  ESCAPE (STUCK-BEFREIUNG)
    # ═══════════════════════════════════════════════════════════════
    def _check_stuck(self, dt):
        """Prüft ob Roboter feststeckt. Gibt True zurück wenn Escape startet."""
        self.stuck_timer += dt
        if self.stuck_timer >= STUCK_CHECK_TIME:
            dist_moved = math.hypot(self.pos_x - self.stuck_ref_x,
                                    self.pos_y - self.stuck_ref_y)
            self.stuck_timer = 0.0
            self.stuck_ref_x = self.pos_x
            self.stuck_ref_y = self.pos_y
            if dist_moved < STUCK_MIN_DIST:
                self.get_logger().warn(
                    f'🔒 STUCK! Nur {dist_moved:.2f}m → Escape')
                self.escape_phase     = 'reverse'
                self.escape_timer     = 0.0
                self.escape_yaw_start = self.yaw
                self.escape_turn_dir  = (1 if self.dist_left >= self.dist_right
                                         else -1)
                self.state = ST_ESCAPE
                return True
        return False

    def _do_escape(self, dt):
        """Escape-Manöver: Rückwärts → Drehen → Weiterfahren."""
        if self.escape_phase == 'reverse':
            self.escape_timer += dt
            if (self.dist_back > OBSTACLE_BACK
                    and self.escape_timer < ESCAPE_REVERSE_T):
                self._publish(REVERSE_SPEED,
                              TURN_SPEED * self.escape_turn_dir * 0.3,
                              MAX_STEER * self.escape_turn_dir * 0.5)
            else:
                self.escape_phase     = 'turn'
                self.escape_yaw_start = self.yaw
                self.get_logger().info('Escape: Rückwärts fertig → Drehen')
        else:  # turn
            turned = abs(self._angle_diff(self.yaw, self.escape_yaw_start))
            if turned < math.radians(ESCAPE_TURN_DEG) - 0.1:
                self._publish(TURN_FORWARD_SPD,
                              TURN_SPEED * self.escape_turn_dir,
                              MAX_STEER * self.escape_turn_dir)
            else:
                self.get_logger().info('Escape fertig → weiterfahren')
                self.stuck_timer = 0.0
                self.stuck_ref_x = self.pos_x
                self.stuck_ref_y = self.pos_y
                # Zurück zum vorherigen Zustand
                if self.state == ST_ESCAPE:
                    self.state = ST_DRIVE_LANE

    # ═══════════════════════════════════════════════════════════════
    #  BATTERIE
    # ═══════════════════════════════════════════════════════════════
    def _dist_to_base(self):
        return math.hypot(self.pos_x - self.base_x, self.pos_y - self.base_y)

    def _battery_for_return(self):
        """Batterie-% die für Rückkehr + Puffer nötig sind."""
        return self._dist_to_base() * BATTERY_PER_METER + BATTERY_RESERVE

    def _should_return_to_base(self):
        """True wenn Batterie für Rückkehr + Puffer nicht mehr reicht."""
        if self.state in (ST_RETURN_BASE, ST_BUG2_RETURN,
                          ST_CHARGING, ST_DONE, ST_EMERGENCY):
            return False
        return self.battery_pct <= self._battery_for_return()

    # ═══════════════════════════════════════════════════════════════
    #  MULTI-ROBOT KOLLISIONSVERMEIDUNG
    # ═══════════════════════════════════════════════════════════════
    def _check_peer_collision(self):
        """Prüft ob ein Peer-Roboter zu nah ist.

        Returns True wenn dieser Roboter anhalten soll (Vorrang).
        """
        for ns, (px, py, _pyaw) in self.peer_positions.items():
            dist = math.hypot(px - self.pos_x, py - self.pos_y)
            if dist < PEER_STOP_DIST:
                # Wer hat Vorrang? Niedrigere Priorität = wichtiger
                # Peer-Index aus Namespace extrahieren
                peer_prio = self._extract_priority(ns)
                if self.priority > peer_prio:
                    # Wir haben niedrigere Priorität → anhalten
                    self.get_logger().info(
                        f'🤖 Peer {ns} zu nah ({dist:.1f}m) – halte an',
                        throttle_duration_sec=2.0)
                    return True
            elif dist < PEER_SAFE_DIST:
                # Verlangsamen (wird in den Drive-Methoden berücksichtigt)
                pass
        return False

    @staticmethod
    def _extract_priority(ns: str):
        """Versucht eine Nummer aus dem Namespace zu extrahieren."""
        digits = ''.join(c for c in ns if c.isdigit())
        return int(digits) if digits else 99

    # ═══════════════════════════════════════════════════════════════
    #  NAVIGATION
    # ═══════════════════════════════════════════════════════════════
    def _drive_toward(self, goal_x, goal_y, speed, dt):
        """Fährt mit PD-Regler zum Zielpunkt."""
        angle_to_goal = math.atan2(goal_y - self.pos_y,
                                   goal_x - self.pos_x)
        yaw_err = self._angle_diff(angle_to_goal, self.yaw)

        # PD-Regler
        d_err = (yaw_err - self.prev_yaw_err) / dt if dt > 0 else 0.0
        self.prev_yaw_err = yaw_err

        correction = HEADING_KP * yaw_err + HEADING_KD * d_err
        angular = max(-TURN_SPEED, min(TURN_SPEED, correction))
        steer   = max(-MAX_STEER, min(MAX_STEER, correction * 0.6))

        # Bei großem Heading-Fehler langsamer fahren
        yaw_err_deg = abs(math.degrees(yaw_err))
        if yaw_err_deg > 45:
            speed *= 0.3
        elif yaw_err_deg > 20:
            speed *= 0.6

        self._publish(speed, angular, steer)

    # ═══════════════════════════════════════════════════════════════
    #  HILFSFUNKTIONEN
    # ═══════════════════════════════════════════════════════════════
    def _publish(self, linear, angular, steer):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)
        s = Float64()
        s.data = float(max(-0.5, min(0.5, steer)))
        self.steer_pub.publish(s)

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    @staticmethod
    def _point_to_line_dist(px, py, lx1, ly1, lx2, ly2):
        """Abstand Punkt (px,py) zur Geraden durch (lx1,ly1)-(lx2,ly2)."""
        dx = lx2 - lx1
        dy = ly2 - ly1
        length = math.hypot(dx, dy)
        if length < 0.001:
            return math.hypot(px - lx1, py - ly1)
        return abs(dy * px - dx * py + lx2 * ly1 - ly2 * lx1) / length


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = AutonomousDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
