#!/usr/bin/env python3
"""
Autonomes Fahren für my_robot (3-Rad, DiffDrive + Lenkung)
===============================================================================
NEU v2:
  - LiDAR-Fix: breiterer Vertikalkegel (LIDAR_HORIZ_TOL 3 → 10)
  - Stuck-Detection im DRIVE-State
  - Emergency Stop via /emergency_stop (std_msgs/Bool)
      auslösen:  ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true" --once
      freigeben: ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: false" --once
  - Batterie-System (distanzbasiert, 0.5% pro Meter, Rückkehr zur Ladestation)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf_transformations

DRIVE_SPEED      = 0.5
AVOID_SPEED      = 0.3
TURN_SPEED       = 0.4

OBSTACLE_FRONT   = 2.5
OBSTACLE_STOP    = 0.8
WALL_TURN_DIST   = 3.0
SIDE_MIN_DIST    = 1.2
Y_MAX_BOUNDARY   = 14.5

FRONT_CONE       = 25
SIDE_CONE_START  = 55
SIDE_CONE_END    = 125
BACK_CONE        = 30
OBSTACLE_BACK    = 0.6

LIDAR_HORIZ_IDX  = 55
LIDAR_HORIZ_TOL  = 10     # war: 3 → jetzt: 10 (Reihen 45–65, Wände nicht mehr übersehen)

AVOID_ARC_TIME   = 2.2
AVOID_STEER      = 0.35
AVOID_ANGULAR    = 0.3

RETURN_YAW_TOL   = 4.0
RETURN_TIMEOUT   = 10.0

MAX_STEER        = 0.48
TURN_FORWARD_SPD = 0.2
NUM_LANES        = 60

STUCK_CHECK_TIME = 3.0
STUCK_DIST_M     = 0.15
REVERSE_SPEED    = -0.25
REVERSE_STEER    = 0.45
REVERSE_YAW_DEG  = 90.0

# NEU: Stuck im DRIVE-State
DRIVE_STUCK_CHECK = 4.0   # s – Prüfintervall
DRIVE_STUCK_MIN   = 0.25  # m – mind. diese Distanz in DRIVE_STUCK_CHECK Sekunden
ESCAPE_REVERSE_T  = 2.5   # s – rückwärts fahren
ESCAPE_TURN_DEG   = 135.0 # ° – dann drehen

# NEU: Batterie
BATTERY_START_PCT   = 100.0
BATTERY_M_PER_PCT   = 2.0    # 1% = 2 Meter → 200m Reichweite bei 100%
BATTERY_RESERVE_PCT = 15.0   # % Sicherheitspuffer für Rückkehr
BATTERY_CHARGE_TIME = 12.0   # s – Aufladezeit
BATTERY_LOW_WARN    = 30.0   # % – Warnschwelle

# Ladestation = Spawn-Punkt (aus gazebo_launch.py)
BASE_X = 22.5
BASE_Y = -22.5
BASE_ARRIVAL_DIST = 1.5  # m – "angekommen" wenn näher

STATE_DRIVE        = 'DRIVE'
STATE_BRAKE        = 'BRAKE'
STATE_TURN_CHECK   = 'TURN_CHECK'
STATE_TURN         = 'TURN'
STATE_REVERSE_TURN = 'REVERSE_TURN'
STATE_AVOID        = 'AVOID'
STATE_RETURN       = 'RETURN'
STATE_ESCAPE       = 'ESCAPE'
STATE_RETURN_BASE  = 'RETURN_BASE'
STATE_WALL_FOLLOW  = 'WALL_FOLLOW'   # NEU: Bug-Algorithmus für RETURN_BASE
STATE_CHARGING     = 'CHARGING'
STATE_DONE         = 'DONE'
STATE_EMERGENCY    = 'EMERGENCY'

# NEU: Wall-Following (Bug-Algorithmus)
WALL_FOLLOW_DIST   = 1.5   # m – Zielabstand zur Wand
WALL_FOLLOW_KP     = 0.5   # P-Regler Verstärkung Wandabstand
WALL_FOLLOW_SPEED  = 0.35  # Vorwärtsgeschwindigkeit beim Wandfolgen
WALL_CLEAR_ANGLE   = 35.0  # °  – Freiwinkel zur Basis zum Abbrechen des Wall-Follow
WALL_CLEAR_FRONT   = 3.0   # m  – Frontfreiheit zum Abbrechen


class autodrive(Node):
    def __init__(self):
        super().__init__('auto_drive')

        self.cmd_pub   = self.create_publisher(Twist,   '/cmd_vel',  10)
        self.steer_pub = self.create_publisher(Float64, '/steering', 10)
        self.create_subscription(LaserScan, '/scan',           self.scan_cb,      10)
        self.create_subscription(Odometry,  '/odom',           self.odom_cb,      10)

        # NEU: Emergency Stop
        # Terminal: ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true" --once
        self.create_subscription(Bool, '/emergency_stop', self.emergency_cb, 10)
        self.emergency_active  = False
        self.state_before_emg  = STATE_DRIVE

        self.state            = STATE_DRIVE
        self.lane_idx         = 0
        self._turn_dir        = 1
        self.avoid_side       = 1
        self.avoid_timer      = 0.0
        self.return_timer     = 0.0
        self.brake_timer      = 0.0
        self.turn_check_timer = 0.0

        self.yaw              = 0.0
        self.yaw_start_turn   = 0.0
        self.lane_yaw         = None
        self.lane_y           = None

        self.pos_x            = 0.0
        self.pos_y            = 0.0
        self.odom_ready       = False

        self.stuck_check_timer = 0.0
        self.stuck_ref_x       = 0.0
        self.stuck_ref_y       = 0.0
        self.stuck_checked     = False

        self.reverse_yaw_start = 0.0
        self.reverse_dir       = 1

        # NEU: Drive-Stuck-Erkennung
        self.drive_stuck_timer  = 0.0
        self.drive_stuck_ref_x  = 0.0
        self.drive_stuck_ref_y  = 0.0
        self.escape_phase       = 'reverse'
        self.escape_reverse_t   = 0.0
        self.escape_yaw_start   = 0.0
        self.escape_turn_dir    = 1

        # NEU: Wall-Follow (Bug-Algorithmus)
        self.wall_follow_side        = 1    # +1 = Wand links, -1 = Wand rechts
        self.wall_follow_dist_start  = 0.0  # Distanz zur Basis beim Start

        # NEU: Batterie
        self.battery_pct        = BATTERY_START_PCT
        self.total_dist_driven  = 0.0
        self.charge_timer       = 0.0
        self._battery_warned    = False
        self.base_yaw_aligned   = False
        self.resume_state       = STATE_DRIVE

        self.dist_front       = 99.0
        self.dist_left        = 99.0
        self.dist_right       = 99.0
        self.dist_back        = 99.0
        self.obstacle_front   = False
        self.obstacle_stop    = False
        self.obstacle_back    = False
        self.wall_near        = False
        self._log_timer       = 0.0

        self.create_timer(0.05, lambda: self.loop(0.05))
        self.get_logger().info(
            'Autodrive v2 bereit\n'
            '  Emergency Stop:  ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: true" --once\n'
            '  Freigeben:       ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: false" --once')

    # ── Emergency Stop ────────────────────────────────────────────────────────
    def emergency_cb(self, msg: Bool):
        if msg.data and not self.emergency_active:
            self.emergency_active = True
            self.state_before_emg = self.state
            self.state = STATE_EMERGENCY
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().error(
                '🛑 EMERGENCY STOP! Roboter gestoppt.\n'
                '   Freigeben: ros2 topic pub /emergency_stop std_msgs/msg/Bool "data: false" --once')
        elif not msg.data and self.emergency_active:
            self.emergency_active = False
            self.state = self.state_before_emg
            self.get_logger().warn(f'✅ Emergency aufgehoben – weiter: {self.state}')

    # ── LiDAR (FIX: TOL 3 → 10) ──────────────────────────────────────────────
    def scan_cb(self, msg: LaserScan):
        n_horiz   = 1200
        n_vert    = 64
        total_pts = len(msg.ranges)

        front_min = 99.0
        left_min  = 99.0
        right_min = 99.0
        back_min  = 99.0

        front_cone  = math.radians(FRONT_CONE)
        side_start  = math.radians(SIDE_CONE_START)
        side_end    = math.radians(SIDE_CONE_END)
        back_thresh = math.radians(180 - BACK_CONE)

        def classify(r, angle):
            nonlocal front_min, left_min, right_min, back_min
            if abs(angle) < front_cone:
                front_min = min(front_min, r)
            elif side_start < angle < side_end:
                left_min  = min(left_min,  r)
            elif -side_end < angle < -side_start:
                right_min = min(right_min, r)
            elif abs(angle) > back_thresh:
                back_min  = min(back_min,  r)

        if total_pts == n_horiz * n_vert:
            for v_idx in range(
                max(0, LIDAR_HORIZ_IDX - LIDAR_HORIZ_TOL),
                min(n_vert, LIDAR_HORIZ_IDX + LIDAR_HORIZ_TOL + 1)
            ):
                for h_idx in range(n_horiz):
                    r = msg.ranges[v_idx * n_horiz + h_idx]
                    if math.isnan(r) or math.isinf(r) or r <= 0.05:
                        continue
                    angle = msg.angle_min + h_idx * msg.angle_increment
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

        self.obstacle_stop  = front_min < OBSTACLE_STOP
        self.obstacle_front = front_min < OBSTACLE_FRONT
        self.obstacle_back  = back_min  < OBSTACLE_BACK
        self.wall_near      = front_min < WALL_TURN_DIST

        if left_min > right_min + 0.5:
            self.avoid_side = 1
        elif right_min > left_min + 0.5:
            self.avoid_side = -1

    # ── Odometrie + Batterie-Tracking ─────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        if self.odom_ready:
            delta = math.hypot(new_x - self.pos_x, new_y - self.pos_y)
            if delta < 0.5:  # Sprung-Filter
                self.total_dist_driven += delta
                self.battery_pct -= delta / BATTERY_M_PER_PCT
                self.battery_pct  = max(0.0, self.battery_pct)

        self.pos_x = new_x
        self.pos_y = new_y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

        if not self.odom_ready:
            self.odom_ready        = True
            self.drive_stuck_ref_x = self.pos_x
            self.drive_stuck_ref_y = self.pos_y
            self.get_logger().info(
                f'Start: x={self.pos_x:.1f} y={self.pos_y:.1f}  '
                f'Basis: x={BASE_X} y={BASE_Y}  '
                f'Batterie: {self.battery_pct:.0f}%')

    # ── Batterie-Hilfsfunktionen ──────────────────────────────────────────────
    def _dist_to_base(self):
        return math.hypot(self.pos_x - BASE_X, self.pos_y - BASE_Y)

    def _battery_critical(self):
        """True wenn Ladung für Rückkehr + Puffer nicht mehr reicht."""
        needed = (self._dist_to_base() / BATTERY_M_PER_PCT) + BATTERY_RESERVE_PCT
        return self.battery_pct <= needed

    # ── Haupt-Loop ────────────────────────────────────────────────────────────
    def loop(self, dt: float):
        if not self.odom_ready:
            return

        # Emergency: nichts tun außer stoppen
        if self.state == STATE_EMERGENCY:
            self._publish(0.0, 0.0, 0.0)
            return

        # Log alle 2s
        self._log_timer += dt
        if self._log_timer >= 2.0:
            self._log_timer = 0.0
            self.get_logger().info(
                f'[SCAN] V={self.dist_front:.1f}  L={self.dist_left:.1f}  '
                f'R={self.dist_right:.1f}  H={self.dist_back:.1f}  '
                f'| {self.state}  x={self.pos_x:.1f} y={self.pos_y:.1f}  '
                f'🔋{self.battery_pct:.1f}%  {self.total_dist_driven:.0f}m')

        # Batterie-Warnung
        if (not self._battery_warned and self.battery_pct <= BATTERY_LOW_WARN
                and self.state not in (STATE_RETURN_BASE, STATE_CHARGING)):
            self._battery_warned = True
            self.get_logger().warn(
                f'⚠️  Batterie niedrig: {self.battery_pct:.1f}% – '
                f'Rückkehr benötigt {self._dist_to_base()/BATTERY_M_PER_PCT:.1f}%')

        # Batterie kritisch → zurück zur Basis
        if (self._battery_critical() and self.battery_pct > 0.5
                and self.state not in (STATE_RETURN_BASE, STATE_WALL_FOLLOW,
                                       STATE_CHARGING, STATE_DONE, STATE_EMERGENCY)):
            self.resume_state     = STATE_DRIVE
            self.base_yaw_aligned = False
            self.state = STATE_RETURN_BASE
            self.get_logger().warn(
                f'🔋 Kritisch ({self.battery_pct:.1f}%) – Rückfahrt zur Basis '
                f'(Dist={self._dist_to_base():.1f}m)')

        # Batterie leer
        if self.battery_pct <= 0.0 and self.state != STATE_CHARGING:
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().error('🔋 BATTERIE LEER!')
            self.state = STATE_DONE
            return

        # Notstopp vorne
        if self.obstacle_stop and self.state not in (
                STATE_TURN, STATE_TURN_CHECK, STATE_REVERSE_TURN,
                STATE_RETURN_BASE, STATE_ESCAPE, STATE_WALL_FOLLOW):
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().warn(f'NOTSTOPP V! {self.dist_front:.2f}m',
                                   throttle_duration_sec=1.0)
            return

        # Notstopp hinten
        if self.obstacle_back and self.state in (STATE_REVERSE_TURN, STATE_ESCAPE):
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().warn(f'NOTSTOPP H! {self.dist_back:.2f}m')
            self.yaw_start_turn    = self.yaw
            self.stuck_check_timer = 0.0
            self.stuck_checked     = False
            self.stuck_ref_x       = self.pos_x
            self.stuck_ref_y       = self.pos_y
            self.state = STATE_TURN
            return

        # ─── ZUSTÄNDE ────────────────────────────────────────────────────────

        if self.state == STATE_DRIVE:
            if self.lane_yaw is None:
                self.lane_yaw = self.yaw
                self.lane_y   = self.pos_y
                self.get_logger().info(f'Bahn-Yaw: {math.degrees(self.lane_yaw):.1f}°')

            # NEU: Stuck-Check im DRIVE
            self.drive_stuck_timer += dt
            if self.drive_stuck_timer >= DRIVE_STUCK_CHECK:
                dist_moved = math.hypot(self.pos_x - self.drive_stuck_ref_x,
                                        self.pos_y - self.drive_stuck_ref_y)
                self.drive_stuck_timer  = 0.0
                self.drive_stuck_ref_x  = self.pos_x
                self.drive_stuck_ref_y  = self.pos_y
                if dist_moved < DRIVE_STUCK_MIN:
                    self.get_logger().warn(
                        f'🔒 DRIVE-STUCK: nur {dist_moved:.2f}m in {DRIVE_STUCK_CHECK}s'
                        ' → Escape-Manöver')
                    self.escape_phase     = 'reverse'
                    self.escape_reverse_t = 0.0
                    self.escape_yaw_start = self.yaw
                    self.escape_turn_dir  = 1 if self.dist_left >= self.dist_right else -1
                    self.state = STATE_ESCAPE
                    return

            if self.pos_y >= Y_MAX_BOUNDARY:
                self._publish(0.0, 0.0, 0.0)
                self.brake_timer = 0.0
                self.state = STATE_BRAKE
                return

            if self.wall_near:
                self._publish(0.0, 0.0, 0.0)
                self.brake_timer = 0.0
                self.state = STATE_BRAKE
                self.get_logger().info(f'Wand {self.dist_front:.1f}m – Bahn {self.lane_idx+1}')
                return

            if self.obstacle_front:
                self._publish(0.0, 0.0, 0.0)
                self.avoid_timer = 0.0
                self.lane_y      = self.pos_y
                self.state = STATE_AVOID
                seite = 'links' if self.avoid_side == 1 else 'rechts'
                self.get_logger().info(f'Hindernis {self.dist_front:.1f}m → {seite}')
                return

            self._publish(DRIVE_SPEED, 0.0, 0.0)

        # NEU: Escape-Manöver (Feststecken im DRIVE)
        elif self.state == STATE_ESCAPE:
            if self.escape_phase == 'reverse':
                self.escape_reverse_t += dt
                self._publish(REVERSE_SPEED,
                              TURN_SPEED * self.escape_turn_dir * 0.5,
                              REVERSE_STEER * self.escape_turn_dir)
                if self.escape_reverse_t >= ESCAPE_REVERSE_T:
                    self.escape_phase     = 'turn'
                    self.escape_yaw_start = self.yaw
                    self.get_logger().info('Escape: rückwärts fertig – drehe')
            else:
                turned = abs(self._angle_diff(self.yaw, self.escape_yaw_start))
                if turned < math.radians(ESCAPE_TURN_DEG) - 0.08:
                    self._publish(TURN_FORWARD_SPD,
                                  TURN_SPEED * self.escape_turn_dir,
                                  MAX_STEER  * self.escape_turn_dir)
                else:
                    self.get_logger().info('Escape fertig – weiterfahren')
                    self.lane_yaw          = self.yaw
                    self.lane_y            = self.pos_y
                    self.drive_stuck_ref_x = self.pos_x
                    self.drive_stuck_ref_y = self.pos_y
                    self.drive_stuck_timer = 0.0
                    self.state = STATE_DRIVE

        elif self.state == STATE_BRAKE:
            self.brake_timer += dt
            self._publish(0.0, 0.0, 0.0)
            if self.brake_timer > 0.4:
                self.turn_check_timer = 0.0
                self.state = STATE_TURN_CHECK

        elif self.state == STATE_TURN_CHECK:
            self.turn_check_timer += dt
            self._publish(0.0, 0.0, 0.0)
            links_ok  = self.dist_left  > SIDE_MIN_DIST
            rechts_ok = self.dist_right > SIDE_MIN_DIST
            if links_ok or rechts_ok:
                self._turn_dir         = 1 if self.dist_left >= self.dist_right else -1
                self.yaw_start_turn    = self.yaw
                self.stuck_check_timer = 0.0
                self.stuck_checked     = False
                self.stuck_ref_x       = self.pos_x
                self.stuck_ref_y       = self.pos_y
                self.state = STATE_TURN
                seite = 'links' if self._turn_dir == 1 else 'rechts'
                self.get_logger().info(
                    f'Wende {seite} (L={self.dist_left:.1f} R={self.dist_right:.1f})')
            elif self.turn_check_timer > 5.0:
                self.get_logger().warn('Kein Wendeplatz – rückwärts')
                self._publish(-0.15, 0.0, 0.0)

        elif self.state == STATE_TURN:
            turned = abs(self._angle_diff(self.yaw, self.yaw_start_turn))
            self.stuck_check_timer += dt
            if not self.stuck_checked and self.stuck_check_timer >= STUCK_CHECK_TIME:
                self.stuck_checked = True
                dist_moved = math.hypot(self.pos_x - self.stuck_ref_x,
                                        self.pos_y - self.stuck_ref_y)
                if dist_moved < STUCK_DIST_M:
                    self.get_logger().warn(
                        f'STUCK in TURN ({dist_moved:.2f}m) – rückwärts')
                    self.reverse_yaw_start = self.yaw
                    self.reverse_dir       = -self._turn_dir
                    self._publish(0.0, 0.0, 0.0)
                    self.state = STATE_REVERSE_TURN
                    return

            if turned < math.pi - 0.10:
                self._publish(TURN_FORWARD_SPD,
                              TURN_SPEED * self._turn_dir,
                              MAX_STEER  * self._turn_dir)
            else:
                self._publish(0.0, 0.0, 0.0)
                self.lane_yaw = self.yaw
                self.lane_y   = self.pos_y
                self.lane_idx += 1
                if self.lane_idx >= NUM_LANES:
                    self.state = STATE_DONE
                    self.get_logger().info('Alle Bahnen fertig!')
                else:
                    self.state = STATE_DRIVE
                    self.get_logger().info(
                        f'Bahn {self.lane_idx+1} – Yaw={math.degrees(self.lane_yaw):.1f}°')

        elif self.state == STATE_REVERSE_TURN:
            turned_back = abs(self._angle_diff(self.yaw, self.reverse_yaw_start))
            if turned_back < math.radians(REVERSE_YAW_DEG) - 0.08:
                self._publish(REVERSE_SPEED,
                              TURN_SPEED * self.reverse_dir,
                              REVERSE_STEER * self.reverse_dir)
            else:
                self._publish(0.0, 0.0, 0.0)
                self.get_logger().info('Rückwärts-Drehung fertig')
                self.yaw_start_turn    = self.yaw
                self.stuck_check_timer = 0.0
                self.stuck_checked     = False
                self.stuck_ref_x       = self.pos_x
                self.stuck_ref_y       = self.pos_y
                self.state = STATE_TURN

        elif self.state == STATE_AVOID:
            self.avoid_timer += dt
            self._publish(AVOID_SPEED,
                          AVOID_ANGULAR * self.avoid_side,
                          AVOID_STEER   * self.avoid_side)
            if self.avoid_timer > AVOID_ARC_TIME and not self.obstacle_front:
                self.return_timer = 0.0
                self.state = STATE_RETURN
                self.get_logger().info(
                    f'Ausgewichen – zurück auf Yaw {math.degrees(self.lane_yaw):.1f}°')

        elif self.state == STATE_RETURN:
            self.return_timer += dt
            if self.obstacle_front:
                self.avoid_timer = 0.0
                self.lane_y      = self.pos_y
                self.state = STATE_AVOID
                return

            yaw_err     = self._angle_diff(self.lane_yaw, self.yaw)
            yaw_err_abs = abs(math.degrees(yaw_err))
            y_err       = (self.lane_y - self.pos_y) if self.lane_y is not None else 0.0
            y_err_abs   = abs(y_err)

            if self.return_timer >= RETURN_TIMEOUT:
                self.get_logger().warn('RETURN-Timeout – weiterfahren')
                self._publish(0.0, 0.0, 0.0)
                self.state = STATE_DRIVE
                return

            if yaw_err_abs > RETURN_YAW_TOL:
                corr = 1.0 if yaw_err > 0 else -1.0
                self._publish(AVOID_SPEED*0.5, AVOID_ANGULAR*corr, AVOID_STEER*corr)
            elif y_err_abs > 0.15:
                side     = 1.0 if (y_err * math.cos(self.lane_yaw)) < 0 else -1.0
                strength = min(1.0, y_err_abs / 0.5)
                self._publish(AVOID_SPEED*0.6,
                              AVOID_ANGULAR*0.4*side*strength,
                              AVOID_STEER*0.4*side*strength)
            else:
                self.get_logger().info('Auf Kurs')
                self._publish(0.0, 0.0, 0.0)
                self.state = STATE_DRIVE

        # Rückkehr zur Ladestation (mit Bug-Algorithmus für Hindernisse)
        elif self.state == STATE_RETURN_BASE:
            dist_base = self._dist_to_base()
            if dist_base < BASE_ARRIVAL_DIST:
                self._publish(0.0, 0.0, 0.0)
                self.charge_timer    = 0.0
                self._battery_warned = False
                self.state = STATE_CHARGING
                self.get_logger().info(
                    f'🔌 Ladestation erreicht ({dist_base:.2f}m) – lade auf...')
                return

            # Hindernis → Wall-Follow (Bug-Algorithmus) aktivieren
            if self.obstacle_front or self.obstacle_stop:
                self.wall_follow_side       = 1 if self.dist_left >= self.dist_right else -1
                self.wall_follow_dist_start = dist_base
                seite = 'links' if self.wall_follow_side == 1 else 'rechts'
                self.get_logger().info(
                    f'🐜 Hindernis ({self.dist_front:.1f}m) – Wall-Follow {seite} '
                    f'(Basis {dist_base:.1f}m)')
                self.state = STATE_WALL_FOLLOW
                return

            angle_to_base = math.atan2(BASE_Y - self.pos_y, BASE_X - self.pos_x)
            yaw_err       = self._angle_diff(angle_to_base, self.yaw)
            yaw_err_abs   = abs(math.degrees(yaw_err))

            if yaw_err_abs > 8.0:
                corr = 1.0 if yaw_err > 0 else -1.0
                self._publish(TURN_FORWARD_SPD*0.5, TURN_SPEED*corr*0.7, MAX_STEER*corr*0.7)
            else:
                steer_corr = max(-0.3, min(0.3, yaw_err * 0.5))
                self._publish(DRIVE_SPEED * 0.8, 0.0, steer_corr)

        # NEU: Wall-Follow (Bug-Algorithmus) – Hindernis umrunden, dann zurück zu RETURN_BASE
        elif self.state == STATE_WALL_FOLLOW:
            dist_base     = self._dist_to_base()
            angle_to_base = math.atan2(BASE_Y - self.pos_y, BASE_X - self.pos_x)
            yaw_err_abs   = abs(math.degrees(self._angle_diff(angle_to_base, self.yaw)))

            # Abbruch: Weg zur Basis frei UND wir haben Fortschritt gemacht
            path_free = (self.dist_front > WALL_CLEAR_FRONT
                         and yaw_err_abs < WALL_CLEAR_ANGLE
                         and dist_base < self.wall_follow_dist_start + 1.0)
            if path_free:
                self.get_logger().info(
                    f'🐜 Hindernis umfahren – zurück zu RETURN_BASE (Basis {dist_base:.1f}m)')
                self.state = STATE_RETURN_BASE
                return

            # Wand auf wall_follow_side halten (P-Regler)
            wall_dist = self.dist_left if self.wall_follow_side == 1 else self.dist_right
            dist_err  = wall_dist - WALL_FOLLOW_DIST   # positiv = zu weit weg → zur Wand lenken
            angular   = WALL_FOLLOW_KP * dist_err * self.wall_follow_side
            angular   = max(-TURN_SPEED, min(TURN_SPEED, angular))
            steer     = max(-MAX_STEER, min(MAX_STEER, angular * 0.6))

            if self.obstacle_stop:
                # Zu nah vorne: auf der Stelle drehen von der Wand weg
                self._publish(0.0,
                              TURN_SPEED * (-self.wall_follow_side),
                              MAX_STEER  * (-self.wall_follow_side))
            elif self.dist_front < WALL_FOLLOW_DIST * 1.5:
                # Ecke: langsamer + stärker drehen
                self._publish(WALL_FOLLOW_SPEED * 0.5,
                              TURN_SPEED * (-self.wall_follow_side) * 0.8,
                              MAX_STEER  * (-self.wall_follow_side))
            else:
                self._publish(WALL_FOLLOW_SPEED, angular, steer)

        # NEU: Aufladen
        elif self.state == STATE_CHARGING:
            self.charge_timer += dt
            self._publish(0.0, 0.0, 0.0)
            self.battery_pct = min(BATTERY_START_PCT,
                                   self.battery_pct + (BATTERY_START_PCT / BATTERY_CHARGE_TIME) * dt)
            if self.charge_timer >= BATTERY_CHARGE_TIME:
                self.battery_pct = BATTERY_START_PCT
                self.state       = self.resume_state
                self.lane_yaw    = None
                self.get_logger().info(f'✅ Voll geladen – weiter: {self.resume_state}')

        elif self.state == STATE_DONE:
            self._publish(0.0, 0.0, 0.0)

    # ── Hilfsfunktionen ───────────────────────────────────────────────────────
    def _publish(self, linear: float, angular: float, steer: float):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)
        s = Float64()
        s.data = max(-0.5, min(0.5, steer))
        self.steer_pub.publish(s)

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d >  math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d


def main():
    rclpy.init()
    node = autodrive()
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
