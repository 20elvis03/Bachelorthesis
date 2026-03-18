#!/usr/bin/env python3
"""
auto_mow.py  –  Autonomer Rasenmäher für my_robot (3-Rad, DiffDrive + Lenkung)
===============================================================================
Roboter-Aufbau:
  - Antrieb:   DiffDrive (linkes + rechtes Hinterrad) -> /cmd_vel
  - Lenkung:   small_base_to_base revolute joint      -> /steering (rad)
  - Lenkbereich: +-0.5 rad (~+-28 Grad)
  - Sensoren:  3D-LiDAR (64 Kanaele) -> /scan, Odometrie -> /odom

Maehbereich: tile_X_0 bis tile_X_7 -> y-Grenze bei +15.0 m
Spawn: x=22.5, y=-22.5, Blickrichtung +Y (Yaw ca. +pi/2)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf_transformations


# ── Konfiguration ────────────────────────────────────────────────────────────
DRIVE_SPEED      = 1.5    # m/s vorwärts (Mähgeschwindigkeit)
AVOID_SPEED      = 0.3    # m/s beim Ausweichen
TURN_SPEED       = 0.4    # rad/s Winkelgeschwindigkeit beim Wenden

OBSTACLE_FRONT   = 2.5    # m – Hindernis erkannt → ausweichen
OBSTACLE_STOP    = 0.8    # m – Notstopp
WALL_TURN_DIST   = 3.0    # m – Bahnende erkannt → Wende einleiten
SIDE_MIN_DIST    = 1.2    # m – Mindestabstand seitlich für Wende

# Mähbereich: maximal bis tile_X_7 (Mitte y=12.5, Grenze zu _8 bei y=15.0)
# Sicherheitsabstand -0.5 m → Roboter dreht spätestens bei y=14.5
X_MIN_BOUNDARY   = -23.5   # m – absoluter Y-Stopp (Odometrie-basiert)
X_MAX_BOUNDARY   = 23.5   # m – absoluter Y-Stopp (Odometrie-basiert)
Y_MIN_BOUNDARY   = -23.5   # m – absoluter Y-Stopp (Odometrie-basiert)
Y_MAX_BOUNDARY   = 15.0   # m – absoluter Y-Stopp (Odometrie-basiert)

# LiDAR-Winkelkegel (Grad) – 360° Abdeckung
FRONT_CONE       = 25     # ±25° vorne
SIDE_CONE_START  = 55     # ab 55° seitlich
SIDE_CONE_END    = 125    # bis 125° seitlich
BACK_CONE        = 30     # ±30° hinten (|angle| > 150°)

# Hindernis-Schwellen
OBSTACLE_BACK    = 0.6    # m – Notstopp rueckwaerts

# LiDAR 3D → 2D
LIDAR_HORIZ_IDX  = 55
LIDAR_HORIZ_TOL  = 3

# Ausweich-Parameter
AVOID_ARC_TIME   = 2.2    # s – Mindestdauer Ausweichbogen
AVOID_STEER      = 0.35   # rad – Lenkwinkel beim Ausweichen
AVOID_ANGULAR    = 0.3    # rad/s – angular.z beim Ausweichen

# Rückkehr-Parameter (Yaw-basiert)
RETURN_YAW_TOL   = 4.0    # ° – Toleranz für "auf Kurs"
RETURN_TIMEOUT   = 10.0   # s – Fallback wenn Yaw nicht erreicht wird

# Wende-Parameter
MAX_STEER        = 0.48   # rad
TURN_FORWARD_SPD = 0.2    # m/s

NUM_LANES        = 60

# Stecken-Erkennung waehrend Wende
# Wenn der Roboter sich X Sekunden dreht aber < STUCK_DIST_M Meter bewegt hat
STUCK_CHECK_TIME = 3.0    # s – nach dieser Zeit Position pruefen
STUCK_DIST_M     = 0.15   # m – weniger als das = feststeckend
# Rueckwaerts-Dreh-Manöver
REVERSE_SPEED    = -0.25  # m/s rueckwaerts
REVERSE_STEER    = 0.45   # rad Lenkeinschlag beim Rueckwaertsdrehen
REVERSE_YAW_DEG  = 90.0   # Grad – um wieviel Grad soll rueckwaerts gedreht werden
# ─────────────────────────────────────────────────────────────────────────────

STATE_DRIVE        = 'DRIVE'
STATE_BRAKE        = 'BRAKE'
STATE_TURN_CHECK   = 'TURN_CHECK'
STATE_TURN         = 'TURN'
STATE_REVERSE_TURN = 'REVERSE_TURN'   # Rueckwaerts + 90-Grad-Drehung
STATE_AVOID        = 'AVOID'
STATE_RETURN       = 'RETURN'
STATE_DONE         = 'DONE'


class AutoMower(Node):
    def __init__(self):
        super().__init__('auto_mower')
	
        self.cmd_pub   = self.create_publisher(Twist,   '/cmd_vel',  10)
        self.steer_pub = self.create_publisher(Float64, '/steering', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)
        self.create_subscription(TFMessage, '/world/pose_info', self.global_pose_cb, 10)

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
        self.lane_yaw         = None   # Fahrtrichtung der aktuellen Bahn
        self.lane_y           = None   # Soll-Y der aktuellen Bahn (Ausweich-Rückkehr)

        self.pos_x            = 0.0
        self.pos_y            = 0.0
        self.odom_ready       = False

        self.global_x         = 0.0
        self.global_y         = 0.0
        self.global_yaw       = 0.0

        # Stecken-Erkennung: Position am Anfang der Wende merken
        self.stuck_check_timer  = 0.0
        self.stuck_ref_x        = 0.0
        self.stuck_ref_y        = 0.0
        self.stuck_checked      = False   # wurde in diesem TURN schon geprueft?

        # Rueckwaerts-Dreh-Zustand
        self.reverse_yaw_start  = 0.0
        self.reverse_dir        = 1      # +1=links, -1=rechts

        self.dist_front       = 99.0
        self.dist_left        = 99.0
        self.dist_right       = 99.0
        self.dist_back        = 99.0
        self.obstacle_front   = False
        self.obstacle_stop    = False
        self.obstacle_back    = False
        self.wall_near        = False

        # Periodisches Sensor-Log (alle 2 s)
        self._log_timer       = 0.0

        self.create_timer(0.05, lambda: self.loop(0.05))
        self.get_logger().info('AutoMower bereit – warte auf Sensoren...')

    # ── LiDAR ────────────────────────────────────────────────────────────────
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
        back_thresh = math.radians(180 - BACK_CONE)   # |angle| > 150° = hinten

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

    # ── Odometrie ────────────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])

        if not self.odom_ready:
            self.odom_ready = True
            self.get_logger().info(
                f'Start: x={self.pos_x:.1f} y={self.pos_y:.1f}  '
                f'vorne={self.dist_front:.1f}m')

    # ── Global Pose ────────────────────────────────────────────────────────────
    def global_pose_cb(self, msg: TFMessage):
      if len(msg.transforms) == 0:
        return
      t = msg.transforms[0]
      p = t.transform.translation
      r = t.transform.rotation
      self.global_x = p.x
      self.global_y = p.y
      _, _, self.global_yaw = tf_transformations.euler_from_quaternion(
          [r.x, r.y, r.z, r.w])

    # ── Haupt-Loop ───────────────────────────────────────────────────────────
    def loop(self, dt: float):
        if not self.odom_ready:
            return

        # Periodisches 360°-Sensor-Log alle 2 Sekunden
        self._log_timer += dt
        if self._log_timer >= 1.0:
            self._log_timer = 0.0
            self.get_logger().info(
                f'[SCAN] V={self.dist_front:.1f}m  L={self.dist_left:.1f}m  '
                f'R={self.dist_right:.1f}m  H={self.dist_back:.1f}m  '
                f'| Zustand={self.state}  x={self.pos_x:.1f} y={self.pos_y:.1f}, x_global={self.global_x:.1f} y_global={self.global_y:.1f}')

        # Notstopp vorne (außer Wende-Zustaende)
        if self.obstacle_stop and self.state not in (
                STATE_TURN, STATE_TURN_CHECK, STATE_REVERSE_TURN):
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().warn(
                f'NOTSTOPP VORNE! {self.dist_front:.2f}m',
                throttle_duration_sec=1.0)
            return

        # Notstopp hinten nur beim Rueckwaertsfahren
        if self.obstacle_back and self.state == STATE_REVERSE_TURN:
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().warn(
                f'NOTSTOPP HINTEN! {self.dist_back:.2f}m – breche Rueckwaerts ab')
            # Trotzdem Wende neu versuchen (vorwaerts)
            self.yaw_start_turn    = self.yaw
            self.stuck_check_timer = 0.0
            self.stuck_checked     = False
            self.stuck_ref_x       = self.pos_x
            self.stuck_ref_y       = self.pos_y
            self.state = STATE_TURN
            return

        # ── Zustandsmaschine ─────────────────────────────────────────────────
        out_of_bounds = (
            self.global_x <= X_MIN_BOUNDARY or
            self.global_x >= X_MAX_BOUNDARY or
            self.global_y <= Y_MIN_BOUNDARY or
            self.global_y >= Y_MAX_BOUNDARY
        )
        if self.state == STATE_DRIVE:
            # Bahn-Yaw beim allerersten Schritt einmalig speichern
            if self.lane_yaw is None:
                self.lane_yaw = self.yaw
                self.lane_y   = self.pos_y
                self.get_logger().info(
                    f'Bahn-Yaw gesetzt: {math.degrees(self.lane_yaw):.1f}°')
                
            # Y-Grenze überschritten → Bahn beenden wie bei Wand
            if out_of_bounds:
                self._publish(0.0, 0.0, 0.0)
                self.brake_timer = 0.0
                self.state = STATE_BRAKE
                self.get_logger().info(
                    f'Y-Grenze erreicht (y={self.global_y:.2f}m) – Wende')
                return

            # Y-Grenze überschritten → Bahn beenden wie bei Wand
            if out_of_bounds and self.wall_near:
                self._publish(0.0, 0.0, 0.0)
                self.brake_timer = 0.0
                self.state = STATE_BRAKE
                self.get_logger().info(
                f'Bahnende bei Boundary (x={self.global_x:.1f}, y={self.global_y:.1f}) '
                f'Wand in {self.dist_front:.1f}m – bremse (Bahn {self.lane_idx+1})')
                return

            # Wand / Bahnende per LiDAR
            if self.wall_near:
                self._publish(0.0, 0.0, 0.0)
                self.brake_timer = 0.0
                self.state = STATE_BRAKE
                self.get_logger().info(
                    f'Wand in {self.dist_front:.1f}m – bremse (Bahn {self.lane_idx+1})')
                return

            # Hindernis (kein Bahnende)
            if self.obstacle_front:
                self._publish(0.0, 0.0, 0.0)
                self.avoid_timer  = 0.0
                self.lane_y       = self.pos_y   # aktuelle Bahn-Y merken
                self.state = STATE_AVOID
                seite = 'links' if self.avoid_side == 1 else 'rechts'
                self.get_logger().info(
                    f'Hindernis {self.dist_front:.1f}m → weiche {seite} aus '
                    f'(Bahn-Yaw={math.degrees(self.lane_yaw):.1f}°)')
                return

            self._publish(DRIVE_SPEED, 0.0, 0.0)

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
                self._turn_dir = 1 if self.dist_left >= self.dist_right else -1
                self.yaw_start_turn    = self.yaw
                self.stuck_check_timer = 0.0
                self.stuck_checked     = False
                self.stuck_ref_x       = self.pos_x
                self.stuck_ref_y       = self.pos_y
                self.state = STATE_TURN
                seite = 'links' if self._turn_dir == 1 else 'rechts'
                self.get_logger().info(
                    f'Wende {seite} (L={self.dist_left:.1f}m R={self.dist_right:.1f}m)')
            elif self.turn_check_timer > 5.0:
                self.get_logger().warn('Kein Wendeplatz – rückwärts')
                self._publish(-0.15, 0.0, 0.0)

        elif self.state == STATE_TURN:
            turned = abs(self._angle_diff(self.yaw, self.yaw_start_turn))

            # Stecken-Erkennung: nach STUCK_CHECK_TIME Sekunden pruefen ob
            # der Roboter sich tatsaechlich fortbewegt hat
            self.stuck_check_timer += dt
            if not self.stuck_checked and self.stuck_check_timer >= STUCK_CHECK_TIME:
                self.stuck_checked = True
                dist_moved = math.hypot(
                    self.pos_x - self.stuck_ref_x,
                    self.pos_y - self.stuck_ref_y
                )
                if dist_moved < STUCK_DIST_M:
                    # Feststeckend – Rueckwaerts-Dreh-Manöver einleiten
                    self.get_logger().warn(
                        f'STUCK erkannt (bewegt={dist_moved:.2f}m in {STUCK_CHECK_TIME}s) '
                        f'– rueckwaerts + 90 Grad drehen')
                    self.reverse_yaw_start = self.yaw
                    # Rueckwaertsrichtung entgegengesetzt zur Wendeseite
                    self.reverse_dir = -self._turn_dir
                    self._publish(0.0, 0.0, 0.0)
                    self.state = STATE_REVERSE_TURN
                    return

            if turned < math.pi - 0.10:
                self._publish(
                    TURN_FORWARD_SPD,
                    TURN_SPEED * self._turn_dir,
                    MAX_STEER * self._turn_dir
                )
            else:
                # Wende fertig – neuen Bahn-Yaw merken
                self._publish(0.0, 0.0, 0.0)
                self.lane_yaw = self.yaw
                self.lane_y   = self.pos_y   # neue Bahn-Y nach der Wende
                self.lane_idx += 1
                if self.lane_idx >= NUM_LANES:
                    self.state = STATE_DONE
                    self.get_logger().info('Fertig – alle Bahnen abgefahren!')
                else:
                    self.state = STATE_DRIVE
                    self.get_logger().info(
                        f'Bahn {self.lane_idx+1} – Yaw={math.degrees(self.lane_yaw):.1f}°')

        elif self.state == STATE_REVERSE_TURN:
            # Rueckwaerts fahren und dabei 90 Grad drehen
            # Kein Notstopp-Check hinten – aber LiDAR-Rueckseite pruefen
            turned_back = abs(self._angle_diff(self.yaw, self.reverse_yaw_start))
            target_rad  = math.radians(REVERSE_YAW_DEG)

            if turned_back < target_rad - 0.08:
                self._publish(
                    REVERSE_SPEED,
                    TURN_SPEED * self.reverse_dir,
                    REVERSE_STEER * self.reverse_dir
                )
            else:
                # 90 Grad erreicht – normaler Wende-Versuch neu starten
                self._publish(0.0, 0.0, 0.0)
                self.get_logger().info(
                    f'Rueckwaerts-Drehung fertig – starte Wende neu')
                # Wende-Parameter fuer neuen Versuch zuruecksetzen
                self.yaw_start_turn    = self.yaw
                self.stuck_check_timer = 0.0
                self.stuck_checked     = False
                self.stuck_ref_x       = self.pos_x
                self.stuck_ref_y       = self.pos_y
                self.state = STATE_TURN

        elif self.state == STATE_AVOID:
            self.avoid_timer += dt
            self._publish(
                AVOID_SPEED,
                AVOID_ANGULAR * self.avoid_side,
                AVOID_STEER   * self.avoid_side
            )
            # Erst wechseln wenn Mindestzeit abgelaufen UND kein Hindernis mehr
            if self.avoid_timer > AVOID_ARC_TIME and not self.obstacle_front:
                self.return_timer = 0.0
                self.state = STATE_RETURN
                self.get_logger().info(
                    f'Ausgewichen – korrigiere auf Yaw {math.degrees(self.lane_yaw):.1f}°')

        elif self.state == STATE_RETURN:
            self.return_timer += dt

            # Wenn während Rückkehr neues Hindernis → wieder ausweichen
            if self.obstacle_front:
                self.avoid_timer = 0.0
                self.lane_y      = self.pos_y   # neue Ist-Y als Basis
                self.state = STATE_AVOID
                self.get_logger().warn('Neues Hindernis in RETURN – weiche erneut aus')
                return

            yaw_err     = self._angle_diff(self.lane_yaw, self.yaw)
            yaw_err_abs = abs(math.degrees(yaw_err))

            # Y-Versatz zur Soll-Bahn berechnen
            y_err = (self.lane_y - self.pos_y) if self.lane_y is not None else 0.0
            y_err_abs = abs(y_err)

            if self.return_timer >= RETURN_TIMEOUT:
                self.get_logger().warn(
                    f'RETURN-Timeout (Yaw={yaw_err_abs:.1f}° Y-Err={y_err_abs:.2f}m) – weiterfahren')
                self._publish(0.0, 0.0, 0.0)
                self.state = STATE_DRIVE
                return

            # Phase 1: Yaw korrigieren
            if yaw_err_abs > RETURN_YAW_TOL:
                corr = 1.0 if yaw_err > 0 else -1.0
                self._publish(
                    AVOID_SPEED * 0.5,
                    AVOID_ANGULAR * corr,
                    AVOID_STEER   * corr
                )
            # Phase 2: Y-Versatz korrigieren (seitlich zurück auf Soll-Bahn)
            elif y_err_abs > 0.15:
                # Lenkung in Richtung Soll-Y, leicht vorwärts
                # lane_yaw bestimmt ob +y links oder rechts ist
                # Wenn Roboter in +X fährt (yaw≈0): +y = links
                # Wenn Roboter in -X fährt (yaw≈π): +y = rechts
                # Vorzeichen über sin(lane_yaw) bestimmen
                side = 1.0 if (y_err * math.cos(self.lane_yaw)) < 0 else -1.0
                # Korrekturstärke proportional zum Fehler, begrenzt
                strength = min(1.0, y_err_abs / 0.5)
                self._publish(
                    AVOID_SPEED * 0.6,
                    AVOID_ANGULAR * 0.4 * side * strength,
                    AVOID_STEER   * 0.4 * side * strength
                )
            else:
                self.get_logger().info(
                    f'Auf Kurs (Yaw={yaw_err_abs:.1f}° Y-Err={y_err_abs:.2f}m)')
                self._publish(0.0, 0.0, 0.0)
                self.state = STATE_DRIVE

        elif self.state == STATE_DONE:
            self._publish(0.0, 0.0, 0.0)

    # ── Hilfsfunktionen ──────────────────────────────────────────────────────
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
    node = AutoMower()
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
