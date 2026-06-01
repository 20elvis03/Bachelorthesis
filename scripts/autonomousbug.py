#!/usr/bin/env python3
import math, rclpy, tf_transformations, struct
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


#ros2 topic pub --once /emergency_reset std_msgs/Float64 "data: 1.0"


# ── Drive ────────────────────────────────────────────────────────────────────
DRIVE_SPEED      = 0.4 # forward speed (m/s)
DRIVE_LANE_KP    = 2.0 # lane-keeping steering gain

# ── Safety ───────────────────────────────────────────────────────────────────
MAX_SPEED_MPS     = 1.3 # absolute max speed (m/s) – emergency shutdown if exceeded

# ── Obstacles ────────────────────────────────────────────────────────────────
OBSTACLE_FRONT   = 2.9 # front obstacle detection distance (m)
OBSTACLE_STOP    = 1.2 # emergency stop distance (m)
OBSTACLE_BACK    = 0.6 # rear obstacle detection (m)
SIDE_MIN_DIST    = 1.2 # minimum side clearance (m)

# ── Boundaries ───────────────────────────────────────────────────────────────
X_MIN               = -23.0
X_MAX               = 23.0
Y_MIN               = -23.0
Y_MAX               = 15.0
BOUNDARY_GRACE_DIST = 3.0 # ignore boundary after U-turn until this far away (m)

# ── Garage / Charging ────────────────────────────────────────────────────
GARAGE_X_MIN        = 10.5
GARAGE_X_MAX        = 24.5
GARAGE_Y_MIN        = -34.5
GARAGE_Y_MAX        = -23.0

CHARGE_ZONE_RADIUS  = 2.5 # sensors disabled within this radius of dock

# ── LiDAR cones (degrees) ───────────────────────────────────────────────────
FRONT_CONE      = 25
SIDE_CONE_START = 55
SIDE_CONE_END   = 125
BACK_CONE       = 30
FDIAG_START     = 25
FDIAG_END       = 55

# ── U-turn ───────────────────────────────────────────────────────────────────
MAX_STEER        = 0.49 # maximum steering angle (rad)
TURN_SPEED       = 2.0 # angular speed during turn (rad/s)
TURN_FORWARD_SPD = 0.6 # forward speed during turn (m/s)
LANE_OFFSET      = 2.0 # X offset for new lane after U‑turn

# ── Stuck / Reverse ─────────────────────────────────────────────────────────
STUCK_CHECK_TIME = 7.0 # time to check for progress (s)
STUCK_DIST_M     = 0.15 # minimum distance to consider progress (m)
REVERSE_SPEED    = -0.4 # reverse speed (m/s)
REVERSE_STEER    = 0.49 # steering while reversing (rad)
REVERSE_YAW_DEG  = 90.0 # yaw change needed to finish reverse (deg)

# ── Bug2 Wall-Following ─────────────────────────────────────────────────────
BUG2_LINE_TOL         = 0.3 # max distance to M-Line to consider "reached"
BUG2_FOLLOW_WALL_LIN  = 0.7 # forward speed while wall-following
BUG2_FOLLOW_TARGET    = 1.7 # desired distance to wall (m)
BUG2_TURN_ANG         = 1.2 # angular speed during initial bug2 turn
BUG2_TURN_FWD         = 0.5 # forward speed during initial bug2 turn

# ── Bug2 Return ──────────────────────────────────────────────────────────────
BUG2_RETURN_SPEED     = 1.0
BUG2_RETURN_KP        = 1.5
BUG2_RETURN_LOOKAHEAD = 3.0 # look-ahead distance for M-Line return

# ── Global Stuck Recovery ───────────────────────────────────────────────
GLOBAL_STUCK_TIME     = 50.0
GLOBAL_STUCK_DIST     = 0.15
GLOBAL_REV_TIME       = 5.0
GLOBAL_REV_SPEED      = -0.5
GLOBAL_REV_STEER      = 0.45

# ── Robot Yielding (multi-robot priority) ───────────────────────────────────
YIELD_CHECK_DIST      = 4.0 # max distance to consider another robot (m)
YIELD_CLEAR_DIST      = 5.0 # distance at which yield ends (m)
YIELD_CONE_DEG        = 50.0 # forward cone for robot detection (degrees)
YIELD_FACING_DEG      = 120.0 # heading difference to consider "facing each other"
YIELD_TIMEOUT         = 15.0 # max yield time before fallback to Bug2 (s)

# ── Go Home (low battery) ───────────────────────────────────────────────────
BAT_LOW_PCT           = 20.0 # start heading home (%)
HOME_SPEED            = 0.6
HOME_KP               = 2.0
CHARGE_RATE_PCT       = 0.2 # battery % gained per second while charging

# ── States ───────────────────────────────────────────────────────────────────
S_DRIVE      = 'DRIVE'
S_BRAKE      = 'BRAKE'
S_TURN_CHECK = 'TURN_CHECK'
S_TURN       = 'TURN'
S_REVERSE    = 'REVERSE_TURN'
S_BUG2       = 'BUG2_WALL'
S_RETURN     = 'BUG2_RETURN'
S_GO_HOME    = 'GO_HOME'
S_DONE       = 'DONE'
S_EMERGENCY  = 'EMERGENCY'
S_CHARGE     = 'CHARGING'
S_YIELD      = 'YIELD'

class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')

        # ── ROS parameters (set per robot via launch file) ───────
        self.declare_parameter('robot_name', 'robot_1')
        self.declare_parameter('spawn_gx', 22.5)
        self.declare_parameter('spawn_gy', -22.5)
        self.declare_parameter('spawn_yaw_deg', 90.0)
        self.declare_parameter('init_x', 99999.0)
        self.declare_parameter('init_y', 99999.0)

        self.robot_name = self.get_parameter('robot_name').value

        self.cmd_pub   = self.create_publisher(Twist,   'cmd_vel',  10)
        self.steer_pub = self.create_publisher(Float64, 'steering', 10)
        self.create_subscription(PointCloud2, 'scan/points', self._scan_cb, 10)
        # Absolute topic: world pose_info is shared across all robots
        self.create_subscription(TFMessage, '/world/pose_info', self._pose_cb, 10)

        # Multi-robot coordination (absolute topics, bypass namespace)
        self.coord_pub = self.create_publisher(PoseStamped, '/robot_coordination', 10)
        self.create_subscription(PoseStamped, '/robot_coordination', self._coord_cb, 10)

        # Emergency
        self.emergency = False
        self.create_subscription(
            Float64, 'emergency_reset', self._emergency_reset_cb, 10)
        
        # Global pose
        self.gx         = 0.0
        self.gy         = 0.0
        self.gyaw       = 0.0
        self.pose_ready = False
        
        # State
        self.state      = S_DRIVE
        self.lane_gx    = None # target X for lane-keeping
        self.lane_yaw   = None # current heading direction
        self.sweep_dir  = 0 # +1 sweep right, -1 sweep left
        self._turn_dir  = 1 # U-turn direction: +1=left, -1=right
        self.avoid_side = 1 # preferred bug2 side based on LiDAR

        # Boundary grace after U-turn
        self.boundary_grace   = False # ignore OOB briefly after U-turn
        self.turn_complete_gy = 0.0 # Y pos where last U-turn ended

        # Battery
        self.bat_pct  = 100.0
        self.bat_gx   = 0.0 # last position for distance-based drain
        self.bat_gy   = 0.0 # last position for distance-based drain
        self.bat_init = False

        # Spawn / Home
        self.spawn_gx   = self.get_parameter('spawn_gx').value
        self.spawn_gy   = self.get_parameter('spawn_gy').value
        self.spawn_gyaw  = math.radians(self.get_parameter('spawn_yaw_deg').value)
        self.init_x     = self.get_parameter('init_x').value
        self.init_y     = self.get_parameter('init_y').value
        self.going_home     = False
        self.home_phase     = 'nav_to_lane' # initial GO_HOME phase
        self.home_uturn_yaw = 0.0 # yaw at start of home U-turn
        self.home_uturn_dir = 1 # home U-turn direction
        self.pre_home_lane_gx   = None
        self.pre_home_lane_yaw  = None
        self.pre_home_sweep_dir = None
        self.pre_home_gy        = 0.0
   
        # Timers
        self.brake_timer      = 0.0
        self.turn_check_timer = 0.0
        self._log_t           = 0.0

        # Stuck detection (shared helper)
        self.stuck_timer    = 0.0
        self.stuck_gx       = 0.0
        self.stuck_gy       = 0.0
        self.stuck_done     = False
        self.yaw_start_turn = 0.0
        self.rev_yaw_start  = 0.0
        self.rev_dir        = 1

        # Global stuck recovery
        self.gstuck_timer   = 0.0
        self.gstuck_gx      = None
        self.gstuck_gy      = None
        self.gstuck_active  = False
        self.gstuck_rev_t   = 0.0
        self.gstuck_rev_dir = 1
        self.gstuck_count   = 0
        self.gstuck_area_gx = 0.0
        self.gstuck_area_gy = 0.0

        # LiDAR distances
        self.d_front        = 99.0
        self.d_left         = 99.0
        self.d_right        = 99.0
        self.d_back         = 99.0
        self.d_fright       = 99.0
        self.d_fleft        = 99.0
        self.obs_front      = False
        self.obs_stop       = False
        self.obs_back       = False

        # Bug2
        self.bug_timer       = 0.0
        self.bug_side        = 1
        self.hit_gx          = 0.0 # M-Line X where bug2 started
        self.pre_bug_lane_gx = None  # saved lane_gx before bug2
        self.bug_start_yaw   = 0.0 # yaw when bug2 started

        self._bug2_phase     = 'turn'
        self._bug2_straight_gx   = 0.0
        self._bug2_straight_gy   = 0.0

        # Multi-robot coordination
        self.other_robots    = {}   # {name: (x, y, yaw)}
        self.yield_to        = None # name of robot we're yielding to
        self.pre_yield_state = None # state to resume after yield
        self.yield_timer     = 0.0  # time spent yielding

        self.create_timer(0.05, lambda: self._loop(0.05))
        self.get_logger().info(
            f'AutoDrive ready [{self.robot_name}] '
            f'init=({self.init_x:.1f},{self.init_y:.1f}) '
            f'dock=({self.spawn_gx:.1f},{self.spawn_gy:.1f})')

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _pub(self, lin, ang, steer):
        if self.emergency:
            t = Twist()
            self.cmd_pub.publish(t)
            s = Float64()
            s.data = 0.0
            self.steer_pub.publish(s)
            return
        
        # Speed limit check
        if abs(lin) > MAX_SPEED_MPS:
            self.emergency = True
            self.state = S_EMERGENCY
            self.get_logger().error(
                f'EMERGENCY SHUTDOWN: speed {abs(lin):.2f} m/s > {MAX_SPEED_MPS} m/s limit! '
                f'Publish 1.0 to /emergency_reset to restart.')
            t = Twist()
            self.cmd_pub.publish(t)
            s = Float64()
            s.data = 0.0
            self.steer_pub.publish(s)
            return

        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)
        s = Float64()
        s.data = max(-0.5, min(0.5, float(steer)))
        self.steer_pub.publish(s)

    @staticmethod
    def _adiff(a, b):
        d = a - b

        while d > math.pi:
            d -= 2*math.pi

        while d < -math.pi:
             d += 2*math.pi

        return d

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _oob(self):
        in_main   = (X_MIN <= self.gx <= X_MAX and Y_MIN <= self.gy <= Y_MAX)
        in_garage = (self.going_home and
                     GARAGE_X_MIN <= self.gx <= GARAGE_X_MAX
                     and GARAGE_Y_MIN <= self.gy <= GARAGE_Y_MAX)
        return not (in_main or in_garage)
    
    def _near_oob_boundary(self, margin: float) -> bool:
        return (self.gx < X_MIN + margin or
                self.gx > X_MAX - margin or
                self.gy < Y_MIN + margin or
                self.gy > Y_MAX - margin)
    
    def _in_charge_zone(self):
        return math.hypot(self.gx - self.spawn_gx, self.gy - self.spawn_gy) < CHARGE_ZONE_RADIUS

    def _reset_stuck(self):
        self.stuck_timer = 0.0
        self.stuck_gx    = self.gx
        self.stuck_gy    = self.gy
        self.stuck_done  = False

    def _reset_gstuck(self):
        self.gstuck_timer = 0.0
        self.gstuck_gx    = self.gx
        self.gstuck_gy    = self.gy

    def _bug2_log(self, phase, ds, dd, df):
        extra = ""
        if phase in ('ALIGN', 'FOLLOW'):
            e_dist = ds - BUG2_FOLLOW_TARGET
            extra = f'e_dist={e_dist:+.2f} '
        self.get_logger().info(
            f'Bug2: {phase} ds={ds:.2f} dd={dd:.2f} df={df:.1f} '
            f'F={self.d_front:.1f}m FL={self.d_fleft:.1f}m FR={self.d_fright:.1f}m '
            f'L={self.d_left:.1f}m R={self.d_right:.1f}m B={self.d_back:.1f}m '
            f'yaw={math.degrees(self.gyaw):.1f}° {extra}'
            f'| BAT={self.bat_pct:.1f}% gx={self.gx:.1f} gy={self.gy:.1f}')

    def _heading_is_positive_y(self):
        return abs(self._adiff(self.gyaw, math.pi/2)) < math.pi/2

    def _robot_in_front(self):
        """Return the name of a robot that is in our forward cone AND facing
        us (collision course), or None."""
        cone = math.radians(YIELD_CONE_DEG)
        facing_thresh = math.radians(YIELD_FACING_DEG)
        for name, (rx, ry, ryaw) in self.other_robots.items():
            dx = rx - self.gx
            dy = ry - self.gy
            dist = math.hypot(dx, dy)
            if dist > YIELD_CHECK_DIST or dist < 0.3:
                continue
            angle_to = math.atan2(dy, dx)
            if abs(self._adiff(angle_to, self.gyaw)) > cone:
                continue
            heading_diff = abs(self._adiff(self.gyaw, ryaw))
            if heading_diff > facing_thresh:
                return name
        return None

    def _should_yield(self, other_name):
        """Return True if we have lower priority than other_name.
        Lower robot name = higher priority (robot_1 > robot_2 > robot_3)."""
        return self.robot_name > other_name

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _emergency_reset_cb(self, msg):
        if msg.data == 1.0 and self.emergency:
            self.emergency = False
            self.state = S_DRIVE
            self.get_logger().warning('EMERGENCY RESET by operator → DRIVE')

    def _coord_cb(self, msg: PoseStamped):
        """Receive other robots' positions from the shared coordination topic."""
        name = msg.header.frame_id
        if name == self.robot_name or not name:
            return
        q = msg.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self.other_robots[name] = (
            msg.pose.position.x, msg.pose.position.y, yaw)

    def _publish_coordination(self):
        """Broadcast own world pose so other robots can see us."""
        msg = PoseStamped()
        msg.header.frame_id = self.robot_name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.gx
        msg.pose.position.y = self.gy
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.gyaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.coord_pub.publish(msg)

    def _scan_cb(self, msg: PointCloud2):
        fm = lm = rm = bm = frm = flm = 99.0
        fc = math.radians(FRONT_CONE)
        ss = math.radians(SIDE_CONE_START)
        se = math.radians(SIDE_CONE_END)
        bt = math.radians(180 - BACK_CONE)
        ds = math.radians(FDIAG_START)
        de = math.radians(FDIAG_END)

        if not hasattr(self, '_pc_offsets'):
            self._pc_offsets = {}
            for f in msg.fields:
                self._pc_offsets[f.name] = f.offset
            self._pc_step = msg.point_step

        ox   = self._pc_offsets.get('x', 0)
        oy   = self._pc_offsets.get('y', 4)
        oz   = self._pc_offsets.get('z', 8)
        step = self._pc_step
        data = msg.data

        for i in range(0, len(data), step):
            if i + oz + 4 > len(data):
                break
            x = struct.unpack_from('f', data, i + ox)[0]
            y = struct.unpack_from('f', data, i + oy)[0]
            z = struct.unpack_from('f', data, i + oz)[0]

            r_horiz = math.sqrt(x * x + y * y)

            if r_horiz < 0.05:
                continue
            if z < -1.85:
                continue
            if z > 0.3:
                continue

            h_angle = math.atan2(y, x)

            if abs(h_angle) < fc:
                fm = min(fm, r_horiz)
            elif ds < h_angle < de:
                flm = min(flm, r_horiz)
            elif -de < h_angle < -ds:
                frm = min(frm, r_horiz)
            elif ss < h_angle < se:
                lm = min(lm, r_horiz)
            elif -se < h_angle < -ss:
                rm = min(rm, r_horiz)
            elif abs(h_angle) > bt:
                bm = min(bm, r_horiz)

        self.d_front        = fm
        self.d_left         = lm
        self.d_right        = rm
        self.d_back         = bm
        self.d_fright       = frm
        self.d_fleft        = flm
        self.obs_stop       = fm < OBSTACLE_STOP
        self.obs_front      = fm < OBSTACLE_FRONT
        self.obs_back       = bm < OBSTACLE_BACK

        if self._in_charge_zone():
            self.obs_front      = False
            self.obs_stop       = False
            self.obs_back       = False

        if lm > rm + 0.5:
            self.avoid_side = 1
        elif rm > lm + 0.5:
            self.avoid_side = -1

    def _pose_cb(self, msg: TFMessage):
        if not msg.transforms:
            return
        ref_x = self.init_x if not self.pose_ready else self.gx
        ref_y = self.init_y if not self.pose_ready else self.gy
        max_d = 2.0 if not self.pose_ready else 3.0

        best_dist, best_tf = 999.0, None
        for tf in msg.transforms:
            p = tf.transform.translation
            d = math.hypot(p.x - ref_x, p.y - ref_y)
            if d < best_dist:
                best_dist, best_tf = d, tf
        if best_tf and best_dist < max_d:
            p = best_tf.transform.translation
            r = best_tf.transform.rotation
            self.gx, self.gy = p.x, p.y
            _, _, self.gyaw = tf_transformations.euler_from_quaternion([r.x, r.y, r.z, r.w])
            if not self.pose_ready:
                self.pose_ready = True
                self.get_logger().info(
                    f'[{self.robot_name}] Pose ready g=({self.gx:.1f},{self.gy:.1f}) dist={best_dist:.2f}m')

    # ── Main Loop ────────────────────────────────────────────────────────────
    def _loop(self, dt):
        if not self.pose_ready:
            return
        self._publish_coordination()
        if not hasattr(self, '_startup_waited'):
            self._startup_waited = 0.0
        if self._startup_waited < 3.0:
            self._startup_waited += dt
            self._pub(0, 0, 0)
            if self._startup_waited >= 3.0:
                self.get_logger().info(f'[{self.robot_name}] Startup wait done → GO')
            return
        if self.emergency:
            self._pub(0, 0, 0)
            return

        if not self.bat_init:
            self.bat_gx   = self.gx
            self.bat_gy   = self.gy
            self.bat_init = True
            self.get_logger().info(
                f'Home station at ({self.spawn_gx:.1f}, {self.spawn_gy:.1f}) yaw={math.degrees(self.spawn_gyaw):.1f}°')
        else:
            d = math.hypot(self.gx - self.bat_gx, self.gy - self.bat_gy)
            if d > 0.05 and self.state != S_CHARGE:
                self.bat_pct -= d * 0.2
                self.bat_gx   = self.gx
                self.bat_gy   = self.gy

        if self.bat_init and self.state != S_CHARGE:
            self.bat_pct -= 0.01 * dt

        if self.bat_pct <= 0:
            self.bat_pct = 0
            self._pub(0, 0, 0)
            if self.state != S_DONE:
                self.state = S_DONE
                self.get_logger().warning('BATTERY DEAD – stopped!')
            return

        if self.bat_pct < BAT_LOW_PCT and self.state not in (S_GO_HOME, S_DONE, S_CHARGE):
            if self.going_home:
                pass
            else:
                self.going_home = True
                self.home_phase = 'nav_to_lane'
                self.pre_home_lane_gx = self.lane_gx
                self.pre_home_lane_yaw = self.lane_yaw
                self.pre_home_sweep_dir = self.sweep_dir
                self.pre_home_gy = self.gy
                self._pub(0, 0, 0)
                prev = self.state
                self.state = S_GO_HOME
                dist = math.hypot(self.gx - self.spawn_gx, self.gy - self.spawn_gy)
                self.get_logger().warning(
                    f'LOW BATTERY {self.bat_pct:.1f}% – heading home '
                    f'from {prev} ({self.spawn_gx:.1f}, {self.spawn_gy:.1f}), dist={dist:.1f}m')
                return

        self._log_t += dt
        if self._log_t >= 1.0:
            self._log_t = 0.0
            self.get_logger().info(
                f'[SCAN] F={max(0,self.d_front-1.0):.1f}m FL={max(0,self.d_fleft-0.7):.1f}m FR={max(0,self.d_fright-0.7):.1f}m '
                f'L={max(0,self.d_left-0.7):.1f}m R={max(0,self.d_right-0.7):.1f}m B={max(0,self.d_back-0.7):.1f}m '
                f'| STATE={self.state} BAT={self.bat_pct:.1f}% gx={self.gx:.1f} gy={self.gy:.1f}')

        if self.obs_stop and self.state not in (S_TURN, S_TURN_CHECK, S_REVERSE, S_CHARGE, S_YIELD):
            other = self._robot_in_front()
            if other and self._should_yield(other):
                self._pub(0, 0, 0)
                if self.state != S_YIELD:
                    self.yield_to = other
                    self.pre_yield_state = self.state
                    self.yield_timer = 0.0
                    self.state = S_YIELD
                    self.get_logger().info(
                        f'[{self.robot_name}] YIELD to {other} (E-STOP, priority)')
                return
            if other and not self._should_yield(other):
                pass
            else:
                self._pub(0, 0, 0)
                self.get_logger().warning(
                    f'E-STOP FRONT {self.d_front:.2f}m', throttle_duration_sec=1.0)
                return
        
        if self.obs_back and self.state == S_REVERSE:
            self._pub(0, 0, 0)
            self.yaw_start_turn = self.gyaw
            self._reset_stuck()
            self.state = S_TURN
            return
        
        # ── GLOBAL STUCK RECOVERY (runs in every state) ──────────────
        if self.gstuck_active:
            self.gstuck_rev_t += dt
            self._pub(GLOBAL_REV_SPEED, 0.4 * self.gstuck_rev_dir,
                      GLOBAL_REV_STEER * self.gstuck_rev_dir)
            if self.gstuck_rev_t  >= GLOBAL_REV_TIME:
                self.gstuck_active = False
                self.gstuck_timer  = 0.0
                self.gstuck_gx     = self.gx
                self.gstuck_gy     = self.gy
                self._pub(0, 0, 0)

                if math.hypot(self.gx - self.gstuck_area_gx,
                              self.gy - self.gstuck_area_gy) < 2.0:
                    self.gstuck_count  += 1
                else:
                    self.gstuck_count   = 1
                    self.gstuck_area_gx = self.gx
                    self.gstuck_area_gy = self.gy

                if self.gstuck_count >= 3:
                    self.gstuck_count = 0
                    if self.going_home:
                        self.state = S_GO_HOME
                        self.home_phase = 'nav_to_lane'
                        self.get_logger().warning(
                            f'GLOBAL STUCK x3 → force GO_HOME turn')
                    else:
                        if self._near_oob_boundary(1.0):
                            self.brake_timer = 0
                            self.state = S_BRAKE
                        else:
                            self._start_bug2()
                        self.get_logger().warning(
                            f'GLOBAL STUCK x3 → force Bug2 escape')
                else:
                    self.get_logger().warning(
                        f'GLOBAL STUCK recovery done ({self.gstuck_count}/3) → resume')
            return

        if self.gstuck_gx is None:
            self.gstuck_gx = self.gx
            self.gstuck_gy = self.gy

        self.gstuck_timer += dt
        
        if self.state in (S_CHARGE, S_DONE, S_EMERGENCY):
            self.gstuck_timer = 0.0
            self.gstuck_gx = self.gx
            self.gstuck_gy = self.gy

        if self.gstuck_timer >= GLOBAL_STUCK_TIME:
            moved = math.hypot(self.gx - self.gstuck_gx, self.gy - self.gstuck_gy)
            if moved < GLOBAL_STUCK_DIST:
                self.gstuck_active  = True
                self.gstuck_rev_t   = 0.0
                self.gstuck_rev_dir = 1 if self.d_left > self.d_right else -1
                self.get_logger().warning(
                    f'GLOBAL STUCK! moved={moved:.2f}m in {GLOBAL_STUCK_TIME}s '
                    f'state={self.state} → reverse recovery')
                return
            self.gstuck_timer = 0.0
            self.gstuck_gx    = self.gx
            self.gstuck_gy    = self.gy

        if not hasattr(self, '_prev_state'):
            self._prev_state = self.state
        if self.state != self._prev_state:
            self._reset_gstuck()
            self._prev_state = self.state

        # ── DRIVE ────────────────────────────────────────────────────────
        if self.state == S_DRIVE:
            if self.lane_yaw is None:
                self.lane_yaw = self.gyaw
                self.lane_gx  = self.gx
                self.sweep_dir = -1 if self.gx > 0 else 1
                self.get_logger().info(
                    f'sweep={"→-X" if self.sweep_dir==-1 else "→+X"}')

            if self.boundary_grace and abs(self.gy - self.turn_complete_gy) > BOUNDARY_GRACE_DIST:
                self.boundary_grace = False

            if self._oob() and not self.boundary_grace:
                self._pub(0, 0, 0)
                self.brake_timer = 0
                self.state = S_BRAKE
                self.get_logger().info(f'Boundary (gx={self.gx:.1f} gy={self.gy:.1f}) → U-turn')
                return

            if self.obs_front:
                other = self._robot_in_front()
                if other and self._should_yield(other):
                    self._pub(0, 0, 0)
                    self.yield_to = other
                    self.pre_yield_state = S_DRIVE
                    self.yield_timer = 0.0
                    self.state = S_YIELD
                    self.get_logger().info(
                        f'[{self.robot_name}] YIELD to {other} in DRIVE')
                    return
                self._pub(0, 0, 0)
                if self._near_oob_boundary(1.0):
                    self.get_logger().warning("Obstacle is OOB wall → U-turn instead of Bug2")
                    self.brake_timer = 0
                    self.state = S_BRAKE
                    return
                self._start_bug2()
                return

            if self.lane_gx is not None:
                tgy = self.gy + 5.0*math.sin(self.gyaw)
                dyaw = math.atan2(tgy-self.gy, self.lane_gx-self.gx)
                se = self._adiff(dyaw, self.gyaw)
                self._pub(DRIVE_SPEED, self._clamp(se*DRIVE_LANE_KP, -0.4, 0.4),
                          self._clamp(se*DRIVE_LANE_KP, -MAX_STEER*0.5, MAX_STEER*0.5))
            else:
                self._pub(DRIVE_SPEED, 0, 0)

        # ── BRAKE ────────────────────────────────────────────────────────
        elif self.state == S_BRAKE:
            self.brake_timer += dt
            self._pub(0, 0, 0)
            if self.brake_timer > 0.4:
                self.turn_check_timer = 0
                self.state = S_TURN_CHECK

        # ── TURN_CHECK (determine direction) ─────────────────────────────
        elif self.state == S_TURN_CHECK:
            self.turn_check_timer += dt
            self._pub(0, 0, 0)
            lok = self.d_left > SIDE_MIN_DIST
            rok = self.d_right > SIDE_MIN_DIST
            if lok or rok:
                if self._heading_is_positive_y():
                    self._turn_dir = -self.sweep_dir
                else:
                    self._turn_dir  = self.sweep_dir
                self.yaw_start_turn = self.gyaw
                self._reset_stuck()
                self.state = S_TURN
                self.get_logger().info(f'U-turn {"L" if self._turn_dir==1 else "R"} (dL={self.d_left:.1f} dR={self.d_right:.1f})')
            elif self.turn_check_timer > 5.0:
                self._pub(-0.15, 0, 0)

        # ── TURN ─────────────────────────────────────────────────────────
        elif self.state == S_TURN:
            turned = abs(self._adiff(self.gyaw, self.yaw_start_turn))
            self.stuck_timer += dt
            if not self.stuck_done and self.stuck_timer >= STUCK_CHECK_TIME:
                self.stuck_done = True
                if math.hypot(self.gx-self.stuck_gx, self.gy-self.stuck_gy) < STUCK_DIST_M:
                    self.get_logger().warning('STUCK – reverse')
                    self.rev_yaw_start = self.gyaw
                    self.rev_dir = -self._turn_dir
                    self._pub(0, 0, 0)
                    self.state = S_REVERSE
                    return
                
            if turned < math.pi - 0.10:
                self._pub(TURN_FORWARD_SPD, TURN_SPEED*self._turn_dir, MAX_STEER*self._turn_dir)
            else:
                self._pub(0, 0, 0)
                self.lane_yaw = self.gyaw
                self.boundary_grace = True
                self.turn_complete_gy = self.gy
                old_gx = self.lane_gx if self.lane_gx is not None else self.gx
                self.lane_gx = old_gx + self.sweep_dir * LANE_OFFSET
                self.state = S_DRIVE
                self.get_logger().info(f'New lane gx={self.lane_gx:.1f} (from {old_gx:.1f})')

        # ── REVERSE ──────────────────────────────────────────────────────
        elif self.state == S_REVERSE:
            if abs(self._adiff(self.gyaw, self.rev_yaw_start)) < math.radians(REVERSE_YAW_DEG)-0.08:
                self._pub(REVERSE_SPEED, TURN_SPEED*self.rev_dir, REVERSE_STEER*self.rev_dir)
            else:
                self._pub(0, 0, 0)
                self.yaw_start_turn = self.gyaw
                self._reset_stuck()
                self.state = S_TURN

        # ── BUG2 WALL-FOLLOWING ──────────────────────────────────────────
        elif self.state == S_BUG2:
            if self._oob():
                self._pub(0, 0, 0)
                if self.going_home:
                    self.home_phase = 'nav_to_lane'
                    self.state = S_GO_HOME
                    self.get_logger().warning(
                        f'Bug2: OOB during GO_HOME → back to GO_HOME nav')
                    return
                self.brake_timer = 0
                self.state = S_BRAKE
                self.get_logger().warning(f'Bug2: OUT OF BOUNDS → U-turn')
                return

            self.bug_timer += dt
            s  = self.bug_side
            ds = self.d_right if s == 1 else self.d_left
            dd = self.d_fright if s == 1 else self.d_fleft
            df = self.d_front

            if self._bug2_phase == 'turn':
                turned = abs(self._adiff(self.gyaw, self.bug_start_yaw))
                if turned < math.radians(50):
                    self._pub(BUG2_TURN_FWD, BUG2_TURN_ANG * s, MAX_STEER * s)
                else:
                    self._bug2_phase = 'straight'
                    self._bug2_straight_gx = self.gx
                    self._bug2_straight_gy = self.gy
                    self._pub(0, 0, 0)
                    self.get_logger().info(f'Bug2: TURN DONE → FOLLOW')

            elif self._bug2_phase == 'straight':
                ds_check = self.d_right if self.bug_side == 1 else self.d_left
                if not self.obs_front and ds_check > 5.0 and self.d_front > 5.0:
                    self._pub(0, 0, 0)
                    nxt = S_GO_HOME if self.going_home else S_DRIVE
                    self.lane_gx = self.pre_bug_lane_gx if self.pre_bug_lane_gx is not None else self.gx
                    self.get_logger().info(
                        f'Bug2: obstacle gone (ds={ds_check:.1f} F={self.d_front:.1f}) → {nxt}')
                    self.state = nxt
                    return
                
                driven = math.hypot(self.gx - self._bug2_straight_gx,
                                    self.gy - self._bug2_straight_gy)
                ds = self.d_right if self.bug_side == 1 else self.d_left
                roughly_parallel = abs(self._adiff(self.gyaw, self.bug_start_yaw)) < math.radians(25)

                # Phase 1
                if driven < 1.5:
                    self._pub(BUG2_FOLLOW_WALL_LIN, 0, 0)

                # Phase 2
                elif not roughly_parallel or ds > BUG2_FOLLOW_TARGET + 0.3:
                    self._pub(BUG2_FOLLOW_WALL_LIN * 0.5,
                              -BUG2_TURN_ANG * 0.6 * self.bug_side,
                              -MAX_STEER * 0.5 * self.bug_side)

                # Phase 3
                else:
                    self._bug2_phase = 'follow'
                    self.get_logger().info(
                        f'Bug2: STRAIGHT→FOLLOW {driven:.1f}m ds={ds:.2f} '
                        f'yaw={math.degrees(self.gyaw):.1f}°')

                if driven > 6.0:
                    self._bug2_phase = 'follow'
                    self.get_logger().info(
                        f'Bug2: STRAIGHT MAX {driven:.1f}m ds={ds:.2f} → follow')
            
            elif self._bug2_phase == 'follow':
                e_dist = ds - BUG2_FOLLOW_TARGET
                
                if ds > 2.5 or dd > 4.0:
                    self._pub(0, 0, 0)
                    self.state = S_RETURN
                    self.get_logger().info(
                        f'Bug2: WALL END ds={ds:.2f}m dd={dd:.2f}m '
                        f'R={self.d_right:.1f}m L={self.d_left:.1f}m '
                        f'→ RETURN to M-Line g=({self.gx:.1f},{self.gy:.1f})')
                else:
                    steer_cmd = self._clamp(-e_dist * 0.5 * s, -MAX_STEER * 0.3, MAX_STEER * 0.3)
                    self._pub(BUG2_FOLLOW_WALL_LIN, steer_cmd, steer_cmd)

            if self._log_t == 0.0:
                self._bug2_log(self._bug2_phase.upper(), ds, dd, df)

        # ── BUG2 RETURN ──────────────────────────────────────────────────
        elif self.state == S_RETURN:
            if self._oob():
                self._pub(0, 0, 0)
                self.brake_timer = 0
                self.state = S_BRAKE
                self.get_logger().warning(f'Bug2 RETURN: OUT OF BOUNDS → U-turn')
                return

            if self.obs_front:
                self.bug_timer = 0
                self.bug_start_yaw = self.gyaw
                self._bug2_phase = 'turn'
                self._reset_stuck()
                if self.d_left > self.d_right + 1.0:
                    self.bug_side = 1
                elif self.d_right > self.d_left + 1.0:
                    self.bug_side = -1
                self.state = S_BUG2
                side = 'L' if self.bug_side == 1 else 'R'
                self.get_logger().info(
                    f'Bug2 RETURN: Front obstacle {self.d_front:.1f}m → BUG2 side={side} '
                    f'(keeping M-Line gx={self.hit_gx:.1f})')
                return

            dtm = abs(self.gx - self.hit_gx)
            tgy = self.gy + BUG2_RETURN_LOOKAHEAD
            dyaw = math.atan2(tgy-self.gy, self.hit_gx-self.gx)
            se = self._adiff(dyaw, self.gyaw)
            he = abs(math.degrees(self._adiff(math.pi/2, self.gyaw)))

            if dtm < BUG2_LINE_TOL:
                self._pub(0, 0, 0)
                nxt = S_GO_HOME if self.going_home else S_DRIVE
                self.lane_gx = self.pre_bug_lane_gx if self.pre_bug_lane_gx is not None else self.gx
                self.get_logger().info(f'Bug2 RETURN: M-Line reached ({dtm:.2f}m) gx={self.gx:.1f} → {nxt}')
                self.state = nxt
                return

            c = self._clamp(se*BUG2_RETURN_KP, -0.5, 0.5)
            st = self._clamp(se*BUG2_RETURN_KP, -MAX_STEER, MAX_STEER)
            self._pub(BUG2_RETURN_SPEED, c, st)

            if self._log_t == 0.0:
                self.get_logger().info(
                    f'Bug2 RET: mline={dtm:.2f}m se={math.degrees(se):.1f}° he={he:.1f}° '
                    f'g=({self.gx:.1f},{self.gy:.1f}) yaw={math.degrees(self.gyaw):.1f}°')

        # ── GO HOME ──────────────────────────────────────────────────
        elif self.state == S_GO_HOME:
            dist = math.hypot(self.gx - self.spawn_gx, self.gy - self.spawn_gy)
            dx = self.spawn_gx - self.gx
            dy = self.spawn_gy - self.gy

            if self.obs_front and self.home_phase in ('nav_to_lane', 'uturn_entry', 'drive_to_pad'):
                other = self._robot_in_front()
                if other and self._should_yield(other):
                    self._pub(0, 0, 0)
                    self.yield_to = other
                    self.pre_yield_state = S_GO_HOME
                    self.yield_timer = 0.0
                    self.state = S_YIELD
                    self.get_logger().info(
                        f'[{self.robot_name}] YIELD to {other} in GO_HOME')
                    return
                self._pub(0, 0, 0)
                self._start_bug2()
                return

            # ── Phase 1: Navigate to above the pad (correct X, Y≈-23)
            if self.home_phase == 'nav_to_lane':
                target_y = -22.5
                if self.gy > target_y + 2.0:
                    target_yaw = -math.pi / 2
                    se = self._adiff(target_yaw, self.gyaw)
                    if abs(se) > math.radians(100):
                        self._pub(0, 0, 0)
                        self.home_phase = 'uturn_entry'
                        self.home_uturn_yaw = self.gyaw
                        self.home_uturn_dir = 1 if self.d_left > self.d_right else -1
                        self.get_logger().info(
                            f'GO_HOME: facing wrong way for Y-drive → uturn_entry')
                    else:
                        c = self._clamp(se * HOME_KP, -0.4, 0.4)
                        self._pub(HOME_SPEED, c,
                                  self._clamp(se * HOME_KP, -MAX_STEER, MAX_STEER))
                else:
                    if abs(self.gx - self.spawn_gx) > 0.75:
                        target_yaw = 0.0 if self.spawn_gx > self.gx else math.pi
                        se = self._adiff(target_yaw, self.gyaw)
                        if abs(se) > math.radians(100):
                            self._pub(0, 0, 0)
                            self.home_phase = 'uturn_entry'
                            self.home_uturn_yaw = self.gyaw
                            self.home_uturn_dir = 1 if self.d_left > self.d_right else -1
                            self.get_logger().info(
                                f'GO_HOME: facing wrong way for X-drive → uturn_entry')
                        else:
                            c = self._clamp(se * HOME_KP, -0.4, 0.4)
                            self._pub(HOME_SPEED, c,
                                      self._clamp(se * HOME_KP, -MAX_STEER, MAX_STEER))
                    else:
                        self._pub(0, 0, 0)
                        self.home_phase = 'drive_to_pad'
                        self.get_logger().info(
                            f'GO_HOME: X aligned ({self.gx:.1f}) → drive_to_pad')

            # ── Phase 1b: U-turn to face correct direction
            elif self.home_phase == 'uturn_entry':
                turned = abs(self._adiff(self.gyaw, self.home_uturn_yaw))
                if turned < math.pi - 0.15:
                    d = self.home_uturn_dir
                    self._pub(TURN_FORWARD_SPD, TURN_SPEED * d, MAX_STEER * d)
                else:
                    self._pub(0, 0, 0)
                    self.boundary_grace = True
                    self.turn_complete_gy = self.gy
                    self.home_phase = 'nav_to_lane'
                    self.get_logger().info(
                        f'GO_HOME: uturn_entry done yaw={math.degrees(self.gyaw):.1f}° → nav_to_lane')

            # ── Phase 2: Drive to charging pad from ANY position/yaw
            elif self.home_phase == 'drive_to_pad':
                pad_x = self.spawn_gx
                pad_y = self.spawn_gy
                dx_pad = pad_x - self.gx
                dy_pad = pad_y - self.gy
                pad_dist = math.hypot(dx_pad, dy_pad)

                if pad_dist < 0.3:
                    self._pub(0, 0, 0)
                    self.state = S_CHARGE
                    self.get_logger().info(
                        f'GO_HOME: on pad at ({self.gx:.1f},{self.gy:.1f}) → CHARGE')
                elif pad_dist < 3.0 and abs(dx_pad) < 0.8:
                    south_yaw = -math.pi / 2
                    x_err = pad_x - self.gx
                    heading_err = self._adiff(south_yaw, self.gyaw)
                    lateral_correction = self._clamp(x_err * 0.3, -0.15, 0.15)
                    steer = self._clamp(heading_err * HOME_KP + lateral_correction,
                                        -MAX_STEER, MAX_STEER)
                    speed = HOME_SPEED * 0.4
                    self._pub(speed,
                              self._clamp(heading_err * HOME_KP + lateral_correction, -0.4, 0.4),
                              steer)
                else:
                    target_yaw = math.atan2(dy_pad, dx_pad)
                    se = self._adiff(target_yaw, self.gyaw)
                    if abs(se) > math.radians(120):
                        self._pub(0, 0, 0)
                        self.home_phase = 'uturn_entry'
                        self.home_uturn_yaw = self.gyaw
                        self.home_uturn_dir = 1 if self.d_left > self.d_right else -1
                        self.get_logger().info(
                            f'GO_HOME: drive_to_pad facing wrong way → uturn_entry')
                    else:
                        speed = HOME_SPEED * 0.6 if pad_dist < 5.0 else HOME_SPEED
                        c = self._clamp(se * HOME_KP, -0.5, 0.5)
                        self._pub(speed, c,
                                  self._clamp(se * HOME_KP, -MAX_STEER, MAX_STEER))

            # ── Phase 3: Reverse U-turn from pad → face north
            elif self.home_phase == 'reverse_uturn':
                turned = abs(self._adiff(self.gyaw, self.home_uturn_yaw))
                if turned > math.pi - 0.2:
                    self._pub(0, 0, 0)
                    self.going_home = False
                    if self.pre_home_lane_gx is not None:
                        self.lane_gx = self.pre_home_lane_gx
                    else:
                        self.lane_gx = self.spawn_gx - 2.0 if self.spawn_gx > 0 else self.spawn_gx + 2.0
                    if self.pre_home_lane_yaw is not None:
                        self.lane_yaw = self.pre_home_lane_yaw
                    else:
                        self.lane_yaw = math.radians(90.0)
                    if self.pre_home_sweep_dir is not None:
                        self.sweep_dir = self.pre_home_sweep_dir
                    self.home_phase = 'exit_garage'
                    self.get_logger().info(
                        f'GO_HOME: reverse_uturn done → exit_garage lane_gx={self.lane_gx:.1f}')
                else:
                    d = self.home_uturn_dir
                    self._pub(-0.4, 0.0, MAX_STEER * d)

            # ── Phase 4: Drive north out of garage to Y=-22
            elif self.home_phase == 'exit_garage':
                if self.gy > -22.5:
                    self._pub(0, 0, 0)
                    self.boundary_grace = True
                    self.turn_complete_gy = self.gy
                    self.state = S_DRIVE
                    self.get_logger().info(
                        f'GO_HOME: exited garage at gy={self.gy:.1f} → DRIVE lane_gx={self.lane_gx:.1f}')
                else:
                    north_yaw = math.pi / 2
                    x_err = self.lane_gx - self.gx
                    heading_err = self._adiff(north_yaw, self.gyaw)
                    lateral = self._clamp(x_err * 0.3, -0.15, 0.15)
                    steer = self._clamp(heading_err * HOME_KP + lateral,
                                        -MAX_STEER, MAX_STEER)
                    self._pub(HOME_SPEED, self._clamp(heading_err * HOME_KP + lateral, -0.4, 0.4), steer)
            
            if self._log_t == 0.0:
                self.get_logger().info(
                    f'GO_HOME[{self.home_phase}]: dx={dx:.1f} dy={dy:.1f} '
                    f'dist={dist:.1f}m BAT={self.bat_pct:.1f}% '
                    f'g=({self.gx:.1f},{self.gy:.1f}) '
                    f'F={self.d_front:.1f} yaw={math.degrees(self.gyaw):.1f}°')
                
        # ── YIELD (wait for higher-priority robot to pass) ────────
        elif self.state == S_YIELD:
            self._pub(0, 0, 0)
            self.yield_timer += dt
            cleared = False
            if self.yield_to in self.other_robots:
                rx, ry, _ = self.other_robots[self.yield_to]
                dist = math.hypot(rx - self.gx, ry - self.gy)
                angle_to = math.atan2(ry - self.gy, rx - self.gx)
                in_front = abs(self._adiff(angle_to, self.gyaw)) < math.radians(YIELD_CONE_DEG)
                if dist > YIELD_CLEAR_DIST or not in_front:
                    cleared = True
            elif not self.obs_front:
                cleared = True

            if cleared:
                nxt = self.pre_yield_state or S_DRIVE
                self.get_logger().info(
                    f'[{self.robot_name}] YIELD done ({self.yield_timer:.1f}s) → {nxt}')
                self.state = nxt
                self.yield_to = None
            elif self.yield_timer >= YIELD_TIMEOUT:
                self.get_logger().warning(
                    f'[{self.robot_name}] YIELD timeout {YIELD_TIMEOUT}s → Bug2')
                self.yield_to = None
                if self._near_oob_boundary(1.0):
                    self.brake_timer = 0
                    self.state = S_BRAKE
                else:
                    self._start_bug2()

            if self._log_t == 0.0 and self.state == S_YIELD:
                self.get_logger().info(
                    f'[{self.robot_name}] YIELD to={self.yield_to} '
                    f't={self.yield_timer:.1f}s F={self.d_front:.1f}m')

        # ── CHARGING ─────────────────────────────────────────────────
        elif self.state == S_CHARGE:
            self._pub(0, 0, 0)
            self.bat_pct = min(100.0, self.bat_pct + CHARGE_RATE_PCT * dt)
            if self._log_t == 0.0 and int(self.bat_pct) % 10 == 0:
                self.get_logger().info(
                    f'CHARGING: BAT={self.bat_pct:.1f}%')
            if self.bat_pct >= 100.0:
                self.bat_pct = 100.0
                self.home_phase = 'reverse_uturn'
                self.home_uturn_yaw = self.gyaw
                self.home_uturn_dir = 1 if self.d_left > self.d_right else -1
                self.state = S_GO_HOME
                self.get_logger().info(
                    f'FULLY CHARGED – reverse U-turn from pad')
                
        # ── DONE ─────────────────────────────────────────────────────────
        elif self.state in (S_DONE, S_EMERGENCY):
            self._pub(0, 0, 0)

    # ── Bug2 start helper ────────────────────────────────────────────────
    def _start_bug2(self):
        self.bug_timer     = 0
        self.hit_gx        = self.gx
        self.bug_start_yaw = self.gyaw
        self._reset_stuck()

        room_right = X_MAX - self.gx
        room_left  = self.gx - X_MIN
        room_up    = Y_MAX - self.gy
        room_down  = self.gy - Y_MIN
        near_boundary = min(room_left, room_right, room_up, room_down) < 3.0

        boundary_side = None
        if near_boundary:
            cy, sy = math.cos(self.gyaw), math.sin(self.gyaw)
            left_vec_x, left_vec_y   = -sy,  cy
            right_vec_x, right_vec_y =  sy, -cy
            space_left  = (room_right if left_vec_x  > 0 else room_left) * abs(left_vec_x) \
                        + (room_up    if left_vec_y  > 0 else room_down) * abs(left_vec_y)
            space_right = (room_right if right_vec_x > 0 else room_left) * abs(right_vec_x) \
                        + (room_up    if right_vec_y > 0 else room_down) * abs(right_vec_y)
            if space_left > space_right + 2.0:
                boundary_side = 1
            elif space_right > space_left + 2.0:
                boundary_side = -1

        if boundary_side is not None:
            self.bug_side = boundary_side
        elif self.going_home:
            dyaw = math.atan2(self.spawn_gy - self.gy, self.spawn_gx - self.gx)
            home_pref = 1 if self._adiff(dyaw, self.gyaw) >= 0 else -1
            if self.d_left  > self.d_right + 1.0: self.bug_side = 1
            elif self.d_right > self.d_left + 1.0: self.bug_side = -1
            else: self.bug_side = home_pref
        else:
            self.bug_side = self.avoid_side

        self.pre_bug_lane_gx = self.lane_gx
        self._bug2_phase = 'turn'
        self.state = S_BUG2
        side = 'L' if self.bug_side == 1 else 'R'
        self.get_logger().info(f'Bug2: START obs={self.d_front:.1f}m side={side} g=({self.gx:.1f},{self.gy:.1f})')

def main():
    rclpy.init()
    node = AutoDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub(0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()