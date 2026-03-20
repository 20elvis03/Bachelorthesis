#!/usr/bin/env python3
"""
Robot Setup:
  - Drive:    DiffDrive (left + right rear wheel) -> /cmd_vel
  - Steering: small_base_to_base revolute joint   -> /steering (rad)
  - Steering range: +-0.5 rad (~+-28 degrees)
  - Sensors:  3D-LiDAR /scan, Odometry -> /odom
Spawn: x=22.5, y=-22.5, facing +Y (Yaw ~+pi/2)
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


# ── Configuration ────────────────────────────────────────────────────────────
DRIVE_SPEED      = 1.5    # m/s forward
DRIVE_LANE_KP    = 2.0    # gain for lane-keeping atan2 correction in DRIVE
AVOID_SPEED      = 1    # m/s during avoidance
TURN_SPEED       = 3.0    # rad/s angular velocity during U-turn (3x faster)

OBSTACLE_FRONT   = 3.0    # m – obstacle detected → avoid
OBSTACLE_STOP    = 0.8    # m – emergency stop
WALL_TURN_DIST   = 3.0    # m – lane end detected → initiate U-turn
SIDE_MIN_DIST    = 1.2    # m – minimum side clearance for U-turn

X_MIN_BOUNDARY   = -23.5   # m – absolute boundary (odometry-based)
X_MAX_BOUNDARY   = 23.5   # m – absolute boundary (odometry-based)
Y_MIN_BOUNDARY   = -23.5   # m – absolute boundary (odometry-based)
Y_MAX_BOUNDARY   = 15.0   # m – absolute boundary (odometry-based)

# LiDAR angle cones (degrees) – 360° coverage
FRONT_CONE       = 25     # ±25° front
SIDE_CONE_START  = 55     # from 55° side
SIDE_CONE_END    = 125    # to 125° side
BACK_CONE        = 30     # ±30° rear (|angle| > 150°)

# Obstacle thresholds
OBSTACLE_BACK    = 0.6    # m – emergency stop reverse

# LiDAR 3D → 2D
LIDAR_HORIZ_IDX  = 55
LIDAR_HORIZ_TOL  = 3

# Avoidance parameters
AVOID_STEER      = 0.35   # rad – steering angle during avoidance
AVOID_ANGULAR    = 0.3    # rad/s – angular.z during avoidance

# Bug2 Wall-Following parameters
BUG2_FRONT_DIST      = 3.0    # m – threshold FRONT → TURN (conservative)
BUG2_FOLLOW_RANGE    = 2.5    # m – threshold SIDE → FOLLOW (stricter, filters room walls)
BUG2_WALL_LOST_MARGIN= 1.5    # m – wall lost when d_wall > obs_dist + MARGIN
BUG2_MIN_TURN_TIME   = 3.0    # s – minimum time in TURN before "wall lost" allowed
BUG2_MIN_FOLLOW_TIME = 30.0   # s – minimum wall-following time before M-line check
BUG2_LINE_TOL        = 0.8    # m – distance to M-Line for return (was 0.4)
BUG2_FOLLOW_WALL_LIN = 0.8    # m/s – forward speed during wall following
BUG2_FOLLOW_TARGET   = 1.5    # m – target distance to wall for P-controller
BUG2_FOLLOW_KP       = 0.5    # proportional gain: steering correction per meter deviation
BUG2_TURN_ANG        = 0.8    # rad/s – turn rate when turning away from wall
BUG2_TURN_FWD        = 0.6    # m/s – forward during turn (car needs motion to steer!)
BUG2_TIMEOUT         = 120.0  # s – fallback: abort Bug2 after this time
BUG2_STUCK_TIME      = 12.0   # s – check position after this time
BUG2_STUCK_DIST      = 0.3    # m – less than this = stuck
BUG2_SAFE_RETURN_DIST= 4.0    # m – minimum distance to nearest obstacle for safe RETURN

# Bug2 M-Line return parameters
BUG2_RETURN_SPEED    = 1.5    # m/s – forward speed during M-Line return (was 1.0)
BUG2_RETURN_YAW_TOL  = 15.0   # ° – tolerance for "yaw on course" (relaxed)
BUG2_RETURN_KP       = 1.5    # gain for yaw correction (strong)
BUG2_RETURN_LOOKAHEAD= 3.0    # m – lookahead on M-Line (smaller = faster convergence)

# LiDAR – additional cones for Bug2 Wall-Following
FRIGHT_CONE_START = 25    # ° – front-right from
FRIGHT_CONE_END   = 55    # ° – front-right to
FLEFT_CONE_START  = 25    # ° – front-left from
FLEFT_CONE_END    = 55    # ° – front-left to

# U-turn parameters
MAX_STEER        = 0.48   # rad
TURN_FORWARD_SPD = 0.8    # m/s (was 0.4 – faster turn radius)
LANE_OFFSET      = 2.0    # m – lateral offset between lanes (boustrophedon)

NUM_LANES        = 60

STUCK_CHECK_TIME = 7.0    # s – check position after this time
STUCK_DIST_M     = 0.1   # m – less than this = stuck

# Reverse turn maneuver
REVERSE_SPEED    = -0.4  # m/s reverse
REVERSE_STEER    = 0.45   # rad steering angle during reverse turn
REVERSE_YAW_DEG  = 90.0   # degrees – how far to reverse-turn

STATE_DRIVE        = 'DRIVE'
STATE_BRAKE        = 'BRAKE'
STATE_TURN_CHECK   = 'TURN_CHECK'
STATE_TURN         = 'TURN'
STATE_REVERSE_TURN = 'REVERSE_TURN'
STATE_AVOID        = 'BUG2_WALL'      # Bug2 Wall-Following
STATE_BUG2_RETURN  = 'BUG2_RETURN'   # Bug2 return to M-Line
STATE_DONE         = 'DONE'

# Bug2 Wall-Following Sub-States
BUG2_WF_TURN   = 1   # Turn away from wall (wall in front)
BUG2_WF_FOLLOW = 2   # Follow wall
BUG2_WF_CLEAR  = 3   # Corner detected → drive straight to clear object

# Bug2 corner/clear detection
BUG2_CORNER_JUMP       = 0.4    # m – d_wall_side jump per tick → corner detected
BUG2_CLEAR_FWD         = 1.0    # m/s – forward speed during CLEAR (drive past object)
BUG2_CLEAR_TIME        = 3.0    # s – how long to drive straight after corner


class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')
	
        self.cmd_pub   = self.create_publisher(Twist,   '/cmd_vel',  10)
        self.steer_pub = self.create_publisher(Float64, '/steering', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)
        self.create_subscription(TFMessage, '/world/pose_info', self.global_pose_cb, 10)

        self.state            = STATE_DRIVE
        self.lane_idx         = 0
        self._turn_dir        = 1
        self.avoid_side       = 1
        self.brake_timer      = 0.0
        self.turn_check_timer = 0.0

        self.yaw              = 0.0
        self.yaw_start_turn   = 0.0
        self.lane_yaw         = None   # heading of current lane
        self.lane_y           = None   # target Y of current lane (avoidance return)
        self.lane_gx          = None   # target global X of current lane (lane-keeping)
        self.boundary_grace   = False  # skip boundary check after U-turn until clear
        self.turn_complete_gy = 0.0    # global Y where last turn completed

        self.pos_x            = 0.0
        self.pos_y            = 0.0
        self.odom_ready       = False

        # Battery simulation
        self.battery_pct      = 100.0  # percent remaining
        self.battery_prev_gx  = 0.0    # previous global position for distance calc
        self.battery_prev_gy  = 0.0
        self.battery_init     = False  # first position captured?

        self.global_x         = 0.0
        self.global_y         = 0.0
        self.global_yaw       = 0.0

        # Stuck detection: remember position at start of turn
        self.stuck_check_timer  = 0.0
        self.stuck_ref_x        = 0.0
        self.stuck_ref_y        = 0.0
        self.stuck_checked      = False   # already checked in this TURN?

        # Reverse turn state
        self.reverse_yaw_start  = 0.0
        self.reverse_dir        = 1      # +1=left, -1=right

        self.dist_front       = 99.0
        self.dist_left        = 99.0
        self.dist_right       = 99.0
        self.dist_back        = 99.0
        self.dist_fright      = 99.0   # front-right für Bug2 Wall-Following
        self.dist_fleft       = 99.0   # front-left für Bug2 Wall-Following
        self.obstacle_front   = False
        self.obstacle_stop    = False
        self.obstacle_back    = False
        self.wall_near        = False

        # Bug2 Wall-Following state
        self.bug2_wf_state      = BUG2_WF_TURN  # sub-state of wall follower
        self.bug2_timer         = 0.0            # time in wall-following
        self.bug2_follow_side   = 1              # +1 = wall right (dodge left)
        self.bug2_hit_x         = 0.0            # position where obstacle was hit
        self.bug2_hit_y         = 0.0
        self.bug2_obs_dist      = 2.0            # measured obstacle distance at entry
        self.bug2_start_yaw     = 0.0            # yaw at Bug2 start (for debug)
        self.bug2_prev_d_wall   = 99.0           # d_wall_side from last tick (corner detection)
        self.bug2_clear_timer   = 0.0            # time in CLEAR state
        self.bug2_hit_gx        = 0.0            # GLOBAL position where obstacle was hit
        self.bug2_hit_gy        = 0.0
        self.bug2_stuck_timer   = 0.0            # stuck detection in Bug2
        self.bug2_stuck_ref_x   = 0.0
        self.bug2_stuck_ref_y   = 0.0
        self.bug2_stuck_checked = False

        self._log_timer       = 0.0

        self.create_timer(0.05, lambda: self.loop(0.05))
        self.get_logger().info('AutoDrive ready – waiting for sensors...')

    # ── LiDAR ────────────────────────────────────────────────────────────────
    def scan_cb(self, msg: LaserScan):
        n_horiz   = 980
        n_vert    = 32
        total_pts = len(msg.ranges)

        front_min  = 99.0
        left_min   = 99.0
        right_min  = 99.0
        back_min   = 99.0
        fright_min = 99.0
        fleft_min  = 99.0

        front_cone    = math.radians(FRONT_CONE)
        side_start    = math.radians(SIDE_CONE_START)
        side_end      = math.radians(SIDE_CONE_END)
        back_thresh   = math.radians(180 - BACK_CONE)
        fright_start  = math.radians(FRIGHT_CONE_START)
        fright_end    = math.radians(FRIGHT_CONE_END)
        fleft_start   = math.radians(FLEFT_CONE_START)
        fleft_end     = math.radians(FLEFT_CONE_END)

        def classify(r, angle):
            nonlocal front_min, left_min, right_min, back_min, fright_min, fleft_min
            if abs(angle) < front_cone:
                front_min  = min(front_min, r)
            elif fleft_start < angle < fleft_end:
                fleft_min  = min(fleft_min, r)
            elif -fright_end < angle < -fright_start:
                fright_min = min(fright_min, r)
            elif side_start < angle < side_end:
                left_min   = min(left_min,  r)
            elif -side_end < angle < -side_start:
                right_min  = min(right_min, r)
            elif abs(angle) > back_thresh:
                back_min   = min(back_min,  r)

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

        self.dist_front  = front_min
        self.dist_left   = left_min
        self.dist_right  = right_min
        self.dist_back   = back_min
        self.dist_fright = fright_min
        self.dist_fleft  = fleft_min

        self.obstacle_stop  = front_min < OBSTACLE_STOP
        self.obstacle_front = front_min < OBSTACLE_FRONT
        self.obstacle_back  = back_min  < OBSTACLE_BACK
        self.wall_near      = front_min < WALL_TURN_DIST

        if left_min > right_min + 0.5:
            self.avoid_side = 1
        elif right_min > left_min + 0.5:
            self.avoid_side = -1

    # ── Odometry ──────────────────────────────────────────────────────────────
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
                f'front={self.dist_front:.1f}m')

    # ── Global Pose ──────────────────────────────────────────────────────────
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

    # ── Main Loop ────────────────────────────────────────────────────────────
    def loop(self, dt: float):
        if not self.odom_ready:
            return

        # ── Battery simulation ───────────────────────────────────────────
        if not self.battery_init:
            self.battery_prev_gx = self.global_x
            self.battery_prev_gy = self.global_y
            self.battery_init = True
        else:
            dx = self.global_x - self.battery_prev_gx
            dy = self.global_y - self.battery_prev_gy
            dist_moved = math.hypot(dx, dy)
            if dist_moved > 0.05:  # ignore tiny jitter
                self.battery_pct -= dist_moved * 0.5  # 0.5% per meter
                self.battery_prev_gx = self.global_x
                self.battery_prev_gy = self.global_y

        if self.battery_pct <= 0.0:
            self.battery_pct = 0.0
            self._publish(0.0, 0.0, 0.0)
            if self.state != STATE_DONE:
                self.state = STATE_DONE
                self.get_logger().warning('BATTERY DEAD (0%) – robot stopped!')
            return

        # Periodic 360° sensor log every second
        self._log_timer += dt
        if self._log_timer >= 1.0:
            self._log_timer = 0.0
            self.get_logger().info(
                f'[SCAN] V={self.dist_front:.1f}m  FL={self.dist_fleft:.1f}m  '
                f'FR={self.dist_fright:.1f}m  L={self.dist_left:.1f}m  '
                f'R={self.dist_right:.1f}m  H={self.dist_back:.1f}m  '
                f'| state={self.state}  BAT={self.battery_pct:.1f}%  '
                f'x={self.pos_x:.1f} y={self.pos_y:.1f}, '
                f'x_global={self.global_x:.1f} y_global={self.global_y:.1f}')

        # Emergency stop front (except turn states)
        if self.obstacle_stop and self.state not in (
                STATE_TURN, STATE_TURN_CHECK, STATE_REVERSE_TURN):
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().warning(
                f'EMERGENCY STOP FRONT! {self.dist_front:.2f}m',
                throttle_duration_sec=1.0)
            return

        # Emergency stop rear only during reverse driving
        if self.obstacle_back and self.state == STATE_REVERSE_TURN:
            self._publish(0.0, 0.0, 0.0)
            self.get_logger().warning(
                f'EMERGENCY STOP REAR! {self.dist_back:.2f}m – aborting reverse')
            # Still retry turn (forward)
            self.yaw_start_turn    = self.yaw
            self.stuck_check_timer = 0.0
            self.stuck_checked     = False
            self.stuck_ref_x       = self.pos_x
            self.stuck_ref_y       = self.pos_y
            self.state = STATE_TURN
            return

        # ── State Machine ─────────────────────────────────────────────────
        out_of_bounds = (
            self.global_x <= X_MIN_BOUNDARY or
            self.global_x >= X_MAX_BOUNDARY or
            self.global_y <= Y_MIN_BOUNDARY or
            self.global_y >= Y_MAX_BOUNDARY
        )
        if self.state == STATE_DRIVE:
            # Save lane yaw once on first step
            if self.lane_yaw is None:
                self.lane_yaw = self.yaw
                self.lane_y   = self.pos_y
                self.lane_gx  = self.global_x
                self.get_logger().info(
                    f'Lane yaw set: {math.degrees(self.lane_yaw):.1f}° lane_gx={self.lane_gx:.1f}')
                
            # Clear boundary grace once robot has moved 3m from where the turn ended
            if self.boundary_grace:
                dist_from_turn = abs(self.global_y - self.turn_complete_gy)
                if dist_from_turn > 3.0:
                    self.boundary_grace = False

            # Boundary exceeded → end lane (but NOT if still in grace period after U-turn)
            if out_of_bounds and not self.boundary_grace:
                self._publish(0.0, 0.0, 0.0)
                self.brake_timer = 0.0
                self.state = STATE_BRAKE
                self.get_logger().info(
                    f'Boundary reached (y={self.global_y:.2f}m) – U-turn')
                return

            # Boundary exceeded → end lane like at wall (alternative)
            #if out_of_bounds and self.wall_near:
                #self._publish(0.0, 0.0, 0.0)
                #self.brake_timer = 0.0
                #self.state = STATE_BRAKE
                #self.get_logger().info(
                #f'Lane end at boundary (x={self.global_x:.1f}, y={self.global_y:.1f}) '
                #f'Wall at {self.dist_front:.1f}m – braking (lane {self.lane_idx+1})')
                #return

            # Wall / lane end via LiDAR
            #if self.wall_near:
                #self._publish(0.0, 0.0, 0.0)
                #self.brake_timer = 0.0
                #self.state = STATE_BRAKE
                #self.get_logger().info(
                #    f'Wall at {self.dist_front:.1f}m – braking (lane {self.lane_idx+1})')
                #return

            # Obstacle (not lane end) → start Bug2 Wall-Following
            if self.obstacle_front:
                self._publish(0.0, 0.0, 0.0)
                self.bug2_timer         = 0.0
                self.bug2_hit_x         = self.pos_x
                self.bug2_hit_y         = self.pos_y
                self.bug2_obs_dist      = self.dist_front  # remember obstacle distance
                self.bug2_start_yaw     = self.yaw         # remember yaw at start
                self.bug2_prev_d_wall   = self.dist_front  # for corner detection
                self.bug2_clear_timer   = 0.0
                self.bug2_hit_gx        = self.global_x    # save GLOBAL hit position
                self.bug2_hit_gy        = self.global_y
                self.bug2_stuck_timer   = 0.0
                self.bug2_stuck_ref_x   = self.pos_x
                self.bug2_stuck_ref_y   = self.pos_y
                self.bug2_stuck_checked = False
                self.lane_y             = self.pos_y
                self.bug2_follow_side   = self.avoid_side
                self.bug2_wf_state      = BUG2_WF_TURN
                self.state = STATE_AVOID
                side = 'left' if self.bug2_follow_side == 1 else 'right'
                wall_thresh = self.dist_front + BUG2_WALL_LOST_MARGIN
                self.get_logger().info(
                    f'Bug2: START obstacle {self.dist_front:.1f}m → Wall-Following {side} '
                    f'(lane_yaw={math.degrees(self.lane_yaw):.1f}°) '
                    f'| obs_dist={self.bug2_obs_dist:.1f}m '
                    f'wall_lost_thresh={wall_thresh:.1f}m '
                    f'| V={self.dist_front:.1f}m FL={self.dist_fleft:.1f}m '
                    f'FR={self.dist_fright:.1f}m L={self.dist_left:.1f}m '
                    f'R={self.dist_right:.1f}m '
                    f'| pos=({self.pos_x:.1f},{self.pos_y:.1f}) '
                    f'global=({self.global_x:.1f},{self.global_y:.1f})')
                return

            # Lane-keeping: steer toward a point on lane_gx, 5m ahead in current heading
            # Uses atan2 so the sign is always correct regardless of heading direction
            if self.lane_gx is not None:
                target_gx = self.lane_gx
                target_gy = self.global_y + 5.0 * math.sin(self.global_yaw)
                desired_yaw = math.atan2(
                    target_gy - self.global_y,
                    target_gx - self.global_x
                )
                steer_err = self._angle_diff(desired_yaw, self.global_yaw)
                lane_corr = max(-0.4, min(0.4, steer_err * DRIVE_LANE_KP))
                lane_steer = max(-MAX_STEER * 0.5, min(MAX_STEER * 0.5, steer_err * DRIVE_LANE_KP))
                self._publish(DRIVE_SPEED, lane_corr, lane_steer)
            else:
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
                side = 'left' if self._turn_dir == 1 else 'right'
                self.get_logger().info(
                    f'U-turn {side} (L={self.dist_left:.1f}m R={self.dist_right:.1f}m)')
            elif self.turn_check_timer > 5.0:
                self.get_logger().warning('No turn space – reversing')
                self._publish(-0.15, 0.0, 0.0)

        elif self.state == STATE_TURN:
            turned = abs(self._angle_diff(self.yaw, self.yaw_start_turn))

            # Stuck detection: after STUCK_CHECK_TIME seconds check if
            # the robot has actually moved
            self.stuck_check_timer += dt
            if not self.stuck_checked and self.stuck_check_timer >= STUCK_CHECK_TIME:
                self.stuck_checked = True
                dist_moved = math.hypot(
                    self.pos_x - self.stuck_ref_x,
                    self.pos_y - self.stuck_ref_y
                )
                if dist_moved < STUCK_DIST_M:
                    # Stuck – initiate reverse turn maneuver
                    self.get_logger().warning(
                        f'STUCK detected (moved={dist_moved:.2f}m in {STUCK_CHECK_TIME}s) '
                        f'– reverse + 90 degree turn')
                    self.reverse_yaw_start = self.yaw
                    # Reverse direction opposite to turn side
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
                # Turn complete – save new lane yaw, offset lane_gx for next lane
                self._publish(0.0, 0.0, 0.0)
                self.lane_yaw = self.yaw
                self.lane_y   = self.pos_y   # new lane Y after turn
                self.boundary_grace = True
                self.turn_complete_gy = self.global_y
                # Offset lane_gx: robot always sweeps in -X global direction
                # (from right room wall toward left room wall)
                old_gx = self.lane_gx if self.lane_gx is not None else self.global_x
                self.lane_gx = old_gx - LANE_OFFSET
                self.lane_idx += 1
                if self.lane_idx >= NUM_LANES:
                    self.state = STATE_DONE
                    self.get_logger().info('Done – all lanes completed!')
                else:
                    self.state = STATE_DRIVE
                    self.get_logger().info(
                        f'Lane {self.lane_idx+1} – Yaw={math.degrees(self.lane_yaw):.1f}° '
                        f'lane_gx={self.lane_gx:.1f} (offset from {old_gx:.1f})')

        elif self.state == STATE_REVERSE_TURN:
            # Reverse drive while turning 90 degrees
            turned_back = abs(self._angle_diff(self.yaw, self.reverse_yaw_start))
            target_rad  = math.radians(REVERSE_YAW_DEG)

            if turned_back < target_rad - 0.08:
                self._publish(
                    REVERSE_SPEED,
                    TURN_SPEED * self.reverse_dir,
                    REVERSE_STEER * self.reverse_dir
                )
            else:
                # 90 degrees reached – restart normal turn attempt
                self._publish(0.0, 0.0, 0.0)
                self.get_logger().info(
                    f'Reverse turn complete – restarting turn')
                # Reset turn parameters for new attempt
                self.yaw_start_turn    = self.yaw
                self.stuck_check_timer = 0.0
                self.stuck_checked     = False
                self.stuck_ref_x       = self.pos_x
                self.stuck_ref_y       = self.pos_y
                self.state = STATE_TURN

        elif self.state == STATE_AVOID:
            # ── Bug2 Wall-Following ──────────────────────────────────────
            self.bug2_timer += dt
            s = self.bug2_follow_side   # +1=dodge left (wall right), -1=dodge right

            # Sensor regions (mirrored depending on follow side)
            # d_wall_diag = front-diagonal (25-55°) → sees front face at an angle
            # d_wall_side = true side (55-125°) → sees actual side wall
            if s == 1:
                d_wall_diag = self.dist_fright
                d_wall_side = self.dist_right
            else:
                d_wall_diag = self.dist_fleft
                d_wall_side = self.dist_left

            d_wall = min(d_wall_diag, d_wall_side)
            d_front = self.dist_front
            d_nearest = min(d_front, self.dist_fright, self.dist_fleft,
                           self.dist_right, self.dist_left)

            # ── Dynamic thresholds ───────────────────────────────────────
            follow_range     = self.bug2_obs_dist + 1.0
            wall_lost_thresh = self.bug2_obs_dist + BUG2_WALL_LOST_MARGIN

            # ── Corner detection ─────────────────────────────────────────
            # Corner = d_wall_side suddenly jumps (object edge passed)
            # → Don't turn toward wall! Instead: drive STRAIGHT to clear object
            d_side_jump = d_wall_side - self.bug2_prev_d_wall
            corner_detected = (
                d_side_jump > BUG2_CORNER_JUMP and
                self.bug2_prev_d_wall < follow_range and
                self.bug2_wf_state == BUG2_WF_FOLLOW and
                d_front > BUG2_FRONT_DIST and
                self.bug2_timer > 3.0
            )

            # ── Wall-Following sub-state machine ─────────────────────────
            prev_wf_state = self.bug2_wf_state

            # --- Priority 1: Corner detected → drive straight to clear object
            if corner_detected:
                self.bug2_wf_state = BUG2_WF_CLEAR
                self.bug2_clear_timer = 0.0
                self.get_logger().info(
                    f'Bug2 WF: *** CORNER → CLEAR *** '
                    f'd_wall_side jumped {self.bug2_prev_d_wall:.2f}→{d_wall_side:.2f}m '
                    f'(+{d_side_jump:.2f}m) → driving straight to clear object')

            # --- Priority 2: Already in CLEAR → drive straight, then RETURN
            elif self.bug2_wf_state == BUG2_WF_CLEAR:
                self.bug2_clear_timer += dt
                if d_front < BUG2_FRONT_DIST or self.obstacle_front:
                    # Front blocked during CLEAR → must TURN
                    self.bug2_wf_state = BUG2_WF_TURN
                    self.get_logger().info(
                        f'Bug2 WF: *** CLEAR → TURN *** '
                        f'Front blocked {d_front:.1f}m during clear')
                elif d_wall_side < follow_range and self.bug2_clear_timer > 1.0:
                    # Found next wall face after some clearing → FOLLOW
                    self.bug2_wf_state = BUG2_WF_FOLLOW
                    self.get_logger().info(
                        f'Bug2 WF: *** CLEAR → FOLLOW *** '
                        f'Next wall found d_side={d_wall_side:.2f}m after '
                        f'{self.bug2_clear_timer:.1f}s')
                elif self.bug2_clear_timer > BUG2_CLEAR_TIME:
                    # Cleared the object → go directly to RETURN
                    self._publish(0.0, 0.0, 0.0)
                    self.state = STATE_BUG2_RETURN
                    self.bug2_prev_d_wall = d_wall_side
                    self.get_logger().info(
                        f'Bug2: CLEAR complete ({self.bug2_clear_timer:.1f}s) '
                        f'd_nearest={d_nearest:.1f}m → RETURN to M-Line '
                        f'(global target=({self.bug2_hit_gx:.1f},{self.bug2_hit_gy:.1f}))')
                    return

            # --- Priority 3: Front or diagonal blocked → TURN
            elif (d_front < BUG2_FRONT_DIST or self.obstacle_front or
                  d_wall_diag < follow_range):
                self.bug2_wf_state = BUG2_WF_TURN
                if d_front < self.bug2_obs_dist:
                    self.bug2_obs_dist = d_front
                    follow_range = self.bug2_obs_dist + 1.0
                    wall_lost_thresh = self.bug2_obs_dist + BUG2_WALL_LOST_MARGIN
                if d_wall_diag < self.bug2_obs_dist:
                    self.bug2_obs_dist = d_wall_diag
                    follow_range = self.bug2_obs_dist + 1.0
                    wall_lost_thresh = self.bug2_obs_dist + BUG2_WALL_LOST_MARGIN

            # --- Priority 4: Side sensor sees wall → FOLLOW
            elif d_wall_side < follow_range:
                self.bug2_wf_state = BUG2_WF_FOLLOW
                if d_wall_side < self.bug2_obs_dist:
                    self.bug2_obs_dist = d_wall_side
                    follow_range = self.bug2_obs_dist + 1.0
                    wall_lost_thresh = self.bug2_obs_dist + BUG2_WALL_LOST_MARGIN

            # --- Priority 5: Side in transition zone → FOLLOW
            elif d_wall_side < wall_lost_thresh:
                self.bug2_wf_state = BUG2_WF_FOLLOW
                if self._log_timer == 0.0:
                    self.get_logger().info(
                        f'Bug2 WF: Side in transition → FOLLOW '
                        f'd_side={d_wall_side:.2f}m (follow={follow_range:.1f}m '
                        f'lost={wall_lost_thresh:.1f}m)')

            # --- Priority 6: Everything far → safety check before RETURN
            else:
                safe_to_return = True
                block_reason = ""
                if self.bug2_timer < BUG2_MIN_TURN_TIME:
                    safe_to_return = False
                    block_reason = f"too early (t={self.bug2_timer:.1f}s)"
                if d_nearest < BUG2_SAFE_RETURN_DIST:
                    safe_to_return = False
                    block_reason = (f"obstacle near d_nearest={d_nearest:.2f}m")

                if safe_to_return:
                    self._publish(0.0, 0.0, 0.0)
                    self.state = STATE_BUG2_RETURN
                    self.bug2_prev_d_wall = d_wall_side
                    self.get_logger().info(
                        f'Bug2: Wall lost (d_side={d_wall_side:.1f}m > '
                        f'lost={wall_lost_thresh:.1f}m) '
                        f'after {self.bug2_timer:.1f}s → RETURN')
                    return
                else:
                    self.bug2_wf_state = BUG2_WF_TURN
                    if self._log_timer == 0.0:
                        self.get_logger().warning(
                            f'Bug2 WF: RETURN BLOCKED: {block_reason} → keep TURN')

            # Log on sub-state change
            if self.bug2_wf_state != prev_wf_state:
                wf_names = {BUG2_WF_TURN: 'TURN', BUG2_WF_FOLLOW: 'FOLLOW',
                            BUG2_WF_CLEAR: 'CLEAR'}
                self.get_logger().info(
                    f'Bug2 WF: *** SWITCH {wf_names.get(prev_wf_state, "?")} → '
                    f'{wf_names.get(self.bug2_wf_state, "?")} *** '
                    f'(front={d_front:.1f}m diag={d_wall_diag:.1f}m '
                    f'side={d_wall_side:.1f}m '
                    f'follow={follow_range:.1f}m obs_ref={self.bug2_obs_dist:.1f}m)')

            # Save d_wall_side for next tick (corner detection)
            self.bug2_prev_d_wall = d_wall_side

            # ── Compute velocities ───────────────────────────────────────
            if self.bug2_wf_state == BUG2_WF_TURN:
                # Turn AWAY from wall
                self._publish(
                    BUG2_TURN_FWD,
                    BUG2_TURN_ANG * s,
                    MAX_STEER * s
                )
            elif self.bug2_wf_state == BUG2_WF_CLEAR:
                # Drive STRAIGHT to clear the object (no turning!)
                self._publish(BUG2_CLEAR_FWD, 0.0, 0.0)
            elif self.bug2_wf_state == BUG2_WF_FOLLOW:
                # P-controller: maintain target distance to side wall
                error = BUG2_FOLLOW_TARGET - d_wall_side
                correction = max(-0.5, min(0.5, error * BUG2_FOLLOW_KP * s))
                steer_corr = max(-MAX_STEER, min(MAX_STEER, error * BUG2_FOLLOW_KP * s))
                self._publish(
                    BUG2_FOLLOW_WALL_LIN,
                    correction,
                    steer_corr
                )

            # ── M-Line reached during Follow? ────────────────────────────
            dist_to_line = self._distance_to_lane_line()
            if self.bug2_timer > BUG2_MIN_FOLLOW_TIME and \
               dist_to_line < BUG2_LINE_TOL and \
               not self.obstacle_front:
                self._publish(0.0, 0.0, 0.0)
                self.get_logger().info(
                    f'Bug2: M-Line reached (dist={dist_to_line:.2f}m, '
                    f't={self.bug2_timer:.1f}s) → DRIVE')
                self.state = STATE_DRIVE
                return

            # ── Bug2 Timeout ─────────────────────────────────────────────
            if self.bug2_timer > BUG2_TIMEOUT:
                self._publish(0.0, 0.0, 0.0)
                self.get_logger().warning(
                    f'Bug2: TIMEOUT after {self.bug2_timer:.1f}s → DRIVE')
                self.state = STATE_DRIVE
                return

            # ── Bug2 Stuck detection ─────────────────────────────────────
            self.bug2_stuck_timer += dt
            if not self.bug2_stuck_checked and \
               self.bug2_stuck_timer >= BUG2_STUCK_TIME:
                self.bug2_stuck_checked = True
                dist_moved = math.hypot(
                    self.pos_x - self.bug2_stuck_ref_x,
                    self.pos_y - self.bug2_stuck_ref_y
                )
                if dist_moved < BUG2_STUCK_DIST:
                    self.bug2_follow_side = -self.bug2_follow_side
                    self.bug2_wf_state    = BUG2_WF_TURN
                    self.bug2_stuck_timer   = 0.0
                    self.bug2_stuck_ref_x   = self.pos_x
                    self.bug2_stuck_ref_y   = self.pos_y
                    self.bug2_stuck_checked = False
                    side = 'left' if self.bug2_follow_side == 1 else 'right'
                    self.get_logger().warning(
                        f'Bug2: STUCK ({dist_moved:.2f}m in {BUG2_STUCK_TIME}s) '
                        f'→ switch side to {side}')
                else:
                    self.bug2_stuck_timer   = 0.0
                    self.bug2_stuck_ref_x   = self.pos_x
                    self.bug2_stuck_ref_y   = self.pos_y
                    self.bug2_stuck_checked = False

            # Periodic debug log
            if self._log_timer == 0.0:
                wf_names = {BUG2_WF_TURN: 'TURN', BUG2_WF_FOLLOW: 'FOLLOW',
                            BUG2_WF_CLEAR: 'CLEAR'}
                side_name = 'right' if s == 1 else 'left'
                yaw_turned = math.degrees(abs(self._angle_diff(self.yaw, self.bug2_start_yaw)))
                clear_info = (f' clear_t={self.bug2_clear_timer:.1f}s'
                              if self.bug2_wf_state == BUG2_WF_CLEAR else '')
                self.get_logger().info(
                    f'Bug2 WF: {wf_names.get(self.bug2_wf_state, "?")} '
                    f'wall_side={side_name} turned={yaw_turned:.0f}° '
                    f'dist_line={self._distance_to_lane_line():.2f}m t={self.bug2_timer:.1f}s '
                    f'| V={d_front:.1f}m FL={self.dist_fleft:.1f}m '
                    f'FR={self.dist_fright:.1f}m L={self.dist_left:.1f}m '
                    f'R={self.dist_right:.1f}m '
                    f'| diag={d_wall_diag:.2f}m side={d_wall_side:.2f}m '
                    f'd_nearest={d_nearest:.2f}m '
                    f'| obs_ref={self.bug2_obs_dist:.1f}m '
                    f'follow={follow_range:.1f}m '
                    f'lost={wall_lost_thresh:.1f}m{clear_info} '
                    f'| global=({self.global_x:.1f},{self.global_y:.1f}) '
                    f'global_yaw={math.degrees(self.global_yaw):.1f}°')

        elif self.state == STATE_BUG2_RETURN:
            # ── Return to M-Line using GLOBAL coordinates ────────────────
            # Navigate toward the global hit point (where Bug2 started)
            # This avoids odometry drift issues with yaw
            d_nearest_return = min(self.dist_front, self.dist_fright, self.dist_fleft,
                                   self.dist_right, self.dist_left)

            # Only re-enter wall-following if something blocks the FRONT path
            # Side walls (room boundaries) are NOT obstacles during return!
            if self.obstacle_front:
                self.bug2_wf_state      = BUG2_WF_TURN
                self.bug2_obs_dist      = self.dist_front
                self.bug2_stuck_timer   = 0.0
                self.bug2_stuck_ref_x   = self.pos_x
                self.bug2_stuck_ref_y   = self.pos_y
                self.bug2_stuck_checked = False
                self.state = STATE_AVOID
                self.get_logger().info(
                    f'Bug2 RETURN: Front obstacle! front={self.dist_front:.1f}m '
                    f'→ back to wall-following')
                return

            # Compute M-Line distance using GLOBAL coordinates
            dist_to_mline = self._distance_to_mline_global()

            # Target: a point FAR ahead on the M-Line
            # Large lookahead = gentle diagonal approach instead of hard turn
            # M-Line is at x=hit_gx, going in +Y direction
            target_gx = self.bug2_hit_gx
            target_gy = self.global_y + BUG2_RETURN_LOOKAHEAD  # always 10m ahead

            desired_yaw_g = math.atan2(
                target_gy - self.global_y,
                target_gx - self.global_x
            )
            steer_err = self._angle_diff(desired_yaw_g, self.global_yaw)
            steer_err_deg = abs(math.degrees(steer_err))

            # Goal reached: close to M-Line and heading roughly in +Y
            # Check heading toward +Y (90°) instead of toward target
            heading_err_deg = abs(math.degrees(
                self._angle_diff(math.radians(90.0), self.global_yaw)))
            if dist_to_mline < BUG2_LINE_TOL and heading_err_deg < BUG2_RETURN_YAW_TOL:
                self._publish(0.0, 0.0, 0.0)
                self.lane_gx = self.global_x  # update lane X after Bug2
                self.get_logger().info(
                    f'Bug2 RETURN: M-Line reached '
                    f'(dist={dist_to_mline:.2f}m, heading_err={heading_err_deg:.1f}°) '
                    f'lane_gx={self.lane_gx:.1f} → DRIVE')
                self.state = STATE_DRIVE
                return

            # P-controller: strong KP, full steer range
            corr  = max(-0.5, min(0.5, steer_err * BUG2_RETURN_KP))
            steer = max(-MAX_STEER, min(MAX_STEER, steer_err * BUG2_RETURN_KP))
            self._publish(BUG2_RETURN_SPEED, corr, steer)

            # Periodic log
            if self._log_timer == 0.0:
                self.get_logger().info(
                    f'Bug2 RETURN: dist_mline={dist_to_mline:.2f}m '
                    f'steer_err={math.degrees(steer_err):.1f}° '
                    f'heading_err={heading_err_deg:.1f}° '
                    f'corr={corr:.2f} steer={steer:.2f} '
                    f'| V={self.dist_front:.1f}m d_nearest={d_nearest_return:.1f}m '
                    f'| target_g=({target_gx:.1f},{target_gy:.1f}) '
                    f'pos_g=({self.global_x:.1f},{self.global_y:.1f}) '
                    f'| global_yaw={math.degrees(self.global_yaw):.1f}°')

        elif self.state == STATE_DONE:
            self._publish(0.0, 0.0, 0.0)

    # ── Helper functions ─────────────────────────────────────────────────────
    def _distance_to_lane_line(self):
        """Perpendicular distance from current position to lane line (M-Line).
        The line passes through (bug2_hit_x, bug2_hit_y) in direction lane_yaw."""
        if self.lane_yaw is None:
            return 99.0
        dx = self.pos_x - self.bug2_hit_x
        dy = self.pos_y - self.bug2_hit_y
        return abs(math.sin(self.lane_yaw) * dx - math.cos(self.lane_yaw) * dy)

    def _distance_to_mline_global(self):
        """Perpendicular distance from current GLOBAL position to M-Line.
        M-Line passes through (bug2_hit_gx, bug2_hit_gy) in +Y direction (global).
        Since the robot drives in +Y global, M-Line is a vertical line at x=hit_gx."""
        return abs(self.global_x - self.bug2_hit_gx)

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
    node = AutoDrive()
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
