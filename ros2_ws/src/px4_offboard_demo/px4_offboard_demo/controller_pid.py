import math
import os
from datetime import datetime
from typing import Optional, List, Tuple
import yaml
from pathlib import Path as FsPath
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path as PathMsg
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, String
from rcl_interfaces.msg import SetParametersResult
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class PID:
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)
        self.integral = 0.0
        self.prev_error = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        p = self.kp * error
        self.integral += error * dt
        # anti-windup
        self.integral = max(min(self.integral, self.i_limit), -self.i_limit)
        i = self.ki * self.integral
        d_term = 0.0
        if self.prev_error is not None:
            d = (error - self.prev_error) / dt
            d_term = self.kd * d
        self.prev_error = error
        return p + i + d_term


class OffboardPIDController(Node):
    def __init__(self):
        super().__init__('offboard_pid_controller')

        # Parameters
        self.declare_parameter('mode', 'hover')  # hover | circle | waypoints | zigzag_repeat
        self.declare_parameter('target_alt', 2.5)
        self.declare_parameter('circle_radius', 2.0)
        self.declare_parameter('circle_period', 10.0)
        try:
            _share = FsPath(get_package_share_directory('px4_offboard_demo'))
            _default_wp = str(_share / 'config' / 'waypoints_demo.yaml')
        except Exception:
            _default_wp = str((FsPath(__file__).resolve().parent.parent / 'config' / 'waypoints_demo.yaml'))
        self.declare_parameter('waypoints_file', _default_wp)
        self.declare_parameter('wp_accept_radius', 0.5)
        self.declare_parameter('wp_hold_time', 2.0)
        self.declare_parameter('wp_loop', True)

        self.declare_parameter('pid_xy.kp', 0.8)
        self.declare_parameter('pid_xy.ki', 0.0)
        self.declare_parameter('pid_xy.kd', 0.1)
        self.declare_parameter('pid_xy.i_limit', 1.0)

        self.declare_parameter('pid_z.kp', 1.0)
        self.declare_parameter('pid_z.ki', 0.0)
        self.declare_parameter('pid_z.kd', 0.1)
        self.declare_parameter('pid_z.i_limit', 1.0)

        self.declare_parameter('limit.v_xy', 2.0)
        self.declare_parameter('limit.v_z', 1.0)
        self.declare_parameter('limit.dv_xy', 1.0)
        self.declare_parameter('limit.dv_z', 1.0)

        # Zigzag/learning params
        self.declare_parameter('zigzag.length_m', 20.0)
        self.declare_parameter('zigzag.width_m', 4.0)
        self.declare_parameter('zigzag.dx', 2.0)  # spacing along X between turns
        self.declare_parameter('zigzag.alt', 2.5)
        self.declare_parameter('zigzag.loops', 0)  # 0 = infinite
        self.declare_parameter('zigzag.end_action', 'hover')  # hover | land | hold
        self.declare_parameter('adapt.enable', True)
        self.declare_parameter('adapt.avg_err_target', 1.0)
        self.declare_parameter('adapt.margin', 0.2)
        self.declare_parameter('adapt.kp_step', 0.05)
        self.declare_parameter('adapt.v_step', 0.2)
        self.declare_parameter('adapt.v_min', 0.8)
        self.declare_parameter('adapt.v_max', 4.0)
        # Metrics/CSV configuration
        self.declare_parameter('metrics.root', os.path.join(os.getcwd(), 'log', 'zigzag'))
        self.declare_parameter('metrics.file', '')  # if empty, a session folder is created under metrics.root

        self.declare_parameter('geofence.x_min', -20.0)
        self.declare_parameter('geofence.x_max', 20.0)
        self.declare_parameter('geofence.y_min', -20.0)
        self.declare_parameter('geofence.y_max', 20.0)
        self.declare_parameter('geofence.z_min', 0.0)
        self.declare_parameter('geofence.z_max', 20.0)

        # QoS profile for MAVROS sensor topics
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )
        self.current_path_pub = self.create_publisher(PathMsg, '/offboard_demo/current_path', 10)
        self.target_path_pub = self.create_publisher(PathMsg, '/offboard_demo/target_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/offboard_demo/marker', 10)
        self.avg_err_pub = self.create_publisher(Float32, '/offboard_demo/avg_err_5s', 10)
        self.episode_pub = self.create_publisher(String, '/offboard_demo/episode_summary', 10)

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self._state_cb, 10
        )
        self.local_pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, sensor_qos
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Internal state
        self.current_state = State()
        self.current_pose: Optional[PoseStamped] = None
        self.home_pose: Optional[PoseStamped] = None
        self.stage = 'init'
        self.stage_start = self.get_clock().now()
        self.last_time = self.get_clock().now()

        # Build controllers
        self.pid_x = PID(
            self.get_parameter('pid_xy.kp').get_parameter_value().double_value,
            self.get_parameter('pid_xy.ki').get_parameter_value().double_value,
            self.get_parameter('pid_xy.kd').get_parameter_value().double_value,
            self.get_parameter('pid_xy.i_limit').get_parameter_value().double_value,
        )
        self.pid_y = PID(
            self.get_parameter('pid_xy.kp').get_parameter_value().double_value,
            self.get_parameter('pid_xy.ki').get_parameter_value().double_value,
            self.get_parameter('pid_xy.kd').get_parameter_value().double_value,
            self.get_parameter('pid_xy.i_limit').get_parameter_value().double_value,
        )
        self.pid_z = PID(
            self.get_parameter('pid_z.kp').get_parameter_value().double_value,
            self.get_parameter('pid_z.ki').get_parameter_value().double_value,
            self.get_parameter('pid_z.kd').get_parameter_value().double_value,
            self.get_parameter('pid_z.i_limit').get_parameter_value().double_value,
        )

        # Timers
        self.timer = self.create_timer(1.0 / 20.0, self._tick)  # 20 Hz

        # Metrics
        self.accum_err = 0.0
        self.accum_cnt = 0

        # Vis buffers
        self.cur_path = PathMsg()
        self.cur_path.header.frame_id = 'map'
        self.tgt_path = PathMsg()
        self.tgt_path.header.frame_id = 'map'

        # Waypoint state
        self._wpts: List[Tuple[float, float, float]] = []  # relative ENU (x,y,alt)
        self._wp_idx = 0
        self._wp_reached_time = None

        # Slew limiter state
        self._last_cmd_vx = 0.0
        self._last_cmd_vy = 0.0
        self._last_cmd_vz = 0.0

        self._load_waypoints()

        # Zigzag state
        self._zigzag_built = False
        self._episode = 0
        self._episode_stage = 'forward'  # forward | return
        self._ep_err_sum = 0.0
        self._ep_err_cnt = 0
        self._ep_max_err = 0.0
        # Build session directory and files
        _metrics_file_param = self.get_parameter('metrics.file').value
        _metrics_root = self.get_parameter('metrics.root').value
        if _metrics_file_param:
            # legacy: write to specified file
            self._session_dir = os.path.dirname(_metrics_file_param) or _metrics_root
            self._metrics_file = _metrics_file_param
        else:
            now = datetime.now()
            date_dir = now.strftime('%Y-%m-%d')
            time_dir = now.strftime('%H%M%S')
            self._session_dir = os.path.join(_metrics_root, date_dir, time_dir)
            self._metrics_file = os.path.join(self._session_dir, 'zigzag_metrics.csv')
        # also track 5s averages
        self._avg_file = os.path.join(self._session_dir, 'avg_err_5s.csv')
        try:
            os.makedirs(self._session_dir, exist_ok=True)
            if not os.path.exists(self._metrics_file):
                with open(self._metrics_file, 'w') as f:
                    f.write('timestamp,episode,stage,avg_err,max_err,v_xy,kp_xy,points\n')
            if not os.path.exists(self._avg_file):
                with open(self._avg_file, 'w') as f:
                    f.write('timestamp,avg_err_5s\n')
            # create/update latest symlink for quick access
            latest_link = os.path.join(_metrics_root, 'latest')
            try:
                if os.path.islink(latest_link) or os.path.exists(latest_link):
                    try:
                        os.remove(latest_link)
                    except IsADirectoryError:
                        pass
                os.symlink(self._session_dir, latest_link)
            except Exception:
                pass
            self.get_logger().info(f'Metrics session: {self._session_dir}')
        except Exception as e:
            self.get_logger().warn(f'Failed to init metrics directory: {e}')

        # State flags
        self._demo_done = False
        self._end_action_done = False

        # Dynamic parameter updates
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info('OffboardPIDController ready. Mode=%s' % self.get_parameter('mode').get_parameter_value().string_value)

    # Parameter update callback
    def _on_params(self, params):
        for p in params:
            if p.name.startswith('pid_xy.') or p.name.startswith('pid_z.'):
                # Rebuild PIDs on-the-fly
                self.pid_x.kp = self.get_parameter('pid_xy.kp').value if p.name != 'pid_xy.kp' else p.value
                self.pid_x.ki = self.get_parameter('pid_xy.ki').value if p.name != 'pid_xy.ki' else p.value
                self.pid_x.kd = self.get_parameter('pid_xy.kd').value if p.name != 'pid_xy.kd' else p.value
                self.pid_x.i_limit = self.get_parameter('pid_xy.i_limit').value if p.name != 'pid_xy.i_limit' else p.value
                self.pid_y.kp = self.pid_x.kp
                self.pid_y.ki = self.pid_x.ki
                self.pid_y.kd = self.pid_x.kd
                self.pid_y.i_limit = self.pid_x.i_limit
                self.pid_z.kp = self.get_parameter('pid_z.kp').value if p.name != 'pid_z.kp' else p.value
                self.pid_z.ki = self.get_parameter('pid_z.ki').value if p.name != 'pid_z.ki' else p.value
                self.pid_z.kd = self.get_parameter('pid_z.kd').value if p.name != 'pid_z.kd' else p.value
                self.pid_z.i_limit = self.get_parameter('pid_z.i_limit').value if p.name != 'pid_z.i_limit' else p.value
                self.get_logger().info('PID updated dynamically')
            if p.name == 'waypoints_file':
                self._load_waypoints(new_file=p.value)
            if p.name.startswith('zigzag.'):
                self._zigzag_built = False
        return SetParametersResult(successful=True)

    # Callbacks
    def _state_cb(self, msg: State):
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg
        if self.home_pose is None:
            self.home_pose = msg

    # Main loop
    def _tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Continuous setpoint stream to keep offboard alive
        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'map'

        if not self.current_state.connected:
            self.cmd_vel_pub.publish(cmd)
            return

        # Warm-up setpoint stream before OFFBOARD
        if self.stage == 'init':
            if (now - self.stage_start) >= Duration(seconds=2.0):
                self._try_set_mode('OFFBOARD')
                self._try_arm(True)
                if self.current_state.mode == 'OFFBOARD' and self.current_state.armed:
                    self.stage = 'run'
                    self.stage_start = now
                    self.get_logger().info('Stage -> run')
            self.cmd_vel_pub.publish(cmd)
            return

        # If demo completed, apply end action and idle
        if self._demo_done:
            action = str(self.get_parameter('zigzag.end_action').value)
            if not self._end_action_done:
                if action == 'land':
                    self._try_set_mode('AUTO.LAND')
                elif action == 'hold':
                    self._try_set_mode('AUTO.LOITER')
                # hover: keep OFFBOARD with zero command
                self._end_action_done = True
            self.cmd_vel_pub.publish(cmd)
            return

        # Ensure we stay in OFFBOARD and armed
        if self.current_state.mode != 'OFFBOARD':
            self._try_set_mode('OFFBOARD')
        if not self.current_state.armed:
            self._try_arm(True)

        # Need pose and home
        if self.current_pose is None or self.home_pose is None or dt <= 0.0:
            self.cmd_vel_pub.publish(cmd)
            return

        # Target generation
        mode = self.get_parameter('mode').get_parameter_value().string_value
        if mode == 'zigzag_repeat' and self.home_pose is not None and not self._zigzag_built:
            self._build_zigzag_path()
        target_alt = float(self.get_parameter('target_alt').value)
        dx = dy = 0.0
        if mode == 'hover':
            # target XY = home, Z = target_alt
            tx = self.home_pose.pose.position.x
            ty = self.home_pose.pose.position.y
        elif mode == 'circle':
            radius = float(self.get_parameter('circle_radius').value)
            period = float(self.get_parameter('circle_period').value)
            omega = 2.0 * math.pi / max(period, 1e-3)
            t = (now - self.stage_start).nanoseconds * 1e-9
            cx = self.home_pose.pose.position.x
            cy = self.home_pose.pose.position.y
            tx = cx + radius * math.cos(omega * t)
            ty = cy + radius * math.sin(omega * t)
        elif mode in ('waypoints', 'zigzag_repeat') and self._wpts:
            tx, ty, alt = self._current_waypoint_target()
            tz = self.home_pose.pose.position.z + alt
        else:
            tx = self.home_pose.pose.position.x
            ty = self.home_pose.pose.position.y

        if mode not in ('waypoints', 'zigzag_repeat'):
            tz = self.home_pose.pose.position.z + target_alt

        # Current position (ENU)
        px = self.current_pose.pose.position.x
        py = self.current_pose.pose.position.y
        pz = self.current_pose.pose.position.z

        # Errors
        ex = tx - px
        ey = ty - py
        ez = tz - pz

        # PID control → velocity setpoints
        vx = self.pid_x.update(ex, dt)
        vy = self.pid_y.update(ey, dt)
        vz = self.pid_z.update(ez, dt)

        # Limits
        v_xy_lim = float(self.get_parameter('limit.v_xy').value)
        v_z_lim = float(self.get_parameter('limit.v_z').value)
        vx = max(min(vx, v_xy_lim), -v_xy_lim)
        vy = max(min(vy, v_xy_lim), -v_xy_lim)
        vz = max(min(vz, v_z_lim), -v_z_lim)

        # Slew rate limiting
        dv_xy_lim = float(self.get_parameter('limit.dv_xy').value)
        dv_z_lim = float(self.get_parameter('limit.dv_z').value)
        vx = self._slew(vx, self._last_cmd_vx, dv_xy_lim, dt)
        vy = self._slew(vy, self._last_cmd_vy, dv_xy_lim, dt)
        vz = self._slew(vz, self._last_cmd_vz, dv_z_lim, dt)
        self._last_cmd_vx, self._last_cmd_vy, self._last_cmd_vz = vx, vy, vz

        # Geofence (clamp velocity if outside box)
        x_min = float(self.get_parameter('geofence.x_min').value)
        x_max = float(self.get_parameter('geofence.x_max').value)
        y_min = float(self.get_parameter('geofence.y_min').value)
        y_max = float(self.get_parameter('geofence.y_max').value)
        z_min = float(self.get_parameter('geofence.z_min').value)
        z_max = float(self.get_parameter('geofence.z_max').value)

        if not (x_min <= px <= x_max):
            vx = 0.0
        if not (y_min <= py <= y_max):
            vy = 0.0
        if not (z_min <= pz <= z_max):
            vz = 0.0

        # Publish command
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        self.cmd_vel_pub.publish(cmd)

        # Visualization: append to paths and publish simple line marker (target)
        from geometry_msgs.msg import PoseStamped as _PoseStamped
        ps = _PoseStamped()
        ps.header = cmd.header
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.position.z = pz
        self.cur_path.header.stamp = cmd.header.stamp
        self.cur_path.poses.append(ps)
        if len(self.cur_path.poses) > 400:
            self.cur_path.poses = self.cur_path.poses[-400:]
        self.current_path_pub.publish(self.cur_path)

        ts = _PoseStamped()
        ts.header = cmd.header
        ts.pose.position.x = tx
        ts.pose.position.y = ty
        ts.pose.position.z = tz
        self.tgt_path.header.stamp = cmd.header.stamp
        self.tgt_path.poses.append(ts)
        if len(self.tgt_path.poses) > 400:
            self.tgt_path.poses = self.tgt_path.poses[-400:]
        self.target_path_pub.publish(self.tgt_path)

        mk = Marker()
        mk.header = cmd.header
        mk.ns = 'target_path'
        mk.id = 1
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.scale.x = 0.03
        mk.color.a = 1.0
        mk.color.r = 1.0
        mk.color.g = 0.2
        mk.color.b = 0.2
        mk.points = [p.pose.position for p in self.tgt_path.poses]
        self.marker_pub.publish(mk)

        # Metrics (position error magnitude in XY)
        err_xy = math.hypot(ex, ey)
        self.accum_err += err_xy
        self.accum_cnt += 1
        if mode == 'zigzag_repeat':
            self._ep_err_sum += err_xy
            self._ep_err_cnt += 1
            if err_xy > self._ep_max_err:
                self._ep_max_err = err_xy
        if self.accum_cnt >= 100:  # ~5 s
            avg_err = self.accum_err / self.accum_cnt
            self.get_logger().info(f'Avg XY error (last ~5s): {avg_err:.2f} m')
            try:
                self.avg_err_pub.publish(Float32(data=float(avg_err)))
                # append to CSV
                ts = datetime.now().isoformat()
                with open(self._avg_file, 'a') as f:
                    f.write(f'{ts},{float(avg_err):.6f}\n')
            except Exception:
                pass
            self.accum_err = 0.0
            self.accum_cnt = 0

        # Waypoint progression
        if mode in ('waypoints', 'zigzag_repeat') and self._wpts:
            acc = float(self.get_parameter('wp_accept_radius').value)
            if math.hypot(ex, ey) <= acc and abs(ez) <= 0.5:
                if self._wp_reached_time is None:
                    self._wp_reached_time = now
                hold = float(self.get_parameter('wp_hold_time').value)
                if (now - self._wp_reached_time) >= Duration(seconds=hold):
                    self._advance_waypoint()
                    self._wp_reached_time = None
            else:
                self._wp_reached_time = None

            if mode == 'zigzag_repeat' and self._wp_idx == len(self._wpts) - 1:
                # last waypoint reached, switch stage or finish episode
                if self._episode_stage == 'forward':
                    # Prepare return: reverse path
                    self._wpts = list(reversed(self._wpts))
                    self._wp_idx = 0
                    self._episode_stage = 'return'
                    self.get_logger().info('Zigzag: stage -> return')
                else:
                    # Episode finished
                    self._finalize_episode()
                    loops = int(self.get_parameter('zigzag.loops').value)
                    if loops > 0 and self._episode >= loops:
                        self._demo_done = True
                        self.get_logger().info(f'Zigzag: completed {self._episode} episodes. Demo done.')
                    else:
                        # Rebuild path for next loop
                        self._build_zigzag_path()

    # Helpers
    def _try_set_mode(self, mode: str):
        if not self.set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def _try_arm(self, arm: bool):
        if not self.arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = arm
        self.arming_client.call_async(req)

    # Utilities
    def _build_zigzag_path(self):
        L = float(self.get_parameter('zigzag.length_m').value)
        W = float(self.get_parameter('zigzag.width_m').value)
        dx = float(self.get_parameter('zigzag.dx').value)
        alt = float(self.get_parameter('zigzag.alt').value)
        # Build path as lawnmower/zigzag along X, oscillating Y between -W/2 and +W/2
        pts: List[Tuple[float, float, float]] = []
        cx = self.home_pose.pose.position.x
        cy = self.home_pose.pose.position.y
        x = 0.0
        y = -W / 2.0
        direction = 1.0  # toward +W/2
        pts.append((cx + x, cy + y, alt))
        while x < L:
            y = (W / 2.0) * direction
            pts.append((cx + x, cy + y, alt))
            x = min(x + dx, L)
            pts.append((cx + x, cy + y, alt))
            direction *= -1.0
        self._wpts = [(px - cx, py - cy, pz) for (px, py, pz) in pts]  # store relative to home
        self._wp_idx = 0
        self._episode_stage = 'forward'
        self._ep_err_sum = 0.0
        self._ep_err_cnt = 0
        self._ep_max_err = 0.0
        self._zigzag_built = True
        self.get_logger().info(f'Zigzag path built: {len(self._wpts)} points')

    def _load_waypoints(self, new_file: Optional[str] = None):
        try:
            wp_file = new_file or self.get_parameter('waypoints_file').get_parameter_value().string_value
            path = FsPath(wp_file)
            if not path.exists():
                self.get_logger().warn(f'Waypoints file not found: {path}')
                return
            data = yaml.safe_load(path.read_text())
            wps = []
            for w in data.get('waypoints', []):
                x = float(w.get('x', 0.0))
                y = float(w.get('y', 0.0))
                alt = float(w.get('alt', self.get_parameter('target_alt').value))
                wps.append((x, y, alt))
            if wps:
                self._wpts = wps
                self._wp_idx = 0
                self.get_logger().info(f'Loaded {len(wps)} waypoints from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')

    def _current_waypoint_target(self) -> Tuple[float, float, float]:
        # Targets are relative to home ENU
        cx = self.home_pose.pose.position.x
        cy = self.home_pose.pose.position.y
        x, y, alt = self._wpts[self._wp_idx]
        return cx + x, cy + y, alt

    def _advance_waypoint(self):
        self._wp_idx += 1
        if self._wp_idx >= len(self._wpts):
            if bool(self.get_parameter('wp_loop').value):
                self._wp_idx = 0
            else:
                self._wp_idx = len(self._wpts) - 1
        self.get_logger().info(f'WP -> {self._wp_idx + 1}/{len(self._wpts)}')

    @staticmethod
    def _slew(target: float, current: float, max_rate: float, dt: float) -> float:
        max_step = max_rate * max(dt, 0.0)
        if target > current + max_step:
            return current + max_step
        if target < current - max_step:
            return current - max_step
        return target

    def _finalize_episode(self):
        self._episode += 1
        avg_err = (self._ep_err_sum / self._ep_err_cnt) if self._ep_err_cnt > 0 else 0.0
        v_xy = float(self.get_parameter('limit.v_xy').value)
        kp_xy = float(self.get_parameter('pid_xy.kp').value)
        self.get_logger().info(f'Zigzag episode {self._episode} done: avg={avg_err:.2f} m, max={self._ep_max_err:.2f} m, v_xy={v_xy:.2f}, kp={kp_xy:.2f}')
        # Log CSV
        try:
            ts = datetime.now().isoformat()
            with open(self._metrics_file, 'a') as f:
                f.write(f'{ts},{self._episode},{self._episode_stage},{avg_err:.3f},{self._ep_max_err:.3f},{v_xy:.2f},{kp_xy:.2f},{self._ep_err_cnt}\n')
        except Exception:
            pass
        # Adaptation
        if bool(self.get_parameter('adapt.enable').value):
            target = float(self.get_parameter('adapt.avg_err_target').value)
            margin = float(self.get_parameter('adapt.margin').value)
            v_step = float(self.get_parameter('adapt.v_step').value)
            v_min = float(self.get_parameter('adapt.v_min').value)
            v_max = float(self.get_parameter('adapt.v_max').value)
            kp_step = float(self.get_parameter('adapt.kp_step').value)
            # If too high error -> slow down slightly and increase kp
            if avg_err > target * (1.0 + margin):
                new_v = max(v_min, v_xy - v_step)
                self.set_parameters([rclpy.parameter.Parameter('limit.v_xy', rclpy.Parameter.Type.DOUBLE, new_v),
                                     rclpy.parameter.Parameter('pid_xy.kp', rclpy.Parameter.Type.DOUBLE, kp_xy + kp_step)])
                self.get_logger().info(f'Adapt: dec v_xy to {new_v:.2f}, inc kp to {kp_xy + kp_step:.2f}')
            elif avg_err < target * max(0.1, (1.0 - margin)):
                new_v = min(v_max, v_xy + v_step)
                self.set_parameters([rclpy.parameter.Parameter('limit.v_xy', rclpy.Parameter.Type.DOUBLE, new_v)])
                self.get_logger().info(f'Adapt: inc v_xy to {new_v:.2f}')
        # Reset episode metrics
        self._ep_err_sum = 0.0
        self._ep_err_cnt = 0
        self._ep_max_err = 0.0
        # Publish episode summary
        try:
            msg = {
                'episode': int(self._episode),
                'stage': self._episode_stage,
                'avg_err': float(avg_err),
                'max_err': float(self._ep_max_err),
                'v_xy': float(self.get_parameter('limit.v_xy').value),
                'kp_xy': float(self.get_parameter('pid_xy.kp').value),
            }
            self.episode_pub.publish(String(data=str(msg)))
        except Exception:
            pass


def main():
    rclpy.init()
    node = OffboardPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
