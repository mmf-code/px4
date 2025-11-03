import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')

        # QoS profiles
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )

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

        self.current_state = State()
        self.current_pose = None

        self.setpoint_timer = self.create_timer(1.0 / 20.0, self._publish_setpoint)  # 20 Hz

        # Controller state
        self.mode_sent = False
        self.armed_sent = False
        self.stage = 'init'  # init -> takeoff -> circle -> land -> done
        self.stage_start = self.get_clock().now()
        self.takeoff_alt = 2.5
        self.circle_duration = 30.0
        self.land_sent = False

        # Velocity command buffer
        self.last_cmd = TwistStamped()
        self.last_cmd.header.frame_id = 'map'
        self.last_cmd.twist.linear.x = 0.0
        self.last_cmd.twist.linear.y = 0.0
        self.last_cmd.twist.linear.z = 0.0

        self.get_logger().info('OffboardController initialized. Waiting for connection...')

    # Callbacks
    def _state_cb(self, msg: State):
        self.current_state = msg

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    # Core loop: publish setpoints and manage simple mission
    def _publish_setpoint(self):
        # Always publish something (>2Hz) to keep offboard happy
        now = self.get_clock().now()

        # If not connected yet, just stream zeros
        if not self.current_state.connected:
            self.cmd_vel_pub.publish(self.last_cmd)
            return

        # Warm-up: stream setpoints for ~2 seconds before OFFBOARD
        if self.stage == 'init':
            if (now - self.stage_start) >= Duration(seconds=2.0):
                self._try_set_mode('OFFBOARD')
                self._try_arm(True)
                if self.current_state.mode == 'OFFBOARD' and self.current_state.armed:
                    self.stage = 'takeoff'
                    self.stage_start = now
                    self.get_logger().info('Stage -> takeoff')
            # Stream zeros during init
            self.last_cmd.twist.linear.x = 0.0
            self.last_cmd.twist.linear.y = 0.0
            self.last_cmd.twist.linear.z = 0.0
            self.cmd_vel_pub.publish(self.last_cmd)
            return

        # Ensure we stay in OFFBOARD and armed during mission
        if self.current_state.mode != 'OFFBOARD':
            self._try_set_mode('OFFBOARD')
        if not self.current_state.armed:
            self._try_arm(True)

        # Takeoff stage: go up until target altitude
        if self.stage == 'takeoff':
            alt = self._current_alt()
            vz = 0.8
            if alt is not None and alt >= self.takeoff_alt:
                self.stage = 'circle'
                self.stage_start = now
                self.get_logger().info('Stage -> circle')
                self.last_cmd.twist.linear.x = 0.0
                self.last_cmd.twist.linear.y = 0.0
                self.last_cmd.twist.linear.z = 0.0
            else:
                # climb
                self.last_cmd.twist.linear.x = 0.0
                self.last_cmd.twist.linear.y = 0.0
                self.last_cmd.twist.linear.z = vz
            self.cmd_vel_pub.publish(self.last_cmd)
            return

        # Circle stage: fly a circle in XY plane for circle_duration seconds
        if self.stage == 'circle':
            t = (now - self.stage_start).nanoseconds * 1e-9
            radius = 2.0
            omega = 2.0 * math.pi / 10.0  # 10 s per revolution
            vx = -radius * omega * math.sin(omega * t)
            vy = radius * omega * math.cos(omega * t)

            self.last_cmd.twist.linear.x = vx
            self.last_cmd.twist.linear.y = vy
            self.last_cmd.twist.linear.z = 0.0
            self.cmd_vel_pub.publish(self.last_cmd)

            if t >= self.circle_duration:
                self.stage = 'land'
                self.stage_start = now
                self.get_logger().info('Stage -> land')
            return

        # Land stage: switch to AUTO.LAND once, then idle
        if self.stage == 'land':
            if not self.land_sent:
                self._try_set_mode('AUTO.LAND')
                self.land_sent = True
            # publish zeros while landing
            self.last_cmd.twist.linear.x = 0.0
            self.last_cmd.twist.linear.y = 0.0
            self.last_cmd.twist.linear.z = 0.0
            self.cmd_vel_pub.publish(self.last_cmd)

            # After some time, consider done
            if (now - self.stage_start) >= Duration(seconds=10.0):
                self.stage = 'done'
                self.get_logger().info('Stage -> done')
            return

        # Done: keep publishing zeros
        if self.stage == 'done':
            self.last_cmd.twist.linear.x = 0.0
            self.last_cmd.twist.linear.y = 0.0
            self.last_cmd.twist.linear.z = 0.0
            self.cmd_vel_pub.publish(self.last_cmd)

    # Helpers
    def _try_set_mode(self, mode: str):
        if not self.set_mode_client.service_is_ready():
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        # No blocking here; best-effort and check via state callback

    def _try_arm(self, arm: bool):
        if not self.arming_client.service_is_ready():
            return
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)
        # No blocking

    def _current_alt(self):
        if self.current_pose is None:
            return None
        # MAVROS local position is ENU; z is up
        return float(self.current_pose.pose.position.z)


def main():
    rclpy.init()
    node = OffboardController()
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
