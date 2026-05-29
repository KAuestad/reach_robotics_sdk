import collections
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import PacketID

from .tool_pose_and_vel import tool_position_from_gripper_pose, tool_velocity_from_gripper_pose_and_vel

ALPHA_BASE_ID = 0x05
QUEUE_SIZE = 10
LOOP_HZ = 10
_W = 76
_SPEED_LIMIT = 100.0  # mm/s — samples above this are discarded as differentiation artefacts


class PoseMonitorNode(Node):

    def __init__(self):
        super().__init__('pose_monitor')

        self.declare_parameter('tool_x', 0.0)
        self.declare_parameter('tool_y', 0.0)
        self.declare_parameter('tool_z', 0.0)

        self._p_tool = np.array([
            self.get_parameter('tool_x').value,
            self.get_parameter('tool_y').value,
            self.get_parameter('tool_z').value,
        ])

        self._pub       = self.create_publisher(Packet,      '/alpha/tx',        QUEUE_SIZE)
        self._pose_pub  = self.create_publisher(PoseStamped, '/alpha/tool_pose',  QUEUE_SIZE)
        self._twist_pub = self.create_publisher(TwistStamped,'/alpha/tool_twist', QUEUE_SIZE)
        self.create_subscription(Packet, '/alpha/rx', self._on_alpha, QUEUE_SIZE)

        self._pos            = None   # latest gripper pose [x,y,z,yaw,pitch,roll]
        self._pos_time       = None   # wall time (s) when _pos was last updated
        self._omega_cart     = np.zeros(3)  # latest angular velocity, Cartesian
        self._vel_buffer     = collections.deque(maxlen=5)  # accepted v_tool samples
        self._first          = True

        self.create_timer(1.0 / LOOP_HZ, self._tick)

        sys.stdout.write(
            f'\r\n{"─" * _W}\r\n'
            f'  Pose Monitor'
            f'   tool offset: '
            f'X {self._p_tool[0]:.1f}  Y {self._p_tool[1]:.1f}  Z {self._p_tool[2]:.1f} mm\r\n'
            f'{"─" * _W}\r\n'
        )
        sys.stdout.flush()

    def _on_alpha(self, packet: Packet):
        if packet.device_id != ALPHA_BASE_ID:
            return
        if packet.packet_id != PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION:
            return
        if len(packet.float_data) < 6:
            return

        now     = self.get_clock().now().nanoseconds * 1e-9
        new_pos = list(packet.float_data[:6])

        # Compute velocity using the actual inter-packet interval as dt.
        # This avoids the timing error that occurs when differentiation is done
        # at display-tick time (tick period ≠ measurement interval).
        if self._pos is not None and self._pos_time is not None:
            dt = now - self._pos_time
            if dt > 1e-6:
                cur = np.array(new_pos)
                prv = np.array(self._pos)
                v_gripper = (cur[:3] - prv[:3]) / dt
                # Packet order: [yaw, pitch, roll] → Cartesian [X, Y, Z] = reversed
                self._omega_cart = (cur[3:] - prv[3:])[::-1] / dt
                v_raw = tool_velocity_from_gripper_pose_and_vel(
                    new_pos[:3], new_pos[3], new_pos[4], new_pos[5],
                    self._p_tool, v_gripper, self._omega_cart,
                )
                if np.linalg.norm(v_raw) <= _SPEED_LIMIT:
                    self._vel_buffer.append(v_raw)

        self._pos      = new_pos
        self._pos_time = now

    def _tick(self):
        self._request_position()
        if self._pos is not None:
            self._display()

    def _request_position(self):
        p = Packet()
        p.device_id = ALPHA_BASE_ID
        p.packet_id = PacketID.REQUEST
        p.int_data  = [PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION]
        self._pub.publish(p)

    @staticmethod
    def _ypr_to_quat(yaw: float, pitch: float, roll: float):
        cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
        return (
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z
            cr * cp * cy + sr * sp * sy,  # w
        )

    def _display(self):
        g = self._pos
        t = tool_position_from_gripper_pose(g[:3], g[3], g[4], g[5], self._p_tool)

        if self._vel_buffer:
            v_tool     = np.mean(self._vel_buffer, axis=0)
            tool_speed = float(np.linalg.norm(v_tool))
        else:
            v_tool     = None
            tool_speed = None

        # ── publish pose and twist ────────────────────────────────────────────
        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = stamp
        pose_msg.header.frame_id = 'base'
        pose_msg.pose.position.x = t[0] / 1000.0
        pose_msg.pose.position.y = t[1] / 1000.0
        pose_msg.pose.position.z = t[2] / 1000.0
        qx, qy, qz, qw = self._ypr_to_quat(g[3], g[4], g[5])
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self._pose_pub.publish(pose_msg)

        twist_msg = TwistStamped()
        twist_msg.header.stamp    = stamp
        twist_msg.header.frame_id = 'base'
        if v_tool is not None:
            twist_msg.twist.linear.x  = v_tool[0] / 1000.0
            twist_msg.twist.linear.y  = v_tool[1] / 1000.0
            twist_msg.twist.linear.z  = v_tool[2] / 1000.0
            twist_msg.twist.angular.x = self._omega_cart[0]
            twist_msg.twist.angular.y = self._omega_cart[1]
            twist_msg.twist.angular.z = self._omega_cart[2]
        self._twist_pub.publish(twist_msg)

        # ── terminal display ──────────────────────────────────────────────────
        if tool_speed is not None:
            speed_line = (
                f'  Speed    {tool_speed:6.1f} mm/s'
                f'   vx {v_tool[0]:6.1f}  vy {v_tool[1]:6.1f}  vz {v_tool[2]:6.1f} mm/s'
            )
        else:
            speed_line = f'  Speed    {"--.-":>6} mm/s'

        lines = [
            f'  Gripper  '
            f'X {g[0]:8.1f}  Y {g[1]:8.1f}  Z {g[2]:8.1f} mm'
            f'   yaw {g[3]:7.3f}  pitch {g[4]:7.3f}  roll {g[5]:7.3f} rad',
            f'  Tool     '
            f'X {t[0]:8.1f}  Y {t[1]:8.1f}  Z {t[2]:8.1f} mm',
            speed_line,
        ]

        if self._first:
            sys.stdout.write('\r\n'.join(lines) + '\r\n')
            self._first = False
        else:
            sys.stdout.write(f'\033[{len(lines)}A\r')
            sys.stdout.write('\r\n'.join(lines) + '\r\n')

        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = PoseMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
