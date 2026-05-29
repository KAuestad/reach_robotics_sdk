import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import PacketID

from .tool_pose_and_vel import tool_position_from_gripper_pose, tool_velocity_from_gripper_pose_and_vel

ALPHA_BASE_ID = 0x05
QUEUE_SIZE = 10
LOOP_HZ = 10
_W = 76


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

        self._pub = self.create_publisher(Packet, '/alpha/tx', QUEUE_SIZE)
        self.create_subscription(Packet, '/alpha/rx', self._on_alpha, QUEUE_SIZE)

        self._pos       = None
        self._prev_pos  = None
        self._prev_time = None
        self._first     = True

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
        self._pos = list(packet.float_data[:6])

    def _tick(self):
        self._request_position()
        if self._pos is not None:
            self._display()
        else:
            sys.stdout.write('\rWaiting for gripper pose...\r')
            sys.stdout.flush()

    def _request_position(self):
        p = Packet()
        p.device_id = ALPHA_BASE_ID
        p.packet_id = PacketID.REQUEST
        p.int_data  = [PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION]
        self._pub.publish(p)

    def _display(self):
        sys.stdout.write('hei')  # debug
        now = self.get_clock().now().nanoseconds * 1e-9

        # Numerical differentiation: v_tool = v_gripper + omega × (R @ p_tool)
        tool_speed = None
        v_tool = None
        if self._prev_pos is not None and self._prev_time is not None:
            dt = now - self._prev_time
            if dt > 1e-6:
                cur = np.array(self._pos)
                prv = np.array(self._prev_pos)
                v_gripper = (cur[:3] - prv[:3]) / dt
                # Packet order: [yaw, pitch, roll] → Cartesian [X, Y, Z] = reversed
                omega_cartesian = (cur[3:] - prv[3:])[::-1] / dt
                v_tool = tool_velocity_from_gripper_pose_and_vel(
                    self._pos[:3], self._pos[3], self._pos[4], self._pos[5],
                    self._p_tool, v_gripper, omega_cartesian,
                )
                tool_speed = float(np.linalg.norm(v_tool))

        self._prev_pos  = list(self._pos)
        self._prev_time = now

        g = self._pos
        t = tool_position_from_gripper_pose(g[:3], g[3], g[4], g[5], self._p_tool)

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
            # Move cursor up and overwrite the previous block in place.
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
