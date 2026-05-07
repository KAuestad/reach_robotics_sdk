import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import PacketID

from .tool_pose_and_vel import tool_position_from_gripper_pose

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

        self._pos   = None
        self._first = True

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

    def _request_position(self):
        p = Packet()
        p.device_id = ALPHA_BASE_ID
        p.packet_id = PacketID.REQUEST
        p.int_data  = [PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION]
        self._pub.publish(p)

    def _display(self):
        g = self._pos
        t = tool_position_from_gripper_pose(g[:3], g[3], g[4], g[5], self._p_tool)

        lines = [
            f'  Gripper  '
            f'X {g[0]:8.1f}  Y {g[1]:8.1f}  Z {g[2]:8.1f} mm'
            f'   yaw {g[3]:7.3f}  pitch {g[4]:7.3f}  roll {g[5]:7.3f} rad',
            f'  Tool     '
            f'X {t[0]:8.1f}  Y {t[1]:8.1f}  Z {t[2]:8.1f} mm',
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
