import math
import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import PacketID, Mode

ALPHA_BASE_ID = 0x05
QUEUE_SIZE = 100
STREAM_HZ = 20


class MoveABNode(Node):

    def __init__(self):
        super().__init__('move_ab')

        self.declare_parameter('ax', 0.0)
        self.declare_parameter('ay', 0.0)
        self.declare_parameter('az', 0.0)
        self.declare_parameter('bx', 0.0)
        self.declare_parameter('by', 0.0)
        self.declare_parameter('bz', 0.0)
        self.declare_parameter('speed', 50.0)     # mm/s
        self.declare_parameter('tolerance', 5.0)  # mm

        self._a = (
            self.get_parameter('ax').value,
            self.get_parameter('ay').value,
            self.get_parameter('az').value,
        )
        self._b = (
            self.get_parameter('bx').value,
            self.get_parameter('by').value,
            self.get_parameter('bz').value,
        )
        self._speed = self.get_parameter('speed').value
        self._tol   = self.get_parameter('tolerance').value

        if math.dist(self._a, self._b) < 1.0:
            self.get_logger().error('Point A and point B are the same. Aborting.')
            raise SystemExit

        self._pub = self.create_publisher(Packet, 'tx', QUEUE_SIZE)
        self.create_subscription(Packet, 'rx', self._on_rx, QUEUE_SIZE)

        self._pos = None          # latest end-effector position from telemetry
        self._state = 'WAIT'

        self.create_timer(1.0 / STREAM_HZ, self._tick)

        self.get_logger().info(
            f'A={self._a} mm  B={self._b} mm  speed={self._speed} mm/s'
        )

    # ------------------------------------------------------------------ #

    def _on_rx(self, packet: Packet):
        if packet.device_id != ALPHA_BASE_ID:
            return
        if packet.packet_id != PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION:
            return
        if len(packet.float_data) < 3:
            return
        self._pos = (packet.float_data[0], packet.float_data[1], packet.float_data[2])

    def _dist(self, target) -> float:
        return math.dist(self._pos, target) if self._pos else float('inf')

    def _set_mode(self, mode: int):
        p = Packet()
        p.device_id = 0xFF           # broadcast — sets all devices
        p.packet_id = PacketID.MODE
        p.int_data  = [mode]
        self._pub.publish(p)

    def _send_position(self, target):
        p = Packet()
        p.device_id  = ALPHA_BASE_ID
        p.packet_id  = PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION
        p.float_data = [target[0], target[1], target[2], 0.0, 0.0, 0.0]
        self._pub.publish(p)

    def _send_velocity(self, vx: float, vy: float, vz: float):
        p = Packet()
        p.device_id  = ALPHA_BASE_ID
        p.packet_id  = PacketID.INVERSE_KINEMATICS_GLOBAL_VELOCITY
        p.float_data = [vx, vy, vz, 0.0, 0.0, 0.0]
        self._pub.publish(p)

    def _request_position(self):
        p = Packet()
        p.device_id = ALPHA_BASE_ID
        p.packet_id = PacketID.REQUEST
        p.int_data  = [PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION]
        self._pub.publish(p)

    # ------------------------------------------------------------------ #

    def _tick(self):
        self._request_position()

        if self._state == 'WAIT':
            if self._pos is not None:
                self._state = 'MOVE_TO_A'
                self._set_mode(Mode.KINEMATIC_POSITION_BASE_FRAME)
                self.get_logger().info(
                    f'Current pos: {tuple(round(v,1) for v in self._pos)} mm — moving to A...'
                )

        elif self._state == 'MOVE_TO_A':
            self._send_position(self._a)
            dist = self._dist(self._a)
            self.get_logger().info(f'[→ A]  distance: {dist:.1f} mm')
            if dist < self._tol:
                self._state = 'MOVE_TO_B'
                self._set_mode(Mode.KINEMATIC_VELOCTIY_BASE_FRAME)
                self.get_logger().info(
                    f'Reached A. Moving to B at {self._speed} mm/s...'
                )

        elif self._state == 'MOVE_TO_B':
            if self._pos is None:
                return
            dist = self._dist(self._b)
            self.get_logger().info(f'[→ B]  distance: {dist:.1f} mm')
            if dist < self._tol:
                self._state = 'DONE'
                self._send_velocity(0.0, 0.0, 0.0)
                self._set_mode(Mode.POSITION_HOLD)
                self.get_logger().info('Reached B. Holding position.')
                return
            dx = self._b[0] - self._pos[0]
            dy = self._b[1] - self._pos[1]
            dz = self._b[2] - self._pos[2]
            scale = self._speed / math.sqrt(dx**2 + dy**2 + dz**2)
            self.get_logger().info(f'Scale: {scale:.3f}  velocity: ({dx*scale:.1f}, {dy*scale:.1f}, {dz*scale:.1f}) mm/s')
            self._send_velocity(dx * scale, dy * scale, dz * scale)

        elif self._state == 'DONE':
            pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MoveABNode()
    except SystemExit:
        rclpy.shutdown()
        return
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
