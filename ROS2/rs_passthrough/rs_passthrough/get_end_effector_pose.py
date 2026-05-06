import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import PacketID

ALPHA_BASE_DEVICE_ID = 0x05
QUEUE_SIZE = 10


class GetEndEffectorPose(Node):

    def __init__(self):
        super().__init__('get_end_effector_pose')
        self._pub = self.create_publisher(Packet, 'tx', QUEUE_SIZE)
        self.create_subscription(Packet, 'rx', self._on_rx, QUEUE_SIZE)

        self.create_timer(0.1, self._send_request)

    def _send_request(self):
        request = Packet()
        request.device_id = ALPHA_BASE_DEVICE_ID
        request.packet_id = PacketID.REQUEST
        request.int_data = [PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION]
        self._pub.publish(request)

    def _on_rx(self, packet: Packet):
        if packet.device_id != ALPHA_BASE_DEVICE_ID:
            return
        if packet.packet_id != PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION:
            return
        if len(packet.float_data) < 3:
            return

        x, y, z = packet.float_data[0], packet.float_data[1], packet.float_data[2]
        msg = f'X: {x:.1f} mm  Y: {y:.1f} mm  Z: {z:.1f} mm'
        if len(packet.float_data) >= 6:
            yaw, pitch, roll = packet.float_data[3], packet.float_data[4], packet.float_data[5]
            msg += f'  Yaw: {yaw:.4f} rad  Pitch: {pitch:.4f} rad  Roll: {roll:.4f} rad'
        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GetEndEffectorPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
