import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import PacketID

QUEUE_SIZE = 100

CONTROL_PACKET_IDS = {
    PacketID.MODE,
    PacketID.VELOCITY,
    PacketID.POSITION,
    PacketID.CURRENT,
    PacketID.TORQUE,
    PacketID.INDEXED_RELATIVE_POSITION,
    PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION,
    PacketID.INVERSE_KINEMATICS_GLOBAL_VELOCITY,
    PacketID.INVERSE_KINEMATICS_LOCAL_VELOCITY,
}


class MasterArmBridgeNode(Node):

    def __init__(self):
        super().__init__('master_arm_bridge')

        self.alpha_tx = self.create_publisher(Packet, '/alpha/tx', QUEUE_SIZE)
        self.create_subscription(Packet, '/master/rx', self._on_master_packet, QUEUE_SIZE)

        self.get_logger().info('Master arm bridge started — forwarding control packets from /master/rx to /alpha/tx')

    def _on_master_packet(self, packet: Packet):
        if packet.packet_id not in CONTROL_PACKET_IDS:
            return
        self.get_logger().debug(
            f'Forwarding device_id={packet.device_id:#04x} packet_id={packet.packet_id:#04x}'
        )
        self.alpha_tx.publish(packet)


def main(args=None):
    rclpy.init(args=args)
    node = MasterArmBridgeNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
