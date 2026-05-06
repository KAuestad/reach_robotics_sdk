import math
import os
import select
import signal
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import PacketID, Mode

ALPHA_BASE_ID = 0x05
QUEUE_SIZE = 100
LOOP_HZ = 20

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

PLAYBACK_BANNER = (
    '\r\n'
    '\r\n╔══════════════════════════════════════════════════╗'
    '\r\n║            PLAYBACK MODE ACTIVE                  ║'
    '\r\n║  Master arm input is disabled.                   ║'
    '\r\n║  Press ESC to abort and return to teleoperation. ║'
    '\r\n╚══════════════════════════════════════════════════╝'
    '\r\n'
)


class _State:
    TELEOP = 'TELEOP'
    MOVE_TO_FIRST = 'MOVE_TO_FIRST'
    PLAYBACK = 'PLAYBACK'


class TeachAndPlayNode(Node):

    def __init__(self):
        super().__init__('teach_and_play')

        self.declare_parameter('speed', 10.0)           # mm/s
        self.declare_parameter('tolerance', 5.0)        # mm
        self.declare_parameter('max_waypoints', 10)
        self.declare_parameter('max_angular_rate', 0.3) # rad/s
        self.declare_parameter('k_orient', 1.0)         # orientation P-gain

        self._speed     = self.get_parameter('speed').value
        self._tol       = self.get_parameter('tolerance').value
        self._max_wp    = self.get_parameter('max_waypoints').value
        self._max_ang   = self.get_parameter('max_angular_rate').value
        self._k_orient  = self.get_parameter('k_orient').value

        self._pub = self.create_publisher(Packet, '/alpha/tx', QUEUE_SIZE)
        self.create_subscription(Packet, '/master/rx', self._on_master, QUEUE_SIZE)
        self.create_subscription(Packet, '/alpha/rx',  self._on_alpha,  QUEUE_SIZE)

        self._state     = _State.TELEOP
        self._pos       = None   # [x, y, z, yaw, pitch, roll]  mm + rad
        self._waypoints = []     # list of 6-element lists
        self._wp_index  = 0

        self._running = True
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.create_timer(1.0 / LOOP_HZ, self._tick)

        self.get_logger().info(
            'TeachAndPlay ready — TELEOP mode\n'
            '  r : record current position\n'
            '  p : start playback\n'
            '  c : clear all waypoints\n'
            '  ESC : abort playback'
        )

    # ── subscriptions ────────────────────────────────────────────────────────

    def _on_master(self, packet: Packet):
        if self._state != _State.TELEOP:
            return
        if packet.packet_id not in CONTROL_PACKET_IDS:
            return
        self._pub.publish(packet)

    def _on_alpha(self, packet: Packet):
        if packet.device_id != ALPHA_BASE_ID:
            return
        if packet.packet_id != PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION:
            return
        if len(packet.float_data) < 6:
            return
        self._pos = list(packet.float_data[:6])

    # ── timer ────────────────────────────────────────────────────────────────

    def _tick(self):
        self._request_position()

        if self._state == _State.TELEOP:
            pass

        elif self._state == _State.MOVE_TO_FIRST:
            if self._pos is None:
                return
            target = self._waypoints[0]
            self._send_position(target)
            dist = self._xyz_dist(self._pos, target)
            self.get_logger().info(f'[→ wp0]  dist: {dist:.1f} mm')
            if dist < self._tol:
                if len(self._waypoints) == 1:
                    self._finish_playback()
                else:
                    self._wp_index = 1
                    self._set_mode(Mode.KINEMATIC_VELOCTIY_BASE_FRAME)
                    self._state = _State.PLAYBACK
                    self.get_logger().info(
                        f'Reached wp0. Streaming through '
                        f'{len(self._waypoints) - 1} remaining waypoint(s) at {self._speed} mm/s...'
                    )

        elif self._state == _State.PLAYBACK:
            if self._pos is None:
                return
            target = self._waypoints[self._wp_index]
            dist = self._xyz_dist(self._pos, target)
            self.get_logger().info(f'[→ wp{self._wp_index}]  dist: {dist:.1f} mm')
            if dist < self._tol:
                self._wp_index += 1
                if self._wp_index >= len(self._waypoints):
                    self._finish_playback()
                    return
                target = self._waypoints[self._wp_index]
                self.get_logger().info(f'Advancing to wp{self._wp_index}')
            self._stream_velocity(target)

    # ── commands ─────────────────────────────────────────────────────────────

    def _request_position(self):
        p = Packet()
        p.device_id = ALPHA_BASE_ID
        p.packet_id = PacketID.REQUEST
        p.int_data  = [PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION]
        self._pub.publish(p)

    def _set_mode(self, mode: int):
        p = Packet()
        p.device_id = 0xFF
        p.packet_id = PacketID.MODE
        p.int_data  = [mode]
        self._pub.publish(p)

    def _send_position(self, target):
        p = Packet()
        p.device_id  = ALPHA_BASE_ID
        p.packet_id  = PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION
        p.float_data = list(target)
        self._pub.publish(p)

    def _stream_velocity(self, target):
        dx = target[0] - self._pos[0]
        dy = target[1] - self._pos[1]
        dz = target[2] - self._pos[2]
        xyz_dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if xyz_dist > 1e-6:
            scale = self._speed / xyz_dist
            vx, vy, vz = dx * scale, dy * scale, dz * scale
        else:
            vx, vy, vz = 0.0, 0.0, 0.0

        # Proportional angular velocity toward target orientation.
        # Packet convention matches the position packet: [yaw, pitch, roll] at indices 3-5.
        def _wrap(a: float) -> float:
            return math.atan2(math.sin(a), math.cos(a))

        ang_vels = [
            max(-self._max_ang, min(self._max_ang, self._k_orient * _wrap(target[i] - self._pos[i])))
            for i in range(3, 6)
        ]

        p = Packet()
        p.device_id  = ALPHA_BASE_ID
        p.packet_id  = PacketID.INVERSE_KINEMATICS_GLOBAL_VELOCITY
        p.float_data = [vx, vy, vz] + ang_vels
        self._pub.publish(p)

    def _finish_playback(self):
        p = Packet()
        p.device_id  = ALPHA_BASE_ID
        p.packet_id  = PacketID.INVERSE_KINEMATICS_GLOBAL_VELOCITY
        p.float_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._pub.publish(p)
        self._set_mode(Mode.POSITION_HOLD)
        self._state = _State.TELEOP
        self.get_logger().info('Playback complete — returning to TELEOP.')

    # ── helpers ──────────────────────────────────────────────────────────────

    @staticmethod
    def _xyz_dist(a, b) -> float:
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    # ── keyboard ─────────────────────────────────────────────────────────────

    def _on_key(self, key: str):
        if key == 'r':
            if self._state != _State.TELEOP:
                return
            if self._pos is None:
                self.get_logger().warn('No position available yet — is the alpha passthrough running?')
                return
            if len(self._waypoints) >= self._max_wp:
                self.get_logger().warn(f'Maximum waypoints ({self._max_wp}) already recorded.')
                return
            self._waypoints.append(list(self._pos))
            idx = len(self._waypoints) - 1
            p = self._pos
            self.get_logger().info(
                f'Recorded wp{idx}: '
                f'({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm  '
                f'yaw={p[3]:.3f} pitch={p[4]:.3f} roll={p[5]:.3f} rad  '
                f'[{len(self._waypoints)}/{self._max_wp}]'
            )

        elif key == 'p':
            if self._state != _State.TELEOP:
                return
            if not self._waypoints:
                self.get_logger().warn('No waypoints recorded — press r to record positions first.')
                return
            self._wp_index = 0
            self._set_mode(Mode.KINEMATIC_POSITION_BASE_FRAME)
            self._state = _State.MOVE_TO_FIRST
            sys.stdout.write(PLAYBACK_BANNER)
            sys.stdout.flush()
            self.get_logger().info(
                f'Starting playback of {len(self._waypoints)} waypoint(s) at {self._speed} mm/s — '
                f'moving to wp0...'
            )

        elif key == 'c':
            if self._state != _State.TELEOP:
                self.get_logger().warn('Cannot clear waypoints during playback.')
                return
            count = len(self._waypoints)
            self._waypoints.clear()
            self.get_logger().info(f'Cleared {count} waypoint(s).')

        elif key == 'ESC':
            if self._state in (_State.MOVE_TO_FIRST, _State.PLAYBACK):
                self.get_logger().info('Playback aborted by user — returning to TELEOP.')
                self._finish_playback()

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        if not os.isatty(fd):
            self.get_logger().warn('stdin is not a TTY — keyboard control unavailable.')
            return
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running:
                if not select.select([sys.stdin], [], [], 0.1)[0]:
                    continue
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    # Consume the rest of any escape sequence (e.g. arrow keys).
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        sys.stdin.read(2)
                    self._on_key('ESC')
                elif ch == '\x03':  # Ctrl+C — restore terminal then send SIGINT
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    os.kill(os.getpid(), signal.SIGINT)
                    return
                elif ch in ('r', 'p', 'c'):
                    self._on_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeachAndPlayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
