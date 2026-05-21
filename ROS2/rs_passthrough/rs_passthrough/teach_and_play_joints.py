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
from sensor_msgs.msg import JointState

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

_W = 56

_STARTUP_BANNER = (
    f'\r\n{"─" * _W}\r\n'
    '  Teach and Play (joint space)  —  TELEOP\r\n'
    f'{"─" * _W}\r\n'
    '  r    record current joint positions\r\n'
    '  p    start playback\r\n'
    '  c    clear all waypoints\r\n'
    '  ESC  abort playback\r\n'
    f'{"─" * _W}\r\n'
)

_PLAYBACK_BANNER = (
    f'\r\n{"─" * _W}\r\n'
    '  PLAYBACK MODE  —  master arm disabled\r\n'
    '  Press ESC to abort and return to TELEOP.\r\n'
    f'{"─" * _W}\r\n'
)


class _State:
    TELEOP = 'TELEOP'
    PLAYBACK = 'PLAYBACK'


class TeachAndPlayJointsNode(Node):

    def __init__(self):
        super().__init__('teach_and_play_joints')

        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('max_waypoints', 10)
        self.declare_parameter('num_joints', 5)

        self._tol      = self.get_parameter('tolerance').value
        self._max_wp   = self.get_parameter('max_waypoints').value
        self._n        = self.get_parameter('num_joints').value

        self._pub = self.create_publisher(Packet, '/alpha/tx', QUEUE_SIZE)
        self.create_subscription(Packet,     '/master/rx',         self._on_master,       QUEUE_SIZE)
        self.create_subscription(JointState, '/alpha/joint_states', self._on_joint_states, QUEUE_SIZE)

        self._state     = _State.TELEOP
        self._joints    = None   # list[float] len == self._n, radians
        self._waypoints = []
        self._wp_index  = 0
        self._inline    = False

        self._running   = True
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.create_timer(1.0 / LOOP_HZ, self._tick)

        sys.stdout.write(_STARTUP_BANNER)
        sys.stdout.flush()

    # ── output helpers ───────────────────────────────────────────────────────

    def _println(self, msg: str):
        prefix = '\r\n' if self._inline else ''
        sys.stdout.write(f'{prefix}{msg}\r\n')
        sys.stdout.flush()
        self._inline = False

    def _print_inline(self, msg: str):
        sys.stdout.write(f'\r{msg:<{_W}}')
        sys.stdout.flush()
        self._inline = True

    def _warn(self, msg: str):
        self._println(f'  [!] {msg}')

    # ── subscriptions ────────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        self._joints = [pos.get(f'joint_{i}', 0.0) for i in range(1, self._n + 1)]

    def _on_master(self, packet: Packet):
        if self._state != _State.TELEOP:
            return
        if packet.packet_id not in CONTROL_PACKET_IDS:
            return
        self._pub.publish(packet)

    # ── timer ────────────────────────────────────────────────────────────────

    def _tick(self):
        if self._state != _State.PLAYBACK:
            return
        if self._joints is None:
            return

        target = self._waypoints[self._wp_index]
        self._send_joint_positions(target)

        error = max(abs(target[i] - self._joints[i]) for i in range(self._n))
        total = len(self._waypoints)
        self._print_inline(f'  wp {self._wp_index}/{total - 1}  —  max err {error:.4f} rad')

        if error < self._tol:
            self._wp_index += 1
            if self._wp_index >= total:
                self._finish_playback('Playback complete — returning to TELEOP.')
                return
            self._println(f'  wp {self._wp_index - 1} reached — advancing to wp {self._wp_index}')

    # ── commands ─────────────────────────────────────────────────────────────

    def _set_mode(self, mode: int):
        p = Packet()
        p.device_id = 0xFF
        p.packet_id = PacketID.MODE
        p.int_data  = [mode]
        self._pub.publish(p)

    def _send_joint_positions(self, positions: list):
        for i, angle in enumerate(positions):
            p = Packet()
            p.device_id  = i + 1          # joint_1 → device_id 0x01, etc.
            p.packet_id  = PacketID.POSITION
            p.float_data = [angle]
            self._pub.publish(p)

    def _finish_playback(self, msg: str):
        self._set_mode(Mode.POSITION_HOLD)
        self._state = _State.TELEOP
        self._println(f'\r\n{"─" * _W}\r\n  {msg}\r\n{"─" * _W}')

    # ── keyboard ─────────────────────────────────────────────────────────────

    def _on_key(self, key: str):
        if key == 'r':
            if self._state != _State.TELEOP:
                return
            if self._joints is None:
                self._warn('No joint state received yet — is joint_telemetry running?')
                return
            if len(self._waypoints) >= self._max_wp:
                self._warn(f'Maximum waypoints ({self._max_wp}) already recorded.')
                return
            self._waypoints.append(list(self._joints))
            idx = len(self._waypoints) - 1
            angles = '  '.join(f'j{i + 1} {self._joints[i]:7.4f}' for i in range(self._n))
            self._println(f'  [wp {idx}]  {angles}  [{len(self._waypoints)}/{self._max_wp}]')

        elif key == 'p':
            if self._state != _State.TELEOP:
                return
            if not self._waypoints:
                self._warn('No waypoints recorded — press r to record positions first.')
                return
            self._wp_index = 0
            self._set_mode(Mode.POSITION)
            self._state = _State.PLAYBACK
            sys.stdout.write(_PLAYBACK_BANNER)
            sys.stdout.flush()
            self._inline = False

        elif key == 'c':
            if self._state != _State.TELEOP:
                self._warn('Cannot clear waypoints during playback.')
                return
            count = len(self._waypoints)
            self._waypoints.clear()
            self._println(f'  Cleared {count} waypoint(s).')

        elif key == 'ESC':
            if self._state == _State.PLAYBACK:
                self._finish_playback('Playback aborted — returning to TELEOP.')

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        if not os.isatty(fd):
            self._warn('stdin is not a TTY — run with: ros2 run rs_passthrough teach_and_play_joints')
            return
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running:
                if not select.select([sys.stdin], [], [], 0.1)[0]:
                    continue
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        sys.stdin.read(2)
                    self._on_key('ESC')
                elif ch == '\x03':
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
    node = TeachAndPlayJointsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
