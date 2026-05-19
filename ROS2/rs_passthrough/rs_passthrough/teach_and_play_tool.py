import math
import os
import select
import signal
import sys
import termios
import threading
import tty

import numpy as np
import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import PacketID, Mode

from .tool_pose_and_vel import tool_position_from_gripper_pose, ypr_to_rotation

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

_W = 56  # banner width

_PLAYBACK_BANNER = (
    f'\r\n{"─" * _W}\r\n'
    '  PLAYBACK MODE  —  master arm disabled\r\n'
    '  Press ESC to abort and return to TELEOP.\r\n'
    f'{"─" * _W}\r\n'
)


def _startup_banner(p_tool: np.ndarray) -> str:
    return (
        f'\r\n{"─" * _W}\r\n'
        '  Teach and Play (tool frame)  —  TELEOP\r\n'
        f'{"─" * _W}\r\n'
        f'  Tool offset (gripper frame):  '
        f'X {p_tool[0]:.1f}  Y {p_tool[1]:.1f}  Z {p_tool[2]:.1f} mm\r\n'
        f'{"─" * _W}\r\n'
        '  r    record current position\r\n'
        '  p    start playback\r\n'
        '  c    clear all waypoints\r\n'
        '  ESC  abort playback\r\n'
        f'{"─" * _W}\r\n'
    )


class _State:
    TELEOP = 'TELEOP'
    MOVE_TO_FIRST = 'MOVE_TO_FIRST'
    PLAYBACK = 'PLAYBACK'


class TeachAndPlayToolNode(Node):

    def __init__(self):
        super().__init__('teach_and_play_tool')

        self.declare_parameter('speed', 10.0)
        self.declare_parameter('tolerance', 5.0)
        self.declare_parameter('max_waypoints', 10)
        self.declare_parameter('max_angular_rate', 0.3)
        self.declare_parameter('k_orient', 1.0)
        # Tool tip offset expressed in the gripper frame (mm).
        self.declare_parameter('tool_x', 0.0)
        self.declare_parameter('tool_y', 0.0)
        self.declare_parameter('tool_z', 0.0)

        self._speed    = self.get_parameter('speed').value
        self._tol      = self.get_parameter('tolerance').value
        self._max_wp   = self.get_parameter('max_waypoints').value
        self._max_ang  = self.get_parameter('max_angular_rate').value
        self._k_orient = self.get_parameter('k_orient').value
        self._p_tool   = np.array([
            self.get_parameter('tool_x').value,
            self.get_parameter('tool_y').value,
            self.get_parameter('tool_z').value,
        ])

        self._pub = self.create_publisher(Packet, '/alpha/tx', QUEUE_SIZE)
        self.create_subscription(Packet, '/master/rx', self._on_master, QUEUE_SIZE)
        self.create_subscription(Packet, '/alpha/rx',  self._on_alpha,  QUEUE_SIZE)

        self._state     = _State.TELEOP
        self._pos       = None   # [x, y, z, yaw, pitch, roll]  mm + rad
        self._waypoints = []     # list of 6-element lists (gripper 6DOF)
        self._wp_index  = 0
        self._inline    = False  # True while the current line is being overwritten in place

        self._running   = True
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.create_timer(1.0 / LOOP_HZ, self._tick)

        #self._set_velocity_limits(self._max_ang*0.001)
        
        sys.stdout.write(_startup_banner(self._p_tool))
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
            dist = self._tool_dist(self._pos, target)
            self._print_inline(f'  Moving to wp 0 ...  {dist:.1f} mm remaining (tool)')
            if dist < self._tol:
                if len(self._waypoints) == 1:
                    self._finish_playback('Playback complete — returning to TELEOP.')
                else:
                    self._wp_index = 1
                    self._set_mode(Mode.KINEMATIC_VELOCTIY_BASE_FRAME)
                    self._state = _State.PLAYBACK
                    self._println(
                        f'  wp 0 reached — streaming {len(self._waypoints) - 1} '
                        f'remaining waypoint(s) at {self._speed:.0f} mm/s (tool)'
                    )

        elif self._state == _State.PLAYBACK:
            if self._pos is None:
                return
            target = self._waypoints[self._wp_index]
            dist = self._tool_dist(self._pos, target)
            total = len(self._waypoints)
            self._print_inline(f'  wp {self._wp_index}/{total - 1}  —  {dist:.1f} mm remaining (tool)')
            if dist < self._tol:
                self._wp_index += 1
                if self._wp_index >= total:
                    self._finish_playback('Playback complete — returning to TELEOP.')
                    return
                self._println(f'  wp {self._wp_index - 1} reached — advancing to wp {self._wp_index}')
                target = self._waypoints[self._wp_index]
            self._stream_velocity(target)
    # ── commands ─────────────────────────────────────────────────────────────

    def _set_velocity_limits(self, max_vel: float, min_vel: float = 0.0):
        for device_id in range(0x01, 0x06):  # 0x01 to 0x05
            p = Packet()
            p.device_id  = device_id
            p.packet_id  = PacketID.VELOCITY_LIMITS
            p.float_data = [max_vel, min_vel]
            self._pub.publish(p)


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
        # ── desired tool-tip velocity ────────────────────────────────────────
        p_tool_cur = self._tool_pos(self._pos)
        p_tool_tgt = self._tool_pos(target)
        delta = p_tool_tgt - p_tool_cur
        dist = np.linalg.norm(delta)

        v_tool = (delta / dist) * self._speed if dist > 1e-6 else np.zeros(3)

        # ── lever arm in base frame ──────────────────────────────────────────
        R = ypr_to_rotation(self._pos[3], self._pos[4], self._pos[5])
        r = R @ self._p_tool

        # ── angular velocity toward target orientation ────────────────────────
        # Computed in packet order [yaw, pitch, roll] (pose indices 3-5).
        # Base-frame Cartesian [X, Y, Z] = [roll, pitch, yaw] = reversed.
        def _wrap(a: float) -> float:
            return math.atan2(math.sin(a), math.cos(a))

        omega_raw = np.array([
            self._k_orient * _wrap(target[i] - self._pos[i])
            for i in range(3, 6)
        ])

        # Scale omega so the lever-arm contribution (omega × r) stays within
        # 20% of tool speed. Without this, large tool offsets and orientation
        # errors produce corrections that overwhelm and reverse the translational
        # command. The formula v_gripper = v_tool - omega×r remains exact —
        # only the rate at which orientation is corrected is reduced.
        omega_cartesian_raw = omega_raw[::-1]  # packet order → Cartesian [X, Y, Z]
        correction_raw = np.cross(omega_cartesian_raw, r)
        correction_speed = np.linalg.norm(correction_raw)
        max_correction = self._speed * 0.2

        if correction_speed > max_correction and correction_speed > 1e-6:
            omega_scale = max_correction / correction_speed
        else:
            omega_scale = 1.0

        # Also enforce max_angular_rate
        omega_pkt_unclipped = omega_raw * omega_scale
        max_component = np.max(np.abs(omega_pkt_unclipped))
        if max_component > self._max_ang:
            omega_scale *= self._max_ang / max_component

        omega_pkt = omega_raw * omega_scale
        omega_cartesian = omega_pkt[::-1]

        # ── back-compute required gripper velocity ───────────────────────────
        # v_tool = v_gripper + omega × r  →  v_gripper = v_tool - omega × r
        correction = np.cross(omega_cartesian, r)
        v_gripper = v_tool - correction

        p = Packet()
        p.device_id  = ALPHA_BASE_ID
        p.packet_id  = PacketID.INVERSE_KINEMATICS_GLOBAL_VELOCITY
        p.float_data = list(v_gripper) + list(omega_pkt)
        self._pub.publish(p)

    def _finish_playback(self, msg: str):
        p = Packet()
        p.device_id  = ALPHA_BASE_ID
        p.packet_id  = PacketID.INVERSE_KINEMATICS_GLOBAL_VELOCITY
        p.float_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self._pub.publish(p)
        self._set_mode(Mode.POSITION_HOLD)
        self._state = _State.TELEOP
        self._println(f'\r\n{"─" * _W}\r\n  {msg}\r\n{"─" * _W}')

    # ── helpers ──────────────────────────────────────────────────────────────

    def _tool_pos(self, gripper_pose: list) -> np.ndarray:
        """Tool tip position in base frame from gripper 6DOF pose."""
        return tool_position_from_gripper_pose(
            gripper_pose[:3], gripper_pose[3], gripper_pose[4], gripper_pose[5],
            self._p_tool,
        )


    def _tool_dist(self, pose_a: list, pose_b: list) -> float:
        """Distance between tool tip positions for two gripper poses."""
        return float(np.linalg.norm(self._tool_pos(pose_a) - self._tool_pos(pose_b)))

    def _tool_dist_separate(self, pose_a: list, pose_b: list) -> tuple[float, float]:
        """Separate translational and rotational distance between two gripper poses."""
        
        translation_error = np.linalg.norm(np.array(pose_a[:3]) - np.array(pose_b[:3]))
        rotation_error = np.linalg.norm(np.array(pose_a[3:]) - np.array(pose_b[3:]))
        return translation_error, rotation_error    
        



    # ── keyboard ─────────────────────────────────────────────────────────────

    def _on_key(self, key: str):
        if key == 'r':
            if self._state != _State.TELEOP:
                return
            if self._pos is None:
                self._warn('No position available yet — is the alpha passthrough running?')
                return
            if len(self._waypoints) >= self._max_wp:
                self._warn(f'Maximum waypoints ({self._max_wp}) already recorded.')
                return
            self._waypoints.append(list(self._pos))
            idx = len(self._waypoints) - 1
            x, y, z, yaw, pitch, roll = self._pos
            tx, ty, tz = self._tool_pos(self._pos)
            self._println(
                f'  [wp {idx}]  gripper  '
                f'X {x:8.1f}  Y {y:8.1f}  Z {z:8.1f} mm'
                f'  [{len(self._waypoints)}/{self._max_wp}]\r\n'
                f'           tool     '
                f'X {tx:8.1f}  Y {ty:8.1f}  Z {tz:8.1f} mm\r\n'
                f'           orient   '
                f'yaw {yaw:7.3f}  pitch {pitch:7.3f}  roll {roll:7.3f} rad'
            )

        elif key == 'p':
            if self._state != _State.TELEOP:
                return
            if not self._waypoints:
                self._warn('No waypoints recorded — press r to record positions first.')
                return
            self._wp_index = 0
            self._set_mode(Mode.KINEMATIC_POSITION_BASE_FRAME)
            self._state = _State.MOVE_TO_FIRST
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
            if self._state in (_State.MOVE_TO_FIRST, _State.PLAYBACK):
                self._finish_playback('Playback aborted — returning to TELEOP.')

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        if not os.isatty(fd):
            self._warn('stdin is not a TTY — run with: ros2 run rs_passthrough teach_and_play_tool')
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
    node = TeachAndPlayToolNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
