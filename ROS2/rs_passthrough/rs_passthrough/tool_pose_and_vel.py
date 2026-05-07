import numpy as np
from numpy.linalg import norm


def rot_x(phi: float) -> np.ndarray:
    """Rotation about X-axis by roll phi [rad]."""
    c, s = np.cos(phi), np.sin(phi)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c],
    ])


def rot_y(alpha: float) -> np.ndarray:
    """Rotation about Y-axis by pitch alpha [rad]."""
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c],
    ])


def rot_z(theta: float) -> np.ndarray:
    """Rotation about Z-axis by yaw theta [rad]."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ])


def ypr_to_rotation(theta: float, alpha: float, phi: float) -> np.ndarray:
    """
    Convert extrinsic Yaw-Pitch-Roll to rotation matrix.

    theta = yaw   [rad], about global/base Z
    alpha = pitch [rad], about global/base Y
    phi   = roll  [rad], about global/base X

    For extrinsic Z-Y-X rotations:
        R_base_gripper = Rz(theta) @ Ry(alpha) @ Rx(phi)
    """
    return rot_z(theta) @ rot_y(alpha) @ rot_x(phi)


def tool_position_from_gripper_pose(
    p_base_gripper: np.ndarray,
    theta: float,
    alpha: float,
    phi: float,
    p_gripper_tool: np.ndarray,
) -> np.ndarray:
    """
    Calculates:
        P_base_tool = P_base_gripper + R_base_gripper @ P_gripper_tool
    """
    p_base_gripper = np.asarray(p_base_gripper, dtype=float).reshape(3)
    p_gripper_tool = np.asarray(p_gripper_tool, dtype=float).reshape(3)

    R_base_gripper = ypr_to_rotation(theta, alpha, phi)

    p_base_tool = p_base_gripper + R_base_gripper @ p_gripper_tool

    return p_base_tool

def tool_velocity_from_gripper_pose_and_vel(
    p_base_gripper: np.ndarray,
    theta: float,
    alpha: float,
    phi: float,
    p_gripper_tool: np.ndarray,
    v_base_gripper: np.ndarray,
    omega_base_gripper: np.ndarray,
) -> np.ndarray:
    """
    Calculates:
        v_base_tool = v_base_gripper + omega_base_gripper x (R_base_gripper @ P_gripper_tool)
    """
    p_base_gripper = np.asarray(p_base_gripper, dtype=float).reshape(3)
    p_gripper_tool = np.asarray(p_gripper_tool, dtype=float).reshape(3)
    v_base_gripper = np.asarray(v_base_gripper, dtype=float).reshape(3)
    omega_base_gripper = np.asarray(omega_base_gripper, dtype=float).reshape(3)

    R_base_gripper = ypr_to_rotation(theta, alpha, phi)

    # Linear velocity of tool due to gripper's linear velocity
    v_linear = v_base_gripper

    # Linear velocity of tool due to gripper's angular velocity
    r_tool_from_gripper = R_base_gripper @ p_gripper_tool
    v_angular = np.cross(omega_base_gripper, r_tool_from_gripper)

    v_base_tool = v_linear + v_angular

    return v_base_tool


if __name__ == "__main__":
    # -------------------------------
    # Example gripper pose from robot
    # -------------------------------
    # Position of gripper in base frame, e.g. mm
    P_base_gripper = np.array([200.0, 50.0, 100.0])

    # Orientation of gripper in radians
    theta = np.deg2rad(0.0)   # yaw
    alpha = np.deg2rad(0.0)   # pitch
    phi = np.deg2rad(0.0)      # roll


    #Velocities
    v_base_gripper_lin= np.array([10.0, 0.0, 0.0]) # mm/s
    omega_base_gripper_ang = np.array([0.0, 0.0, np.deg2rad(0)]) # rad/s

    # Tool offset expressed in gripper frame, e.g. mm
    # Example: tool tip is 120 mm along gripper local X-axis
    P_gripper_tool = np.array([120.0, 0.0, 0.0])

    # Calculate tool position in base frame
    P_base_tool = tool_position_from_gripper_pose(
        P_base_gripper,
        theta,
        alpha,
        phi,
        P_gripper_tool,
    )
    v_base_tool = tool_velocity_from_gripper_pose_and_vel(P_base_gripper, theta, alpha, phi, P_gripper_tool, v_base_gripper_lin, omega_base_gripper_ang)

    print("P_base_gripper:", P_base_gripper)
    print("P_gripper_tool:", P_gripper_tool)
    print("P_base_tool:", P_base_tool)
    print("Distance gripper-to-tool:", norm(P_base_tool - P_base_gripper))
    
    print("v_base_gripper_lin:", v_base_gripper_lin)
    print("omega_base_gripper_ang:", omega_base_gripper_ang)
    print("v_base_tool:", v_base_tool)