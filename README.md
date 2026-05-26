[![Logo](.docsrc/source/images/Reach.Robotics-Logo.Navy-01.png)](#) 

The Reach Robotics SDK provides an implementation of the Reach System Protocol [V1.12.1] in Python and C, as well as a ROS 2 communication bridge. Additionally, the SDK provides some basic examples of how to use the Reach System Protocol to communicate with Reach Robotics hardware over serial and UDP connections. Follow the [instructions to get started](https://reach-robotics.github.io/reach_robotics_sdk/).

<div align="center">
    <img src=".docsrc/source/images/python-logo-only.png" style="width: 10%;">
    <img src=".docsrc/source/images/C_Logo.png" style="width: 10%;">
    <img src=".docsrc/source/images/ISO_C++_Logo.png" style="width: 10%;">
    <img src=".docsrc/source/images/Ros_logo.png" style="width: 42%;">
</div>

&nbsp;

> **Note:** Integration support is not included as part of this SDK. The software is supplied as-is, without warranty of any kind. Integration support may be purchased through one of Reach Robotics' support packages. However, all services are subject to availability. For more information contact our sales team (sales@reachrobotics.com).

>Reach Robotics provides the following support options:
>- Python/C/C++ - 1 hour of engineering support included with any core or comprehensive support package.
>- ROS2 Foxy - paid-for support only for Alpha 5, Bravo 5 and Bravo 7 standard configurations.  

>The prices of supplementary support packages are dependent upon the scope of the request.

## General System Info
Manip Cam:
IP:192.168.2.42
user:admin
pw: Stinger67
gst-launch-1.0 -v rtspsrc location="rtsp://admin:Stinger67@192.168.2.42:554" protocols=tcp latecy=100 ! application/x-rtp,media=video,encoding-name=H265 ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false


Manip Moxa: 192.168.2.75
user: NA
pw: Stinger67

## USB-port Setup

## See the name of the connected USB ports:

sudo dmesg | grep tty


### Create a new USB port with a more suitable name:
sudo ln -sf /dev/ttyr00 /dev/ttyUSB_ALPHA5
sudo chmod 666 /dev/ttyr00
ls -l /dev/ttyUSB_ALPHA5



## Teach and Play

Three variants are available:

| Node | Control space | Speed unit |
|------|--------------|-----------|
| `teach_and_play` | Cartesian — gripper flange | mm/s |
| `teach_and_play_tool` | Cartesian — tool tip | mm/s |
| `teach_and_play_joints` | Joint space | rad/s |

All nodes share the same keyboard controls and workflow. The sections below describe each variant.

### Prerequisites

- Two serial passthroughs must be running: one for the master arm, one for the Alpha.
- The teach-and-play node must be run in its own terminal so it can read keyboard input.

### Starting up

**Terminal 1** — start the serial passthroughs:

```bash
source install/setup.bash
ros2 launch rs_passthrough teach_and_play.launch.py \
  master_port:=/dev/ttyUSB0 \
  alpha_port:=/dev/ttyUSB_ALPHA5
```

**Terminal 2** — start the teach and play node (choose one):

```bash
# Gripper-frame control (no tool)
source install/setup.bash
ros2 run rs_passthrough teach_and_play

# Tool-tip control (see "Tool-Frame Variant" below for setting the tool offset)
source install/setup.bash
ros2 run rs_passthrough teach_and_play_tool --ros-args \
  -p tool_x:=120.0 -p tool_y:=0.0 -p tool_z:=0.0

# Joint-space control (see "Joint-Space Variant" below)
source install/setup.bash
ros2 run rs_passthrough teach_and_play_joints --ros-args \
  -p max_speed:=0.3 -p tolerance:=0.02
```

Replace `/dev/ttyUSB0` and `/dev/ttyUSB1` with the serial ports for your hardware.

### Keyboard controls

| Key | Action |
|-----|--------|
| `r` | Record the current pose as the next waypoint (up to 10) |
| `p` | Start playback of all recorded waypoints |
| `c` | Clear all recorded waypoints |
| `ESC` | Abort playback and return to teleoperation |
| `Ctrl+C` | Shut down the node |

### Workflow

1. **Teleoperate** the arm to the desired positions using the master arm.
2. **Press `r`** at each position you want to save. The terminal confirms each recorded waypoint with its index, XYZ position (mm), and orientation (rad). Up to 10 waypoints can be recorded.
3. **Press `p`** to start playback. The node prints a banner, disables master arm input, and begins executing the sequence:
   - For `teach_and_play` and `teach_and_play_tool`: the arm moves to the first waypoint under position control, then streams Cartesian velocity through all remaining waypoints at a constant speed.
   - For `teach_and_play_joints`: the arm streams per-joint velocity commands through all waypoints, with velocities scaled each tick so all joints arrive simultaneously.
   - On completion the arm holds its final position and teleoperation resumes automatically.
4. **Press `ESC`** at any time during playback to stop immediately, hold position, and return to teleoperation.
5. **Press `c`** to clear all waypoints and start a new recording session.

---

### Gripper-frame variant (`teach_and_play`)

The `teach_and_play` node controls speed at the gripper flange. Use this when no tool is attached or when the tool geometry does not need to be accounted for.

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `speed` | `10.0` | Playback translation speed at the gripper flange (mm/s) |
| `tolerance` | `5.0` | Distance threshold to consider a waypoint reached (mm) |
| `max_waypoints` | `10` | Maximum number of waypoints that can be recorded |
| `max_angular_rate` | `1.3` | Maximum orientation correction rate during playback (rad/s) |
| `k_orient` | `1.0` | Proportional gain for orientation correction |

Example — increase speed to 20 mm/s:

```bash
ros2 run rs_passthrough teach_and_play --ros-args -p speed:=20.0
```

---

### Tool-frame variant (`teach_and_play_tool`)

The `teach_and_play_tool` node applies a rigid-body transform so that the **tool tip** (not the gripper flange) moves at the configured constant speed. Waypoints and distance tolerance are also expressed in terms of the tool-tip position.

#### Setting the tool offset

The tool offset is the position of the tool tip expressed in the **gripper frame** (mm). Measure it with the gripper in a known orientation, or read it from the tool drawing:

```
             ┌──────────────────────────────────────┐
  Gripper    │   Z  ↑                               │  Tool tip
  origin ────┤      │     ←──── tool_x ────→       ├──── ×
             │      └──→ X                          │
             └──────────────────────────────────────┘
```

For example, if the tool tip is 120 mm along the gripper's local X-axis and centered on Y and Z:

```bash
ros2 run rs_passthrough teach_and_play_tool --ros-args \
  -p tool_x:=120.0 -p tool_y:=0.0 -p tool_z:=0.0
```

The startup banner confirms the configured offset:

```
────────────────────────────────────────────────────────
  Teach and Play (tool frame)  —  TELEOP
────────────────────────────────────────────────────────
  Tool offset (gripper frame):  X 120.0  Y 0.0  Z 0.0 mm
────────────────────────────────────────────────────────
```

When you press `r` to record a waypoint, both the gripper and tool-tip positions are printed so you can verify the offset is correct:

```
  [wp 0]  gripper  X   200.0  Y    50.0  Z   100.0 mm  [1/10]
           tool     X   320.0  Y    50.0  Z   100.0 mm
           orient   yaw   0.000  pitch   0.000  roll   0.000 rad
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `speed` | `10.0` | Playback translation speed at the tool tip (mm/s) |
| `tolerance` | `5.0` | Tool-tip distance threshold to consider a waypoint reached (mm) |
| `max_waypoints` | `10` | Maximum number of waypoints that can be recorded |
| `max_angular_rate` | `1.3` | Maximum orientation correction rate during playback (rad/s) |
| `k_orient` | `1.0` | Proportional gain for orientation correction |
| `tool_x` | `0.0` | Tool tip X offset in the gripper frame (mm) |
| `tool_y` | `0.0` | Tool tip Y offset in the gripper frame (mm) |
| `tool_z` | `0.0` | Tool tip Z offset in the gripper frame (mm) |

#### How the velocity transform works

At each control tick the node:

1. Computes the desired tool-tip velocity vector (pointing from current tool position toward target at `speed` mm/s).
2. Computes a proportional angular velocity toward the target orientation.
3. Back-computes the required gripper velocity using the lever-arm relationship:
   `v_gripper = v_tool − ω × (R · p_tool)`
   where `R` is the current gripper rotation matrix and `p_tool` is the tool offset.

The angular correction rate is automatically scaled so its lever-arm contribution stays within 20 % of the tool speed. This prevents large orientation errors from overwhelming the translational command when the tool offset is long.

---

---

### Joint-space variant (`teach_and_play_joints`)

The `teach_and_play_joints` node records and replays **individual joint angles** read from the `/alpha/joint_states` topic. During playback the node streams per-joint velocity commands so that all joints arrive at each waypoint simultaneously.

The `teach_and_play.launch.py` file already starts the `joint_telemetry` node in the `alpha` namespace, which publishes `/alpha/joint_states` at 20 Hz. No additional setup is required.

#### Synchronized motion

Playback uses `Mode.VELOCITY` (joint velocity streaming) rather than `Mode.POSITION`. At every 20 Hz tick the node:

1. Computes the remaining angular distance to the target for each joint.
2. Scales velocities so the joint with the largest remaining travel moves at `max_speed` and all others move proportionally slower — guaranteeing simultaneous arrival.
3. Recomputes the proportions from live joint state every tick, so the synchronisation self-corrects for any tracking error.

`VELOCITY_LIMITS` is set to `max_speed` at playback start as a hardware safety cap only — it does **not** drive the synchronisation (the hardware's internal position controller ignores `VELOCITY_LIMITS`, which is why velocity streaming is used instead).

On completion or ESC, zero-velocity commands are sent to all joints before switching to `POSITION_HOLD`.

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_speed` | `1.0` | Maximum joint velocity during playback (rad/s) |
| `tolerance` | `0.05` | Max joint error to consider a waypoint reached (rad) |
| `max_waypoints` | `10` | Maximum number of waypoints that can be recorded |
| `num_joints` | `5` | Number of joints to control (5 for Alpha 5, 7 for Bravo 7) |

Example:

```bash
ros2 run rs_passthrough teach_and_play_joints --ros-args \
  -p max_speed:=0.3 -p tolerance:=0.02 -p num_joints:=5
```

The startup banner confirms the configured speed:

```
────────────────────────────────────────────────────────
  Teach and Play (joint space)  —  TELEOP
────────────────────────────────────────────────────────
  r    record current joint positions
  p    start playback
  c    clear all waypoints
  ESC  abort playback
────────────────────────────────────────────────────────
  Max joint speed: 0.30 rad/s
────────────────────────────────────────────────────────
```

When you press `r` to record a waypoint, all joint angles are printed in radians:

```
  [wp 0]  j1  0.0000  j2 -0.7854  j3  1.5708  j4  0.0000  j5  0.0000  [1/10]
```

---

### Notes

- `teach_and_play` and `teach_and_play_tool` waypoints store a full 6-DOF Cartesian pose (XYZ + yaw/pitch/roll). Orientation is tracked during playback using proportional control.
- `teach_and_play_joints` waypoints store one angle per joint in radians.
- Waypoints are held in memory only. They are lost when the node is shut down.
- The master arm is fully ignored while playback is active.

---

## Pose Monitor

The `monitor_pose` node continuously displays the current gripper flange position and the tool-tip position side by side in the terminal, updated in place at 10 Hz.

```bash
source install/setup.bash
ros2 run rs_passthrough monitor_pose --ros-args \
  -p tool_x:=120.0 -p tool_y:=0.0 -p tool_z:=0.0
```

Example output:

```
────────────────────────────────────────────────────────────────────────────
  Pose Monitor   tool offset: X 120.0  Y 0.0  Z 0.0 mm
────────────────────────────────────────────────────────────────────────────
  Gripper  X   200.3  Y    49.8  Z   100.1 mm   yaw   0.002  pitch  -0.001  roll   0.000 rad
  Tool     X   320.2  Y    49.8  Z   100.1 mm
```

Set `tool_x`, `tool_y`, `tool_z` to the same values used with `teach_and_play_tool`. If no tool is attached, omit the parameters (defaults to zero offset, so Tool and Gripper positions will match).

## Documentation

Detailed SDK documentation is available on our GitHub pages
https://reach-robotics.github.io/reach_robotics_sdk/. All Reach Robotics product documentation can be downloaded from our [website](https://reachrobotics.com/resources/knowledge-hub/?tab=datasheets-and-manuals/).

## License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), limited exclusively to use with products produced by Reach Robotics Pty Ltd, subject to the following conditions:

1. The Software may only be used in conjunction with products manufactured or developed by Reach Robotics Pty Ltd.

2. Redistributions or use of the Software in any other context, including but not limited to, integration, combination, or use with other products or software, are strictly prohibited without prior written authorization from Reach Robotics Pty Ltd.

3. All copies of the Software, in whole or in part, must retain this notice and the above copyright notice.

THIS SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL REACH ROBOTICS PTY LTD BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
