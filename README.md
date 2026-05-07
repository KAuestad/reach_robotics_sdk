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

## Teach and Play

The `teach_and_play` node lets you teleoperate the Alpha 5 with a master arm, record end-effector poses during operation, and replay them in sequence at a steady continuous speed.

### Prerequisites

- Two serial passthroughs must be running: one for the master arm, one for the Alpha.
- The `teach_and_play` node must be run in its own terminal so it can read keyboard input.

### Starting up

**Terminal 1** — start the serial passthroughs:

```bash
source install/setup.bash
ros2 launch rs_passthrough teach_and_play.launch.py \
  master_port:=/dev/ttyUSB0 \
  alpha_port:=/dev/ttyUSB1
```

**Terminal 2** — start the teach and play node:

```bash
source install/setup.bash
ros2 run rs_passthrough teach_and_play
```

Replace `/dev/ttyUSB0` and `/dev/ttyUSB1` with the serial ports for your hardware.

### Keyboard controls

| Key | Action |
|-----|--------|
| `r` | Record the current end-effector pose as the next waypoint (up to 10) |
| `p` | Start playback of all recorded waypoints |
| `c` | Clear all recorded waypoints |
| `ESC` | Abort playback and return to teleoperation |
| `Ctrl+C` | Shut down the node |

### Workflow

1. **Teleoperate** the arm to the desired positions using the master arm.
2. **Press `r`** at each position you want to save. The terminal will confirm each recorded waypoint with its index, XYZ position (mm), and orientation (rad). Up to 10 waypoints can be recorded.
3. **Press `p`** to start playback. The node prints a banner, disables master arm input, and begins executing the sequence:
   - The arm moves to the first waypoint under position control.
   - Once within 5 mm, it switches to velocity streaming and traverses all remaining waypoints at 10 mm/s without stopping between them.
   - On completion the arm holds its final position and teleoperation resumes automatically.
4. **Press `ESC`** at any time during playback to stop immediately, hold position, and return to teleoperation.
5. **Press `c`** to clear all waypoints and start a new recording session.

### Parameters

The following parameters can be set at launch via `--ros-args -p <name>:=<value>`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `speed` | `10.0` | Playback translation speed (mm/s) |
| `tolerance` | `5.0` | Distance threshold to consider a waypoint reached (mm) |
| `max_waypoints` | `10` | Maximum number of waypoints that can be recorded |
| `max_angular_rate` | `0.3` | Maximum orientation correction rate during playback (rad/s) |

Example — increase speed to 20 mm/s:

```bash
ros2 run rs_passthrough teach_and_play --ros-args -p speed:=20.0
```

### Notes

- Waypoints store the full 6-DOF pose (XYZ + yaw/pitch/roll). Orientation is tracked during playback using proportional control toward the recorded orientation at each waypoint.
- Waypoints are held in memory only. They are lost when the node is shut down.
- The master arm is fully ignored while playback is active.

## Documentation

Detailed SDK documentation is available on our GitHub pages
https://reach-robotics.github.io/reach_robotics_sdk/. All Reach Robotics product documentation can be downloaded from our [website](https://reachrobotics.com/resources/knowledge-hub/?tab=datasheets-and-manuals/).

## License

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), limited exclusively to use with products produced by Reach Robotics Pty Ltd, subject to the following conditions:

1. The Software may only be used in conjunction with products manufactured or developed by Reach Robotics Pty Ltd.

2. Redistributions or use of the Software in any other context, including but not limited to, integration, combination, or use with other products or software, are strictly prohibited without prior written authorization from Reach Robotics Pty Ltd.

3. All copies of the Software, in whole or in part, must retain this notice and the above copyright notice.

THIS SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL REACH ROBOTICS PTY LTD BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
