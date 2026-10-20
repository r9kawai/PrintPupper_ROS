# pp_champ_balance_controller (Stage 2)

ROS Melodic / Ubuntu 18.04 catkin package. Tested Stage-1 behavior is preserved with `enable_balance:=false` (default). No servo safety is guaranteed by software.

## Build

```bash
cd ~/rosws1
catkin_make --only-pkg-with-deps pp_champ_balance_controller
source devel/setup.bash
```

## Launch

First start CHAMP and your already verified `balance_teleop.launch` (the latter must remap `body_pose` to `/body_pose_manual`). Do not start the original teleop concurrently.

```bash
roslaunch pp_champ_balance_controller relay.launch
# Only after validating the baseline and with servos DISABLED:
roslaunch pp_champ_balance_controller relay.launch enable_balance:=true
```

Do NOT run two instances of relay.launch at once. Stop the previous relay process before changing mode. `enable_balance` is read only on startup (not dynamically reconfigured).

## Interfaces

- Subscribe: `/body_pose_manual` (`geometry_msgs/Pose`), `/imu/data` (`sensor_msgs/Imu`)
- Publish: `/body_pose` (`geometry_msgs/Pose`)
- `false`: publishes each manual Pose immediately and **unchanged**.
- `true`: outputs at `control_rate_hz` when a manual Pose has been received. If a fresh, valid IMU orientation is unavailable, publishes unmodified manual Pose.
- CHAMP's manual position and yaw fields are kept unchanged. Roll/Pitch are changed by bounded P feedback.
- The correction is `clamp(Kp*(manual_angle - imu_angle), +/- max_correction_deg)`, then rate-limited. Angles are radians internally; limits in config are degrees.
- `imu_timeout_sec` checks ROS receipt time, not `header.stamp`; delayed-but-still-arriving sensor packets cannot be detected this way.
- `orientation_covariance[0] == -1` indicates that orientation is absent.
- Output quaternion is normalized. IMU yaw is not used.

## Important precautions

- First test **with servo power disabled**. Watch `/body_pose` and the joint command topic, and physically tilt the IMU to verify correct *direction*. The chosen feedback sign is provisional and depends on the CHAMP/IMU physical conventions.
- `/body_pose` is an IK body pose *command*, not a physical body position estimate. The RViz visualization of physical tilt depends on TF/EKF configuration.
- Keep only **one publisher** on `/body_pose`.
- Pose messages have no timestamp. This node does not currently timeout manual input; it retains the latest manual Pose.
- If IMU is lost, it immediately returns to the unmodified manual Pose; evaluate this transition with servos disabled before moving to physical testing.
- This does not implement stance detection, contact control or dynamic walking balance; validate with stationary four-foot contact before trying movement.
