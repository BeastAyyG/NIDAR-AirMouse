# airmouse_sensors

Data-only ROS 2 package: the Gazebo -> ROS bridge mapping for the AirMouse
X500 RGB-D payload, plus a standalone bridge launch file.  It ships no
algorithm code.

## Topic map (`config/gz_bridge.yaml`)

The simulated X500 (`airmouse_sim/models/airmouse_x500_rgbd`) exposes one
RGB-D sensor with `<topic>camera</topic>` and one IMU with `<topic>imu</topic>`.
Gazebo Harmonic appends the stream names to the sensor topic, so the bridge
below is the only place where roadmap ROS names are produced.

| ROS topic | gz topic | ROS type | gz type | Direction |
|---|---|---|---|---|
| `/clock` | `/clock` | `rosgraph_msgs/Clock` | `gz.msgs.Clock` | GZ -> ROS |
| `/camera/color/image_raw` | `/camera/image` | `sensor_msgs/Image` | `gz.msgs.Image` | GZ -> ROS |
| `/camera/depth/image_rect_raw` | `/camera/depth_image` | `sensor_msgs/Image` | `gz.msgs.Image` | GZ -> ROS |
| `/camera/color/camera_info` | `/camera/camera_info` | `sensor_msgs/CameraInfo` | `gz.msgs.CameraInfo` | GZ -> ROS |
| `/camera/depth/camera_info` | `/camera/depth_camera_info` | `sensor_msgs/CameraInfo` | `gz.msgs.CameraInfo` | GZ -> ROS |
| `/camera/depth/color/points` | `/camera/points` | `sensor_msgs/PointCloud2` | `gz.msgs.PointCloudPacked` | GZ -> ROS |
| `/camera/imu` | `/imu` | `sensor_msgs/Imu` | `gz.msgs.IMU` | GZ -> ROS |

Sensor characteristics authored in the model: 640x480 RGB-D at 20 Hz,
depth clip 0.2-8.0 m, gaussian noise stddev 0.007 (0.03 in the `_noisy`
variant), IMU at 200 Hz with gaussian noise.  All streams are one-way: the
autonomy stack never commands the sensors.

## Launch

```bash
# Bridge only (Gazebo must already be running and may be remote)
ros2 launch airmouse_sensors sensors.launch.py

# Bridge plus Gazebo plus all autonomy nodes
ros2 launch airmouse_sim mission.launch.py
```

`sim_bringup.launch.py` starts the same bridge, so do not run both bridge
launches against one simulator (duplicate subscriptions on the ROS side).

`bridge_config` accepts a bare filename resolved inside the installed
`share/airmouse_sensors/config` directory, or an absolute path.  The config
is passed to `ros_gz_bridge/parameter_bridge` through its `config_file`
parameter:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$(ros2 pkg prefix airmouse_sensors)/share/airmouse_sensors/config/gz_bridge.yaml
```

If `config_file` is unavailable on an older bridge build, start the bridge
manually with the equivalent positional arguments, e.g.
`/camera/image@sensor_msgs/msg/Image[gz.msgs.Image`.

## PX4 SITL

PX4 SITL is launched **separately** from this workspace, exactly as in the
existing AirMouse setup (PX4 + Micro XRCE-DDS Agent + `px4` gz model).  These
packages only provide the scenario worlds, the sensor rig and the gz -> ROS
bridge; they do not start, configure or command PX4.  `gz sim` and PX4 must
share the same simulator instance and the same `GZ_SIM_RESOURCE_PATH`
(`airmouse_sim/launch/sim_bringup.launch.py` prepends
`share/airmouse_sim/models` for `model://` resolution).

## Runtime note (UNVERIFIED-OFFLINE)

This package was authored on a Windows box without ROS 2 / Gazebo; all XML/YAML
and Python launch files are parse-verified offline, but the bridge itself has
not been executed.  Verify on the Ubuntu 22.04 + Gazebo Harmonic machine with
`ros2 topic list` / `ros2 topic hz /camera/color/image_raw`.
