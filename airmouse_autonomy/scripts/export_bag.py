"""Export a rosbag2 recording to an airmouse-clip directory.

RUNS ON THE ROS MACHINE ONLY (needs rclpy + rosbag2_py). The renderer itself
does not need ROS; this is the one bridge step from bag to clip.

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    python airmouse_autonomy/scripts/export_bag.py bags/mission_xxx clip_out

Default topic set (override with flags):

    --rgb        /camera/color/image_raw        sensor_msgs/Image   (rgb8/bgr8)
    --depth      /camera/depth/image_rect_raw   sensor_msgs/Image   (32FC1/16UC1)
    --info       /camera/color/camera_info      sensor_msgs/CameraInfo
    --imu        /camera/imu                    sensor_msgs/Imu
    --map        /map                           nav_msgs/OccupancyGrid
    --tracks     /survivors                     airmouse_interfaces/SurvivorTrack
    --state      /mission_state                 std_msgs/String
    --status     /mission_status                airmouse_interfaces/MissionStatus
    --pose       /ground_truth/pose             geometry_msgs/PoseStamped
    --path       /path                          nav_msgs/Path
    --goal       /goal                          geometry_msgs/PoseStamped

Frame rule: freeze the timeline at ``--fps`` (default 10); for each frame pick
the most recent message of every stream at or before the frame time. Missing
streams stay absent and the renderer draws placeholders.

UNVERIFIED-OFFLINE: authored without ROS on the development machine. Validate
on the Ubuntu box (see docs/VIDEO-PIPELINE.md, "Manoj's computer").
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def _import_ros_modules():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    return rosbag2_py, deserialize_message, get_message


def _image_to_numpy(msg, depth: bool):
    h, w = int(msg.height), int(msg.width)
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if depth:
        if msg.encoding.lower() in ("32fc1", "32fc"):
            return np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(h, w)
        if msg.encoding.lower() in ("16uc1", "16u"):
            return (np.frombuffer(bytes(msg.data), dtype=np.uint16)
                    .reshape(h, w).astype(np.float32) / 1000.0)
        raise ValueError(f"unsupported depth encoding {msg.encoding!r}")
    raw = data.reshape(h, w, -1)
    if msg.encoding.lower() == "bgr8":
        raw = raw[:, :, ::-1]
    return np.ascontiguousarray(raw[:, :, :3])


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def export(bag_dir: str, out_dir: str, fps: float, width: int, height: int,
           topics: dict) -> Path:
    rosbag2_py, deserialize_message, get_message = _import_ros_modules()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_class = {name: get_message(kind) for name, kind in type_by_topic.items()}

    latest = {}      # topic -> last message
    tracks = {}      # survivor_id -> dict
    imu_buffer = []  # samples since the previous frame
    end_time = None
    start_time = None
    frames = []

    out = Path(out_dir)
    (out / "frames").mkdir(parents=True, exist_ok=True)
    (out / "maps").mkdir(parents=True, exist_ok=True)

    def topic(name):
        return topics.get(name)

    def snapshot(now_ns):
        nonlocal imu_buffer
        t = (now_ns - start_time) / 1e9
        frame = {"t": round(t, 3)}
        if topic("rgb") in latest:
            frame["_rgb"] = _image_to_numpy(latest[topic("rgb")], depth=False)
        if topic("depth") in latest:
            frame["_depth"] = _image_to_numpy(latest[topic("depth")], depth=True)
        if topic("pose") in latest:
            pose = latest[topic("pose")].pose
            yaw = _quaternion_yaw(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w)
            frame["pose"] = [pose.position.x, pose.position.y, yaw]
        if topic("goal") in latest:
            frame["goal"] = [latest[topic("goal")].pose.position.x,
                             latest[topic("goal")].pose.position.y]
        if topic("path") in latest:
            frame["path"] = [[p.pose.position.x, p.pose.position.y]
                             for p in latest[topic("path")].poses]
        if tracks:
            frame["tracks"] = list(tracks.values())
        if topic("state") in latest:
            frame["state"] = latest[topic("state")].data
        if topic("status") in latest:
            status = latest[topic("status")]
            frame["coverage_pct"] = float(status.explored_pct)
            frame["survivors"] = int(status.survivor_count)
        if imu_buffer:
            frame["imu"] = [[s.linear_acceleration.x, s.linear_acceleration.y,
                             s.linear_acceleration.z, s.angular_velocity.x,
                             s.angular_velocity.y, s.angular_velocity.z]
                            for s in imu_buffer[-20:]]
            imu_buffer = []
        frames.append(frame)

    next_frame_ns = None
    while reader.has_next():
        topic_name, raw, stamp_ns = reader.read_next()
        if start_time is None:
            start_time = stamp_ns
            next_frame_ns = stamp_ns
        cls = msg_class.get(topic_name)
        if cls is None:
            continue
        try:
            msg = deserialize_message(raw, cls)
        except Exception:
            continue
        latest[topic_name] = msg
        if topic_name == topic("imu"):
            imu_buffer.append(msg)
        elif topic_name == topic("tracks"):
            tracks[int(msg.survivor_id)] = {
                "id": int(msg.survivor_id), "x": float(msg.position_x),
                "y": float(msg.position_y), "confidence": float(msg.confidence),
                "state": str(msg.state),
            }
        while stamp_ns >= next_frame_ns:
            snapshot(next_frame_ns)
            next_frame_ns += int(round(1e9 / fps))
        end_time = stamp_ns

    if start_time is None or not frames:
        raise SystemExit(f"no usable messages in {bag_dir} for the requested topics")

    camera = {"fx": width, "fy": width, "cx": width / 2.0, "cy": height / 2.0,
              "depth_max_m": 8.0}
    info_topic = topic("info")
    if info_topic in latest:
        info = latest[info_topic]
        camera = {"fx": float(info.k[0]) or camera["fx"],
                  "fy": float(info.k[4]) or camera["fy"],
                  "cx": float(info.k[2]) or camera["cx"],
                  "cy": float(info.k[5]) or camera["cy"],
                  "depth_max_m": 8.0}
        width, height = int(info.width), int(info.height)

    # Save arrays + manifest frame entries (map every 5th frame to save space).
    manifest_frames = []
    for index, frame in enumerate(frames):
        entry = {k: v for k, v in frame.items() if not k.startswith("_")}
        if "_rgb" in frame:
            rel = f"frames/{index:06d}_rgb.npy"
            np.save(out / rel, np.asarray(frame["_rgb"], dtype=np.uint8))
            entry["rgb"] = rel
        if "_depth" in frame:
            rel = f"frames/{index:06d}_depth.npy"
            np.save(out / rel, np.asarray(frame["_depth"], dtype=np.float32))
            entry["depth"] = rel
        map_topic = topic("map")
        if index % 5 == 0 and map_topic in latest:
            grid = latest[map_topic]
            rel = f"maps/{index:06d}_map.npy"
            arr = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height,
                                                               grid.info.width)
            np.save(out / rel, arr)
            entry["map"] = rel
        manifest_frames.append(entry)

    manifest = {
        "format": "airmouse-clip/1",
        "fps": float(fps),
        "width": int(width),
        "height": int(height),
        "camera": camera,
        "map_cell_m": (float(latest[topic("map")].info.resolution)
                       if topic("map") in latest else 0.1),
        "map_origin": [0.0, 0.0],
        "meta": {"bag": str(bag_dir), "frames": len(manifest_frames)},
        "frames": manifest_frames,
    }
    with open(out / "clip.json", "w", encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=1)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("out", help="output clip directory")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--rgb", default="/camera/color/image_raw")
    parser.add_argument("--depth", default="/camera/depth/image_rect_raw")
    parser.add_argument("--info", default="/camera/color/camera_info")
    parser.add_argument("--imu", default="/camera/imu")
    parser.add_argument("--map", default="/map")
    parser.add_argument("--tracks", default="/survivors")
    parser.add_argument("--state", default="/mission_state")
    parser.add_argument("--status", default="/mission_status")
    parser.add_argument("--pose", default="/ground_truth/pose")
    parser.add_argument("--path", default="/path")
    parser.add_argument("--goal", default="/goal")
    args = parser.parse_args()
    topics = {"rgb": args.rgb, "depth": args.depth, "info": args.info,
              "imu": args.imu, "map": args.map, "tracks": args.tracks,
              "state": args.state, "status": args.status, "pose": args.pose,
              "path": args.path, "goal": args.goal}
    out = export(args.bag, args.out, args.fps, 640, 480, topics)
    print(f"clip written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())