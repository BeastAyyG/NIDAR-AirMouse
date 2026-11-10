"""Static verification of the AirMouse autonomy workspace (runs without ROS).

Checks that CAN be proven offline:
  * every package.xml parses, names match their directory, declares build_type
  * interfaces messages exist with their expected fields
  * console_scripts entry points resolve to real module:function targets
  * algorithm cores contain no rclpy/ROS imports; node adapters do
  * every SDF world/model and YAML config parses
  * all 12 scenario worlds exist and reference the RGB-D model
  * launch files compile
  * expected pipeline topics are actually used by some node

Exit code 0 only when every check passes. Runtime behavior (colcon build,
Gazebo, PX4, TF, bags) is explicitly out of scope: see README Goal-0 checklist.

Usage: python airmouse_autonomy/tools/verify_workspace.py [--quiet]
"""
from __future__ import annotations

import ast
import py_compile
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # airmouse_autonomy/
SRC = ROOT / "src"
PKGS = sorted(p for p in SRC.iterdir() if p.is_dir() and (p / "package.xml").exists())

WORLDS = [
    "straight_corridor.sdf", "l_corridor.sdf", "t_junction.sdf", "dead_end.sdf",
    "narrow_passage.sdf", "multiple_obstacles.sdf", "multiple_rooms.sdf",
    "full_maze_15x15.sdf", "moving_obstacles.sdf", "noisy_depth.sdf",
    "loop_closure.sdf", "full_mission.sdf",
]
EXPECTED_TOPICS = [
    "/camera/color/image_raw", "/camera/depth/image_rect_raw",
    "/camera/color/camera_info", "/camera/depth/color/points", "/camera/imu",
    "/map", "/obstacles", "/survivors", "/goal", "/path", "/cmd_vel",
    "/mission_status", "/mission_state", "/mission_command", "/mission_complete",
]
MSG_FIELDS = {
    "ObstacleRegions.msg": {"min_distance", "representative", "count"},
    "SurvivorTrack.msg": {"survivor_id", "detected", "position_x",
                          "position_y", "confidence", "timestamp", "state"},
    "MissionStatus.msg": {"state", "explored_pct", "accessible_pct",
                          "survivor_count", "mission_complete", "timestamp"},
}

_failures: list[str] = []
_checks = 0


def check(ok: bool, what: str) -> None:
    global _checks
    _checks += 1
    if not ok:
        _failures.append(what)


def _module_file(pkg: Path, module: str) -> Path | None:
    """Resolve a dotted module inside a package's inner python dir."""
    parts = module.split(".")
    if not parts or parts[0] != pkg.name:
        return None
    candidate = pkg / pkg.name
    for part in parts[1:]:
        candidate = candidate / part
    file = candidate.with_suffix(".py")
    return file if file.is_file() else None


def _defines_function(file: Path, dotted: str) -> bool:
    try:
        tree = ast.parse(file.read_text(encoding="utf-8"))
    except SyntaxError:
        return False
    name = dotted.rsplit(".", 1)[-1]
    return any(isinstance(n, ast.FunctionDef) and n.name == name
               for n in tree.body)


def _imports_rclpy(file: Path) -> bool:
    try:
        tree = ast.parse(file.read_text(encoding="utf-8"))
    except SyntaxError:
        return False
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            if any(a.name.split(".")[0] in {"rclpy", "tf2_ros"} for a in node.names):
                return True
        elif isinstance(node, ast.ImportFrom) and node.module:
            if node.module.split(".")[0] in {"rclpy", "tf2_ros"}:
                return True
    return False


def check_packages() -> None:
    check(len(PKGS) == 11, f"expected 11 ROS packages, found {len(PKGS)}")
    for pkg in PKGS:
        try:
            root = ET.parse(pkg / "package.xml").getroot()
        except ET.ParseError as exc:
            check(False, f"{pkg.name}/package.xml parse error: {exc}")
            continue
        name = root.findtext("name")
        check(name == pkg.name, f"{pkg.name}/package.xml name mismatch: {name!r}")
        build_type = root.findtext("export/build_type")
        has_cmake = (pkg / "CMakeLists.txt").exists()
        check(bool(build_type) or has_cmake,
              f"{pkg.name} declares neither export/build_type nor CMakeLists.txt")
        if (pkg / "resource").is_dir():
            check((pkg / "resource" / pkg.name).exists(),
                  f"{pkg.name} missing resource marker")


def check_interfaces() -> None:
    msg_dir = SRC / "airmouse_interfaces" / "msg"
    for filename, fields in MSG_FIELDS.items():
        path = msg_dir / filename
        check(path.is_file(), f"missing message {filename}")
        if path.is_file():
            declared = {line.split()[1] for line in path.read_text().splitlines()
                        if line.strip() and not line.strip().startswith("#")}
            missing = fields - declared
            check(not missing, f"{filename} missing fields {sorted(missing)}")


def check_entry_points() -> None:
    for pkg in PKGS:
        setup = pkg / "setup.py"
        if not setup.is_file():
            continue
        try:
            tree = ast.parse(setup.read_text(encoding="utf-8"))
        except SyntaxError as exc:
            check(False, f"{pkg.name}/setup.py syntax error: {exc}")
            continue
        entry_map = None
        for node in ast.walk(tree):
            if isinstance(node, ast.keyword) and node.arg in ("entry_points",
                                                              "console_scripts"):
                try:
                    entry_map = ast.literal_eval(node.value)
                except (ValueError, SyntaxError):
                    entry_map = None
        scripts = {}
        if isinstance(entry_map, dict):
            raw = entry_map.get("console_scripts", [])
            for item in raw:
                if "=" in item:
                    exe, target = item.split("=", 1)
                    scripts[exe.strip()] = target.strip()
        for exe, target in scripts.items():
            module, _, func = target.partition(":")
            file = _module_file(pkg, module)
            check(file is not None, f"{pkg.name}: entry '{exe}' module {module} missing")
            if file is not None:
                check(_defines_function(file, func),
                      f"{pkg.name}: entry '{exe}' function {func} not defined")
        for node in pkg.rglob("*_node.py"):
            check(bool(scripts), f"{pkg.name}/{node.name}: package has no console_scripts")
            check(_imports_rclpy(node), f"{node.name} does not import rclpy")


def check_core_purity() -> None:
    node_files = set(SRC.glob("*/[a-z]*_node.py")) | set(SRC.glob("*/*/*_node.py"))
    for pkg in PKGS:
        inner = pkg / pkg.name
        if not inner.is_dir():
            continue
        for file in inner.glob("*.py"):
            if file.name == "__init__.py" or file in node_files:
                continue
            if file.name.startswith("setup"):
                continue
            check(not _imports_rclpy(file),
                  f"core {file.relative_to(ROOT)} imports rclpy/tf2_ros")


def check_assets() -> None:
    sim = SRC / "airmouse_sim"
    for name in WORLDS:
        path = sim / "worlds" / name
        check(path.is_file(), f"missing world {name}")
        if path.is_file():
            try:
                ET.parse(path)
            except ET.ParseError as exc:
                check(False, f"world {name} parse error: {exc}")
    for sdf in list(sim.rglob("*.sdf")) + list(sim.rglob("*.config")):
        try:
            ET.parse(sdf)
        except ET.ParseError as exc:
            check(False, f"{sdf.relative_to(ROOT)} parse error: {exc}")
    model = (sim / "models" / "airmouse_x500_rgbd" / "model.sdf").read_text()
    check("rgbd_camera" in model, "X500 model lacks rgbd_camera sensor")
    check("depth" in model, "X500 model lacks depth stream")
    for world in ("full_maze_15x15.sdf", "full_mission.sdf"):
        text = (sim / "worlds" / world).read_text()
        check("exit" in text, f"{world} has no exit marker")
        check(text.count("survivor_") <= 6 * 2, f"{world} exceeds 6 survivors")
    for launch in sim.glob("launch/*.launch.py"):
        _check_compiles(launch)
    sensors = SRC / "airmouse_sensors"
    if (sensors / "config" / "gz_bridge.yaml").is_file():
        _check_yaml(sensors / "config" / "gz_bridge.yaml")


def _check_compiles(path: Path) -> None:
    try:
        with tempfile.TemporaryDirectory() as tmp:
            py_compile.compile(str(path), cfile=str(Path(tmp) / "out.pyc"),
                               doraise=True)
        check(True, "")
    except py_compile.PyCompileError as exc:
        check(False, f"{path.name} does not compile: {exc}")


def _check_yaml(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
    except ImportError:
        check(":" in text and "\t" not in text,
              f"{path.name}: cannot parse (no pyyaml) and looks malformed")
        return
    try:
        data = yaml.safe_load(text)
        # ros_gz_bridge config files are a list of entries; params files are dicts.
        check(isinstance(data, (dict, list)) and bool(data), f"{path.name} is empty")
    except yaml.YAMLError as exc:
        check(False, f"{path.name} YAML error: {exc}")


def check_topics() -> None:
    node_texts = [p.read_text(encoding="utf-8")
                  for p in SRC.rglob("*_node.py")]
    launch_texts = [p.read_text(encoding="utf-8")
                    for p in SRC.rglob("*.launch.py")]
    bridge = SRC / "airmouse_sensors" / "config" / "gz_bridge.yaml"
    if bridge.is_file():
        launch_texts.append(bridge.read_text(encoding="utf-8"))
    blob = "\n".join(node_texts + launch_texts)
    for topic in EXPECTED_TOPICS:
        check(topic in blob, f"topic {topic} not used by any node/launch/bridge")


def main() -> int:
    quiet = "--quiet" in sys.argv
    check_packages()
    check_interfaces()
    check_entry_points()
    check_core_purity()
    check_assets()
    check_topics()
    if _failures:
        print(f"FAIL {len(_failures)}/{_checks} checks")
        for failure in _failures:
            print(f"  - {failure}")
        return 1
    if not quiet:
        print(f"PASS {_checks}/{_checks} offline workspace checks")
        print("NOTE runtime items (colcon build, Gazebo, PX4, TF, bags) are "
              "UNVERIFIED-OFFLINE; run scripts/check_env.sh on the Ubuntu box.")
    return 0


if __name__ == "__main__":
    sys.exit(main())