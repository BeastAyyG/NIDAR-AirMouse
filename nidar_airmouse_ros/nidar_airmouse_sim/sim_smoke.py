"""Fast offline acceptance test for the AirMouse grid convention."""

import json

from .grid import GridConfig, cell_key, grid_to_world, world_to_grid


def main() -> None:
    config = GridConfig(origin_x=0.0, origin_y=0.0, cell_size=1.0)
    checks = [
        (0.1, 0.1, (0, 0)),
        (0.999, 1.999, (0, 1)),
        (-0.001, -0.001, (-1, -1)),
        (3.4, -2.2, (3, -3)),
    ]
    for x, y, expected in checks:
        actual = world_to_grid(x, y, config)
        assert actual == expected, f"{(x, y)} -> {actual}; expected {expected}"
    assert grid_to_world(3, -3, config) == (3.5, -2.5)
    print(json.dumps({"status": "PASS", "grid_checks": len(checks), "sample_cell": cell_key(3, -3)}))
