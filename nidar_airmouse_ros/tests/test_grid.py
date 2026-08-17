from nidar_airmouse_sim.grid import GridConfig, grid_to_world, world_to_grid


def test_book_grid_convention():
    config = GridConfig()
    assert world_to_grid(0.0, 0.0, config) == (0, 0)
    assert world_to_grid(0.999, 1.999, config) == (0, 1)
    assert world_to_grid(-0.001, -0.001, config) == (-1, -1)
    assert grid_to_world(3, -3, config) == (3.5, -2.5)
