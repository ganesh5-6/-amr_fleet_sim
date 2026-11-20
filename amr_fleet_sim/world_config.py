"""Which files belong to which world. Shared by the launch files, nodes and tests.

    warehouse      original 16 x 12 m hall: config/*.yaml (flat, hand-written), SLAM map
    big_warehouse  42 x 32 m hall: config/big_warehouse/*.yaml + map, made by tools/gen_world.py
"""
import os

WORLDS = ('big_warehouse', 'warehouse')
DEFAULT_WORLD = 'big_warehouse'

# fleet_manager parameters that depend on the size of the hall
FLEET_PARAMS = {
    'warehouse': {},
    # 0.5 %/m would drain a battery in a few 40 m trips; also allow a longer wait before re-planning
    'big_warehouse': {'drain_per_m': 0.15, 'patience': 10.0},
}


def world_files(share, world):
    if world not in WORLDS:
        raise ValueError(f'unknown world {world!r}, expected one of {WORLDS}')
    cfg = os.path.join(share, 'config') if world == 'warehouse' else os.path.join(share, 'config', world)
    return {
        'robots': os.path.join(cfg, 'robots.yaml'),
        'stations': os.path.join(cfg, 'stations.yaml'),
        'roadmap': os.path.join(cfg, 'roadmap.yaml'),
        'map': os.path.join(share, 'maps', f'{world}.yaml'),
        'world': os.path.join(share, 'worlds', f'{world}.world'),
    }
