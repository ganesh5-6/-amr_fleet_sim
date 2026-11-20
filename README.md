# amr_fleet_sim

Multi-robot (AMR) warehouse fleet simulation on ROS 2 + Gazebo + Nav2, with a central
fleet manager, roadmap-based traffic control and a browser dashboard.

## Worlds

| `world:=` | Hall | Roadmap | Robots | Map |
|---|---|---|---|---|
| `big_warehouse` (default) | 42 × 32 m, 24 racks, 2 m aisles | 103 nodes / 138 edges grid | 4 (docks for up to 7) | generated from the geometry |
| `warehouse` | 16 × 12 m, 6 shelves | 14 nodes | 3 | SLAM |

The big hall has inbound bays on the east wall (P1–P4), shipping docks on the north wall (D1–D4),
and pick / put-away faces inside the aisles (P5–P8, D5–D8). Because the roadmap is a grid, there are
many routes between any two points, so the traffic manager's Dijkstra re-planning around blocked
nodes actually has alternatives to choose from.

Everything in the big world (roadmap, stations, robots, occupancy map, Gazebo world) comes from one
spec, `BIG` in [tools/gen_world.py](tools/gen_world.py). To change the layout, edit it and run:

```bash
python3 tools/gen_world.py --world big_warehouse
```

## Features

- **Gazebo warehouse** with N differential-drive AMRs (add robots in `config/robots.yaml`, no code changes)
- **Mapping** with slam_toolbox, **navigation** with one Nav2 stack per robot
- **Task allocation** — pluggable allocators: `nearest` (greedy), `hungarian` (optimal per round), `auction`
  - battery-aware cost model (energy to pickup → drop → dock must stay above a reserve)
- **Traffic management** on a roadmap graph (`config/roadmap.yaml`)
  - node reservations (no two robots in one junction/segment), one-node lookahead
  - wait-for-graph deadlock detection, victim re-plan / yield, priority aging against starvation
- **Operator commands**: goto, dock, stop, manual/auto mode, fleet-wide E-STOP, live allocator switch
- **Web dashboard** (rosbridge): live map, robots, task queue, KPIs, logs
- **Metrics**: per-task CSV logs in `~/fleet_logs` (wait time, cycle time, retries)

## Layout

```
amr_fleet_sim/   fleet_core.py      allocators + cost model (pure Python)
                 traffic_core.py    roadmap + reservations + deadlock handling (pure Python)
                 fleet_manager.py   ROS 2 node wiring it all to Nav2
                 task_generator.py  seeded random workload
                 dashboard_server.py static server for the web UI
                 world_config.py    which config / map / world files belong to which world
config/          robots, stations, roadmap (small hall), Nav2 / SLAM params
config/big_warehouse/  generated robots, stations, roadmap for the big hall
tools/gen_world.py     generates the Gazebo worlds (+ everything for the big hall)
launch/          sim, mapping, navigation, fleet manager, dashboard
test/            offline tests (no ROS needed)
```

## Build

```bash
mkdir -p ~/amr_ws/src && cd ~/amr_ws/src
git clone https://github.com/ganesh5-6/-amr_fleet_sim.git amr_fleet_sim
cd .. && rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash
```

## Run

```bash
ros2 launch amr_fleet_sim nav_fleet.launch.py            # Gazebo + Nav2 for every robot
ros2 launch amr_fleet_sim fleet_manager.launch.py allocator:=hungarian generate_tasks:=true
ros2 launch amr_fleet_sim dashboard.launch.py            # http://localhost:8080
```

Every launch file takes `world:=big_warehouse|warehouse`. Use the same value for all three.
For a lighter run, add `gui:=false camera:=false rviz:=false` to `nav_fleet.launch.py`.

Each robot's Nav2 stack starts once that robot's odometry is flowing in Gazebo, staggered by 6 s,
so on a 4-core machine the big hall needs about a minute before every robot reports `ready`.
Verified headless (4 cores / 8 GB, WSL2): 8 generated tasks, 4 robots, all done, 0 failures.

Send a task by hand:

```bash
ros2 topic pub --once /fleet/task_request std_msgs/String \
  "{data: '{\"pickup\":\"P1\",\"dropoff\":\"D2\",\"priority\":2}'}"
```

Mapping (small hall only; the big hall's map is generated): `ros2 launch amr_fleet_sim mapping.launch.py`.

## Tests (offline)

```bash
python3 test/test_manager_sim.py   # real FleetManager against a fake ROS / Nav2
python3 test/sim_traffic.py        # traffic manager stress test (both worlds, up to 10 robots)
python3 test/bench_allocators.py   # allocator comparison
```

## License

MIT
