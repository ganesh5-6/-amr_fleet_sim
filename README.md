# amr_fleet_sim

Multi-robot (AMR) warehouse fleet simulation on ROS 2 + Gazebo + Nav2, with a central
fleet manager, roadmap-based traffic control and a browser dashboard.

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
config/          robots, stations, roadmap, Nav2 / SLAM params
launch/          sim, mapping, navigation, fleet manager, dashboard
test/            offline tests (no ROS needed)
```

## Build

```bash
mkdir -p ~/amr_ws/src && cd ~/amr_ws/src
git clone https://github.com/ganesh5-6/amr_fleet_sim.git
cd .. && rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash
```

## Run

```bash
ros2 launch amr_fleet_sim nav_fleet.launch.py            # Gazebo + Nav2 for every robot
ros2 launch amr_fleet_sim fleet_manager.launch.py allocator:=hungarian generate_tasks:=true
ros2 launch amr_fleet_sim dashboard.launch.py            # http://localhost:8080
```

Send a task by hand:

```bash
ros2 topic pub --once /fleet/task_request std_msgs/String \
  "{data: '{\"pickup\":\"P1\",\"dropoff\":\"D2\",\"priority\":2}'}"
```

Mapping a new world: `ros2 launch amr_fleet_sim mapping.launch.py`.

## Tests (offline)

```bash
python3 test/test_manager_sim.py   # real FleetManager against a fake ROS / Nav2
python3 test/sim_traffic.py        # traffic manager stress test
python3 test/bench_allocators.py   # allocator comparison
```

## License

MIT
