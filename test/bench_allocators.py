"""Offline comparison of allocators (no ROS needed):  python3 test/bench_allocators.py"""
import os, random, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'amr_fleet_sim'))
from fleet_core import Robot, Task, Context, ALLOCATORS, cost

STATIONS_P = [(7, -4.5), (7, 4.5), (4, 0)]
STATIONS_D = [(-4, 0), (-4, -4.5), (-4, 4.5)]

def trial(seed):
    rnd = random.Random(seed)
    robots = [Robot(f'amr{i+1}', ready=True, xy=(rnd.uniform(-7, 7), rnd.uniform(-5, 5)),
                    dock_xy=(-6, -4 + 2 * i)) for i in range(3)]
    tasks = [Task(i, 'P', 'D', rnd.choice(STATIONS_P), rnd.choice(STATIONS_D),
                  priority=rnd.randint(1, 3), created=float(i)) for i in range(4)]
    return robots, tasks

ctx = Context()
totals = {k: 0.0 for k in ALLOCATORS}
for s in range(200):
    for name, fn in ALLOCATORS.items():
        robots, tasks = trial(s)
        pairs = fn(tasks, robots, ctx)
        assert len({r.name for _, r in pairs}) == len(pairs), 'robot double-booked'
        assert len({t.id for t, _ in pairs}) == len(pairs), 'task double-booked'
        assert len(pairs) == 3, 'should assign 3 robots'
        totals[name] += sum(cost(r, t, ctx) for t, r in pairs)
for k, v in totals.items():
    print(f'{k:10s} mean round cost = {v/200:7.2f}')
assert totals['hungarian'] <= totals['nearest'] + 1e-6
assert totals['hungarian'] <= totals['auction'] + 1e-6
print('OK: hungarian <= others')

# battery feasibility
r = Robot('amr1', ready=True, xy=(-6, 0), dock_xy=(-6, 0), battery=20.0)
t = Task(1, 'P', 'D', (7, 4.5), (-4, 0))
assert ALLOCATORS['hungarian']([t], [r], ctx) == [], 'low battery robot must be refused'
print('OK: battery gate')
