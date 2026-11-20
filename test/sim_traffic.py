"""Offline stress test of the traffic manager: python3 test/sim_traffic.py"""
import math, os, random, sys
import yaml
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'amr_fleet_sim'))
from traffic_core import RoadMap, TrafficManager

cfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'roadmap.yaml')))
rm = RoadMap(cfg['nodes'], cfg['edges'])
assert all(rm.shortest_path('dock_amr1', n) for n in rm.nodes), 'graph not connected'

SPEED, DT = 0.5, 0.2

def run(starts, goal_fn, sim_time=900.0, seed=0, rm=rm):
    rnd = random.Random(seed)
    tm = TrafficManager(rm, patience=6.0)
    pos, seg, goals_done, wait_time = {}, {}, {}, {}
    for i, (n, node) in enumerate(starts.items()):
        tm.register(n, node, priority=1 + i % 3)
        pos[n] = list(rm.nodes[node]); seg[n] = None; goals_done[n] = 0; wait_time[n] = 0.0
        tm.set_goal(n, goal_fn(rnd, node), priority=1 + i % 3)
    t, min_gap = 0.0, 1e9
    while t < sim_time:
        for n, a in tm.agents.items():
            if seg[n] is None:
                kind, val = tm.request(n, t)
                if kind == 'GO':
                    seg[n] = (list(pos[n]), rm.nodes[val])
                elif kind == 'DONE':
                    goals_done[n] += 1
                    tm.set_goal(n, goal_fn(rnd, a.node), priority=a.priority)
                elif kind == 'WAIT':
                    wait_time[n] += DT
                else:
                    raise AssertionError(f'{n}: {kind}')
            else:
                (x0, y0), (x1, y1) = seg[n]
                dx, dy = x1 - pos[n][0], y1 - pos[n][1]
                L = math.hypot(dx, dy)
                step = SPEED * DT
                if L <= step:
                    pos[n] = [x1, y1]; seg[n] = None; tm.arrived(n)
                else:
                    pos[n][0] += dx / L * step; pos[n][1] += dy / L * step
        names = list(pos)
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                g = math.dist(pos[names[i]], pos[names[j]])
                min_gap = min(min_gap, g)
        owners = list(tm.owner.values())
        assert len(set(tm.owner)) == len(tm.owner)
        t += DT
    return min_gap, goals_done, wait_time

stations = [n for n in rm.nodes if not n.startswith('dock')]
pick = lambda rnd, cur: rnd.choice([s for s in stations if s != cur])

# 1) head-on: two robots must swap ends of the bottom corridor
def swap(rnd, cur):
    return 'P1' if cur == 'D2' else 'D2'
gap, done, _ = run({'amr1': 'D2', 'amr2': 'P1'}, swap, sim_time=400)
print(f'head-on swap     : min gap {gap:.2f} m, trips {done}')
assert gap > 1.0 and min(done.values()) >= 1

# 2) three robots, random goals, many seeds
worst, mind = 1e9, 99
for seed in range(30):
    gap, done, w = run({'amr1': 'dock_amr1', 'amr2': 'dock_amr2', 'amr3': 'dock_amr3'}, pick,
                       sim_time=900, seed=seed)
    worst = min(worst, gap); mind = min(mind, min(done.values()))
print(f'30 random runs   : min gap {worst:.2f} m, fewest goals reached by any robot {mind}')
assert worst > 1.0, 'robots got too close'
assert mind >= 5, 'a robot starved / deadlocked'

# 3) 3 robots forced through the same single node (stress)
def hub(rnd, cur):
    return rnd.choice(['C0', 'P3', 'D1']) if cur not in ('C0', 'P3', 'D1') else rnd.choice(['P1', 'P2', 'D3', 'D2'])
gap, done, _ = run({'amr1': 'dock_amr1', 'amr2': 'dock_amr2', 'amr3': 'dock_amr3'}, hub, sim_time=900, seed=3)
print(f'hub stress       : min gap {gap:.2f} m, trips {done}')
assert gap > 1.0 and min(done.values()) >= 5

# 4) settle() must clear wait state, else a parked robot forms a phantom deadlock cycle
tm = TrafficManager(rm, patience=1e9)
tm.register('a', 'D2'); tm.register('b', 'B0')
tm.set_goal('a', 'B4'); assert tm.request('a', 0.0) == ('WAIT', 'b')
tm.settle('a', 'D2')                        # e.g. operator pressed Stop / Dock on robot a
assert tm.agents['a'].wait_on is None, 'settle left a stale wait_on'
tm.set_goal('b', 'D1'); assert tm.request('b', 1.0)[0] == 'WAIT'
assert tm._cycle('b') is None, 'phantom wait-for cycle through a settled robot'
print('settle           : wait state cleared, no phantom cycle')

# 5) big warehouse: 103-node grid, many alternative routes for Dijkstra to re-plan through
import time
bcfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'big_warehouse', 'roadmap.yaml')))
big = RoadMap(bcfg['nodes'], bcfg['edges'])
assert all(big.shortest_path('dock_amr1', n) for n in big.nodes), 'big graph not connected'
t0 = time.perf_counter(); n_q = 0
for a in list(big.nodes)[::7]:
    for b in big.nodes:
        big.shortest_path(a, b); n_q += 1
print(f'big dijkstra     : {len(big.nodes)} nodes, {(time.perf_counter() - t0) / n_q * 1e3:.2f} ms per query')
bst = [n for n in big.nodes if n[0] in 'PD' and n[1:].isdigit()]
bpick = lambda rnd, cur: rnd.choice([s for s in bst if s != cur])
worst, mind = 1e9, 99
for seed in range(10):
    starts = {f'amr{i + 1}': f'dock_amr{i + 1}' for i in range(4)}
    gap, done, w = run(starts, bpick, sim_time=1200, seed=seed, rm=big)
    worst = min(worst, gap); mind = min(mind, min(done.values()))
print(f'big, 4 robots    : 10 runs, min gap {worst:.2f} m, fewest goals reached by any robot {mind}')
assert worst > 1.0 and mind >= 5
rnd = random.Random(1)
starts = dict(zip([f'r{i}' for i in range(10)], rnd.sample([n for n in big.nodes if n.startswith('N_')], 10)))
gap, done, w = run(starts, bpick, sim_time=1500, seed=1, rm=big)
print(f'big, 10 robots   : min gap {gap:.2f} m, goals {sorted(done.values())}')
assert gap > 1.0 and min(done.values()) >= 3
print('ALL TRAFFIC TESTS PASSED')
