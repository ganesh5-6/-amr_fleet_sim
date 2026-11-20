"""Runs the REAL FleetManager against a fake ROS (clock, timers, Nav2 servers, moving robots).
    python3 test/test_manager_sim.py
"""
import json, math, os, sys, types
from types import SimpleNamespace as NS

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)

# ------------------------------------------------------------------ fake world / scheduler
class Sim:
    pass
sim = Sim()

def reset():
    sim.t = 1000.0
    sim.timers, sim.subs, sim.pubs = [], {}, {}
    sim.world, sim.goals, sim.accept = {}, {}, []
    sim.reject = set()              # robots whose Nav2 rejects every goal
    sim.min_gap = 1e9
    sim.zero_pubs = 0

class Future:
    def __init__(self): self.done, self.val, self.cbs = False, None, []
    def add_done_callback(self, cb):
        if self.done: cb(self)
        else: self.cbs.append(cb)
    def set(self, v):
        self.done, self.val = True, v
        for cb in self.cbs: cb(self)
    def result(self): return self.val

class GoalHandle:
    def __init__(self, robot, goal):
        self.robot, self.goal, self.accepted = robot, goal, True
        self.res = Future(); self.finished = False
    def get_result_async(self): return self.res
    def cancel_goal_async(self):
        if not self.finished:
            self.finished = True
            if sim.goals.get(self.robot) is self: del sim.goals[self.robot]
            self.res.set(NS(status=5))

class ActionClient:
    def __init__(self, node, typ, name): self.robot = name.split('/')[1]
    def server_is_ready(self): return True
    def send_goal_async(self, goal):
        fut = Future(); h = GoalHandle(self.robot, goal)
        if self.robot in sim.reject: h.accepted = False; fut.set(h); return fut
        sim.accept.append((fut, h)); return fut

class Timer:
    def __init__(self, period, cb): self.period, self.cb, self.cancelled, self.next = period, cb, False, sim.t + period
    def cancel(self): self.cancelled = True

class Pub:
    def __init__(self, topic): self.topic, self.msgs = topic, []
    def publish(self, m):
        self.msgs.append(m)
        if self.topic.endswith('cmd_vel'): sim.zero_pubs += 1

class Node:
    OVERRIDES = {'log_dir': '/tmp/fleet_test_logs', 'load_time': 1.0}
    WORLD = {}                      # per-test file / parameter overrides (see new_fleet)
    def __init__(self, name): self._p = {}
    def declare_parameter(self, n, d): self._p[n] = self.WORLD.get(n, self.OVERRIDES.get(n, d))
    def get_parameter(self, n): return NS(value=self._p[n])
    def create_timer(self, period, cb):
        t = Timer(period, cb); sim.timers.append(t); return t
    def destroy_timer(self, t): t.cancelled = True
    def create_subscription(self, typ, topic, cb, qos): sim.subs[topic] = cb
    def create_publisher(self, typ, topic, qos):
        p = Pub(topic); sim.pubs[topic] = p; return p
    def get_clock(self): return NS(now=lambda: NS(nanoseconds=int(sim.t * 1e9)))
    _sites = {}
    def get_logger(self):
        def pr(lvl):
            def log(m):
                # like rclpy: one call site may not log at two different severities
                f = sys._getframe(1); site = (f.f_code.co_filename, f.f_lineno)
                if Node._sites.setdefault(site, lvl) != lvl:
                    raise ValueError('Logger severity cannot be changed between calls.')
                if VERBOSE: print(f'   [{sim.t-1000:6.1f}] {lvl}: {m}')
            return log
        return NS(info=pr('INFO'), warn=pr('WARN'), warning=pr('WARN'), error=pr('ERR'))
    def destroy_node(self): pass

class Buffer:
    def lookup_transform(self, a, b, t):
        r = b.split('/')[0]
        if r not in sim.world: raise RuntimeError('no tf')
        x, y = sim.world[r]
        return NS(transform=NS(translation=NS(x=x, y=y), rotation=NS(x=0, y=0, z=0, w=1)))

def mod(name, **attrs):
    m = types.ModuleType(name); m.__dict__.update(attrs); sys.modules[name] = m; return m

class Msg:
    def __init__(self, **kw): self.__dict__.update(kw)
class MarkerArray:
    def __init__(self): self.markers = []
class Marker(Msg):
    LINE_LIST, SPHERE_LIST, TEXT_VIEW_FACING, ADD = 5, 7, 9, 0
    def __init__(self, **kw): self.header = NS(frame_id=''); self.pose = NS(position=NS(x=0, y=0, z=0), orientation=NS(w=1)); self.scale = NS(x=0, y=0, z=0); self.points, self.colors = [], []; super().__init__(**kw)
class PoseStamped:
    def __init__(self): self.header = NS(frame_id=''); self.pose = NS(position=NS(x=0, y=0), orientation=NS(z=0, w=1))
class Goal:
    def __init__(self): self.pose = None
class NavigateToPose: Goal = Goal

rclpy = mod('rclpy', init=lambda: None, spin=lambda n: None, shutdown=lambda: None)
rclpy.time = mod('rclpy.time', Time=lambda: None)
mod('rclpy.executors', ExternalShutdownException=type('ExternalShutdownException', (Exception,), {})); mod('rclpy.node', Node=Node); mod('rclpy.action', ActionClient=ActionClient)
mod('action_msgs'); mod('action_msgs.msg', GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5))
mod('ament_index_python'); mod('ament_index_python.packages', get_package_share_directory=lambda p: ROOT)
mod('geometry_msgs'); mod('geometry_msgs.msg', Point=Msg, PoseStamped=PoseStamped, Twist=Msg)
mod('nav2_msgs'); mod('nav2_msgs.action', NavigateToPose=NavigateToPose)
mod('std_msgs'); mod('std_msgs.msg', ColorRGBA=Msg, String=Msg)
mod('tf2_ros', Buffer=Buffer, TransformListener=lambda b, n: None)
mod('visualization_msgs'); mod('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
VERBOSE = '-v' in sys.argv

import yaml
from amr_fleet_sim.fleet_manager import FleetManager

SPEED = 0.5
def step(dt=0.1):
    sim.t += dt
    for fut, h in sim.accept:
        old = sim.goals.get(h.robot)
        if old and not old.finished:
            old.finished = True; old.res.set(NS(status=6))
        sim.goals[h.robot] = h; fut.set(h)
    sim.accept.clear()
    for r, h in list(sim.goals.items()):
        p = sim.world[r]; g = h.goal.pose.pose.position
        dx, dy = g.x - p[0], g.y - p[1]; d = math.hypot(dx, dy); s = SPEED * dt
        if d <= s + 0.02:
            sim.world[r] = [g.x, g.y]; h.finished = True; del sim.goals[r]; h.res.set(NS(status=4))
        else:
            p[0] += dx / d * s; p[1] += dy / d * s
    for t in list(sim.timers):
        while not t.cancelled and t.next <= sim.t + 1e-9:
            t.next += t.period; t.cb()
    sim.timers[:] = [t for t in sim.timers if not t.cancelled]
    names = list(sim.world)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            sim.min_gap = min(sim.min_gap, math.dist(sim.world[names[i]], sim.world[names[j]]))

def run(seconds):
    for _ in range(int(seconds / 0.1)): step()

from amr_fleet_sim.world_config import FLEET_PARAMS, world_files

def new_fleet(world='warehouse'):
    reset()
    f = world_files(ROOT, world)
    Node.WORLD = {'robots_file': f['robots'], 'stations_file': f['stations'], 'roadmap_file': f['roadmap'],
                  **FLEET_PARAMS[world]}
    robots = yaml.safe_load(open(f['robots']))['robots']
    for r in robots: sim.world[r['name']] = [r['x'], r['y']]
    fm = FleetManager(); run(2); return fm

def task(p, d, prio=1): sim.subs['/fleet/task_request'](NS(data=json.dumps({'pickup': p, 'dropoff': d, 'priority': prio, 'payload': 'box'})))
def cmd(**kw): sim.subs['/fleet/command'](NS(data=json.dumps(kw)))
def snap(fm): fm.publish_status(); return json.loads(fm.status_pub.msgs[-1].data)
def near(a, b, tol=0.3): return math.dist(a, b) <= tol

passed = []
def check(name, cond):
    print(('  PASS ' if cond else '  FAIL ') + name)
    if not cond: sys.exit(1)
    passed.append(name)

# 1 ---------------------------------------------------------------------- one task
print('1. single task'); fm = new_fleet()
task('P1', 'D1', 2); run(200); s = snap(fm)
check('task done', s['stats']['done'] == 1)
check('all robots end idle/charging at docks', all(r['status'] in ('IDLE', 'CHARGING') for r in s['robots']) and
      all(near(sim.world[r.name], r.dock_xy) for r in fm.robots.values()))

# 2 ---------------------------------------------------------------------- head-on
print('2. head-on swap in the bottom corridor'); fm = new_fleet()
task('P1', 'D2', 1); task('D2', 'P1', 2); sim.min_gap = 1e9; run(600); s = snap(fm)
check('both tasks done', s['stats']['done'] == 2)
check(f'robots never closer than 0.9 m (min {sim.min_gap:.2f})', sim.min_gap > 0.9)

# 3 ---------------------------------------------------------------------- goto then dock mid-way
print('3. click-to-go, then Dock while moving'); fm = new_fleet()
cmd(cmd='goto', robot='amr1', x=3.0, y=-4.4); run(8); s = snap(fm)
r1 = [r for r in s['robots'] if r['name'] == 'amr1'][0]
check('amr1 is TO_POINT with a target distance', r1['status'] == 'TO_POINT' and r1['target'] and r1['target']['dist'] > 0)
cmd(cmd='dock', robot='amr1'); run(2); s = snap(fm)
check('amr1 switched to TO_DOCK', [r for r in s['robots'] if r['name'] == 'amr1'][0]['status'] == 'TO_DOCK')
run(120); s = snap(fm)
check('amr1 back at dock', near(sim.world['amr1'], fm.robots['amr1'].dock_xy))
check('amr1 holds only its dock node', [n for n, o in fm.traffic.owner.items() if o == 'amr1'] == [fm.robots['amr1'].dock_node])
cmd(cmd='goto', robot='amr1', x=3.0, y=-4.4); run(150); s = snap(fm)
r1 = [r for r in s['robots'] if r['name'] == 'amr1'][0]
check('goto completes and robot is IDLE at the point', r1['status'] == 'IDLE' and near(sim.world['amr1'], (3.0, -4.4), 0.2))

# 4 ---------------------------------------------------------------------- manual mode
print('4. manual mode requeues the task and blocks allocation'); fm = new_fleet()
task('P1', 'D1', 1); run(6)
busy = [r for r in fm.robots.values() if r.task][0].name
cmd(cmd='mode', robot=busy, mode='manual'); run(2); s = snap(fm)
rb = [r for r in s['robots'] if r['name'] == busy][0]
check('robot is MANUAL, mode reported', rb['status'] == 'MANUAL' and rb['mode'] == 'manual')
run(3)
check('task was re-allocated to a different robot', fm.tasks[1].robot not in (None, busy))
cmd(cmd='goto', robot=busy, x=0, y=0); run(1)
check('goto refused in manual mode', fm.robots[busy].status == 'MANUAL')
run(300)
check('task still completed', snap(fm)['stats']['done'] == 1)
cmd(cmd='mode', robot=busy, mode='auto'); run(1)
check('back to AUTO / IDLE', fm.robots[busy].status == 'IDLE' and not fm.robots[busy].manual)

print('4b. Dock works from MANUAL mode'); fm = new_fleet()
cmd(cmd='goto', robot='amr2', x=3.0, y=-4.4); run(6); cmd(cmd='mode', robot='amr2', mode='manual'); run(1)
sim.world['amr2'][0] -= 0.7; sim.world['amr2'][1] += 0.3     # operator drove it away from the node by hand
check('amr2 is manual', fm.robots['amr2'].manual and fm.robots['amr2'].status == 'MANUAL')
cmd(cmd='dock', robot='amr2'); run(1)
check('dock switched it to AUTO and started TO_DOCK', (not fm.robots['amr2'].manual) and fm.robots['amr2'].status == 'TO_DOCK')
run(150)
check('amr2 docked exactly (final approach after manual drive)', near(sim.world['amr2'], fm.robots['amr2'].dock_xy, 0.1))
check('pose feed published (5 Hz) with all robots', len(sim.pubs['/fleet/poses'].msgs) > 300 and
      set(json.loads(sim.pubs['/fleet/poses'].msgs[-1].data)['p']) == {'amr1', 'amr2', 'amr3'})

# 5 ---------------------------------------------------------------------- e-stop
print('5. E-STOP halts everything, RESUME recovers'); fm = new_fleet()
task('P1', 'D1', 1); task('P2', 'D2', 1); task('P3', 'D3', 1); run(10)
cmd(cmd='estop'); run(1); s = snap(fm)
check('all robots ESTOP and flag set', s['estop'] and all(r['status'] == 'ESTOP' for r in s['robots']))
before = {k: list(v) for k, v in sim.world.items()}; z0 = sim.zero_pubs; run(5)
check('nobody moved during E-STOP', all(near(before[k], sim.world[k], 0.05) for k in before))
check('zero velocity commands streamed', sim.zero_pubs > z0)
check('no goals active', not sim.goals)
cmd(cmd='goto', robot='amr1', x=0, y=0); run(1)
check('commands refused during E-STOP', fm.robots['amr1'].status == 'ESTOP')
cmd(cmd='resume'); run(400); s = snap(fm)
check('after resume all 3 tasks finish', s['stats']['done'] == 3 and s['stats']['failed'] == 0)

# 6 ---------------------------------------------------------------------- misc
print('6. misc commands'); fm = new_fleet()
cmd(cmd='allocator', name='auction'); check('allocator switch', snap(fm)['allocator'] == 'auction')
cmd(cmd='allocator', name='bogus'); check('bad allocator ignored', snap(fm)['allocator'] == 'auction')
cmd(cmd='goto', robot='ghost', x=0, y=0); check('unknown robot ignored', True)
task('P2', 'D3', 3); run(8); cmd(cmd='stop', robot=[r for r in fm.robots.values() if r.task][0].name); run(1)
check('stop returns task to queue (or reassigns it)', fm.tasks[1].status in ('PENDING', 'ASSIGNED'))
run(300); check('task eventually completes', snap(fm)['stats']['done'] == 1)
logs = [json.loads(m.data) for m in fm.log_pub.msgs]
check('log feed has info + warn entries', {'info', 'warn'} <= {l['level'] for l in logs})

# 7 ---------------------------------------------------------------------- failed auto-dock backs off
print('7. low battery + unreachable dock does not retry every tick'); fm = new_fleet()
real_set_goal = fm.traffic.set_goal
fm.traffic.set_goal = lambda name, *a, **k: False if name == 'amr1' else real_set_goal(name, *a, **k)
fm.robots['amr1'].battery = 20.0; n0 = len(fm.log_pub.msgs); run(5)
tries = sum('docking to charge' in json.loads(m.data)['msg'] and 'amr1' in json.loads(m.data)['msg']
            for m in fm.log_pub.msgs[n0:])
check(f'only one dock attempt in 5 s (got {tries})', tries == 1)
fm.traffic.set_goal = real_set_goal; run(15)
check('retries after the back-off and ends up charging', fm.robots['amr1'].status == 'CHARGING')

# 7b --------------------------------------------------------------------- a broken robot is benched
print('7b. robot whose Nav2 rejects every goal gets benched, task goes elsewhere'); fm = new_fleet()
task('P1', 'D1', 2); run(3)
bad = fm.tasks[1].robot; sim.reject.add(bad); run(300); s = snap(fm)
fails_on_bad = sum(f'Task 1 on {bad}' in json.loads(m.data)['msg'] for m in fm.log_pub.msgs)
check(f'task done by another robot (done={s["stats"]["done"]}, robot={fm.tasks[1].robot})',
      s['stats']['done'] == 1 and fm.tasks[1].robot != bad)
check(f'bad robot failed it only once (got {fails_on_bad})', fails_on_bad == 1)
check('bad robot reported not ready while benched', not fm.robots[bad].ready or s['t'] > 15)

# 8 ---------------------------------------------------------------------- big warehouse end to end
print('8. big warehouse: 4 robots, 12 mixed tasks'); fm = new_fleet('big_warehouse')
check('4 robots on a 100+ node roadmap', len(fm.robots) == 4 and len(fm.roadmap.nodes) > 100)
import random as _r; rr = _r.Random(5)
P = [k for k in fm.stations if k.startswith('P')]; D = [k for k in fm.stations if k.startswith('D')]
for _ in range(12): task(rr.choice(P), rr.choice(D), rr.choice([1, 1, 2, 3]))
sim.min_gap = 1e9; run(1500); s = snap(fm)
check(f"all 12 tasks done (done={s['stats']['done']}, failed={s['stats']['failed']})", s['stats']['done'] == 12)
check(f'robots never closer than 0.9 m (min {sim.min_gap:.2f})', sim.min_gap > 0.9)
check('every robot did some work', all(r.dist_travelled > 20 for r in fm.robots.values()))
check('nobody ran flat', all(r.battery > 15 for r in fm.robots.values()))
print(f'\nALL {len(passed)} MANAGER CHECKS PASSED')
