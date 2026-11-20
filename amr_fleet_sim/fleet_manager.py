#!/usr/bin/env python3
"""Central fleet manager (allocation + traffic + operator commands).

Topics
  in  /fleet/task_request  JSON {"pickup":"P1","dropoff":"D2","priority":2,"payload":"pallet"}
  in  /fleet/command       JSON {"cmd": goto|dock|stop|mode|estop|resume|allocator, ...}
        goto      {"robot":"amr1","x":1.5,"y":-2.0}     drive there (roadmap + final free leg)
        dock      {"robot":"amr1"}                       go home (cancels its task; also leaves MANUAL mode)
        stop      {"robot":"amr1"}                       cancel current motion
        mode      {"robot":"amr1","mode":"manual"|"auto"}
        estop / resume                                   whole fleet
        allocator {"name":"nearest"|"hungarian"|"auction"}
  out /fleet/status        JSON snapshot, 1 Hz
  out /fleet/poses         {"t":..,"p":{"amr1":[x,y,yaw,speed],..}}  5 Hz (smooth map + lidar alignment)
  out /fleet/log           JSON {"t":..,"level":"info|warn|error","msg":".."}
  out /fleet/roadmap       RViz markers
"""
import csv
import json
import math
import os
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from amr_fleet_sim.fleet_core import ALLOCATORS, Context, Robot, Task, dist
from amr_fleet_sim.traffic_core import RoadMap, TrafficManager

PKG = 'amr_fleet_sim'
ROBOT_COLORS = [(1.0, 0.2, 0.2), (0.2, 0.85, 0.2), (0.2, 0.45, 1.0), (1.0, 0.8, 0.1), (0.8, 0.3, 1.0)]
TARGET_STATES = ('TO_PICKUP', 'TO_DROP', 'TO_DOCK', 'TO_POINT')


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        share = get_package_share_directory(PKG)

        self.declare_parameter('allocator', 'hungarian')
        self.declare_parameter('stations_file', os.path.join(share, 'config', 'stations.yaml'))
        self.declare_parameter('robots_file', os.path.join(share, 'config', 'robots.yaml'))
        self.declare_parameter('roadmap_file', os.path.join(share, 'config', 'roadmap.yaml'))
        self.declare_parameter('patience', 8.0)
        self.declare_parameter('allocation_period', 1.0)
        self.declare_parameter('load_time', 3.0)
        self.declare_parameter('drain_per_m', 0.5)
        self.declare_parameter('battery_reserve', 15.0)
        self.declare_parameter('low_battery', 30.0)
        self.declare_parameter('charge_rate', 4.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('dock_retry_s', 10.0)    # wait this long before retrying a failed dock
        self.declare_parameter('log_dir', os.path.expanduser('~/fleet_logs'))

        g = self.get_parameter
        alloc_name = g('allocator').value
        if alloc_name not in ALLOCATORS:
            raise ValueError(f'allocator must be one of {list(ALLOCATORS)}')
        self.allocate_fn = ALLOCATORS[alloc_name]
        self.alloc_name = alloc_name
        self.load_time = g('load_time').value
        self.low_battery = g('low_battery').value
        self.charge_rate = g('charge_rate').value
        self.max_retries = g('max_retries').value
        self.dock_retry_s = g('dock_retry_s').value
        self.dock_retry_at = {}                          # robot -> earliest time to retry auto-docking

        with open(g('stations_file').value) as f:
            self.stations = yaml.safe_load(f)['stations']
        with open(g('robots_file').value) as f:
            robot_cfg = yaml.safe_load(f)['robots']
        with open(g('roadmap_file').value) as f:
            rm = yaml.safe_load(f)
        self.roadmap = RoadMap(rm['nodes'], rm['edges'])
        for s in self.stations:
            if s not in self.roadmap.nodes:
                raise ValueError(f'station {s} is not a node in roadmap.yaml')

        self.ctx = Context(drain_per_m=g('drain_per_m').value, reserve=g('battery_reserve').value,
                           dist_fn=self.roadmap.road_dist)
        self.traffic = TrafficManager(self.roadmap, patience=g('patience').value)

        self.robots, self.nav_clients, self.cmd_pubs = {}, {}, {}
        for rc in robot_cfg:
            n = rc['name']
            dock_xy = (float(rc['x']), float(rc['y']))
            node = self.roadmap.nearest_node(dock_xy)
            if dist(dock_xy, self.roadmap.nodes[node]) > 0.5:
                self.get_logger().warn(f'{n}: spawn pose is >0.5 m from nearest roadmap node {node}')
            self.robots[n] = Robot(name=n, dock_xy=dock_xy, dock_yaw=float(rc.get('yaw', 0.0)),
                                   dock_node=node, xy=dock_xy)
            self.traffic.register(n, node)
            self.nav_clients[n] = ActionClient(self, NavigateToPose, f'/{n}/navigate_to_pose')
            self.cmd_pubs[n] = self.create_publisher(Twist, f'/{n}/cmd_vel', 10)
        self.color = {n: ROBOT_COLORS[i % len(ROBOT_COLORS)] for i, n in enumerate(self.robots)}

        self.estop = False
        self.tasks, self.next_id, self.last_xy, self.last_tick = {}, 1, {}, None
        self.t0 = self._now()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        os.makedirs(g('log_dir').value, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(g('log_dir').value, f'tasks_{alloc_name}_{stamp}.csv')
        with open(self.log_path, 'w', newline='') as f:
            csv.writer(f).writerow(['id', 'robot', 'priority', 'pickup', 'dropoff', 'created_s',
                                    'assigned_s', 'completed_s', 'wait_s', 'cycle_s', 'retries'])

        self.create_subscription(String, '/fleet/task_request', self.on_request, 10)
        self.create_subscription(String, '/fleet/command', self.on_command, 10)
        self.status_pub = self.create_publisher(String, '/fleet/status', 10)
        self.log_pub = self.create_publisher(String, '/fleet/log', 50)
        self.pose_pub = self.create_publisher(String, '/fleet/poses', 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/roadmap', 10)
        self.create_timer(0.2, self.update_state)
        self.create_timer(g('allocation_period').value, self.allocate)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(1.0, self.publish_markers)
        self.create_timer(0.1, self.estop_hold)
        self.flog('info', f'Fleet manager up | allocator={alloc_name} | robots={list(self.robots)}')

    # ------------------------------------------------------------ helpers
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _rel(self):
        return self._now() - self.t0

    def flog(self, level, msg):
        {'info': self.get_logger().info, 'warn': self.get_logger().warn,
         'error': self.get_logger().error}.get(level, self.get_logger().info)(msg)
        self.log_pub.publish(String(data=json.dumps({'t': round(self._rel(), 1), 'level': level, 'msg': msg})))

    def call_later(self, delay, fn):
        timer = None

        def cb():
            timer.cancel()
            self.destroy_timer(timer)
            fn()
        timer = self.create_timer(delay, cb)

    # ------------------------------------------------------------ intake
    def on_request(self, msg):
        try:
            d = json.loads(msg.data)
            p, q = d['pickup'], d['dropoff']
            prio = int(d.get('priority', 1))
            sp, sq = self.stations[p], self.stations[q]
        except Exception as e:                                  # noqa
            self.flog('warn', f'Bad task request {msg.data!r}: {e}')
            return
        t = Task(id=self.next_id, pickup=p, dropoff=q,
                 pickup_xy=(sp['x'], sp['y']), dropoff_xy=(sq['x'], sq['y']),
                 priority=prio, created=self._rel(), payload=str(d.get('payload', ''))[:60])
        self.next_id += 1
        self.tasks[t.id] = t
        self.flog('info', f'Task {t.id} queued: {p} -> {q}, priority {prio}'
                          + (f', "{t.payload}"' if t.payload else ''))

    # ------------------------------------------------------------ operator commands
    def on_command(self, msg):
        try:
            d = json.loads(msg.data)
            cmd = d['cmd']
        except Exception as e:                                  # noqa
            self.flog('warn', f'Bad command {msg.data!r}: {e}')
            return
        if cmd == 'estop':
            return self.do_estop(True)
        if cmd == 'resume':
            return self.do_estop(False)
        if cmd == 'allocator':
            name = d.get('name')
            if name in ALLOCATORS:
                self.allocate_fn, self.alloc_name = ALLOCATORS[name], name
                self.flog('info', f'Allocator set to {name}')
            else:
                self.flog('warn', f'Unknown allocator {name!r}')
            return
        r = self.robots.get(d.get('robot'))
        if r is None:
            self.flog('warn', f'Command {cmd}: unknown robot {d.get("robot")!r}')
            return
        if self.estop and cmd != 'mode':
            self.flog('warn', f'{r.name}: {cmd} refused, E-STOP is active')
            return
        if cmd == 'mode':
            self.cmd_mode(r, d.get('mode'))
        elif cmd == 'dock':
            self.cmd_dock(r)                      # works from MANUAL too (switches to AUTO first)
        elif r.manual:
            self.flog('warn', f'{r.name}: {cmd} refused, robot is in MANUAL mode')
        elif cmd == 'goto':
            try:
                self.cmd_goto(r, float(d['x']), float(d['y']))
            except (KeyError, ValueError):
                self.flog('warn', f'{r.name}: goto needs numeric x and y')
        elif cmd == 'stop':
            self.cmd_stop(r)
        else:
            self.flog('warn', f'Unknown command {cmd!r}')

    def preempt(self, r):
        """Cancel whatever the robot is doing and make its reservations consistent."""
        r.route_id += 1                     # every pending callback of the old route becomes stale
        if r.goal_handle is not None:
            try:
                r.goal_handle.cancel_goal_async()
            except Exception:               # noqa
                pass
            r.goal_handle = None
        self.traffic.settle(r.name, self.roadmap.nearest_node(r.xy))
        r.target_xy = None

    def release_task(self, r, count_retry=False):
        t = r.task
        if t is None:
            return
        t.status, t.robot, t.assigned = 'PENDING', None, None
        if count_retry:
            t.retries += 1
            if t.retries > self.max_retries:
                t.status = 'FAILED'
        r.task = None

    def cmd_goto(self, r, x, y):
        if r.task is not None:
            self.flog('warn', f'{r.name}: busy with task {r.task.id}; use Dock/Stop first')
            return
        self.preempt(r)
        node = self.roadmap.nearest_node((x, y))
        r.status = 'TO_POINT'
        self.flog('info', f'{r.name}: go to ({x:.2f}, {y:.2f}) via {node}')

        def after_node(ok):
            if not ok:
                r.status = 'IDLE'
                self.flog('warn', f'{r.name}: could not reach the point')
                return
            rid = r.route_id
            yaw = math.atan2(y - r.xy[1], x - r.xy[0])

            def fin(ok2):
                if rid != r.route_id:
                    return
                r.status = 'IDLE'
                r.target_xy = None
                self.flog('info' if ok2 else 'warn',
                          f'{r.name}: ' + ('arrived at point' if ok2 else 'final leg failed'))
            self.go_to_pose(r, (x, y), yaw, fin, rid)
        self.drive_to(r, node, 0.0, 0, after_node, target_xy=(x, y))

    def cmd_dock(self, r):
        if r.manual:
            self.cmd_mode(r, 'auto')
        if r.task is not None:
            self.flog('warn', f'{r.name}: task {r.task.id} returned to the queue')
            self.release_task(r)
        self.preempt(r)
        self.flog('info', f'{r.name}: returning to dock')
        self.send_to_dock(r)

    def cmd_stop(self, r):
        if r.task is not None:
            self.flog('warn', f'{r.name}: task {r.task.id} returned to the queue')
            self.release_task(r)
        self.preempt(r)
        r.status = 'IDLE'
        self.flog('info', f'{r.name}: stopped')

    def cmd_mode(self, r, mode):
        if mode == 'manual':
            if r.task is not None:
                self.flog('warn', f'{r.name}: task {r.task.id} returned to the queue')
                self.release_task(r)
            self.preempt(r)
            r.manual, r.status = True, 'MANUAL'
            self.flog('info', f'{r.name}: MANUAL mode (operator drives)')
        elif mode == 'auto':
            self.preempt(r)
            r.manual = False
            r.status = 'ESTOP' if self.estop else 'IDLE'
            self.flog('info', f'{r.name}: AUTONOMOUS mode')
        else:
            self.flog('warn', f'{r.name}: unknown mode {mode!r}')

    def do_estop(self, on):
        self.estop = on
        for r in self.robots.values():
            if on:
                if r.task is not None:
                    self.release_task(r)
                self.preempt(r)
                r.estop, r.status = True, 'ESTOP'
            else:
                r.estop = False
                r.status = 'MANUAL' if r.manual else 'IDLE'
        self.flog('error' if on else 'info', 'E-STOP ACTIVATED - all robots halted' if on
                  else 'E-STOP released - robots idle')

    def estop_hold(self):
        if self.estop:
            for p in self.cmd_pubs.values():
                p.publish(Twist())

    # ------------------------------------------------------------ state
    def update_state(self):
        now = self._now()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        for n, r in self.robots.items():
            try:
                tf = self.tf_buffer.lookup_transform('map', f'{n}/base_link', rclpy.time.Time())
                xy = (tf.transform.translation.x, tf.transform.translation.y)
                q = tf.transform.rotation
                r.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                if n in self.last_xy:
                    d = dist(xy, self.last_xy[n])
                    if d < 1.0:
                        r.dist_travelled += d
                        r.battery = max(0.0, r.battery - d * self.ctx.drain_per_m)
                        if dt > 0:
                            r.speed = 0.6 * r.speed + 0.4 * (d / dt)
                self.last_xy[n] = xy
                r.xy = xy
                has_pose = True
            except Exception:                                   # noqa
                has_pose = False
            r.ready = has_pose and self.nav_clients[n].server_is_ready()

            if r.status == 'CHARGING':
                r.battery = min(100.0, r.battery + self.charge_rate * dt)
                if r.battery >= 99.5:
                    r.status = 'IDLE'
                    self.flog('info', f'{n} fully charged')
            elif (r.status == 'IDLE' and r.ready and not r.manual and not r.estop
                  and r.battery < self.low_battery and now >= self.dock_retry_at.get(n, 0.0)):
                self.flog('warn', f'{n} battery {r.battery:.0f}% - docking to charge')
                self.send_to_dock(r)

        self.pose_pub.publish(String(data=json.dumps({
            't': round(self._rel(), 2),
            'p': {n: [round(r.xy[0], 3), round(r.xy[1], 3), round(r.yaw, 3), round(r.speed, 2)]
                  for n, r in self.robots.items()}})))

    # ------------------------------------------------------------ allocation
    def allocate(self):
        if self.estop:
            return
        pending = [t for t in self.tasks.values() if t.status == 'PENDING']
        if not pending:
            return
        for task, robot in self.allocate_fn(pending, list(self.robots.values()), self.ctx):
            self.flog('info', f'Task {task.id} -> {robot.name}  [{self.alloc_name}]')
            task.status, task.robot, task.assigned = 'ASSIGNED', robot.name, self._rel()
            robot.task = task
            robot.status = 'TO_PICKUP'
            self.drive_to(robot, task.pickup, self.stations[task.pickup].get('yaw', 0.0),
                          task.priority, lambda ok, r=robot, t=task: self.after_pickup(r, t, ok),
                          target_xy=task.pickup_xy)

    # ------------------------------------------------------------ Nav2 single goal
    def go_to_pose(self, robot, xy, yaw, on_done, rid):
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y = float(xy[0]), float(xy[1])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose
        self.nav_clients[robot.name].send_goal_async(goal).add_done_callback(
            lambda f: self._goal_response(robot, f, on_done, rid))

    def _goal_response(self, robot, fut, on_done, rid):
        handle = fut.result()
        if rid != robot.route_id:                    # superseded while the goal was being sent
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            on_done(False)
            return
        robot.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._goal_result(robot, f, on_done, rid))

    def _goal_result(self, robot, fut, on_done, rid):
        if rid != robot.route_id:
            return
        robot.goal_handle = None
        on_done(fut.result().status == GoalStatus.STATUS_SUCCEEDED)

    # ------------------------------------------------------------ roadmap driving
    def drive_to(self, robot, node, yaw, priority, on_done, target_xy=None):
        robot.route_id += 1
        rid = robot.route_id
        robot.target_xy = target_xy or self.roadmap.nodes[node]
        if not self.traffic.set_goal(robot.name, node, priority):
            self.flog('error', f'{robot.name}: no route to {node}')
            self.traffic.settle(robot.name, self.roadmap.nearest_node(robot.xy))
            on_done(False)
            return
        self._advance(robot, rid, yaw, on_done)

    def _advance(self, robot, rid, yaw, on_done):
        if rid != robot.route_id:
            return
        kind, val = self.traffic.request(robot.name, self._now())
        if kind == 'DONE':
            nx, ny = self.roadmap.nodes[self.traffic.agents[robot.name].node]
            if dist(robot.xy, (nx, ny)) > 0.4:       # e.g. after manual driving: park exactly on the node
                self.go_to_pose(robot, (nx, ny), yaw, on_done, rid)
            else:
                on_done(True)
        elif kind == 'GO':
            self._segment(robot, rid, val, yaw, on_done, 0)
        elif kind == 'WAIT':
            self.call_later(0.5, lambda: self._advance(robot, rid, yaw, on_done))
        else:                                        # NOROUTE: drop goal + stale wait state
            self.traffic.settle(robot.name, self.roadmap.nearest_node(robot.xy))
            on_done(False)

    def _segment(self, robot, rid, node, final_yaw, on_done, tries):
        ag = self.traffic.agents[robot.name]
        x, y = self.roadmap.nodes[node]
        px, py = self.roadmap.nodes[ag.node]
        yaw = final_yaw if node == ag.goal else math.atan2(y - py, x - px)
        self.go_to_pose(robot, (x, y), yaw,
                        lambda ok: self._segment_done(robot, rid, node, final_yaw, on_done, tries, ok), rid)

    def _segment_done(self, robot, rid, node, final_yaw, on_done, tries, ok):
        if rid != robot.route_id:
            return
        if ok:
            self.traffic.arrived(robot.name)
            self._advance(robot, rid, final_yaw, on_done)
        elif tries < 2:
            self.flog('warn', f'{robot.name}: segment to {node} failed, retry {tries + 1}')
            self._segment(robot, rid, node, final_yaw, on_done, tries + 1)
        else:
            self.traffic.settle(robot.name, self.roadmap.nearest_node(robot.xy))
            on_done(False)

    # ------------------------------------------------------------ task flow
    def after_pickup(self, robot, task, ok):
        if robot.task is not task:
            return
        if not ok:
            return self.fail_task(robot, task, 'pickup navigation failed')
        robot.status = 'LOADING'
        robot.target_xy = None
        self.call_later(self.load_time, lambda: self.start_drop(robot, task))

    def start_drop(self, robot, task):
        if robot.task is not task or robot.status != 'LOADING':
            return
        robot.status = 'TO_DROP'
        self.drive_to(robot, task.dropoff, self.stations[task.dropoff].get('yaw', 0.0),
                      task.priority, lambda ok: self.after_drop(robot, task, ok),
                      target_xy=task.dropoff_xy)

    def after_drop(self, robot, task, ok):
        if robot.task is not task:
            return
        if not ok:
            return self.fail_task(robot, task, 'drop-off navigation failed')
        task.status, task.completed = 'DONE', self._rel()
        robot.task, robot.status, robot.target_xy = None, 'IDLE', None
        wait, cycle = task.assigned - task.created, task.completed - task.created
        with open(self.log_path, 'a', newline='') as f:
            csv.writer(f).writerow([task.id, robot.name, task.priority, task.pickup, task.dropoff,
                                    f'{task.created:.1f}', f'{task.assigned:.1f}',
                                    f'{task.completed:.1f}', f'{wait:.1f}', f'{cycle:.1f}',
                                    task.retries])
        self.flog('info', f'Task {task.id} DONE by {robot.name} (cycle {cycle:.0f}s)')
        self.allocate()                       # maybe there is more work right here
        if robot.status == 'IDLE' and not robot.manual and not robot.estop:
            self.send_to_dock(robot)          # no: get out of the corridor, go home

    def fail_task(self, robot, task, why):
        self.flog('warn', f'Task {task.id} on {robot.name}: {why}')
        self.release_task(robot, count_retry=True)
        robot.status, robot.target_xy = 'IDLE', None

    def send_to_dock(self, robot):
        robot.status = 'TO_DOCK'

        def done(ok):
            robot.target_xy = None
            robot.status = 'CHARGING' if (ok and robot.battery < 99.0) else 'IDLE'
            if ok:
                self.dock_retry_at.pop(robot.name, None)
            else:                                   # back off, or the low-battery check retries every tick
                self.dock_retry_at[robot.name] = self._now() + self.dock_retry_s
                self.flog('warn', f'{robot.name}: docking failed, retry in {self.dock_retry_s:.0f}s')
        self.drive_to(robot, robot.dock_node, robot.dock_yaw, 0, done, target_xy=robot.dock_xy)

    # ------------------------------------------------------------ monitoring
    def publish_status(self):
        done = [t for t in self.tasks.values() if t.status == 'DONE']
        waits = [t.assigned - t.created for t in done]
        cycles = [t.completed - t.created for t in done]
        robots = []
        for r in self.robots.values():
            tgt = None
            if r.target_xy is not None and r.status in TARGET_STATES:
                tgt = {'x': round(r.target_xy[0], 2), 'y': round(r.target_xy[1], 2),
                       'dist': round(self.roadmap.road_dist(r.xy, r.target_xy), 2)}
            robots.append({'name': r.name, 'x': round(r.xy[0], 2), 'y': round(r.xy[1], 2),
                           'yaw': round(r.yaw, 2), 'speed': round(r.speed, 2),
                           'status': r.status, 'mode': 'manual' if r.manual else 'auto',
                           'battery': round(r.battery, 1),
                           'task': r.task.id if r.task else None, 'target': tgt,
                           'dist': round(r.dist_travelled, 1), 'ready': r.ready})
        snap = {
            't': round(self._rel(), 1),
            'allocator': self.alloc_name,
            'estop': self.estop,
            'robots': robots,
            'traffic': self.traffic.snapshot(),
            'tasks': [{'id': t.id, 'pickup': t.pickup, 'dropoff': t.dropoff, 'payload': t.payload,
                       'priority': t.priority, 'status': t.status, 'robot': t.robot}
                      for t in self.tasks.values() if t.status in ('PENDING', 'ASSIGNED')],
            'stats': {'done': len(done),
                      'failed': sum(t.status == 'FAILED' for t in self.tasks.values()),
                      'pending': sum(t.status == 'PENDING' for t in self.tasks.values()),
                      'avg_wait_s': round(sum(waits) / len(waits), 1) if waits else 0.0,
                      'avg_cycle_s': round(sum(cycles) / len(cycles), 1) if cycles else 0.0,
                      'total_dist_m': round(sum(r.dist_travelled for r in self.robots.values()), 1)},
        }
        self.status_pub.publish(String(data=json.dumps(snap)))

    def publish_markers(self):
        """Roadmap in RViz: grey lines, nodes coloured by the robot that has reserved them."""
        ma = MarkerArray()

        def base(mid, ns, mtype):
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m
        lines = base(0, 'edges', Marker.LINE_LIST)
        lines.scale.x = 0.04
        lines.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.8)
        for a, b in self.roadmap.edges:
            for n in (a, b):
                x, y = self.roadmap.nodes[n]
                lines.points.append(Point(x=x, y=y, z=0.05))
        ma.markers.append(lines)

        nodes = base(1, 'nodes', Marker.SPHERE_LIST)
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.35
        for n, (x, y) in self.roadmap.nodes.items():
            nodes.points.append(Point(x=x, y=y, z=0.08))
            owner = self.traffic.owner.get(n)
            c = self.color[owner] if owner in self.color else (0.75, 0.75, 0.75)
            nodes.colors.append(ColorRGBA(r=c[0], g=c[1], b=c[2], a=1.0 if owner else 0.5))
        ma.markers.append(nodes)

        for i, (n, (x, y)) in enumerate(self.roadmap.nodes.items()):
            t = base(100 + i, 'labels', Marker.TEXT_VIEW_FACING)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = x, y + 0.35, 0.2
            t.scale.z = 0.28
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            t.text = n.replace('dock_', 'H:')
            ma.markers.append(t)
        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = FleetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
