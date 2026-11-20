"""Traffic management (no ROS): roadmap graph + node reservations + deadlock handling.

Rule: a robot may only drive from node A to the next node B after it has *reserved* B.
It keeps A until it arrives at B (so it always holds the node it stands on and the one it
is driving to). A node has at most one owner -> two robots can never be in the same
junction/corridor segment at once, and head-on meetings are impossible.

One-node lookahead: a robot will not step into a node if the node after it is held by another
robot (prevents plugging the only entrance of a dead-end station).

Deadlock handling (two robots waiting for each other's node):
  1. detect a cycle in the wait-for graph;
  2. the lowest-priority robot in the cycle is the victim;
  3. victim re-plans avoiding every node held by others; if there is no such route it
     yields: steps aside to a free neighbour node, then re-plans.
Also: any robot blocked longer than `patience` seconds tries to re-plan around the blocker.
"""
import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


def _d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class RoadMap:
    def __init__(self, nodes: Dict[str, Tuple[float, float]], edges):
        self.nodes = {k: (float(v[0]), float(v[1])) for k, v in nodes.items()}
        self.adj = {k: {} for k in self.nodes}
        for a, b in edges:
            if a not in self.nodes or b not in self.nodes:
                raise ValueError(f'edge {a}-{b} uses unknown node')
            w = _d(self.nodes[a], self.nodes[b])
            self.adj[a][b] = w
            self.adj[b][a] = w
        self.edges = [tuple(e) for e in edges]

    def nearest_node(self, xy) -> str:
        return min(self.nodes, key=lambda n: _d(xy, self.nodes[n]))

    def shortest_path(self, src, dst, blocked=frozenset()) -> Optional[List[str]]:
        """Dijkstra. `blocked` nodes cannot be used (src is always allowed)."""
        if src == dst:
            return [src]
        if dst in blocked:
            return None
        best = {src: 0.0}
        prev = {}
        pq = [(0.0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == dst:
                break
            if d > best.get(u, math.inf):
                continue
            for v, w in self.adj[u].items():
                if v in blocked:
                    continue
                nd = d + w
                if nd < best.get(v, math.inf):
                    best[v], prev[v] = nd, u
                    heapq.heappush(pq, (nd, v))
        if dst not in prev:
            return None
        path = [dst]
        while path[-1] != src:
            path.append(prev[path[-1]])
        return path[::-1]

    def path_length(self, path) -> float:
        return sum(self.adj[a][b] for a, b in zip(path, path[1:]))

    def road_dist(self, a_xy, b_xy) -> float:
        """Travel distance along the roadmap between two arbitrary points (for allocation cost)."""
        na, nb = self.nearest_node(a_xy), self.nearest_node(b_xy)
        p = self.shortest_path(na, nb)
        if p is None:
            return 1e5
        return _d(a_xy, self.nodes[na]) + self.path_length(p) + _d(self.nodes[nb], b_xy)


@dataclass
class _Agent:
    name: str
    node: str
    priority: int = 1
    goal: Optional[str] = None
    route: List[str] = field(default_factory=list)   # nodes still to visit (route[0] = next)
    moving_to: Optional[str] = None
    wait_since: Optional[float] = None
    wait_on: Optional[str] = None


class TrafficManager:
    def __init__(self, roadmap: RoadMap, patience: float = 8.0, aging: float = 20.0):
        self.map = roadmap
        self.patience = patience
        self.aging = aging            # seconds of waiting worth +1 priority level
        self.last_move = {}           # robot -> time it last made progress
        self._clock = 0.0
        self.agents: Dict[str, _Agent] = {}
        self.owner: Dict[str, str] = {}

    # ---------------------------------------------------------------- setup
    def register(self, name, node, priority=1):
        if node in self.owner:
            raise ValueError(f'{node} already owned by {self.owner[node]}')
        self.agents[name] = _Agent(name, node, priority)
        self.owner[node] = name

    def set_goal(self, name, goal, priority=1) -> bool:
        a = self.agents[name]
        if a.moving_to is not None:
            return False
        path = self.map.shortest_path(a.node, goal)
        if path is None:
            return False
        a.goal, a.priority = goal, priority
        a.route = path[1:]
        a.wait_since = a.wait_on = None
        return True

    # ---------------------------------------------------------------- main API
    def request(self, name, now) -> Tuple[str, Optional[str]]:
        """Returns ('DONE'|'GO'|'WAIT'|'NOROUTE', node-or-blocker)."""
        a = self.agents[name]
        self._clock = now
        self.last_move.setdefault(name, now)
        for _ in range(3):
            if a.node == a.goal:
                return 'DONE', None
            if not a.route:                       # e.g. after a yield: plan again
                if not (self._replan(a, True) or self._replan(a, False)):
                    return 'NOROUTE', None
            nxt = a.route[0]
            holder = self.owner.get(nxt)
            if holder in (None, name):
                # one-node lookahead: do not step into `nxt` if the node after it is held by
                # someone else - we would just block the corridor (and that robot's only exit,
                # e.g. a dead-end station) while waiting.
                look = a.route[1] if len(a.route) > 1 else None
                holder = self.owner.get(look) if look else None
                if holder in (None, name):
                    self.owner[nxt] = name
                    a.moving_to = nxt
                    a.wait_since = a.wait_on = None
                    return 'GO', nxt

            # blocked
            if a.wait_since is None:
                a.wait_since = now
            a.wait_on = holder
            cyc = self._cycle(name)
            victim = bool(cyc) and self._victim(cyc, now) == name
            timed_out = (now - a.wait_since) >= self.patience
            if not (victim or timed_out):
                return 'WAIT', holder
            if self._replan(a, True):
                a.wait_since = a.wait_on = None
                continue
            if victim and self._yield(a):
                a.wait_since = a.wait_on = None
                continue
            if timed_out:
                a.wait_since = now                 # try again after another patience period
            return 'WAIT', holder
        return 'WAIT', a.wait_on

    def arrived(self, name):
        """Robot reached the node it was granted."""
        a = self.agents[name]
        old, new = a.node, a.moving_to
        if new is None:
            return
        a.node, a.moving_to = new, None
        self.last_move[name] = self._clock
        if a.route and a.route[0] == new:
            a.route.pop(0)
        if old != new and self.owner.get(old) == name:
            del self.owner[old]

    def settle(self, name, node):
        """Navigation failed: robot is somewhere near `node`; drop every other reservation."""
        a = self.agents[name]
        mine = [n for n, o in self.owner.items() if o == name]
        if node not in mine:
            node = a.node if a.node in mine else mine[0]
        for n in mine:
            if n != node:
                del self.owner[n]
        a.node, a.moving_to = node, None
        a.route, a.goal = [], None

    # ---------------------------------------------------------------- internals
    def _others(self, name):
        return {n for n, o in self.owner.items() if o != name}

    def _replan(self, a, avoid_others) -> bool:
        blocked = self._others(a.name) if avoid_others else frozenset()
        p = self.map.shortest_path(a.node, a.goal, blocked)
        if p is None or p[1:] == a.route:
            return False
        a.route = p[1:]
        return True

    def _yield(self, a) -> bool:
        blocked_next = a.route[0] if a.route else None
        free = [n for n in self.map.adj[a.node]
                if n != blocked_next and n not in self.owner]
        if not free:
            return False

        def remaining(n):
            p = self.map.shortest_path(n, a.goal, frozenset({a.node}))
            return self.map.adj[a.node][n] + (self.map.path_length(p) if p else 1e3)
        a.route = [min(free, key=remaining)]
        return True

    def _cycle(self, name) -> Optional[List[str]]:
        seen = [name]
        cur = name
        while True:
            nxt = self.agents[cur].wait_on
            if nxt is None or nxt not in self.agents:
                return None
            if nxt == name:
                return seen
            if nxt in seen:
                return None
            seen.append(nxt)
            cur = nxt

    def _can_replan(self, a) -> bool:
        p = self.map.shortest_path(a.node, a.goal, self._others(a.name))
        return p is not None and p[1:] != a.route

    def _can_yield(self, a) -> bool:
        nxt = a.route[0] if a.route else None
        return any(n != nxt and n not in self.owner for n in self.map.adj[a.node])

    def _eff_priority(self, name, now):
        a = self.agents[name]
        return a.priority + (now - self.last_move.get(name, now)) / self.aging

    def _victim(self, cyc, now):
        """Who gets out of the way when robots wait for each other?
        1) robots that can drive AROUND the blocker (productive) - lowest effective priority first,
        2) else robots that can only step aside (to a free neighbour),
        3) else nobody can; the lowest priority keeps trying.
        Effective priority = task priority + time since last progress / aging (no starvation)."""
        ordered = sorted(cyc, key=lambda n: (self._eff_priority(n, now), n))
        for n in ordered:
            if self._can_replan(self.agents[n]):
                return n
        for n in ordered:
            if self._can_yield(self.agents[n]):
                return n
        return ordered[0]

    def snapshot(self):
        return {n: {'node': a.node, 'goal': a.goal, 'moving_to': a.moving_to,
                    'route': list(a.route), 'waiting_on': a.wait_on}
                for n, a in self.agents.items()}
