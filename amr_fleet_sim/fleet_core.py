"""Pure-python fleet logic: data types, cost model and task allocators.

No ROS imports here so it can be unit-tested and benchmarked offline.
"""
import itertools
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

INFEASIBLE = 1e6


@dataclass
class Task:
    id: int
    pickup: str
    dropoff: str
    pickup_xy: Tuple[float, float]
    dropoff_xy: Tuple[float, float]
    priority: int = 1
    created: float = 0.0
    assigned: Optional[float] = None
    completed: Optional[float] = None
    status: str = 'PENDING'   # PENDING ASSIGNED DONE FAILED
    robot: Optional[str] = None
    retries: int = 0
    payload: str = ''


@dataclass
class Robot:
    name: str
    dock_xy: Tuple[float, float] = (0.0, 0.0)
    dock_yaw: float = 0.0
    dock_node: str = ''
    xy: Tuple[float, float] = (0.0, 0.0)
    status: str = 'IDLE'      # IDLE TO_PICKUP LOADING TO_DROP TO_DOCK CHARGING
    battery: float = 100.0
    task: Optional[Task] = None
    dist_travelled: float = 0.0
    ready: bool = False       # has a pose AND its Nav2 action server is up
    goal_handle: object = None
    manual: bool = False
    estop: bool = False
    route_id: int = 0
    target_xy: Optional[Tuple[float, float]] = None
    yaw: float = 0.0
    speed: float = 0.0

    @property
    def available(self) -> bool:
        return (self.ready and self.status == 'IDLE' and self.task is None
                and not self.manual and not self.estop)


@dataclass
class Context:
    drain_per_m: float = 0.5      # battery % lost per metre
    reserve: float = 15.0         # keep at least this much battery after a task
    w_priority: float = 2.0       # metres of "discount" per priority level
    dist_fn: Callable = None      # (xy, xy) -> metres; set to roadmap distance by the manager

    def d(self, a, b):
        return (self.dist_fn or dist)(a, b)


def dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def energy_needed(robot: Robot, task: Task, ctx: Context) -> float:
    metres = (ctx.d(robot.xy, task.pickup_xy)
              + ctx.d(task.pickup_xy, task.dropoff_xy)
              + ctx.d(task.dropoff_xy, robot.dock_xy))
    return metres * ctx.drain_per_m


def feasible(robot: Robot, task: Task, ctx: Context) -> bool:
    return robot.battery - energy_needed(robot, task, ctx) >= ctx.reserve


def cost(robot: Robot, task: Task, ctx: Context) -> float:
    """Lower is better. Swap this for real planner path length later."""
    if not feasible(robot, task, ctx):
        return INFEASIBLE
    return ctx.d(robot.xy, task.pickup_xy) - ctx.w_priority * task.priority


def _order(tasks: List[Task]) -> List[Task]:
    return sorted(tasks, key=lambda t: (-t.priority, t.created, t.id))


# ---------------------------------------------------------------- allocators
def greedy_nearest(tasks, robots, ctx):
    """Tasks in priority/FIFO order; each takes the cheapest free robot."""
    free = [r for r in robots if r.available]
    out = []
    for t in _order(tasks):
        if not free:
            break
        best = min(free, key=lambda r: cost(r, t, ctx))
        if cost(best, t, ctx) >= INFEASIBLE:
            continue
        out.append((t, best))
        free.remove(best)
    return out


def _min_cost_matching(c):
    """Rows x cols cost matrix -> list of (row, col). scipy if present, else brute force."""
    n = len(c)
    m = len(c[0]) if n else 0
    if n == 0 or m == 0:
        return []
    try:
        from scipy.optimize import linear_sum_assignment
        import numpy as np
        rows, cols = linear_sum_assignment(np.array(c))
        return list(zip(rows.tolist(), cols.tolist()))
    except ImportError:
        pass
    transpose = n > m
    if transpose:
        c = [[c[i][j] for i in range(n)] for j in range(m)]
        n, m = m, n
    best, best_cols = None, None
    for cols in itertools.permutations(range(m), n):
        s = sum(c[i][cols[i]] for i in range(n))
        if best is None or s < best:
            best, best_cols = s, cols
    pairs = [(i, best_cols[i]) for i in range(n)]
    return [(j, i) for i, j in pairs] if transpose else pairs


def hungarian(tasks, robots, ctx):
    """Globally optimal robot<->task assignment for this round (min total cost)."""
    free = [r for r in robots if r.available]
    ts = _order(tasks)
    if not free or not ts:
        return []
    c = [[cost(r, t, ctx) for t in ts] for r in free]
    out = []
    for i, j in _min_cost_matching(c):
        if c[i][j] < INFEASIBLE:
            out.append((ts[j], free[i]))
    return out


def auction(tasks, robots, ctx):
    """Parallel single-item auction.

    Each round every free robot bids (its cost) on the one task it likes best;
    each task is awarded to its lowest bidder. Repeat until no bids remain.
    Decentralisable: a robot only needs the task list and its own cost.
    """
    free = [r for r in robots if r.available]
    remaining = list(_order(tasks))
    out = []
    while free and remaining:
        bids: Dict[int, List[Tuple[float, Robot]]] = {}
        for r in free:
            options = [(cost(r, t, ctx), t) for t in remaining]
            options = [o for o in options if o[0] < INFEASIBLE]
            if not options:
                continue
            c, t = min(options, key=lambda o: o[0])
            bids.setdefault(t.id, []).append((c, r))
        if not bids:
            break
        by_id = {t.id: t for t in remaining}
        for tid, lst in bids.items():
            c, winner = min(lst, key=lambda x: x[0])
            out.append((by_id[tid], winner))
            free.remove(winner)
            remaining.remove(by_id[tid])
    return out


ALLOCATORS: Dict[str, Callable] = {
    'nearest': greedy_nearest,
    'hungarian': hungarian,
    'auction': auction,
}
