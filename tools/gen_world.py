#!/usr/bin/env python3
"""Generate Gazebo worlds (and, for generated layouts, the roadmap / stations / robots / map).

    python3 tools/gen_world.py                       # worlds/warehouse.world from config/*.yaml
    python3 tools/gen_world.py --world big_warehouse # config/big_warehouse/*.yaml, maps/big_warehouse.*,
                                                     # worlds/big_warehouse.world - all from BIG below

warehouse      the original 16 x 12 m hall. Its collision boxes never change so the SLAM map
               in maps/warehouse.* stays valid.
big_warehouse  42 x 32 m hall, 24 racks, ~100-node roadmap grid. Fully generated: the occupancy
               map is rasterised straight from the collision boxes, so no SLAM run is needed.

Everything robots can bump into (walls, racks) has an invisible collision box. Everything else
(lane tape, dock pads, station pads, stock on shelves, crates, hazard stripes) is VISUAL ONLY:
lidar and physics do not see it.
"""
import argparse
import math
import os
import random

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# same palette as the dashboard / robot bodies: cyan, violet, amber, green, pink
ROBOT_RGB = [(0.13, 0.83, 0.93), (0.55, 0.36, 0.96), (0.96, 0.62, 0.04), (0.06, 0.73, 0.50), (0.93, 0.27, 0.60)]
GOODS = [(0.72, 0.55, 0.35), (0.62, 0.46, 0.28), (0.20, 0.42, 0.72), (0.90, 0.90, 0.88), (0.90, 0.46, 0.10), (0.25, 0.60, 0.38)]
TAPE = (0.95, 0.78, 0.10)
WALL_T, WALL_H = 0.2, 1.5


def col(c, a=1.0):
    return f'{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a}'


def mat(c):
    return f'<material><ambient>{col(c)}</ambient><diffuse>{col(c)}</diffuse><specular>0.05 0.05 0.05 1</specular></material>'


class World:
    """Collects visuals + collision boxes; renders SDF and an occupancy map."""

    def __init__(self, seed=7):
        self.rnd = random.Random(seed)
        self.n = 0
        self.models = []
        self.collisions = {}          # name -> (cx, cy, cz, sx, sy, sz), all axis-aligned

    def vbox(self, x, y, z, sx, sy, sz, c, yaw=0.0, name=None):
        self.n += 1
        return (f'<visual name="{name or "v%d" % self.n}"><cast_shadows>false</cast_shadows>'
                f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>'
                f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>{mat(c)}</visual>')

    def vcyl(self, x, y, z, r, length, c):
        self.n += 1
        return (f'<visual name="v{self.n}"><cast_shadows>false</cast_shadows><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>'
                f'<geometry><cylinder><radius>{r}</radius><length>{length}</length></cylinder></geometry>{mat(c)}</visual>')

    def model(self, name, x, y, link_body, z=0.0):
        self.models.append(f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>'
                           f'<link name="l">{link_body}</link></model>')

    # ------------------------------------------------------------------ structure
    def walls(self, W, H, stripe=0.5):
        for name, cx, cy, sx, sy in [('wall_n', 0, H / 2, W, WALL_T), ('wall_s', 0, -H / 2, W, WALL_T),
                                     ('wall_e', W / 2, 0, WALL_T, H), ('wall_w', -W / 2, 0, WALL_T, H)]:
            body = f'<collision name="c"><pose>0 0 {WALL_H / 2} 0 0 0</pose><geometry><box><size>{sx} {sy} {WALL_H}</size></box></geometry></collision>'
            body += self.vbox(0, 0, WALL_H / 2, sx, sy, WALL_H, (0.27, 0.31, 0.40))
            body += self.vbox(0, 0, WALL_H + 0.02, sx + 0.04, sy + 0.04, 0.04, (0.62, 0.66, 0.74))   # cap
            horiz = sx > sy
            length = sx if horiz else sy
            inward = -1 if (cy > 0 if horiz else cx > 0) else 1                                      # towards the room
            n = int(length / stripe)
            for i in range(n):                                                                        # hazard stripe
                t = -length / 2 + stripe * (i + 0.5)
                c = (0.95, 0.75, 0.08) if i % 2 == 0 else (0.08, 0.08, 0.08)
                off = inward * (WALL_T / 2 + 0.005)
                body += (self.vbox(t, off, 0.09, stripe, 0.012, 0.18, c) if horiz
                         else self.vbox(off, t, 0.09, 0.012, stripe, 0.18, c))
            self.model(name, cx, cy, body)
            self.collisions[name] = (cx, cy, WALL_H / 2, sx, sy, WALL_H)

    def shelf(self, name, x, y, sx=2.0, sy=1.0, h=1.2, levels=3):
        """Rack: uprights every ~2 m, `levels` loaded boards + a top board, one solid collision box."""
        bays = max(1, round(sx / 2.0))
        body = f'<collision name="c"><pose>0 0 {h / 2} 0 0 0</pose><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>'
        ex, ey = sx / 2 - 0.03, sy / 2 - 0.03
        for ux in [-ex + 2 * ex * k / bays for k in range(bays + 1)]:
            for uy in (-ey, ey):
                body += self.vbox(ux, uy, h / 2, 0.06, 0.06, h, (0.12, 0.30, 0.72))
        step = (h - 0.06) / levels if levels != 3 or h != 1.2 else 0.4
        boards = [0.03 + step * k for k in range(levels)]
        for zb in boards + [h - 0.03]:
            body += self.vbox(0, 0, zb, sx - 0.06, sy - 0.06, 0.04, (0.74, 0.77, 0.82))
        gx, gy = sx / 2 - 0.25, sy / 2 - 0.25
        for zb in boards:
            for _ in range(self.rnd.randint(3, 5) * bays):
                bx, by, bz = self.rnd.uniform(0.28, 0.5), self.rnd.uniform(0.3, 0.6), self.rnd.uniform(0.18, 0.32)
                body += self.vbox(self.rnd.uniform(-gx, gx), self.rnd.uniform(-gy, gy), zb + 0.02 + bz / 2,
                                  bx, by, bz, self.rnd.choice(GOODS))
        self.model(name, x, y, body)
        self.collisions[name] = (x, y, h / 2, sx, sy, h)

    # ------------------------------------------------------------------ floor decoration (one model, visual only)
    def floor(self, W, H, roadmap, stations, robots, crates):
        d, vbox, rnd = [], self.vbox, self.rnd
        d.append(vbox(0, 0, 0.001, W - 0.2, H - 0.2, 0.002, (0.50, 0.53, 0.58)))                  # concrete slab
        for gx in range(-int(W / 2) + 1, int(W / 2), 2):                                          # tile seams
            d.append(vbox(gx, 0, 0.0022, 0.012, H - 0.2, 0.001, (0.40, 0.43, 0.48)))
        for gy in range(-int(H / 2) + 1, int(H / 2), 2):
            d.append(vbox(0, gy, 0.0022, W - 0.2, 0.012, 0.001, (0.40, 0.43, 0.48)))
        nodes = roadmap['nodes']
        for a, b in roadmap['edges']:                                                             # lane tape on every edge
            (x1, y1), (x2, y2) = nodes[a], nodes[b]
            d.append(vbox((x1 + x2) / 2, (y1 + y2) / 2, 0.0035, math.hypot(x2 - x1, y2 - y1), 0.09, 0.002, TAPE,
                          yaw=math.atan2(y2 - y1, x2 - x1)))
        for k, (x, y) in nodes.items():                                                           # junction discs
            if not k.startswith('dock_') and k not in stations:
                d.append(self.vcyl(x, y, 0.0038, 0.22, 0.002, TAPE))

        # dock bays: coloured pad, charging strip, rear stopper, charger tower, N bars = robot number
        for idx, r in enumerate(robots):
            c = ROBOT_RGB[idx % len(ROBOT_RGB)]
            x, y = r['x'], r['y']
            d.append(vbox(x, y, 0.0045, 1.15, 1.15, 0.003, tuple(v * 0.75 for v in c)))
            for sx_, sy_, sw, sl in ((0, 0.575, 1.15, 0.05), (0, -0.575, 1.15, 0.05)):              # pad border
                d.append(vbox(x + sx_, y + sy_, 0.0065, sw, sl, 0.002, c))
            d.append(vbox(x + 0.575, y, 0.0065, 0.05, 1.15, 0.002, c))
            d.append(vbox(x - 0.20, y, 0.0075, 0.55, 0.12, 0.003, (0.12, 0.12, 0.14)))            # charging strip
            d.append(vbox(x - 0.55, y, 0.05, 0.08, 1.15, 0.10, (0.14, 0.15, 0.18)))               # stopper
            d.append(vbox(x - 0.72, y, 0.30, 0.16, 0.42, 0.60, (0.20, 0.22, 0.27)))               # charger tower
            d.append(vbox(x - 0.72, y, 0.62, 0.17, 0.43, 0.06, c))
            for n in range(idx + 1):                                                               # number bars
                d.append(vbox(x + 0.42, y + (n - idx / 2) * 0.12, 0.0085, 0.05, 0.08, 0.003, (0.96, 0.96, 0.96)))

        # stations: pads with border + corner posts. P* = green (pickup), D* = orange (drop-off)
        for k, s in stations.items():
            c = (0.10, 0.70, 0.40) if k.startswith('P') else (0.95, 0.52, 0.12)
            x, y = s['x'], s['y']
            d.append(vbox(x, y, 0.0045, 1.5, 1.5, 0.003, tuple(v * 0.7 for v in c)))
            for dx, dy, sx_, sy_ in ((0, 0.72, 1.5, 0.06), (0, -0.72, 1.5, 0.06), (0.72, 0, 0.06, 1.5), (-0.72, 0, 0.06, 1.5)):
                d.append(vbox(x + dx, y + dy, 0.0065, sx_, sy_, 0.002, (0.96, 0.96, 0.96)))
            for dx in (-0.72, 0.72):
                for dy in (-0.72, 0.72):
                    d.append(vbox(x + dx, y + dy, 0.2, 0.07, 0.07, 0.4, (0.9, 0.9, 0.9)))
                    d.append(vbox(x + dx, y + dy, 0.42, 0.10, 0.10, 0.05, c))
            for n in range(int(k[1:]) if k[1:].isdigit() else 1):                                  # bars = station number
                d.append(vbox(x - 0.45 + n * 0.12, y + 0.45, 0.0085, 0.06, 0.24, 0.003, (0.96, 0.96, 0.96)))

        # stock crates (clear of every lane)
        for cx, cy in crates:
            for lvl in range(rnd.randint(1, 2)):
                s = rnd.uniform(0.40, 0.55)
                d.append(vbox(cx + rnd.uniform(-0.04, 0.04), cy, s / 2 + lvl * 0.45, s, s, s, rnd.choice(GOODS),
                              yaw=rnd.uniform(-0.3, 0.3)))
        self.model('decor_floor', 0, 0, ''.join(d))

    # ------------------------------------------------------------------ output
    def sdf(self, camera):
        return f"""<?xml version="1.0"?>
<!-- GENERATED by tools/gen_world.py - edit the script, not this file. -->
<sdf version="1.6">
  <world name="warehouse">
    <scene><ambient>0.62 0.62 0.66 1</ambient><background>0.08 0.10 0.16 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.75 0.75 0.72 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 0.2 -1</direction></light>
    <include><uri>model://ground_plane</uri></include>
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gui><camera name="user_camera"><pose>{camera}</pose></camera></gui>
    {''.join(self.models)}
  </world>
</sdf>
"""

    def occupancy(self, W, H, res=0.05, margin=0.5):
        """Trinary PGM straight from the collision boxes, drawn the way a lidar / SLAM map sees them.

        Hall free, outside unknown, and every obstacle as an occupied OUTLINE with an unknown
        interior. Solid blocks would break AMCL: its likelihood-field model scores laser endpoints
        by distance to the nearest occupied cell without ray casting, so a particle INSIDE a solid
        4 x 2 m rack sees every endpoint "on an obstacle" and wins - the filter jumps into racks.
        """
        x0, y0 = -W / 2 - margin, -H / 2 - margin
        cols, rows = int(round((W + 2 * margin) / res)), int(round((H + 2 * margin) / res))
        FREE, OCC, UNK = 254, 0, 205
        img = bytearray([UNK]) * (cols * rows)

        def fill(xa, xb, ya, yb, v):
            ca, cb = max(0, int(math.floor((xa - x0) / res))), min(cols, int(math.ceil((xb - x0) / res)))
            for wy in range(max(0, int(math.floor((ya - y0) / res))), min(rows, int(math.ceil((yb - y0) / res)))):
                r = rows - 1 - wy                                                 # image row 0 = top (max y)
                img[r * cols + ca:r * cols + cb] = bytes([v]) * (cb - ca)
        fill(-W / 2, W / 2, -H / 2, H / 2, FREE)
        edge = 0.1                                                         # outline thickness (2 cells)
        for cx, cy, _, sx, sy, _ in self.collisions.values():
            fill(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, OCC)
            if sx > 2 * edge + res and sy > 2 * edge + res:
                fill(cx - sx / 2 + edge, cx + sx / 2 - edge, cy - sy / 2 + edge, cy + sy / 2 - edge, UNK)
        pgm = f'P5\n{cols} {rows}\n255\n'.encode() + bytes(img)
        info = {'resolution': res, 'origin': [round(x0, 3), round(y0, 3), 0]}
        return pgm, info


def _write(path, text, mode='w'):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, mode) as f:
        f.write(text)
    return path


# ====================================================================== small hall (hand-made configs)
def small():
    cfg = lambda n: yaml.safe_load(open(os.path.join(ROOT, 'config', n)))
    roadmap, stations, robots = cfg('roadmap.yaml'), cfg('stations.yaml')['stations'], cfg('robots.yaml')['robots']
    W, H = 16.0, 12.0
    w = World(seed=7)
    w.walls(W, H)
    i = 0
    for x in (-2.0, 2.0, 6.0):
        for y in (-3.0, 3.0):
            i += 1
            w.shelf(f'shelf_{i}', x, y)
    w.floor(W, H, roadmap, stations, robots,
            crates=((-7.3, 5.2), (-6.5, 5.45), (-7.3, -5.3), (-6.6, -5.45), (7.3, 5.4), (7.3, -5.4)))
    sdf = w.sdf('0 -9.5 12 0 0.95 1.5708')
    print('wrote', _write(os.path.join(ROOT, 'worlds', 'warehouse.world'), sdf), f'({w.n} visuals)')


# ====================================================================== big hall (everything generated)
BIG = dict(
    W=42.0, H=32.0,
    cols=[-17.0, -11.0, -5.0, 1.0, 7.0, 13.0, 17.0],    # north-south lanes (west spine ... east spine)
    rows=[-12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0],      # east-west aisles
    rack=(4.0, 2.0, 1.8),                               # rack length, depth, height (2 m aisles left over)
    n_robots=4,                                         # docks exist for up to 7 (one per aisle row)
)


def big():
    W, H, xs, ys = BIG['W'], BIG['H'], BIG['cols'], BIG['rows']
    rl, rd, rh = BIG['rack']
    rack_x = [(a + b) / 2 for a, b in zip(xs[1:-2], xs[2:-1])]      # racks between the inner lanes
    rack_y = [(a + b) / 2 for a, b in zip(ys, ys[1:])]

    nodes, edges = {}, []
    name = lambda x, y: f'N_{x:+.0f}_{y:+.0f}'.replace('+', 'p').replace('-', 'm')
    mids = [(a + b) / 2 for a, b in zip(xs, xs[1:])]                 # an aisle node in front of every rack + staging
    for y in ys:
        row = sorted([(x, name(x, y)) for x in xs + mids])
        for x, n in row:
            nodes[n] = [x, y]
        edges += [[a, b] for (_, a), (_, b) in zip(row, row[1:])]
    for x in xs:                                                     # lanes link the aisles
        edges += [[name(x, a), name(x, b)] for a, b in zip(ys, ys[1:])]

    stations, labels = {}, {}

    def station(key, at, label, yaw, spur_from=None):
        nonlocal nodes
        if spur_from:                                                # dead-end bay off a lane
            nodes[key] = list(at)
            edges.append([spur_from, key])
        else:                                                        # rename an aisle node
            old = name(*at)
            nodes[key] = nodes.pop(old)
            for e in edges:
                e[:] = [key if v == old else v for v in e]
        stations[key] = {'x': float(at[0]), 'y': float(at[1]), 'yaw': yaw, 'label': label}

    east, north = W / 2 - 1.5, H / 2 - 1.7
    for i, y in enumerate([-8.0, -4.0, 4.0, 8.0]):                  # inbound trucks, east wall
        station(f'P{i + 1}', (east, y), f'Inbound Bay {"ABCD"[i]}', 0.0, spur_from=name(xs[-1], y))
    for i, x in enumerate(rack_x):                                   # outbound / shipping, north wall
        station(f'D{i + 1}', (x, north), f'Shipping Dock {i + 1}', 1.5708, spur_from=name(x, ys[-1]))
    aisle = 'ABCDEFG'
    for i, (x, y) in enumerate([(rack_x[0], -4.0), (rack_x[2], -8.0), (rack_x[1], 4.0), (rack_x[3], 8.0)]):
        station(f'P{i + 5}', (x, y), f'Pick Face {aisle[ys.index(y)]}{rack_x.index(x) + 1}', 0.0)
    for i, (x, y) in enumerate([(rack_x[1], -8.0), (rack_x[3], -4.0), (rack_x[0], 4.0), (rack_x[2], 0.0)]):
        station(f'D{i + 5}', (x, y), f'Put-away {aisle[ys.index(y)]}{rack_x.index(x) + 1}', 0.0)

    robots = []
    dock_x = -W / 2 + 1.5
    for i, y in enumerate([-8.0, -4.0, 0.0, 4.0, 8.0, -12.0, 12.0][:BIG['n_robots']]):
        n = f'amr{i + 1}'
        robots.append({'name': n, 'x': dock_x, 'y': y, 'yaw': 0.0})
        nodes[f'dock_{n}'] = [dock_x, y]
        edges.append([f'dock_{n}', name(xs[0], y)])
    roadmap = {'nodes': nodes, 'edges': edges}

    # -------- world
    w = World(seed=11)
    w.walls(W, H, stripe=1.0)
    k = 0
    for y in rack_y:
        for x in rack_x:
            k += 1
            w.shelf(f'rack_{k}', x, y, rl, rd, rh, levels=3)
    hx, hy = W / 2 - 0.7, H / 2 - 0.7
    w.floor(W, H, roadmap, stations, robots,
            crates=((-hx, hy), (-hx + 0.8, hy), (-hx, -hy), (-hx + 0.8, -hy), (hx, -hy), (hx - 0.8, -hy)))
    sdf = w.sdf('0 -30 34 0 0.92 1.5708')
    pgm, info = w.occupancy(W, H)

    # -------- write everything
    cdir = os.path.join(ROOT, 'config', 'big_warehouse')
    hdr = '# GENERATED by tools/gen_world.py --world big_warehouse - edit BIG in the script, not this file.\n'
    fmt = lambda v: f'[{v[0]:.1f}, {v[1]:.1f}]'
    rm_txt = hdr + f'# {len(nodes)} nodes, {len(edges)} edges. Coordinates in the map frame.\nnodes:\n'
    rm_txt += ''.join(f'  {n}: {fmt(v)}\n' for n, v in nodes.items())
    rm_txt += 'edges:\n' + ''.join(f'  - [{a}, {b}]\n' for a, b in edges)
    st_txt = hdr + '# P* = pickup, D* = drop-off. Every station is a roadmap node.\nstations:\n' + ''.join(
        f'  {k}: {{x: {s["x"]:.1f}, y: {s["y"]:.1f}, yaw: {s["yaw"]}, label: "{s["label"]}"}}\n' for k, s in stations.items())
    ro_txt = hdr + '# Docks are on the west wall, one per aisle row (up to 7 robots: raise BIG["n_robots"]).\nrobots:\n' + ''.join(
        f'  - {{name: {r["name"]}, x: {r["x"]:.1f}, y: {r["y"]:.1f}, yaw: {r["yaw"]}}}\n' for r in robots)
    map_yaml = (f'image: big_warehouse.pgm\nmode: trinary\nresolution: {info["resolution"]}\n'
                f'origin: {info["origin"]}\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    for p in (_write(os.path.join(cdir, 'roadmap.yaml'), rm_txt),
              _write(os.path.join(cdir, 'stations.yaml'), st_txt),
              _write(os.path.join(cdir, 'robots.yaml'), ro_txt),
              _write(os.path.join(ROOT, 'maps', 'big_warehouse.pgm'), pgm, 'wb'),
              _write(os.path.join(ROOT, 'maps', 'big_warehouse.yaml'), map_yaml),
              _write(os.path.join(ROOT, 'worlds', 'big_warehouse.world'), sdf)):
        print('wrote', os.path.relpath(p, ROOT))
    print(f'big_warehouse: {len(nodes)} nodes, {len(edges)} edges, {len(stations)} stations, '
          f'{len(robots)} robots, {k} racks, {w.n} visuals, world {len(sdf) / 1024:.0f} KB')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='warehouse', choices=['warehouse', 'big_warehouse'])
    {'warehouse': small, 'big_warehouse': big}[ap.parse_args().world]()
