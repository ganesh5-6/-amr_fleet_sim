#!/usr/bin/env python3
"""Generate worlds/warehouse.world from the project configs.

    python3 tools/gen_world.py            # writes worlds/warehouse.world

Everything that robots can bump into (walls, shelves) keeps the exact same invisible
collision boxes as the original world, so a map saved earlier stays valid. All new things
(lane tape, dock pads, station pads, stock on shelves, crates, hazard stripes) are
VISUAL ONLY: lidar and physics do not see them.
"""
import math
import os
import random
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
cfg = lambda n: yaml.safe_load(open(os.path.join(ROOT, 'config', n)))
roadmap, stations, robots = cfg('roadmap.yaml'), cfg('stations.yaml')['stations'], cfg('robots.yaml')['robots']

# same palette as the dashboard / robot bodies: cyan, violet, amber, green, pink
ROBOT_RGB = [(0.13, 0.83, 0.93), (0.55, 0.36, 0.96), (0.96, 0.62, 0.04), (0.06, 0.73, 0.50), (0.93, 0.27, 0.60)]
W, H, WALL_T, WALL_H = 16.0, 12.0, 0.2, 1.5
rnd = random.Random(7)


def col(c, a=1.0):
    return f'{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a}'


def mat(c):
    return f'<material><ambient>{col(c)}</ambient><diffuse>{col(c)}</diffuse><specular>0.05 0.05 0.05 1</specular></material>'


_n = [0]


def vbox(x, y, z, sx, sy, sz, c, yaw=0.0, name=None):
    _n[0] += 1
    return (f'<visual name="{name or "v%d" % _n[0]}"><cast_shadows>false</cast_shadows>'
            f'<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>'
            f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>{mat(c)}</visual>')


def vcyl(x, y, z, r, l, c):
    _n[0] += 1
    return (f'<visual name="v{_n[0]}"><cast_shadows>false</cast_shadows><pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>'
            f'<geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry>{mat(c)}</visual>')


def model(name, x, y, link_body, z=0.0):
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>'
            f'<link name="l">{link_body}</link></model>')


out = []

# ------------------------------------------------------------------ walls (collision identical to old world)
WALLS = [('wall_n', 0, H / 2, W, WALL_T), ('wall_s', 0, -H / 2, W, WALL_T),
         ('wall_e', W / 2, 0, WALL_T, H), ('wall_w', -W / 2, 0, WALL_T, H)]
collisions = {}
for name, cx, cy, sx, sy in WALLS:
    body = f'<collision name="c"><pose>0 0 {WALL_H / 2} 0 0 0</pose><geometry><box><size>{sx} {sy} {WALL_H}</size></box></geometry></collision>'
    body += vbox(0, 0, WALL_H / 2, sx, sy, WALL_H, (0.27, 0.31, 0.40))
    body += vbox(0, 0, WALL_H + 0.02, sx + 0.04, sy + 0.04, 0.04, (0.62, 0.66, 0.74))        # cap
    horiz = sx > sy
    length = sx if horiz else sy
    inward = -1 if (cy > 0 if horiz else cx > 0) else 1                                      # towards the room
    seg, n = 0.5, int(length / 0.5)
    for i in range(n):                                                                        # hazard stripe
        t = -length / 2 + seg * (i + 0.5)
        c = (0.95, 0.75, 0.08) if i % 2 == 0 else (0.08, 0.08, 0.08)
        off = inward * (WALL_T / 2 + 0.005)
        body += vbox(t, off, 0.09, seg, 0.012, 0.18, c) if horiz else vbox(off, t, 0.09, 0.012, seg, 0.18, c)
    out.append(model(name, cx, cy, body))
    collisions[name] = (cx, cy, WALL_H / 2, sx, sy, WALL_H)

# ------------------------------------------------------------------ shelves (collision identical to old world)
GOODS = [(0.72, 0.55, 0.35), (0.62, 0.46, 0.28), (0.20, 0.42, 0.72), (0.90, 0.90, 0.88), (0.90, 0.46, 0.10), (0.25, 0.60, 0.38)]
i = 0
for x in (-2.0, 2.0, 6.0):
    for y in (-3.0, 3.0):
        i += 1
        body = '<collision name="c"><pose>0 0 0.6 0 0 0</pose><geometry><box><size>2.0 1.0 1.2</size></box></geometry></collision>'
        for ux in (-0.97, 0.97):
            for uy in (-0.47, 0.47):
                body += vbox(ux, uy, 0.6, 0.06, 0.06, 1.2, (0.12, 0.30, 0.72))
        for zb in (0.03, 0.43, 0.83, 1.17):
            body += vbox(0, 0, zb, 1.94, 0.94, 0.04, (0.74, 0.77, 0.82))
        for zb in (0.05, 0.45, 0.85):
            for _ in range(rnd.randint(3, 5)):
                sx, sy, sz = rnd.uniform(0.28, 0.5), rnd.uniform(0.3, 0.6), rnd.uniform(0.18, 0.32)
                body += vbox(rnd.uniform(-0.75, 0.75), rnd.uniform(-0.25, 0.25), zb + sz / 2, sx, sy, sz, rnd.choice(GOODS))
        out.append(model(f'shelf_{i}', x, y, body))
        collisions[f'shelf_{i}'] = (x, y, 0.6, 2.0, 1.0, 1.2)

# ------------------------------------------------------------------ floor decoration (one model, visual only)
d = []
d.append(vbox(0, 0, 0.001, W - 0.2, H - 0.2, 0.002, (0.50, 0.53, 0.58)))                     # concrete slab
for gx in range(-7, 8, 2):                                                                    # tile seams
    d.append(vbox(gx, 0, 0.0022, 0.012, H - 0.2, 0.001, (0.40, 0.43, 0.48)))
for gy in range(-5, 6, 2):
    d.append(vbox(0, gy, 0.0022, W - 0.2, 0.012, 0.001, (0.40, 0.43, 0.48)))
nodes = roadmap['nodes']
TAPE = (0.95, 0.78, 0.10)
for a, b in roadmap['edges']:                                                                 # lane tape on every roadmap edge
    (x1, y1), (x2, y2) = nodes[a], nodes[b]
    d.append(vbox((x1 + x2) / 2, (y1 + y2) / 2, 0.0035, math.hypot(x2 - x1, y2 - y1), 0.09, 0.002, TAPE, yaw=math.atan2(y2 - y1, x2 - x1)))
for k, (x, y) in nodes.items():                                                               # junction discs
    if not k.startswith('dock_') and k not in stations:
        d.append(vcyl(x, y, 0.0038, 0.22, 0.002, TAPE))

# dock bays: coloured pad, charging strip + LED, rear stopper, charger tower, N bars = robot number
for idx, r in enumerate(robots):
    c = ROBOT_RGB[idx % len(ROBOT_RGB)]
    x, y = r['x'], r['y']
    light = tuple(min(1, v + 0.35) for v in c)
    d.append(vbox(x, y, 0.0045, 1.15, 1.15, 0.003, tuple(v * 0.75 for v in c)))
    for sx_, sy_, sw, sl in ((0, 0.575, 1.15, 0.05), (0, -0.575, 1.15, 0.05)):                # pad border
        d.append(vbox(x + sx_, y + sy_, 0.0065, sw, sl, 0.002, c))
    d.append(vbox(x + 0.575, y, 0.0065, 0.05, 1.15, 0.002, c))
    d.append(vbox(x - 0.20, y, 0.0075, 0.55, 0.12, 0.003, (0.12, 0.12, 0.14)))              # charging strip
    d.append(vbox(x + 0.05, y, 0.0085, 0.06, 0.12, 0.004, light))                           # LED
    d.append(vbox(x - 0.55, y, 0.05, 0.08, 1.15, 0.10, (0.14, 0.15, 0.18)))                 # stopper
    d.append(vbox(x - 0.72, y, 0.30, 0.16, 0.42, 0.60, (0.20, 0.22, 0.27)))                 # charger tower
    d.append(vbox(x - 0.72, y, 0.62, 0.17, 0.43, 0.06, c))
    d.append(vbox(x - 0.64, y, 0.40, 0.02, 0.18, 0.14, light))
    for n in range(idx + 1):                                                                 # number bars
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
    for n in range(int(k[1:]) if k[1:].isdigit() else 1):                                    # bars = station number
        d.append(vbox(x - 0.45 + n * 0.12, y + 0.45, 0.0085, 0.06, 0.24, 0.003, (0.96, 0.96, 0.96)))

# stock crates in the corners (clear of every lane)
for cx, cy in ((-7.3, 5.2), (-6.5, 5.45), (-7.3, -5.3), (-6.6, -5.45), (7.3, 5.4), (7.3, -5.4)):
    for lvl in range(rnd.randint(1, 2)):
        s = rnd.uniform(0.40, 0.55)
        d.append(vbox(cx + rnd.uniform(-0.04, 0.04), cy, s / 2 + lvl * 0.45, s, s, s, rnd.choice(GOODS), yaw=rnd.uniform(-0.3, 0.3)))
out.append(model('decor_floor', 0, 0, ''.join(d)))

# ------------------------------------------------------------------ assemble
lights = ''.join(
    f'<light type="point" name="lamp{i}"><pose>{x} {y} 3.2 0 0 0</pose><diffuse>0.95 0.93 0.85 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
    f'<attenuation><range>14</range><constant>0.5</constant><linear>0.02</linear><quadratic>0.002</quadratic></attenuation><cast_shadows>false</cast_shadows></light>'
    for i, (x, y) in enumerate(((-4, -3), (4, -3), (-4, 3), (4, 3))))
world = f"""<?xml version="1.0"?>
<!-- GENERATED by tools/gen_world.py - edit the script, not this file. -->
<sdf version="1.6">
  <world name="warehouse">
    <scene><ambient>0.62 0.62 0.66 1</ambient><background>0.08 0.10 0.16 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.75 0.75 0.72 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>-0.3 0.2 -1</direction></light>
    {lights}
    <include><uri>model://ground_plane</uri></include>
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gui><camera name="user_camera"><pose>0 -9.5 12 0 0.95 1.5708</pose></camera></gui>
    {''.join(out)}
  </world>
</sdf>
"""
path = os.path.join(ROOT, 'worlds', 'warehouse.world')
open(path, 'w').write(world)
print(f'wrote {path}  ({len(world) / 1024:.0f} KB, {_n[0]} visuals)')
