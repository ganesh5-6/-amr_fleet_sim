#!/usr/bin/env python3
"""Tiny web server for the fleet dashboard (no ROS needed at runtime).

Serves  /              dashboard/index.html
        /map.pgm       the saved occupancy map
        /map_info.json resolution / origin of that map
        /roadmap.json  nodes + edges
        /robots.json   robot names + dock poses
        /stations.json pickup / drop stations
The browser talks to ROS separately through rosbridge (ws://<host>:9090).
"""
import argparse
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('amr_fleet_sim')
    except Exception:                                           # running from source tree
        return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def make_handler(share, map_yaml):
    dash = os.path.join(share, 'dashboard', 'index.html')
    cfg = os.path.join(share, 'config')

    def load_yaml(path):
        with open(path) as f:
            return yaml.safe_load(f)

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):                              # quiet
            pass

        def _send(self, code, body, ctype):
            if isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj), 'application/json')

        def do_GET(self):
            path = self.path.split('?')[0]
            try:
                if path in ('/', '/index.html'):
                    with open(dash, 'rb') as f:
                        return self._send(200, f.read(), 'text/html; charset=utf-8')
                if path == '/roadmap.json':
                    rm = load_yaml(os.path.join(cfg, 'roadmap.yaml'))
                    return self._json({'nodes': rm['nodes'], 'edges': rm['edges']})
                if path == '/robots.json':
                    return self._json(load_yaml(os.path.join(cfg, 'robots.yaml'))['robots'])
                if path == '/stations.json':
                    return self._json(load_yaml(os.path.join(cfg, 'stations.yaml'))['stations'])
                if path in ('/map_info.json', '/map.pgm'):
                    if not os.path.exists(map_yaml):
                        return self._json({'error': f'map not found: {map_yaml}'}, 404)
                    m = load_yaml(map_yaml)
                    if path == '/map_info.json':
                        return self._json({'resolution': m['resolution'], 'origin': m['origin'][:2],
                                           'negate': m.get('negate', 0)})
                    img = os.path.join(os.path.dirname(map_yaml), m['image'])
                    with open(img, 'rb') as f:
                        return self._send(200, f.read(), 'application/octet-stream')
                return self._json({'error': 'not found'}, 404)
            except Exception as e:                              # noqa
                return self._json({'error': str(e)}, 500)
    return H


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--map', default=None, help='path to map .yaml (default: installed warehouse.yaml)')
    args, _ = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    share = share_dir()
    map_yaml = args.map or os.path.join(share, 'maps', 'warehouse.yaml')
    srv = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(share, map_yaml))
    print(f'Dashboard: http://localhost:{args.port}   (map: {map_yaml})', flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
