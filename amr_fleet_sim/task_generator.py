#!/usr/bin/env python3
"""Publishes random tasks so you can benchmark allocators with the same workload (fixed seed)."""
import json
import random

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import os


class TaskGenerator(Node):
    def __init__(self):
        super().__init__('task_generator')
        share = get_package_share_directory('amr_fleet_sim')
        self.declare_parameter('period', 15.0)       # seconds between tasks
        self.declare_parameter('max_tasks', 20)
        self.declare_parameter('seed', 42)
        self.declare_parameter('initial_delay', 40.0)  # let Nav2 come up first
        self.declare_parameter('stations_file', os.path.join(share, 'config', 'stations.yaml'))
        with open(self.get_parameter('stations_file').value) as f:
            st = yaml.safe_load(f)['stations']
        self.pickups = sorted(k for k in st if k.startswith('P'))
        self.drops = sorted(k for k in st if k.startswith('D'))
        self.rnd = random.Random(self.get_parameter('seed').value)
        self.max_tasks = self.get_parameter('max_tasks').value
        self.count = 0
        self.pub = self.create_publisher(String, '/fleet/task_request', 10)
        self.timer = self.create_timer(self.get_parameter('initial_delay').value, self.first)

    def first(self):
        self.timer.cancel()
        self.timer = self.create_timer(self.get_parameter('period').value, self.tick)
        self.tick()

    def tick(self):
        if self.count >= self.max_tasks:
            self.timer.cancel()
            self.get_logger().info('All tasks generated')
            return
        self.count += 1
        msg = {'pickup': self.rnd.choice(self.pickups), 'dropoff': self.rnd.choice(self.drops),
               'priority': self.rnd.choice([1, 1, 2, 3])}
        self.pub.publish(String(data=json.dumps(msg)))
        self.get_logger().info(f'Generated {self.count}/{self.max_tasks}: {msg}')


def main():
    rclpy.init()
    node = TaskGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C / launch shutdown
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
