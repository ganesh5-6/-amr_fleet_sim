#!/usr/bin/env python3
"""Send a NavigateToPose goal to one robot.

Usage: ros2 run amr_fleet_sim send_goal amr1 3.0 2.0 [yaw_deg]
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalSender(Node):
    def __init__(self, robot):
        super().__init__(f'goal_sender_{robot}')
        self.robot = robot
        self.client = ActionClient(self, NavigateToPose, f'/{robot}/navigate_to_pose')

    def send(self, x, y, yaw_deg):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'/{self.robot}/navigate_to_pose not available')
            return False
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'          # zero stamp = "latest transform"
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = pose

        fut = self.client.send_goal_async(goal, feedback_callback=self._fb)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        self.get_logger().info('Goal accepted')
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        status = res_fut.result().status
        self.get_logger().info(f'Finished with status {status} (4 = SUCCEEDED)')
        return status == 4

    def _fb(self, msg):
        d = msg.feedback.distance_remaining
        self.get_logger().info(f'{self.robot}: {d:.2f} m remaining', throttle_duration_sec=1.0)


def main():
    if len(sys.argv) < 4:
        print('usage: send_goal <robot> <x> <y> [yaw_deg]')
        return
    robot, x, y = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    yaw = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    rclpy.init()
    node = GoalSender(robot)
    node.send(x, y, yaw)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
