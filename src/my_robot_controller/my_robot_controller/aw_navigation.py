#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tier4_system_msgs.srv import ChangeOperationMode

import time
import os
import yaml
from ament_index_python.packages import get_package_share_directory


class AWNavigationNode(Node):
  
    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("our navigation is started")

        self.goal_poses = []
        self.current_goal_index = 0
        
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal", 10)
        
        self.odom_listener = self.create_subscription(
            Odometry, "/localization/kinematic_state", self.odom_callback, 10)
        
        self.change_mode_srv = self.create_client(
            ChangeOperationMode,
            '/system/operation_mode/change_operation_mode'
        )
        self.change_mode_req = ChangeOperationMode.Request()

        # -------------------------
        # LOAD YAML
        # -------------------------
        pkg_path = get_package_share_directory('my_robot_controller')
        file_path = os.path.join(pkg_path, 'poses.yaml')

        initial_pose, self.goal_poses = self.load_poses(file_path)

        # Add start as first goal (for looping)
        self.goal_poses.insert(0, {
            'x': initial_pose.pose.pose.position.x,
            'y': initial_pose.pose.pose.position.y,
            'z': initial_pose.pose.pose.orientation.z,
            'w': initial_pose.pose.pose.orientation.w
        })

        # -------------------------
        # START NAVIGATION
        # -------------------------
        time.sleep(5)
        self.initial_pose_publisher.publish(initial_pose)

        time.sleep(5)
        self.publish_goal()
      

    # -------------------------
    # LOAD POSES FROM YAML
    # -------------------------
    def load_poses(self, file_path):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        # Initial pose
        ip = data['initial_pose']
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = ip['position']['x']
        initial_pose.pose.pose.position.y = ip['position']['y']
        initial_pose.pose.pose.position.z = ip['position']['z']
        initial_pose.pose.pose.orientation.x = ip['orientation']['x']
        initial_pose.pose.pose.orientation.y = ip['orientation']['y']
        initial_pose.pose.pose.orientation.z = ip['orientation']['z']
        initial_pose.pose.pose.orientation.w = ip['orientation']['w']

        # Goals
        goals = []
        for g in data['goals']:
            goals.append({
                'x': g['position']['x'],
                'y': g['position']['y'],
                'z': g['orientation']['z'],
                'w': g['orientation']['w']
            })

        return initial_pose, goals


    def send_request(self):
        self.change_mode_req.mode = 2
        future = self.change_mode_srv.call_async(self.change_mode_req)
      

    def odom_callback(self, msg: Odometry):
        current_pose = msg.pose.pose
        goal_pose = self.goal_poses[self.current_goal_index]

        distance_to_goal = (
            (current_pose.position.x - goal_pose['x']) ** 2 +
            (current_pose.position.y - goal_pose['y']) ** 2
        ) ** 0.5

        if distance_to_goal < 0.3:
            print(distance_to_goal)
            self.publish_next_goal()
          

    def publish_next_goal(self):
        # 🔁 LOOP instead of stopping
        self.current_goal_index = (self.current_goal_index + 1) % len(self.goal_poses)
        self.publish_goal()


    def publish_goal(self):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.goal_poses[self.current_goal_index]['x']
        pose_msg.pose.position.y = self.goal_poses[self.current_goal_index]['y']

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = self.goal_poses[self.current_goal_index]['z']
        pose_msg.pose.orientation.w = self.goal_poses[self.current_goal_index]['w']

        pose_msg.header.frame_id = 'map'

        self.goal_pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published goal: {self.current_goal_index}")

        time.sleep(5)
        self.send_request()


    def stop(self):
        self.get_logger().info("stopping the node")
        rclpy.shutdown()
        raise KeyboardInterrupt
          


def main(args=None):
    rclpy.init(args=args)
    node = AWNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
  

if __name__ == '__main__':
    main()