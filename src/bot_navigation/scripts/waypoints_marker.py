#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
## get package prefix
from ament_index_python.packages import get_package_prefix

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')
        self.publisher_ = self.create_publisher(PoseStamped, 'target_waypoints', 10)
        self.timer_ = self.create_timer(1, self.publish_poses)
        self.get_logger().info('Waypoints Publisher Node Started')

        # Read poses from YAML file
        waypoints_file_path = get_package_prefix('bot_navigation') + '/share/bot_navigation/params/waypoints.yaml'
        self.poses = self.read_poses_from_yaml(waypoints_file_path)
        self.pose_index = 0

    def read_poses_from_yaml(self, file_path):
        with open(file_path, 'r') as file:
            poses_data = yaml.safe_load(file)
            poses = []
            for pose_data in poses_data:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = pose_data['position']['x']
                pose.pose.position.y = pose_data['position']['y']
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.w = pose_data['orientation']['w']
                pose.pose.orientation.z = pose_data['orientation']['z']
                poses.append(pose)
        return poses

    def publish_poses(self):
        if self.pose_index < len(self.poses):
            # Publish PoseStamped messages with a timestamp
            pose_stamped = PoseStamped()
            pose_stamped.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')
            pose_stamped.pose = self.poses[self.pose_index]
            self.publisher_.publish(pose_stamped)
            self.get_logger().info(f'Publishing pose: {pose_stamped}')

            self.pose_index += 1
        else:
            self.get_logger().info('All poses published. Stopping the node.')
            self.timer_.cancel()

def main(args=None):
    rclpy.init(args=args)
    pose_publisher_node = PosePublisherNode()
    try:
        rclpy.spin(pose_publisher_node)
    except KeyboardInterrupt:
        pass
    pose_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
