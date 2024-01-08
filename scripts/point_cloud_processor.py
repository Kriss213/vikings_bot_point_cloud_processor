#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.node import Node
import numpy as np


class PointCloudListener(Node):
    def __init__(self):
        super().__init__('point_cloud_listener')

        # self.declare_parameter('use_sim_time') # this doesn't need declaring
        self.declare_parameter('robot_name')

        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.robot_name = self.get_parameter('robot_name').value

        self.get_logger().info(f"Using sim time: {self.use_sim_time}")
        self.get_logger().info(f"Robot name: {self.robot_name}")

        self.subscription = self.create_subscription(
            PointCloud2,
            f'{self.robot_name}/camera/depth/points',
            #'/camera/camera/depth/color/points', for actual realsense
            self.process_cloud,
            10
        )

        self.publisher_ = self.create_publisher(
            PointCloud2,
            f'{self.robot_name}/camera/processed/color/depth/points',
            10)

        self.get_logger().info(f"Processing point clouds for {self.robot_name}")
        

    def process_cloud(self, msg):
        # Process point cloud
        # publish processed point cloud
    
        if self.use_sim_time:
            msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    PC_processor = PointCloudListener()
    
    rclpy.spin(PC_processor)

    PC_processor.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

