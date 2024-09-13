#!/usr/bin/env python3

import rclpy
import rclpy.qos
from sensor_msgs.msg import PointCloud2, LaserScan, CameraInfo, Image
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from classes import PointCloudProcessor, FrameTransformer, ImageProcessor

from functools import reduce
from operator import and_

class LidarProcessorNode(Node):
    """
    Process Lidar LaserScan messages
    """
    def __init__(self):
        super().__init__('LidarProcessorNode')
        self.declare_parameter('robot_name', value='default_robot_name')

        self.robot_name = self.get_parameter('robot_name').value

        self.__lidar_pc_processor = PointCloudProcessor()

        self.__color_frame = None
        self.__lidar_frame = None
        self.__lidar_color_tranform = None

        self.__color_map_transform = None

        self._TF_buffer = Buffer()
        self._tf_listener = TransformListener(self._TF_buffer, self)

        self.__filter_mask = ImageProcessor()

        # Create subscription
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/lidar_scan',
            self.lidar_scan_callback,
            10
        )
        self.subscription_rgb_camera_info = self.create_subscription(
            CameraInfo,
            f'/{self.robot_name}/camera/color/camera_info',
            self.RGB_camera_info_callback,
            10
        )
        self.subscription_filter_mask = self.create_subscription(
            Image,
            f'/{self.robot_name}/camera/filter_mask',
            self.filter_mask_callback,
            10
        )

        self.publisher_safe_object_points = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/safe_obstacle_points',
            rclpy.qos.qos_profile_sensor_data
        )

        # create a timer for init_transforms
        self.timer = self.create_timer(0.033, self.__init_static_transforms) #30 Hz
    
    def lidar_scan_callback(self, msg:LaserScan):
        self.__lidar_frame = msg.header.frame_id

        if self.__color_map_transform == None and self.__color_frame:
            self.__color_map_transform = FrameTransformer(self.__color_frame, "map")
        # find color <--> map transform
        try:
            self.__color_map_transform.find_transform(TF_buffer=self._TF_buffer)
        except:
            self.get_logger().warn("Couldn't find transforms from color to map, dropping message")
            return

        # Check if necessary values are initialized
        conditions = [
            self.__lidar_pc_processor.is_camera_info_set,
            self.__lidar_color_tranform != None,
            self.__filter_mask.is_image_set,
            self.__color_map_transform != None
        ]
        if not reduce(and_, conditions):
            return
  
        pc_proc = self.__lidar_pc_processor
        #transform = self.__lidar_color_tranform
        
        #=====================================================
        # using the same object because it needs other properties set e.g camera info
        # === Get points representing safe objects ===
        pc_proc.from_LaserScan(msg)

        # transform to color frame
        self.__lidar_color_tranform.transform_points(pc_proc)
        # get inverse of filter mask
        if self.__filter_mask == None:
            return
        inverese_filter_mask = 1 - self.__filter_mask.image
        #keep_only_filtered - drop points that are out of camera's (and filter mask) FOV
        pc_proc.apply_filter(filter_mask=inverese_filter_mask, keep_only_filtered=True, z_max=2.5, omit_lidar=True) # do not consider point farther that 2.5 m away

        # transform to map frame
        self.__color_map_transform.transform_points(pc_proc)

        # furher transformations are handled by RmSafeObstacles plugin

        #convert to pointCloud2 message
        obstacle_points_msg = pc_proc.to_PointCloud2(msg_time=self.get_clock().now().to_msg())

        if len(obstacle_points_msg.data) != 0:
           # avoid publishing empty messages
           self.publisher_safe_object_points.publish(obstacle_points_msg)

    def RGB_camera_info_callback(self, msg:CameraInfo):
        self.__lidar_pc_processor.set_camera_info(msg, overwrite=False)
        self.__color_frame = msg.header.frame_id
        self.destroy_subscription(self.subscription_rgb_camera_info)

    def filter_mask_callback(self, msg:Image):
        self.__filter_mask.from_img_message(msg)
        
    def __init_static_transforms(self) -> bool:
        """
        Intitialize static transforms. Returns True if transforms are initialized.

        :return bool: `True` if transforms initialized successfully. `False` otherwise.
        """
        if self.__color_frame and self.__lidar_frame:
            # check if transforms are initialized
            if self.__lidar_color_tranform != None:
                return True
            self.__lidar_color_tranform = FrameTransformer(source_frame=self.__lidar_frame, target_frame=self.__color_frame)
            #while not self.__lidar_color_tranform.is_ready:
            try:
                self.__lidar_color_tranform.find_transform(TF_buffer=self._TF_buffer, timeout=1.0)
                self.get_logger().info("Lidar <--> color transforms found!")
                self.timer.cancel()
                return True
            except:
                self.__lidar_color_tranform = None
                self.get_logger().info("Couldn't find transforms, retrying")
        
        return False

def main(args=None):
    rclpy.init(args=args)

    lidar_proc_node = LidarProcessorNode()
    
    try:
        rclpy.spin(lidar_proc_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Clean shutdown: lidar_processor")
    else:
        rclpy.shutdown()

    lidar_proc_node.destroy_node()



if __name__ == "__main__":
    main()