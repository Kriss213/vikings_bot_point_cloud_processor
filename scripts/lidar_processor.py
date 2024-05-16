#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import PointCloud2, LaserScan, CameraInfo, Image
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from classes import PointCloudProcessor, FrameTransformer, ImageProcessor


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
        # Create publisher
        # TODO recalculate to LaserScan!!
        self.publisher_lidar_cloud = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/lidar_scan/filtered/points',
            10
        )

        # create a timer for init_transforms
        self.timer = self.create_timer(0.33, self.__init_transforms) #30 Hz
    
    def lidar_scan_callback(self, msg:LaserScan):
        self.__lidar_frame = msg.header.frame_id

        # Check if necessary values are initialized
        if not self.__lidar_pc_processor.is_camera_info_set or self.__lidar_color_tranform == None or not self.__filter_mask.is_image_set:
            return
  
        pc_proc = self.__lidar_pc_processor
        transform = self.__lidar_color_tranform
        
        pc_proc.from_LaserScan(msg)

        # transform to color frame
        transform.transform_points(pc_proc)

        if self.__filter_mask != None:
            pc_proc.apply_filter(
                filter_mask=self.__filter_mask.image,
            ) # filtered points

        
        #transform back to lidar frame:
        transform.transform_points(pc_proc, inverse=True)

        # convert 3D points to PointCloud2 Message
        new_msg = pc_proc.to_PointCloud2(msg_time=self.get_clock().now().to_msg())

        # publish
        self.publisher_lidar_cloud.publish(new_msg)

    def RGB_camera_info_callback(self, msg:CameraInfo):
        self.__lidar_pc_processor.set_camera_info(msg, overwrite=False)
        self.__color_frame = msg.header.frame_id
        self.destroy_subscription(self.subscription_rgb_camera_info)

    def filter_mask_callback(self, msg:Image):
        self.__filter_mask.from_img_message(msg)
        
    def __init_transforms(self) -> bool:
        """
        Intitialize transforms. Returns True if transforms are initialized.

        Returns:
        * `True` if transforms initialized successfully
        * `False` otherwise
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
    except KeyboardInterrupt:
        pass

    lidar_proc_node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()