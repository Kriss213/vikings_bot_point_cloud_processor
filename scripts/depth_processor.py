#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from classes import PointCloudProcessor, FrameTransformer, ImageProcessor

class DepthProcessorNode(Node):
    """
    Process depth camera's depth image messages
    """
    def __init__(self):
        super().__init__('DepthProcessorNode')
        self.declare_parameter('robot_name', value='default_robot_name')

        self.robot_name = self.get_parameter('robot_name').value

        # add params for topics

        # self.__depth_pc_processor = PointCloudProcessor()

        self.__color_frame = None
        self.__depth_frame = None
        self.__depth_color_tranform = None
        self.__color_map_transform = None

        self._TF_buffer = Buffer()
        self._tf_listener = TransformListener(self._TF_buffer, self)

        self.__filter_mask = ImageProcessor()
        self.__depth_image_in_color_frame = ImageProcessor()

        # Subscribe to topics
        self.subscription_depth_point_cloud = self.create_subscription(
                PointCloud2,
                f'/{self.robot_name}/camera/depth/color/points',
                self.depth_point_cloud_callback,
                10
            )
        # DEPTH IMAGE ALIGNED IN RGB FRAME:
        self.subscription_depth_img_in_color_frame = self.create_subscription(
                Image,
                f'/{self.robot_name}/camera/aligned_depth_to_color/image_raw',
                self.depth_image_in_color_frame_callback,
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
        self.publisher_safe_object_points = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/safe_obstacle_points',
            rclpy.qos.qos_profile_sensor_data
        )

        # create a timer for init_transforms
        self.timer = self.create_timer(0.033, self.__init_transforms) #30 Hz

    def depth_point_cloud_callback(self, msg:PointCloud2):
        self.__depth_frame = msg.header.frame_id
        self.destroy_subscription(self.subscription_depth_point_cloud)


    def depth_image_in_color_frame_callback(self, msg:Image):
        if not self.__depth_image_in_color_frame.is_camera_info_set or self.__depth_color_tranform == None or not self.__filter_mask.is_image_set:
            return
        if self.__color_map_transform == None:
            self.__color_map_transform = FrameTransformer(msg.header.frame_id, "map")
        # find color <--> map transform
        else:
            try:
                self.__color_map_transform.find_transform(TF_buffer=self._TF_buffer)
            except Exception as e:
                self.get_logger().warn(f"Couldn't find transforms from {self.__color_map_transform.source_frame} to {self.__color_map_transform.target_frame}, dropping message:{e}")
                return
        ################ publish points that only belong to obstacle #################
        self.__depth_image_in_color_frame.from_img_message(msg)

        # invert filter mask
        filter_mask_inv = 1 - self.__filter_mask.image

        self.__depth_image_in_color_frame.apply_filter(filter_mask_inv, remove_above_mean=True)

        point_cloud = self.__depth_image_in_color_frame.to_3D(
            z_lim=3.0,
            z_channel=0,
            z_mul=0.001
        )
        
        # Transform points to map frame
        self.__color_map_transform.transform_points(point_cloud)
      
        obstacle_points_msg = point_cloud.to_PointCloud2(
            msg_time=self.get_clock().now().to_msg(),
            projection=2, # project points into XY plane
            reduce_density=15)# keep only every 15th point because
                              # points from depth cloud are very dense

        if len(obstacle_points_msg.data) != 0:
            # avoid publishing empty messages
            self.publisher_safe_object_points.publish(obstacle_points_msg)

    def RGB_camera_info_callback(self, msg:CameraInfo):
        #self.__depth_pc_processor.set_camera_info(msg, overwrite=False)
        self.__depth_image_in_color_frame.set_camera_info(msg, overwrite=False)
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
        if self.__color_frame and self.__depth_frame:
            # check if transforms are initialized
            if self.__depth_color_tranform != None:
                return True
            self.__depth_color_tranform = FrameTransformer(source_frame=self.__depth_frame, target_frame=self.__color_frame)
            
            try:
                self.__depth_color_tranform.find_transform(TF_buffer=self._TF_buffer, timeout=1.0)
                self.get_logger().info("Depth <--> color transforms found!")
                self.timer.cancel()
                return True
            except:
                self.__depth_color_tranform = None
                self.get_logger().info("Couldn't find transforms, retrying")
        
        return False

def main(args=None):
    rclpy.init(args=args)

    depth_proc_node = DepthProcessorNode()
    
    try:
        rclpy.spin(depth_proc_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Clean shutdown: depth_processor")
    else:
        rclpy.shutdown()

    depth_proc_node.destroy_node()



if __name__ == "__main__":
    main()