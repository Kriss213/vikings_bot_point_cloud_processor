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

        self._TF_buffer = Buffer()
        self._tf_listener = TransformListener(self._TF_buffer, self)

        self.__filter_mask = ImageProcessor()
        self.__depth_image_in_color_frame = ImageProcessor()

        # Subscribe to topics
        self.subscription_depth_point_cloud = self.create_subscription(
                PointCloud2,
                f'/{self.robot_name}/camera/depth/points',
                self.depth_point_cloud_callback,
                10
            )
        # DEPTH IMAGE ALIGNED IN RGB FRAME:
        self.subscription_depth_img_in_color_frame = self.create_subscription(
                Image,
                f'/{self.robot_name}/camera/depth/image_raw', # for sim assume that this is in color frame
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
            f'/{self.robot_name}/camera/color/filter_mask',
            self.filter_mask_callback,
            10
        )
        # Create publisher
        self.publisher_depth_cloud = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/camera/filtered/depth/points',
            10
        )

        # create a timer for init_transforms
        self.timer = self.create_timer(0.33, self.__init_transforms) #30 Hz

    def depth_point_cloud_callback(self, msg:PointCloud2):
        self.__depth_frame = msg.header.frame_id
        self.destroy_subscription(self.subscription_depth_point_cloud)
        return
        # NOTE: not using the code below, because filtering depth image in color frame and transforming it to point cloud is a lot faster
        """
        if not self.__depth_pc_processor.is_camera_info_set or self.__depth_color_tranform == None or not self.__filter_mask.is_image_set:
            return
        
        pc_proc = self.__depth_pc_processor
        transform = self.__depth_color_tranform
        
        pc_proc.from_PointCloud2(msg)

        # transform to color frame
        transform.transform_points(pc_proc)
        
        if self.__filter_mask != None:
            pc_proc.apply_filter(
                filter_mask=self.__filter_mask.image,
            ) # filtered points

        #transform back to depth frame:
        transform.transform_points(pc_proc, inverse=True)

        # convert 3D points to PointCloud2 Message
        new_msg = pc_proc.to_PointCloud2(msg_time=self.get_clock().now().to_msg())

        # publish
        self.publisher_depth_cloud.publish(new_msg)
        """

    def depth_image_in_color_frame_callback(self, msg:Image):
        if not self.__depth_image_in_color_frame.is_camera_info_set or self.__depth_color_tranform == None or not self.__filter_mask.is_image_set:
            return
        
        self.__depth_image_in_color_frame.from_img_message(msg)

        self.__depth_image_in_color_frame.apply_filter(self.__filter_mask.image)

        point_cloud = self.__depth_image_in_color_frame.to_3D(
            z_lim=3.0,
            z_channel=0,
            z_mul=0.001
        )

        # simulation: using depth image which is in depth frame:
        # no transform necessary
        # realsense: using depth image aligned with rgb:
        # transform to depth frame:
        # NOTE: there is no actual need to transform it back since message contains frame id that can be used by nav2
        #self.__depth_color_tranform.transform_points(point_cloud, inverse=True)

        msg = point_cloud.to_PointCloud2(msg_time=self.get_clock().now().to_msg())

        self.publisher_depth_cloud.publish(msg)

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
    except KeyboardInterrupt:
        pass

    depth_proc_node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()