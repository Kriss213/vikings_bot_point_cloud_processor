#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from torchvision import transforms
from torchvision.models.segmentation import deeplabv3_mobilenet_v3_large, DeepLabV3_MobileNet_V3_Large_Weights
from classes import ImageProcessor, SemanticSegmentation, PointFilter
import time

class CameraProcessorNode(Node):
    """
    Processes RGB  image
    """
    def __init__(self):
        super().__init__('CameraProcessorNode')
        self.declare_parameter('robot_name', value='default_robot_name')
        self.declare_parameter('camera_name', value='camera')
        
        self.declare_parameter(name='safe_classes',
                               value=[-1],
                               descriptor=ParameterDescriptor(
                                   description="Semantic segmentation model classes that are considered safe (not detected as an obstacle)",
                                   type=Parameter.Type.INTEGER_ARRAY.value
                               ))       
        self.declare_parameter(name='vis_sem_seg',
                               value=False,
                               descriptor=ParameterDescriptor(
                                   description="Visualize semantic segmentation.",
                                   type=Parameter.Type.BOOL.value
                               )) 
        self.declare_parameter(name='seg_bb_type',
                               value=0,
                               descriptor=ParameterDescriptor(
                                   description="Add a bounding box around detected object (to enhance sensor data filtering). 0 - no bounding box (default), 1 - filled bounding box, 2 - outlined bounding box",
                                   type=Parameter.Type.INTEGER.value
                               ))
        self.declare_parameter(name='seg_bb_pad',
                               value=0,
                               descriptor=ParameterDescriptor(
                                   description="Add padding for bounding box (to enhance sensor data filtering). Default 0 px.",
                                   type=Parameter.Type.INTEGER.value
                               ))
        self.declare_parameter(name='filter_buffer_len',
                               value=60,
                               descriptor=ParameterDescriptor(
                                   description="Length of point filter mask buffer.",
                                   type=Parameter.Type.INTEGER.value
                               ))
        self.declare_parameter(name='filter_prob_threshold',
                               value=0.5,
                               descriptor=ParameterDescriptor(
                                   description="Required probability of class for it to be considered correctly detected [0-1].",
                                   type=Parameter.Type.DOUBLE.value
                               ))
        
        # Can be False when only semantic segmentation is launched (without filtering)
        self.declare_parameter(name='publish_filter_mask',
                               value=False,
                               descriptor=ParameterDescriptor(
                                   description="Whether filter mask must be published (set automatically, do not override!)",
                                   type=Parameter.Type.BOOL.value
                               ))

        self.vis_sem_seg = self.get_parameter('vis_sem_seg').value
        self.bounding_box_type = self.get_parameter('seg_bb_type').value
        self.bounding_box_padding = self.get_parameter('seg_bb_pad').value

        self.robot_name = self.get_parameter('robot_name').value
        self.camera_name = self.get_parameter('camera_name').value
        self.safe_classes = self.get_parameter('safe_classes').value
        
        self.must_publish_filter_mask = self.get_parameter("publish_filter_mask").value

        # Image processor initialization
        self.img_proc = ImageProcessor()

        self.filter_mask_msg = Image()
        self.point_filter = PointFilter(buffer_len=self.get_parameter('filter_buffer_len').value)
        self.point_filter.safe_classes = self.safe_classes
        self.point_filter.threshold = self.get_parameter('filter_prob_threshold').value
        

        # Semantic segmentator initialization
        preprocess = transforms.Compose([
                transforms.ToPILImage(),
                #transforms.Resize((320, 240)),
                transforms.ToTensor(),
                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ])
        
        self.segmentator = SemanticSegmentation(
            model=deeplabv3_mobilenet_v3_large,
            weights=DeepLabV3_MobileNet_V3_Large_Weights,
            labels=DeepLabV3_MobileNet_V3_Large_Weights.COCO_WITH_VOC_LABELS_V1.meta['categories'],
            preprocess=preprocess,
            safe_classes=self.safe_classes,
            threshold=self.point_filter.threshold,
            alpha=0.7
        )
        self.__last_info_pub_time:float = time.time()
        
        # Create subscribers:
        self.subscription_image = self.create_subscription(
            Image,
            f'/{self.robot_name}/{self.camera_name}/color/image_raw',
            self.RGB_image_callback,
            10
        )

        self.subscription_rgb_camera_info = self.create_subscription(
            CameraInfo,
            f'/{self.robot_name}/{self.camera_name}/color/camera_info',
            self.RGB_camera_info_callback,
            10
        )

        # Create publisher
        if self.must_publish_filter_mask:
            self.filter_mask_publisher = self.create_publisher(
                Image,
                f'/{self.robot_name}/{self.camera_name}/filter_mask',
                10
            )

        # Create a publisher for visualizing semantic segmentation info board
        self.segmentation_info_board_publisher = self.create_publisher(
            Image,
            f'/{self.robot_name}/{self.camera_name}/color/semantic_segmentation_info_board',
            10
        )


    # CALLBACK methods
    def RGB_image_callback(self, msg:Image):
        self.color_frame = msg.header.frame_id
        self.img_proc.from_img_message(msg)

        img = self.img_proc.image
        seg_mask, probs = self.segmentator.predict(
            image=img,
            bounding_box_type=self.bounding_box_type,
            bounding_box_padding=self.bounding_box_padding
        )

        # publish seg_mask and probs
        self.point_filter.segmentation_mask = seg_mask
        self.point_filter.probabilities = probs
        
        header = Header()
        header.frame_id = msg.header.frame_id
        header.stamp = self.get_clock().now().to_msg()
        img_msg = self.point_filter.to_Image(header=header)
        
        if self.must_publish_filter_mask:
            self.filter_mask_publisher.publish(img_msg)

        current_time = time.time()
        if current_time - self.__last_info_pub_time >= 1: 
            info_board_msg = self.segmentator.visualize(show=self.vis_sem_seg)
            self.segmentation_info_board_publisher.publish(info_board_msg)
            self.__last_info_pub_time = current_time
        
    def RGB_camera_info_callback(self, msg:CameraInfo):
        self.img_proc.set_camera_info(msg, overwrite=False)
        

def main(args=None):
    rclpy.init(args=args)

    cam_proc_node = CameraProcessorNode()
    
    try:
        rclpy.spin(cam_proc_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Clean shutdown: camera_processor")
    else:
        rclpy.shutdown()

    cam_proc_node.destroy_node()



if __name__ == "__main__":
    main()