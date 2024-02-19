#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Image, CameraInfo
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from torchvision import transforms
from torchvision.models.segmentation import deeplabv3_mobilenet_v3_large, DeepLabV3_MobileNet_V3_Large_Weights
from classes import ImageProcessor, SemanticSegmentation, PointFilter

class CameraProcessorNode(Node):
    """
    Processes RGB  image
    """
    def __init__(self):
        super().__init__('CameraProcessorNode')
        self.declare_parameter('robot_name', value='default_robot_name')
        
        self.declare_parameter(name='safe_classes',
                               value=[-1],
                               descriptor=ParameterDescriptor(
                                   description="A list of semantic segmentation classes for depth data filter to mark as safe.",
                                   type=Parameter.Type.INTEGER_ARRAY.value
                               ))

        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.robot_name = self.get_parameter('robot_name').value
        self.safe_classes = self.get_parameter('safe_classes').value
        
        # Image processor initialization
        self.img_proc = ImageProcessor()

        self.filter_mask_msg = Image()
        self.point_filter = PointFilter()
        self.point_filter.safe_classes = self.safe_classes
        

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
        )
        
        # Create subscribers:
        self.subscription_image = self.create_subscription(
            Image,
            f'/{self.robot_name}/camera/color/image_raw',
            self.RGB_image_callback,
            10
        )

        self.subscription_rgb_camera_info = self.create_subscription(
            CameraInfo,
            f'/{self.robot_name}/camera/color/camera_info',
            self.RGB_camera_info_callback,
            10
        )

        # Create publisher
        self.filter_mask_publisher = self.create_publisher(
            Image,
            f'/{self.robot_name}/camera/color/filter_mask',
            10
        )

    # CALLBACK methods
    def RGB_image_callback(self, msg:Image):
        self.color_frame = msg.header.frame_id
        self.img_proc.from_img_message(msg)

        img = self.img_proc.image
        seg_mask, probs = self.segmentator.predict(image=img)

        # publish seg_mask and probs
        self.point_filter.segmentation_mask = seg_mask
        self.point_filter.probabilities = probs
        
        img_msg = self.point_filter.to_Image(header=msg.header)
        
        self.filter_mask_publisher.publish(img_msg)
        self.segmentator.visualize()
        
    def RGB_camera_info_callback(self, msg:CameraInfo):
        self.img_proc.set_camera_info(msg, overwrite=False)
        

def main(args=None):
    rclpy.init(args=args)

    cam_proc_node = CameraProcessorNode()
    
    try:
        rclpy.spin(cam_proc_node)
    except KeyboardInterrupt:
        pass

    cam_proc_node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()