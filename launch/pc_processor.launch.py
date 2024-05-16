#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot_name',
        description="Robot name used in topics etc.")
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description="Use simulation time / run as simulation")
    safe_classes_arg = DeclareLaunchArgument(
        'safe_classes',
        default_value='[-1]',
        description="Semantic segmentation model classes that are considered safe (not detected as an obstacle)")
    vis_sem_seg_arg = DeclareLaunchArgument(
        'vis_sem_seg',
        default_value='false',
        description="Visualize semantic segmentation in seperate window (ROS2 Image topic with same info will be published regardless of this argument)") 
    seg_bb_type_arg = DeclareLaunchArgument(
        'seg_bb_type',
        default_value='0',
        description="Add a bounding box around detected object (to enhance sensor data filtering). 0 - no bounding box (default), 1 - filled bounding box, 2 - outlined bounding box")
    seg_bb_pad_arg = DeclareLaunchArgument(
        'seg_bb_pad',
        default_value='0',
        description='Add padding for bounding box (to enhance sensor data filtering). Default 0 px.')
    filter_buffer_len_arg = DeclareLaunchArgument(
        'filter_buffer_len',
        default_value='60',
        description='Length of point filter mask buffer.')
    filter_prob_threshold_arg = DeclareLaunchArgument(
        'filter_prob_threshold',
        default_value='0.5',
        description='Required probability of class for it to be considered correctly detected [0-1].')

    # define arguments

    # Arguments that determine whether node should be launched
    filter_lidar_arg = DeclareLaunchArgument(
        'filter_lidar',
        default_value='False',
        description="Filter lidar data (LaserScan) based on semantic segmentation model.")
    filter_depth_cam_arg = DeclareLaunchArgument(
        'filter_depth_cam',
        default_value='False',
        description="Filter depth camera data (PointCloud2) based on semantic segmentation model.")
    perf_seg_sem_arg = DeclareLaunchArgument(
        'perf_seg_sem',
        default_value='False',
        description="Perform semantic segmentation and publish results over topic.")


    # Common arguments
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Camera processor arguments
    safe_classes = LaunchConfiguration('safe_classes')
    vis_sem_seg = LaunchConfiguration('vis_sem_seg')
    seg_bb_type = LaunchConfiguration('seg_bb_type')
    seg_bb_pad = LaunchConfiguration('seg_bb_pad')
    filter_buffer_len = LaunchConfiguration('filter_buffer_len')
    filter_prob_threshold = LaunchConfiguration('filter_prob_threshold')

    filter_lidar = LaunchConfiguration('filter_lidar')
    filter_depth_cam = LaunchConfiguration('filter_depth_cam')
    perf_seg_sem = LaunchConfiguration('perf_seg_sem')

    # Depth and Lidar processor have no additional parameters



    camera_proc_launch_cond = IfCondition(
        PythonExpression(
            ["('", filter_lidar, "'.lower() == 'true') or",
             "('", filter_depth_cam, "'.lower() == 'true') or",
             "('", perf_seg_sem, "'.lower() == 'true')"]
        )
    )
    publish_filter_mask_cond = IfCondition(
        PythonExpression(
            ["('", filter_lidar, "'.lower() == 'true') or",
             "('", filter_depth_cam, "'.lower() == 'true')"]
        )
    )
    
    camera_proc_node_act = GroupAction(
        actions=[
            SetParameter(
                name='publish_filter_mask',
                value='True',
                condition=publish_filter_mask_cond
            ),

            Node(
                package='vikings_bot_point_cloud_processor',
                executable='camera_processor.py',
                namespace=robot_name,
                name='data_filter_camera_processor',
                condition=camera_proc_launch_cond,
                output='screen',
                parameters=[{
                    'use_sim_time':use_sim_time,
                    'robot_name':robot_name,
                    'safe_classes':safe_classes,
                    'vis_sem_seg':vis_sem_seg,
                    'seg_bb_type':seg_bb_type,
                    'seg_bb_pad':seg_bb_pad,
                    'filter_buffer_len':filter_buffer_len,
                    'filter_prob_threshold':filter_prob_threshold
                    #'publish_filter_mask': TODO should be true only if filter_lidar or filter_depth is set to true
                }]
            )
        ]
    )
    
    lidar_proc_node = Node(
        package='vikings_bot_point_cloud_processor',
        executable='lidar_processor.py',
        namespace=robot_name,
        name='data_filter_lidar_processor',
        condition=IfCondition(filter_lidar),
        output='screen',
        parameters=[{
            'use_sim_time':use_sim_time,
            'robot_name':robot_name,
        }]
    )

    depth_cam_proc_node = Node(
        package='vikings_bot_point_cloud_processor',
        executable='depth_processor.py',
        namespace=robot_name,
        name='data_filter_depth_processor',
        condition=IfCondition(filter_depth_cam),
        output='screen',
        parameters=[{
            'use_sim_time':use_sim_time,
            'robot_name':robot_name,
        }]
    )
      
    return LaunchDescription([
        filter_lidar_arg,
        filter_depth_cam_arg,
        perf_seg_sem_arg,
        robot_name_arg,
        use_sim_time_arg,
        safe_classes_arg,
        vis_sem_seg_arg,
        seg_bb_type_arg,
        seg_bb_pad_arg,
        filter_buffer_len_arg,
        filter_prob_threshold_arg,
        
        # Nodes:
        camera_proc_node_act,
        lidar_proc_node,
        depth_cam_proc_node
    ])



#     Classes for deeplabv3_mobilenet_v3_large:
#     __background__: 0,
#     aeroplane: 1,
#     bicycle: 2,
#     bird: 3,
#     boat: 4,
#     bottle: 5,
#     bus: 6,
#     car: 7,
#     cat: 8,
#     chair: 9,
#     cow: 10,
#     diningtable: 11,
#     dog: 12,
#     horse: 13,
#     motorbike: 14,
#     person: 15,
#     pottedplant: 16,
#     sheep: 17,
#     sofa: 18,
#     train: 19,
#     tvmonitor: 20,
