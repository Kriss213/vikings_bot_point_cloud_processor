# vikings_bot_point_cloud_processor

Applies filter to LaserScan or PointCloud2 based on semantic segmentation model.

## Dependencies
#### Install required Python dependencies:
```
pip install -r requirements.txt
```
#### Install required ROS2 dependencies:
```
rosdep install -y --from-paths src --ignore-src
```

## Launch
```
ros2 launch vikings_bot_point_cloud_processor pc_processor.launch.py robot_name:=vikings_bot_1 perf_seg_sem:=true
```

### Parameters
* __```robot_name```__ - Used in namespace (default: default_robot_name).
* ```use_sim_time``` - Use simulation or real time (default: false).
* __```filter_lidar```__ - Filter lidar data (LaserScan) based on semantic segmentation model (default: false).
* __```filter_depth_cam```__ - Filter depth camera data (PointCloud2) based on semantic segmentation model (default false).
* __```perf_seg_sem```__ - Perform semantic segmentation and publish results over topic (defaulft: false).
* __```safe_classes```__ - A list of semantic segmentation model classes that are considered safe (default: [-1]). See list of classes.
*  ```vis_sem_seg``` - Visualize semantic segmentation in seperate window (ROS2 Image topic with same info will be published regardless of this argument) (default: false).
* ```seg_bb_type``` - Add a bounding box around detected object (to enhance sensor data filtering). 0 - no bounding box (default), 1 - filled bounding box, 2 - outlined bounding box"
* ```seg_bb_pad``` - Add padding in px for bounding box (default: 0).
* ```filter_buffer_len``` - Length of point filter mask buffer (default: 60). Each new filter mask is based on n previous ones in buffer.
* __```filter_prob_threshold```__ - Required probability of class for it to be considered correctly detected [0-1] (default: 0.5).


## RmSafeObstacleLayer plugin
This ROS2 package comes with ROS2 Nav2 Costmap2D plugin that clears points from costmap that are received from specified topic as PointCloud2.

### Parameters:
* ```enabled``` - default: true - Enable/disable plugin.
* ```point_topic``` - default: '/safe_obstacle_points' - Topic over which removable points are published as PointCloud2. Set this to same topic as default but with correct namespace.
* ```inflation_radius``` - default: 5 - Clear extra pixels around each clearable point.

## ROS2 nodes and published topics (listed without robot name in namespace):
#### ```/data_filter_camera_processor```
Responsible for performing semantic segmentation publishing filter mask.

__Launch condition:__ ```perf_seg_sem``` or ```filter_lidar``` or ```filter_depth_cam``` is true.

__Topics:__

* ```/camera/filter_mask``` - sensor_msgs.msg.Image - A binary filter mask based on semantic segmentation model and safe classes (0 - delete, 1 - keep).
    * Condition: ```filter_lidar``` or ```filter_depth_cam``` is true.
* ```/camera/color/semantic_segmentation_info_board``` - sensor_msgs.msg.Image - Visualizes camera, semantic segmentation classes, FPS and what device is used (CPU or CUDA)
<hr>

#### ```data_filter_lidar_processor```
Filters lidar data - removes points belonging to any of safe classes.

__Launch condition:__ ```filter_lidar``` is true.

__Topics:__
* ```/lidar_scan_filtered``` - sensor_msgs.msg.LaserScan - Lidar points where points that belong to safe obstacle are removed.
* ```/safe_obstacle_points``` - sensor_msgs.msg.PointCloud2 - Only those Lidar points that belong to safe obstacle. This topic is used by RmSafeObstacleLayer plugin to clear costmap.
<hr>

#### ```data_filter_depth_processor```
Filters Realsense depth camera data - removes points belonging to any of safe classes.

__Launch condition:__ ```filter_depth``` is true.

__Topics:__
* ```/camera/filtered/depth/points``` - sensor_msgs.msg.PointCloud2 - Depth points where points that belong to safe obstacle are removed.
* ```/safe_obstacle_points``` - sensor_msgs.msg.PointCloud2 - Only those depth points that belong to safe obstacle. This topic is used by RmSafeObstacleLayer plugin to clear costmap.


## Semantic segmentation model
DeepLabV3_MobileNet_V3_Large with COCO_WITH_VOC_LABELS_V1.
https://pytorch.org/vision/master/models/generated/torchvision.models.segmentation.deeplabv3_mobilenet_v3_large.html

#### List of classes:

    __background__: 0,
    aeroplane: 1,
    bicycle: 2,
    bird: 3,
    boat: 4,
    bottle: 5,
    bus: 6,
    car: 7,
    cat: 8,
    chair: 9,
    cow: 10,
    diningtable: 11,
    dog: 12,
    horse: 13,
    motorbike: 14,
    person: 15,
    pottedplant: 16,
    sheep: 17,
    sofa: 18,
    train: 19,
    tvmonitor: 20