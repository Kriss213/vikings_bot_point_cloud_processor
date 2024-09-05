import numpy as np
import cv2
import torch
from torchvision import transforms
import colorsys
import image_geometry
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo, LaserScan
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Transform
import array
from typing import ForwardRef, Union
from tf2_ros import Buffer
import rclpy
import time
from collections import deque
from cv_bridge import CvBridge

DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

class ImageProcessor:
    """
    Process sensor_msgs.msg.Image message
    """
    def __init__(self) -> None:
        self._width:int = None
        self._height:int = None
        self._channels:int = None
        self._encoding:str = None
        self._step:int = None
        self._data_raw = None
        self._message:Image = None
        self._frame:str = None

        self.is_camera_info_set = False
        self.__dx_over_fx:np.ndarray = None
        self.__dy_over_fy:np.ndarray = None

        self.__init_from_img = False

        self.is_image_set = False
        
        self.__endcodings_channels = {
            'rgb8':3,
            '8UC1':1,
            '16UC1':1,
            #TODO
        }
        self.__endcodings_dtypes = {
            'rgb8':np.uint8,
            '8UC1':np.uint8,
            '16UC1':np.uint16,
            #TODO
        }

        self.image:np.ndarray = None
        
    def from_img_message(self, image_message:Image) -> None:
        """
        Initialize class from sensor_msgs.msg.Image message

        :param image_message: A ROS2 Image message.
        
        :return None:
        """
        self._message = image_message
        self._width = image_message.width
        self._height = image_message.height
        self._encoding = image_message.encoding
        self._channels = self.__endcodings_channels[self._encoding]
        self._step = image_message.step
        self._data_raw = image_message.data
        self._frame = image_message.header.frame_id
        
        self.__init_from_img = False
        
        self.is_image_set = True
        self.image = self.__get_image()

    def from_image(self, image:np.ndarray) -> None:
        """
        Initialize class from image in numpy.ndarray.

        :param image: A numpy.ndarray with 2 or 3 dimensions.
        
        :return None:
        """
        self._height, self._width = image.shape
        #self._data = image
        if len(image.shape) == 2:
            self.image = image[:,:,np.newaxis]
        elif len(image.shape) == 3:
            self.image = image
        else:
            raise Exception(f"Invalid image shape (must have 2 or 3 dims): {image.shape}")
        self.__init_from_img = True

        self.is_image_set = True

    def set_camera_info(self, camera_info_msg:CameraInfo, overwrite:bool=True) -> None:
        """
        Update camera info parametrs needed for converting pixels to 3D points

        :param camera_info_msg: A ROS2 CameraInfo message
        :param overwrite: If False, camera info will not be overwritten (default=True)

        :return None:
        """
        if not np.any(self.__dy_over_fy) or overwrite:
            # Calculate parametrs for converting 2D to 3D:
            u_indices = np.arange(0, camera_info_msg.width)
            v_indices = np.arange(0, camera_info_msg.height)
            uv_indices = np.array(np.meshgrid(u_indices, v_indices)).T.reshape(-1, 2)
            img_geom = image_geometry.PinholeCameraModel()
            img_geom.fromCameraInfo(camera_info_msg)

            self.__dx_over_fx = (uv_indices[:,0] - img_geom.cx()) / img_geom.fx()
            self.__dy_over_fy = (uv_indices[:,1] - img_geom.cy()) / img_geom.fy()

            self._frame = camera_info_msg.header.frame_id
            self.is_camera_info_set= True

    def __get_image(self) -> np.ndarray:
        """
        Get numpy.ndarray like (height, width, channels)

        :return np.ndarray: with shape like (height, width, channels)
        """
        
        if self.__init_from_img:
            return self.image
        
        if not self.is_image_set:
            raise Exception("Set image using 'from_img_message() or from_image() before calling get_image()'") # TODO: add other methods
        
        if self._encoding not in self.__endcodings_channels.keys():
            raise Exception(f"Encoding '{self._encoding}' currently is not supported!")
        
        return np.frombuffer(self._data_raw, dtype=self.__endcodings_dtypes[self._encoding]).reshape(self._height, self._width, self._channels) 
        
    def apply_filter(self, filter_mask:np.ndarray, remove_above_mean:bool=False) -> None:
        """
        Filter image based on segmentation model result.
        
        :param filter_mask: A np.ndarray like image with values 0 (delete) and 1 (keep)
        :param remove_above_mean: __Assuming a depth image__. If True, keep only points that are closer than mean distance after filtering. This is to minimize noise points behind an object.

        :return None:
        """

        if self._channels != 1:
            raise Exception(f"Cannot filter image with more than 1 channel. Image has {self._channels}")
        
        # filter here:
        if self.image.shape != filter_mask.shape:
            raise Exception(f"Camera info shape {self.image.shape} must match with filter mask shape {filter_mask.shape}")
        
        # apply filter
        self.image = filter_mask * self.image

        if remove_above_mean:
            non_zero_mean = np.mean(self.image[self.image > 0])
            self.image[self.image > non_zero_mean] = 0

    def to_3D(self, z_lim:float=float('inf'), z_channel:int=0, z_mul:float=1.0) -> 'PointCloudProcessor':
        """
        Transform 2D image to 3D points in camera frame using z_channel as Z data.

        :param z_lim: Limit point depth (default=float('inf')).
        :param z_channel: Which channel use as point depth value (default=0).
        :param z_mul: Multiply Z value to change units etc.

        :return pc_processor_obj: A PointCloudProcessor object.
        """

        if not self.is_camera_info_set:#(np.any(self.__dx_over_fx) and np.any(self.__dy_over_fy)):
            raise Exception("Use 'set_camera_info()' before using 'to_3D()'!")
        
        depth_values = (self.image[:, :, z_channel] * z_mul).flatten('F')


        # Calculate point coordinates
        cond = (depth_values!=0) & (depth_values < z_lim)
        depth_vals_pos = depth_values[cond]
        Xc = depth_vals_pos * self.__dx_over_fx[cond]
        Yc = depth_vals_pos * self.__dy_over_fy[cond]
        Zc = depth_vals_pos

        points = np.column_stack([Xc, Yc, Zc])

        pc_processor_obj = PointCloudProcessor()
        pc_processor_obj.from_points(points, self._frame)

        return pc_processor_obj

class FrameTransformer:
    """
    Manage 3D point transformations from source frame to target frame. Meant for static transforms!
    """
    def __init__(self, source_frame, target_frame) -> None:
        self._source_frame = source_frame
        self._target_frame = target_frame
        self._rotation = None
        self._translation = None
        self._rotation_inv = None
        self._translation_inv = None
        self.is_ready = False

    def __from_transform(self, transform_fw:Transform, transform_inv:Transform) -> None:
        """
        Initialize class from geometrey_msgs.msg.Transform

        :return None:
        """
        self._translation = np.array([
            transform_fw.translation.x,
            transform_fw.translation.y,
            transform_fw.translation.z,
        ])
        rotation = np.array([
            transform_fw.rotation.x,
            transform_fw.rotation.y,
            transform_fw.rotation.z,
            transform_fw.rotation.w
        ])
        self._rotation = Rotation.from_quat(rotation).as_matrix()

        self._translation_inv = np.array([
            transform_inv.translation.x,
            transform_inv.translation.y,
            transform_inv.translation.z,
        ])
        rotation_inv = np.array([
            transform_inv.rotation.x,
            transform_inv.rotation.y,
            transform_inv.rotation.z,
            transform_inv.rotation.w
        ])
        self._rotation_inv = Rotation.from_quat(rotation_inv).as_matrix()
        self.is_ready = True

    def find_transform(self, TF_buffer:Buffer, timeout:float=1.0) -> None:
        """
        Find transform between source and target frame.
        
        :param TF_buffer: A buffer where to look for transform.
        :param timeout: Timeout to wait for transform in buffer (default=1.0).

        :return None:
        """
        
        t_fw = TF_buffer.lookup_transform(
                target_frame=self._target_frame,
                source_frame=self._source_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
        t_inv = TF_buffer.lookup_transform(
                target_frame=self._source_frame,
                source_frame=self._target_frame,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
            
        self.__from_transform(transform_fw=t_fw.transform, transform_inv=t_inv.transform)
    
    def transform_points(self, points:Union[np.ndarray, 'PointCloudProcessor'], inverse:bool=False) -> tuple:
        """
        Transform points from source frame to target frame.
        
        :param points: A np.ndarray of 3D points like (n, 3) or PointCloudProcessor object. If `points` is an instance of `PointCloudProcessor`, instance's points are transformed.
        :param inverse: If True, transfrom from target_frame to source_frame

        :return tuple: (PointCloudProcessor object, target frame)
        """
        if not(np.any(self._rotation) and np.any(self._translation)):
            raise Exception("Use find_transform() before transforming points!")
        
        #   ROS2 humble doesn't have tf2_sensor_msgs on python
        #   therefore frame conversion is implemented manually
        #   see:
        #   https://github.com/ros2/geometry2/blob/rolling/tf2_sensor_msgs/tf2_sensor_msgs/tf2_sensor_msgs.py#L86
    
        if isinstance(points, PointCloudProcessor):
            points_to_transfer = points.points
        else:
            points_to_transfer = points
            points = PointCloudProcessor()

        if inverse:
            new_frame = self._source_frame
            new_points = np.einsum(
                'ij, pj -> pi',
                self._rotation_inv,
                points_to_transfer
                ) + self._translation_inv
            points.from_points(new_points, new_frame)

        else:
            new_frame = self._target_frame
            new_points = np.einsum(
                'ij, pj -> pi',
                self._rotation,
                points_to_transfer
                ) + self._translation
            points.from_points(new_points, new_frame)
        
        return points, new_frame

class PointCloudProcessor:
    """
    Process point clouds from PointCloud2 or LaserScan message.
    """
    def __init__(self) -> None:
        self._points = None
        self._frame = None

        self.__data_type_bytes = {
            1: 1,
            2: 1,
            3: 2,
            4: 2,
            5: 4,
            6: 4,
            7: 4,
            8: 8
        }

        self.__camera_info_msg = None
        self.__camera_params = None
        self.is_camera_info_set= False

        self.__init_from_LaserScan_msg = False
        self.__angle_min = None
        self.__angle_max = None
        self.__angle_inc = None
        self.__range_min = None
        self.__range_max = None
    @property
    def points(self):
        return self._points
    @points.setter
    def points(self, nv):
        raise AttributeError("'points' is a read-only attribute.")
    
    @property
    def frame(self):
        return self._frame
    @frame.setter
    def frame(self, nv):
        raise AttributeError("'frame' is a read-only attribute.")
    
    def from_PointCloud2(self, pc2_msg:PointCloud2, omit_RGB:bool=True) -> None:
        """
        Initialize from sensor_msgs.msg.PointCloud2 message.

        :param pc2_msg: A PointCloud2 message

        :return None:
        """
        if not omit_RGB:
            raise NotImplementedError("The inclusion of RGB data in point cloud is currently not supported")
        
        # set frame
        self._frame = pc2_msg.header.frame_id

        fields = pc2_msg.fields
        data_raw = pc2_msg.data
        
        # Extract offsets and datatypes for x, y, z
        offsets_bytes = {'x':None, 'y':None, 'z':None}
        for point_field in fields:
            # point_field is of sensor_msgs.msg.PointField type
            name = point_field.name.lower()
            if name in offsets_bytes.keys():
                offsets_bytes[name] = (
                    point_field.offset,
                    self.__data_type_bytes[point_field.datatype]
                )
        x_off, x_bytes = offsets_bytes['x']
        y_off, y_bytes = offsets_bytes['y']
        z_off, z_bytes = offsets_bytes['z']
        
        # Convert to numpy array
        points = np.frombuffer(data_raw, dtype=np.uint8).reshape((-1, pc2_msg.point_step))

        # Remove columns that are not xyz
        x = points[:, x_off:x_off+x_bytes]
        y = points[:, y_off:y_off+y_bytes]
        z = points[:, z_off:z_off+z_bytes]

        self._points = np.column_stack([x, y, z]).view(np.float32)

    def from_LaserScan(self, ls_msg:LaserScan) -> None:
        """
        Initialize from sensor_msgs.msg.LaserScan message.

        :param ls_msg: A LaserScan message.

        :return None:
        """
        # set frame
        self._frame = ls_msg.header.frame_id
        self.__angle_min = ls_msg.angle_min
        self.__angle_max = ls_msg.angle_max
        self.__angle_inc = ls_msg.angle_increment
        self.__range_min = ls_msg.range_min
        self.__range_max = ls_msg.range_max

        ranges = np.frombuffer(ls_msg.ranges, dtype=np.float32)
        angles = np.arange(start=ls_msg.angle_min, stop=ls_msg.angle_max, step=ls_msg.angle_increment, dtype=np.float64)

        
        # adjust angles/ranges vector to be of the same shape
        # ideally both should be equal already
        ang_len = angles.shape[0]
        rng_len = ranges.shape[0]
        if rng_len > ang_len:
            ranges = ranges[:ang_len]

        elif ang_len > rng_len:
            angles = angles[:rng_len]

        X_l = ranges * np.cos(angles)
        Y_l = ranges * np.sin(angles)
        Z_l = np.zeros_like(X_l) # 0 in lidar frame


        # Remove invalid points
        X_l[ranges < ls_msg.range_min] = float('nan')
        X_l[ranges > ls_msg.range_max] = float('nan')
        Y_l[ranges < ls_msg.range_min] = float('nan')
        Y_l[ranges > ls_msg.range_max] = float('nan')
        
        # Stack XYZ values
        self._points = np.column_stack((X_l, Y_l, Z_l))
        self.__init_from_LaserScan_msg = True
        
    def from_points(self, points:np.ndarray, frame:str) -> None:
        """
        Initialize from numpy.ndarray of points. Must have shape like (point_count, 3)

        :param points: numpy.ndarray of XYZ points.
        :param frame: Frame in which the points are given.

        :return None:
        """
        if type(points) != np.ndarray:
            self._points = np.ndarray(points)
        else:
            self._points = points
        if points.shape[1] != 3:
            raise Exception(f"Invalid points shape: {self._points.shape}. Second dim must be 3")
        
        if type(frame) != str:
            raise Exception(f"Invalid frame given: {frame}. Must be a str!")
        self._frame = frame

    def to_2D(self) -> ImageProcessor:
        """
        Convert 3D points to 2D image plane. Ignoring points that are out of FOV.

        Parameters:

        Returns:
        * `image`:ImageProcessor - Image with Z values as depth

        """
        if not self.is_camera_info_set:
            raise Exception(f"Use set_camera_info() before converting to 2D'!")
        
        fx, fy, cx, cy, frame, width, height = self.__camera_params.values()
        if frame != self._frame:
            raise Exception(f"Point frame '{self._frame}' must match with camera frame '{frame}'!")


        image = np.zeros((height, width))

        x = self.points[:, 0]
        y = self.points[:, 1]
        z = self.points[:, 2]

        u_unlim = np.round(fx * x / z + cx).astype(np.int32)
        v_unlim = np.round(fy * y / z + cy).astype(np.int32)
        
        cond = (0 <= u_unlim) & (u_unlim < width) & (0 <= v_unlim) & (v_unlim < height)
        u = u_unlim[cond]
        v = v_unlim[cond]

        image[v, u] = z[cond]

        image_obj = ImageProcessor()
        image_obj.from_image(image)
        image_obj.set_camera_info(self.__camera_info_msg)
        return image_obj
    
        #return image

    def set_camera_info(self, camera_info:CameraInfo, overwrite:bool=True) -> None:
        """
        Set camera info for converting to 2D or applying filter.

        Parameters:
        * `camera_info`:sensor_msgs.msg.CameraInfo - camera information message
        * `overwrite`:bool - If False, camera info will not be overwritten (default=True)

        Returns: None
        """
        if self.__camera_params == None or overwrite:
            image_geom = image_geometry.PinholeCameraModel()
            image_geom.fromCameraInfo(camera_info)
            self.__camera_params = {
                'fx': image_geom.fx(),
                'fy': image_geom.fy(),
                'cx': image_geom.cx(),
                'cy': image_geom.cy(),
                'frame':camera_info.header.frame_id,
                'width':image_geom.width,
                'height':image_geom.height
            }
            self.__camera_info_msg = camera_info
            self.is_camera_info_set= True
        return

    def apply_filter(self, filter_mask:np.ndarray, z_max:float=float('inf'), keep_only_filtered:bool=False, omit_lidar:bool=False) -> None:
        """
        Filter points based on segmentation model result.
        
        :param z_max: Limit point depth (default=float('inf')).
        :param filter_mask: Binary matrix with points to keep (1) and points to delete (0).
        :param keep_only_filtered: If True keep only those points that pass the filter.
        :param omit_lidar: If True, do not populate PointCloudProcessor.points with nan values to keep correct LaserScan structure. If True, afterwards converting to LaserScan wil result in invalid data.
        
        :return None:
        """

        if not self.is_camera_info_set:
            raise Exception(f"Use set_camera_info() before applying filter'!")
        
        fx, fy, cx, cy, frame, width, height = self.__camera_params.values()

        if frame != self._frame:
            raise Exception(f"Point frame '{self._frame}' must match with camera frame '{frame}'!")
        
        # Check filter mask
        filter_mask = filter_mask.squeeze()
        if (height, width) != filter_mask.shape:
            raise Exception(f"Camera info h,w ({height}, {width}) must match with filter mask shape {filter_mask.shape}")
        
        points = self.points
        x, y, z = points[:, 0], points[:, 1], points[:, 2]

        # calculate projections on image plane for all points.
        # since depth cam, lidar etc. FOV usually is greater than rgb camera's FOV,
        # negative values and values greater than images width and height will be present
        u_unlim = np.rint(fx * x / (z+1e-8) + cx).astype(int)
        v_unlim = np.rint(fy * y / (z+1e-8) + cy).astype(int)
        
        # select those points that are "visible" in camera
        cond_matches_filter = (0 <= u_unlim) & (u_unlim < width) & (0 <= v_unlim) & (v_unlim < height)
        
        # find pixel u and v coordinates for points that are visible in camera
        u_f = u_unlim[cond_matches_filter]
        v_f = v_unlim[cond_matches_filter]
        z_f = z[cond_matches_filter]

        # in filter mask find points that are visible in camera
        matching_points_in_filter = filter_mask[v_f, u_f]

        # since filter mask is binary (0: delete, 1: keep), multiply to remove points
        u_unlim[cond_matches_filter] = matching_points_in_filter * u_f
        v_unlim[cond_matches_filter] = matching_points_in_filter * v_f
        z[cond_matches_filter] = matching_points_in_filter * z_f
        
        # remove points that are out of camera's FOV
        if keep_only_filtered:
            u_unlim = u_unlim[cond_matches_filter]
            v_unlim = v_unlim[cond_matches_filter]
            z = z[cond_matches_filter]

        
        z_cond = (z <= z_max) & (z != 0)
        if not self.__init_from_LaserScan_msg or omit_lidar:
            # remove points that are farther than z_lim or have 0 depth
            u_filtered = u_unlim[z_cond]
            v_filtered = v_unlim[z_cond]
            z_filtered = z[z_cond]
        else:
            # keep deleted points but marked as nan
            u_filtered = u_unlim.astype(float)
            v_filtered = v_unlim.astype(float)
            z_filtered = z.astype(float)
            inv_z_cond = np.logical_not(z_cond)
            u_filtered[inv_z_cond] = float('nan')
            v_filtered[inv_z_cond] = float('nan')
            z_filtered[inv_z_cond] = float('nan')

        # project points back into 3D space
        z = z_filtered
        x = ((u_filtered - cx) * z_filtered) / fx
        y = ((v_filtered - cy) * z_filtered) / fy

        self._points = np.column_stack([x, y, z])

    def to_PointCloud2(self, msg_time:str=rclpy.clock.Clock().now().to_msg(), projection:int=None, reduce_density:int=1) -> PointCloud2:
        """
        Create a ROS2 PointCloud2 message.
        
        :param msg_time: Message time to use.
        :param projection: Project points to plane. 0 - YZ, 1 - XZ, 2 - XY. Other values not accepted
        :param reduce_density: Keep only every ith element (1 = no reduction).

        :return point_cloud_msg: A PointCloud2 message.
        """
        if type(reduce_density) != int or reduce_density < 1:
            raise Exception(f"reduce_density factor must be a positive integer (got {reduce_density}).")

        if projection not in (None, 0, 1, 2):
            raise Exception(f"Provide correct projection value: 0 - YZ, 1 - XZ, 2 - XY. Got {projection}")

        point_cloud_msg = PointCloud2()
        points = self._points

        point_cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=7, count=1),
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1),
        ]

        point_step = len(point_cloud_msg.fields) * 4
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = point_step
        point_cloud_msg.row_step = point_step * point_cloud_msg.width * point_cloud_msg.height
        
        if projection:
            # remove points with duplicate xy
            # (set precision to 1 cm for calculating duplicates)
            _, indices = np.unique(np.around(points,decimals=2)[:,:2], axis=0, return_index=True)
            
            points = points[indices]

            #flatten all points to same height
            points[:, projection]  = 0
        
        # get every reduce_density point
        points = points[::reduce_density]

        point_cloud_msg.is_dense = not (np.any(np.isnan(points)) or np.any(np.isinf(points)) or np.any(np.isneginf(self.points)) )
        # set point cloud size after manipulations
        point_cloud_msg.width = points.shape[0]
        point_cloud_msg.height = 1

        msg_data_np = points.astype(np.float32).view(np.uint8).reshape(-1)

        # Using array and memoryview to *significantly* improve performance over numpy.ndarray.tolist()
        mem_view = memoryview(msg_data_np)
        message_data = array.array('B')
        message_data.frombytes(mem_view.cast('B'))

        point_cloud_msg.data = message_data

        # set time and frame
        point_cloud_msg.header.frame_id = self.frame        
        point_cloud_msg.header.stamp = msg_time

        return point_cloud_msg

    def to_LaserScan(self, msg_time:str=rclpy.clock.Clock().now().to_msg()) -> LaserScan:
        """
        Get a LaserScan message from 3D points.

        :param msg_time: Time stamp for message.

        :return new_msg: A new LaserScan message.

        :raises Exception('Can only convert to LaserScan message, if instance was initiated from LaserScan message'):
        """
        if not self.__init_from_LaserScan_msg:
            raise Exception("Can only convert to LaserScan message, if instance was initiated from LaserScan message")

        new_msg = LaserScan()

        new_msg.angle_min = self.__angle_min
        new_msg.angle_max = self.__angle_max
        new_msg.angle_increment = self.__angle_inc
        new_msg.range_min = self.__range_min
        new_msg.range_max = self.__range_max

        new_msg.header.frame_id = self.frame
        
        x = self.points[:, 0]
        y = self.points[:, 1]

        ranges = np.hypot(x, y).astype(np.float32)
        mem_view_ranges = memoryview(ranges)
        ranges_arr = array.array('f')

        ranges_arr.frombytes(mem_view_ranges.cast('B'))
        
        new_msg.ranges=ranges_arr

        new_msg.header.stamp = msg_time

        return new_msg

class SemanticSegmentation:
    """
    Segment an image with model.
    """
    NO_BOUNDING_BOX=0
    BOUNDING_BOX_FILLED=1
    BOUNDING_BOX_OUTLINE=2

    __fill_types = {
        BOUNDING_BOX_FILLED: cv2.FILLED,
        BOUNDING_BOX_OUTLINE:2
    }
    def __init__(self, model, weights, labels:list, preprocess:transforms.Compose=None) -> None:
        self.weights = weights
        self.model = model(weights=weights)
        self.model.eval().to(DEVICE)
        self.labels = labels
        self.__window_name = None # CV2 visualization window

        # automatically generate class colors for all labels
        self.__class_colors = {}
        HSV_tuples = [(x * 1.0 / len(self.labels), 0.5, 0.5) for x in range(len(self.labels))]
        for i, rgb in enumerate(HSV_tuples):
            rgb = map(lambda x: int(x * 255), colorsys.hsv_to_rgb(*rgb))
            self.__class_colors[i] = tuple(rgb)

        if preprocess == None:
            # use preprocess that comes with model weights
            self.preprocess = self.weights.transforms()
        else:
            self.preprocess = preprocess

        self.segmentation_mask = None
        self.probabilities = None

        self.__last_visualiztion_frame_time = 0

        #self.overlay_image = Image()
        #self.overlay_image.encoding = '8UC1'
        self.cv_bridge = CvBridge()

    def predict(self, image:np.ndarray, bounding_box_type:int=NO_BOUNDING_BOX, bounding_box_padding:int=0) -> tuple:
        """
        Apply semantic segmentation model to image.

        :param image: Image (numpy.ndarray) to segment.
        :param bounding_box_type: Add a bounding box around object. Use: `SemanticSegmentation.NO_BOUNDING_BOX` (default), `SemanticSegmentation.BOUNDING_BOX_FILLED`, `SemanticSegmentation.BOUNDING_BOX_OUTLINE`),
        :param bounding_box_padding: Extra padding for bounding box, px (default=0)

        :return (segmentation_mask, max_probabilities):
        
        * segmentation mask with same shape as input image - contains classes for each pixel
        * probabilities mask with same shape as input image - containts probabilities for each class)
        """
        # preprocess image and add batch dim
        im_preprocessed = self.preprocess(image)
        im_preprocessed = im_preprocessed.unsqueeze(dim=0).to(DEVICE)
        
        # apply segmentation to image
        with torch.no_grad():
            predictions = self.model(im_preprocessed)['out'].cpu()
        
        # apply softmax to get probability distribution for each class
        predictions_softmax = predictions.softmax(dim=1)
        # extract classes with highest probabilities for each pixel
        max_probabilities, indices = predictions_softmax.max(dim=1)
        
        # remove batch dim as it was only needed to pass image
        # through model; convert to numpy ndarray.
        segmentation_mask = indices.squeeze().numpy().astype(np.uint8)
        max_probabilities = max_probabilities.squeeze().numpy()
        
        # resize probabilities and segmenation mask to input size
        max_probabilities = cv2.resize(src=max_probabilities, dsize=image.shape[:2][::-1], interpolation=cv2.INTER_LINEAR)
        segmentation_mask = cv2.resize(src=segmentation_mask, dsize=image.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)

        if bounding_box_type in (SemanticSegmentation.BOUNDING_BOX_OUTLINE, SemanticSegmentation.BOUNDING_BOX_FILLED):
            bounding_box_padding = int(bounding_box_padding)
            contours, _ = cv2.findContours(segmentation_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                x_box, y_box, w_box, h_box = cv2.boundingRect(contour)
                M = cv2.moments(contour)
                cX = int(M["m10"] / (M["m00"] + 1e-8))
                cY = int(M["m01"] / (M["m00"] + 1e-8))
                class_id = int(segmentation_mask[cY, cX])
                x1 = max(x_box-bounding_box_padding, 0)
                y1 = max(y_box-bounding_box_padding, 0)
                y_lim, x_lim = segmentation_mask.shape
                x2 = min(x_box+w_box+bounding_box_padding, x_lim)
                y2 = min(y_box+h_box+bounding_box_padding, y_lim)
                
                # add rectangle to segmentation mask
                # and update the probabilites for rectangle area to match mean probability of class
                mean_probability = np.mean(max_probabilities[segmentation_mask==class_id])
                
                cv2.rectangle(
                    img=segmentation_mask,
                    pt1=(x1, y1),
                    pt2=(x2, y2),
                    color=class_id,
                    thickness=SemanticSegmentation.__fill_types[bounding_box_type])
                max_probabilities[segmentation_mask==class_id] = mean_probability
               

        self.image = image
        self.segmentation_mask = segmentation_mask
        self.probabilities = max_probabilities

        return segmentation_mask, max_probabilities

    def visualize(self, show:bool=False) -> Image:
        """
        Generate a ROS2 Image message containing Semantic segmentation visualization (original image, segmentation mask and identified classes).

        :param show: Show a CV2 window in addition to publishing Image message.

        :return ros_2_image: a ROS2 Image message
        """
        if type(self.segmentation_mask) != np.ndarray or type(self.probabilities) != np.ndarray:
            raise Exception(f"Perform semantic segmenation using 'predict()' before visualizing!")

        if self.__window_name == None and show:
            self.__window_name = "Semantic segmentation"
            cv2.namedWindow(self.__window_name, cv2.WINDOW_NORMAL)
        
        # generate colormap and add labels to overlay
        label_window = np.ones(self.image.shape, dtype=np.uint8) * 255
        h, w, _ = self.image.shape
        square_size = 50
        text_offset = 10
        row_space = 20
        max_rows = int(h / (square_size + row_space))
        column_space = square_size+text_offset + 150
        

        color_map = np.zeros((self.segmentation_mask.shape[0], self.segmentation_mask.shape[1],3), dtype=np.uint8)
        unique_clases = np.unique(self.segmentation_mask)

        for j, class_idx in enumerate(unique_clases):
            color_map[self.segmentation_mask == class_idx] = self.__class_colors[class_idx]

            # calculate probability for class.
            # NOTE: this is calculates the accuarcy for whole class since
            # semantic segmentation does not distinguish instances of class
            mean_probability = np.mean(self.probabilities[self.segmentation_mask==class_idx])

            x0 = text_offset + column_space * int( j / max_rows)
            if int( j / max_rows) == 0:
                y0 = j * (square_size + row_space)
            else:
                y0 = (j-max_rows*int( j / max_rows)) * (square_size + row_space)
            cv2.rectangle(
                img=label_window,
                pt1=(x0, y0),
                pt2=(x0+square_size, y0+square_size),
                color=self.__class_colors[class_idx],
                thickness=-1,
                )

            text = self.labels[class_idx] if self.labels[class_idx] != "__background__" else "background"
            cv2.putText(
                img=label_window,
                text=text,
                org=(x0 + square_size + text_offset, y0 + int(square_size/4)),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.6,
                color=(0,0,0),
                thickness=2)
            first_line_height = cv2.getTextSize(text, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.6, thickness=2)[0][1]
            cv2.putText(
                img=label_window,
                text=f"{mean_probability:.4f}",
                org=(x0 + square_size + text_offset, y0 + int(square_size/4) + first_line_height+8),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.6,
                color=(0,0,0),
                thickness=2
            )

        # show which device is being used
        cv2.putText(
            img=label_window,
            text="Device:",
            org=(w-125, 20),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.6,
            color=(0,0,0),
            thickness=2
        )
        cv2.putText(
            img=label_window,
            text=f"{DEVICE.upper()}",
            org=(w-55, 20),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.6,
            color=(127,127,127) if DEVICE == 'cpu' else (0,200,40),
            thickness=2
        )
        
        # add FPS:
        fps = int(1 / (time.time() - self.__last_visualiztion_frame_time) )
        cv2.putText(
            img=label_window,
            text=f"FPS: {fps}",
            org=(w-125, 40),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.6,
            color=(0,0,0),
            thickness=2
        )
        self.__last_visualiztion_frame_time = time.time()

        # add label window next to image and color map
        overlay = np.hstack((cv2.cvtColor(self.image,cv2.COLOR_RGB2BGR), color_map, label_window))

        ros_2_image = self.cv_bridge.cv2_to_imgmsg(overlay,'bgr8')
        
        if show:
            cv2.imshow(self.__window_name, overlay)
            cv2.waitKey(1)
        
        return ros_2_image

class PointFilter:
    """
    Class that contains information about point cloud filter based on semantic segmentation result
    """
    def __init__(self, buffer_len:int=1):
        """
        Parameters:
        :param buffer_len`: Specify filter mask buffer length (default=1)
        """
        if buffer_len < 1:
            raise Exception(f"Condition not met: buffer_len>=1 (buffer_len={buffer_len})")
        self._segmentation_mask:np.ndarray = None
        self._probabilities:np.ndarray = None
        self._safe_classes = [0]
        self._threshold = 0.5
        self.__filter_mask = None

        self.__filter_mask_buffer = deque(maxlen=buffer_len)

        self.img_msg = Image()

    @property
    def segmentation_mask(self) -> np.ndarray:
        return self._segmentation_mask
    @segmentation_mask.setter
    def segmentation_mask(self, nv:np.ndarray):
        if np.any(self._probabilities) and nv.shape != self._probabilities.shape:
            raise Exception(f"segmentation_mask and probabilities shape must be the same: {nv.shape} != {self._probabilities.shape}")    
        self._segmentation_mask = nv
        if not np.any(self.__filter_mask):
            self.__filter_mask = np.ones_like(nv, dtype=np.uint8)

    @property
    def probabilities(self) -> np.ndarray:
        return self._probabilities
    @probabilities.setter
    def probabilities(self, nv:np.ndarray):
        if np.any(self._segmentation_mask) and nv.shape != self._segmentation_mask.shape:
            raise Exception(f"segmentation_mask and probabilities shape must be the same: {self._segmentation_mask.shape} != {nv.shape}")
        self._probabilities = nv

    @property
    def safe_classes(self):
        return self._safe_classes
    @safe_classes.setter
    def safe_classes(self, nv:Union[list, tuple, np.ndarray, set]):
        if type(nv) not in [list, tuple, np.ndarray, set]:
            raise Exception(f"Invalid safe_classes type passed: {type(nv)}")
        self._safe_classes = nv

    @property
    def threshold(self) -> float:
        return self._threshold
    @threshold.setter
    def threshold(self, nv:float) -> None:
        """
        Set probability threshold. If probability < threshold, class will be same as "unsafe" class
        """
        if nv < 0 or nv > 1:
            raise Exception(f"Threshold must be between [0; 1]: {nv}")
        self._threshold = nv

    def get_filter_mask(self) -> np.ndarray:
        """
        Get filter mask. 1 if point must be kept, 0 otherwise
        
        :return filter mask: np.ndarray.
        """
        
        # Get filter mask based on current perception
        self.__filter_mask.fill(1)

        cond1 = np.isin(self.segmentation_mask, self._safe_classes)
        cond2 = self.probabilities > self.threshold
        self.__filter_mask[cond1 & cond2] = 0

        # add current perception filter mask to buffer
        self.__filter_mask_buffer.append(self.__filter_mask)

        # calculate sum for each index across buffer (occurances of 1)
        filter_mask_sum = np.sum(self.__filter_mask_buffer, axis=0)

        # get new filter mask based on buffer:
        new_filter_mask = np.zeros(self.__filter_mask.shape, dtype=np.uint8)

        # set pixels to keep based on occurances of 1
        buffer_len = len(self.__filter_mask_buffer)
        new_filter_mask[filter_mask_sum > buffer_len/2] = 1 # buffer_len/2 -> 50%

        return new_filter_mask
    
    def to_Image(self, header:Header) -> Image:
        """
        Create ROS2 Image message from filter mask
        
        :param header: A header for image (contains frame_id and message time).

        :return img_msg: A ROS2 Image message.
        """
        self.get_filter_mask()
        img_msg = self.img_msg
        img_msg.header = header
        img_msg.height, img_msg.width = self.__filter_mask.shape
        img_msg.encoding = '8UC1'
        img_msg.step = img_msg.width
        
        mem_view = memoryview(self.__filter_mask.reshape(-1))
        message_data = array.array('B')
        message_data.frombytes(mem_view.cast('B'))

        img_msg.data = message_data

        return img_msg
