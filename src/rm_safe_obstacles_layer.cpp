#include "vikings_bot_point_cloud_processor/rm_safe_obstacles_layer.hpp"

#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::FREE_SPACE;


namespace vikings_bot_point_cloud_processor
{

RmSafeObstaclesLayer::RmSafeObstaclesLayer(){
}

void RmSafeObstaclesLayer::onInitialize() {
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("point_topic", rclcpp::ParameterValue(std::string("/safe_obstacle_points")));
  declareParameter("inflation_px", rclcpp::ParameterValue(5));
  declareParameter("buffer_time_limit", rclcpp::ParameterValue(10));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "point_topic", point_topic_);
  node->get_parameter(name_ + "." + "inflation_px", inflation_px_);
  node->get_parameter(name_ + "." + "buffer_time_limit", buffer_time_limit_);
  
  // subscribe to topic
  point_subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&RmSafeObstaclesLayer::PointCloudCallback, this, std::placeholders::_1)
  );

  // get costmap frame
  costmap_frame_ = layered_costmap_->getGlobalFrameID();

  float resolution = layered_costmap_->getCostmap()->getResolution();
  resolution = resolution == -1 ? 0.1 : resolution; // if using default, set to 0.1
  inflation_m_ = inflation_px_ * resolution;

  current_ = true;
  
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "RmSafeObstaclesLayer initialized!");
}

void RmSafeObstaclesLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y) {
    (void) min_x;
    (void) min_y;
    (void) max_x;
    (void) max_y; //prevent unused variable warning
}

void RmSafeObstaclesLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) {
    (void) min_i;
    (void) min_j;
    (void) max_i;
    (void) max_j; //prevent unused variable warning
    if (!enabled_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (point_time_buffer_.empty()) {
      return; // nothing to do
    }

    int current_time = getCurrentTime();

    // parse bounding point clouds
    for (auto it=point_time_buffer_.begin(); it!=point_time_buffer_.end();) {

      PointCloud pc_w_timestamp = *it;
      BoundingPoints pointcloud = pc_w_timestamp.bounding_points;
      int timestamp = pc_w_timestamp.timestamp;

      if (current_time - timestamp > buffer_time_limit_){
        it = point_time_buffer_.erase(it);
      } else{
        ++it;
        // transform bounding point cloud to costmap frame
        BoundingPoints cloud_in_costmap_frame = transformToFrame(pointcloud, costmap_frame_);

        std::vector<geometry_msgs::msg::Point> convex_polygon = {
          geometry_msgs::build<geometry_msgs::msg::Point>().x(cloud_in_costmap_frame.left.x-inflation_m_).y(cloud_in_costmap_frame.left.y).z(0),
          geometry_msgs::build<geometry_msgs::msg::Point>().x(cloud_in_costmap_frame.top.x).y(cloud_in_costmap_frame.top.y+inflation_m_).z(0),
          geometry_msgs::build<geometry_msgs::msg::Point>().x(cloud_in_costmap_frame.right.x+inflation_m_).y(cloud_in_costmap_frame.right.y).z(0),
          geometry_msgs::build<geometry_msgs::msg::Point>().x(cloud_in_costmap_frame.bottom.x).y(cloud_in_costmap_frame.bottom.y-inflation_m_).z(0),
        };
        
        //clear area
        master_grid.setConvexPolygonCost(convex_polygon, FREE_SPACE);
      }

    }

}
  
void RmSafeObstaclesLayer::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    int current_time = getCurrentTime();

    // get 4 bounding points
    BoundingPoints bounding_cloud = getBoundingPointCloud(msg);

    // update point cloud frame
    point_cloud_frame_ = msg->header.frame_id;

    PointCloud pc_w_time ={bounding_cloud, current_time};

    // insert newest cloud at beginning
    point_time_buffer_.push_front(pc_w_time);
}

RmSafeObstaclesLayer::BoundingPoints RmSafeObstaclesLayer::transformToFrame(const BoundingPoints& points, const std::string& target_frame) {
  geometry_msgs::msg::TransformStamped transform_stamped;
  RmSafeObstaclesLayer::BoundingPoints transformed_points;

    try {
        transform_stamped = tf_->lookupTransform(target_frame, point_cloud_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "Transform failed: %s", ex.what());
        return transformed_points; // returns empty BoundingPoints struct
    }
    
    // Convert the transform to an Eigen::Affine3d matrix
    Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
    Eigen::Affine3f transform_eigen_f = transform_eigen.cast<float>(); // Convert to float for PCL compatibility

    transformed_points.top = pcl::transformPoint(points.top, transform_eigen_f);
    transformed_points.bottom = pcl::transformPoint(points.bottom, transform_eigen_f);
    transformed_points.left = pcl::transformPoint(points.left, transform_eigen_f);
    transformed_points.right = pcl::transformPoint(points.right, transform_eigen_f);

    return transformed_points;

}

RmSafeObstaclesLayer::BoundingPoints RmSafeObstaclesLayer::getBoundingPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();

    pcl::PointXYZ left, right, top, bottom;
    left.z = 0; right.z = 0; top.z = 0; bottom.z = 0;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    // Iterate through the point cloud to find the extreme points
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        float x = *iter_x;
        float y = *iter_y;

        if (x < min_x) {
            min_x = x;
            left.x = x;
            left.y = y;
        }
        if (x > max_x) {
            max_x = x;
            right.x = x;
            right.y = y;
        }
        if (y < min_y) {
            min_y = y;
            bottom.x = x;
            bottom.y = y;
        }
        if (y > max_y) {
            max_y = y;
            top.x = x;
            top.y = y;
        }
    }
    BoundingPoints bounding_points = {
      top, bottom, left, right
    };

    return bounding_points;
}

}  // namespace vikings_bot_point_cloud_processor



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vikings_bot_point_cloud_processor::RmSafeObstaclesLayer, nav2_costmap_2d::Layer)