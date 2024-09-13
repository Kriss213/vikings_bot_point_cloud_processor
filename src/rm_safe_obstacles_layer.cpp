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

    int map_width = layered_costmap_->getCostmap()->getSizeInCellsX();
    int map_height = layered_costmap_->getCostmap()->getSizeInCellsY();
    
    // a vector of points that will need to be inflated
    std::vector<std::pair<unsigned int, unsigned int>> inflatable_points;

    // parse stored point clouds
    RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "Parsing %ld point clouds from pointcloud time buffer", point_time_buffer_.size());
    for (auto it=point_time_buffer_.begin(); it!=point_time_buffer_.end();) {
      //bool increase_it = true;
      PointCloud pc_w_timestamp = *it;
      sensor_msgs::msg::PointCloud2::SharedPtr pc2 = pc_w_timestamp.pointcloud_msg;
      int timestamp = pc_w_timestamp.timestamp;

      if (current_time - timestamp > buffer_time_limit_){ //cast to uint to prevent warning
        it = point_time_buffer_.erase(it);
      } else{
        ++it;
        //  pc2 is in map frame, it needs to be transformed to costmap frame
        //  because costmap can change its position (like local costmap)
        
        sensor_msgs::msg::PointCloud2 transformed_cloud_costmap = transformToFrame(pc2, costmap_frame_);

        // get iterators for point cloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud_costmap, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud_costmap, "y");

        // parse point cloud
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
          unsigned int mx, my; // costmap coordinates
          if (layered_costmap_->getCostmap()->worldToMap(*iter_x, *iter_y, mx, my)) {
            std::pair<unsigned int, unsigned int> indice = {mx, my};

            if (inflatable_points_.find(indice) == inflatable_points_.end()) {
              inflatable_points_.insert(indice); // prevent duplicates
              master_grid.setCost(mx, my, FREE_SPACE);
            }
          }
        }
      }
    }

    for (const auto& map_point : inflatable_points_) {   
    // inflate to clear around the pixel as well
      for (int dx=-inflation_px_; dx <= inflation_px_; ++dx) {
        for (int dy=-inflation_px_; dy <= inflation_px_; ++dy) {
          // check if cell is within inflation radius
          if (std::sqrt(dx*dx + dy*dy) > inflation_px_) {continue;}

          // new costmap indices:
          int nx = map_point.first + dx;
          int ny = map_point.second + dy;

          // check if new indices are within costmap bounds
          if (nx < 0 || nx > map_width || ny < 0 || ny > map_height) {continue;}

          // clear surrounding pixel
          master_grid.setCost(nx, ny, FREE_SPACE);

        }
      }
    }
    //clear_indices_.clear();
    inflatable_points_.clear();
}
  
void RmSafeObstaclesLayer::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    int current_time = getCurrentTime();

    PointCloud pc_w_time ={msg, current_time};

    // insert newest cloud at beginning
    point_time_buffer_.insert(point_time_buffer_.begin(), pc_w_time);
    // NOTE: if slow, use deque instead
}

sensor_msgs::msg::PointCloud2 RmSafeObstaclesLayer::transformToFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::string frame) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    try {
        transform_stamped = tf_->lookupTransform(frame, msg->header.frame_id, tf2::TimePointZero);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "Transform failed: %s", ex.what());
        return transformed_cloud; // returns empty cloud
    }
    
    pcl_ros::transformPointCloud(costmap_frame_, transform_stamped, *msg, transformed_cloud);
    return transformed_cloud;
}


}  // namespace vikings_bot_point_cloud_processor



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vikings_bot_point_cloud_processor::RmSafeObstaclesLayer, nav2_costmap_2d::Layer)