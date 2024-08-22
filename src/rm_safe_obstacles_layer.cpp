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
  declareParameter("inflation_radius", rclcpp::ParameterValue(5));
  
  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "point_topic", point_topic_);
  node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
  
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

    int map_width = layered_costmap_->getCostmap()->getSizeInCellsX();
    int map_height = layered_costmap_->getCostmap()->getSizeInCellsY();

    for (const auto& index : clear_indices_) {
      // clear pixel
      master_grid.setCost(index.first, index.second, FREE_SPACE);

      // inflate to clear around the pixel as well
      for (int dx=-inflation_radius_; dx <= inflation_radius_; ++dx) {
        for (int dy=-inflation_radius_; dy <= inflation_radius_; ++dy) {
          // check if cell is within inflation radius
          if (std::sqrt(dx*dx + dy*dy) > inflation_radius_) {continue;}

          // new costmap indices:
          int nx = index.first + dx;
          int ny = index.second + dy;

          // check if new indices are within costmap bounds
          if (nx < 0 || nx > map_width || ny < 0 || ny > map_height) {continue;}

          // clear surrounding pixel
          master_grid.setCost(nx, ny, FREE_SPACE);

        }
      }
    }
    clear_indices_.clear();

}
  
void RmSafeObstaclesLayer::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(mutex_);
    
    geometry_msgs::msg::TransformStamped transform_stamped;
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    try {
        transform_stamped = tf_->lookupTransform(costmap_frame_, msg->header.frame_id, tf2::TimePointZero);
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "Transform failed: %s", ex.what());
        return;
    }
    
    pcl_ros::transformPointCloud(costmap_frame_, transform_stamped, *msg, transformed_cloud);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed_cloud, "z");
    
    
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      unsigned int mx, my; // costmap coordinates
      if (layered_costmap_->getCostmap()->worldToMap(*iter_x, *iter_y, mx, my)) {
        clear_indices_.emplace_back(mx, my);
      }   
    }
}
}  // namespace vikings_bot_point_cloud_processor



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vikings_bot_point_cloud_processor::RmSafeObstaclesLayer, nav2_costmap_2d::Layer)