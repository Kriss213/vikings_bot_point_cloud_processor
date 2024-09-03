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
  declareParameter("buffer_time_limit", rclcpp::ParameterValue(15));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "point_topic", point_topic_);
  node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);
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

    if (indice_time_buffer_.empty() && clear_indices_.empty() ) {
      return; // nothing to do
    }

    // get current time
    auto now = std::chrono::system_clock::now();
    auto now_in_seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    int current_time = static_cast<int>(now_in_seconds);

    int map_width = layered_costmap_->getCostmap()->getSizeInCellsX();
    int map_height = layered_costmap_->getCostmap()->getSizeInCellsY();
    
    // a vector of points that will need to be inflated
    std::vector<std::pair<unsigned int, unsigned int>> inflatable_points;

    for (const auto& map_point : clear_indices_) {
      // clear pixel
      unsigned int nx = map_point.first;
      unsigned int ny = map_point.second;
      master_grid.setCost(nx, ny, FREE_SPACE);
      
      // add point to time buffer
      indice_time_buffer_.emplace_back(std::vector<unsigned int>{nx, ny, static_cast<unsigned int>(current_time)});
      
      // add to inflatable points
      inflatable_points.push_back(map_point);
    }

    // iterate over time point buffer
    for (auto it=indice_time_buffer_.begin(); it!=indice_time_buffer_.end();) {
      std::vector<unsigned int> point_n_time = *it;
      unsigned int nx = point_n_time[0];
      unsigned int ny = point_n_time[1];
      int point_time = point_n_time[2];

      if (current_time - point_time > buffer_time_limit_){ //cast to uint to prevent warning
        it = indice_time_buffer_.erase(it);
      } else{
        // clear point
        master_grid.setCost(nx, ny, FREE_SPACE);
        inflatable_points.emplace_back(nx, ny);
        
        ++it;
      }
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("nav2_costmap_2d"),
      "Time buffer has %ld indices. Clear indices has %ld elements.",
      indice_time_buffer_.size(),
      clear_indices_.size());

    //Inflate points
    for (const auto& map_point : inflatable_points) {
    
    // inflate to clear around the pixel as well
      for (int dx=-inflation_radius_; dx <= inflation_radius_; ++dx) {
        for (int dy=-inflation_radius_; dy <= inflation_radius_; ++dy) {
          // check if cell is within inflation radius
          if (std::sqrt(dx*dx + dy*dy) > inflation_radius_) {continue;}

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
    clear_indices_.clear();
    inflatable_points.clear();
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
    
    clear_indices_.clear(); // make sure old messages are not stored
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      unsigned int mx, my; // costmap coordinates
      if (layered_costmap_->getCostmap()->worldToMap(*iter_x, *iter_y, mx, my)) {
        std::pair<unsigned int, unsigned int> indice = {mx, my};

        if (clear_indices_.find(indice) == clear_indices_.end()) {
          clear_indices_.insert(indice); // prevent duplicates
        }
      }   
    }
}
}  // namespace vikings_bot_point_cloud_processor



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vikings_bot_point_cloud_processor::RmSafeObstaclesLayer, nav2_costmap_2d::Layer)