#ifndef RM_SAFE_OBSTACLES_LAYER_HPP_
#define RM_SAFE_OBSTACLES_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>


namespace vikings_bot_point_cloud_processor
{

class RmSafeObstaclesLayer : public nav2_costmap_2d::Layer
{
public:
	RmSafeObstaclesLayer();

	virtual void onInitialize();
	virtual void updateBounds(
		double robot_x, double robot_y, double robot_yaw, double * min_x,
		double * min_y,
		double * max_x,
		double * max_y);
	virtual void updateCosts(
		nav2_costmap_2d::Costmap2D & master_grid,
		int min_i, int min_j, int max_i, int max_j);

	virtual void reset() { return; }

	virtual bool isClearable() {return true;}

	std::string point_topic_; //PointCloud2 topic for points to clear
	int inflation_radius_; // clearable point inflation radius in px
	

private:
	void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_subscription_;
    std::mutex mutex_;
    std::vector<std::pair<unsigned int, unsigned int>> clear_indices_;
	std::string costmap_frame_;
};

}

#endif  // RM_SAFE_OBSTACLES_LAYER_HPP_