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

	/**
	* @brief  Get current system time in seconds.
   	* @return Current time in seconds as int.
   	*/
	static int getCurrentTime() {
		auto now = std::chrono::system_clock::now();
    	auto now_in_seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    	int current_time = static_cast<int>(now_in_seconds);

		return current_time;
	}

	/**
	* @brief Transform sensor_msgs PointCloud2 to different frame.
	* @param msg PointCloud2 message.
	* @param frame Frame to which point cloud needs to be transformed.
   	* @return PointCloud2 message transformed to new frame.
   	*/
	sensor_msgs::msg::PointCloud2 transformToFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::string frame);

	std::string point_topic_; //PointCloud2 topic for points to clear
	int inflation_px_; // clearable point inflation radius in px
	int buffer_time_limit_; // how long a point should be kept clear even if safe obstacle is out of sight

private:
	void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_subscription_;
    std::mutex mutex_;

	struct pair_hash {
		template <class T1, class T2>
		std::size_t operator()(const std::pair<T1, T2>& p) const {
			auto hash1 = std::hash<T1>{}(p.first);
			auto hash2 = std::hash<T2>{}(p.second);
			return hash1 ^ hash2; // Combine the two hash values
		}
	};

	struct PointCloud {
		sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
		int timestamp; //not using message time 
	};

	std::unordered_set<std::pair<unsigned int, unsigned int>, pair_hash> inflatable_points_;
	std::string costmap_frame_;

	// A buffer to store point clouds and time stamps that need to be deleted
	std::vector<PointCloud> point_time_buffer_;
};

}

#endif  // RM_SAFE_OBSTACLES_LAYER_HPP_