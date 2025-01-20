#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_memory_core.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <utility> // For std::pair
#include <memory>  // For smart pointers
#include <cmath>
#include <algorithm>

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Occupancy Grid Data initialization
    int width_ = 100, height_ = 100;
    double resolution_ = 0.1;

    // New initializations
    double dist_;
    std::pair<double, double> pos_; // Global position
    double theta_;

    // Occupancy grids variables
    bool update_map_, update_map_state_;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr local_map;

    robot::MapMemoryCore map_memory_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void costmapCallBack(const nav_msgs::msg::OccupancyGrid &msg);
    void odomCallBack(const nav_msgs::msg::Odometry &msg);
    void updateMap();
    void linearFusion();
};

#endif