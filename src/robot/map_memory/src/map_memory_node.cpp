#include "map_memory_node.hpp"

using namespace std;

static const int THRESHOLD = 1.5;

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
    // Declare and read parameters
    width_      = this->declare_parameter<int>("width", 500);
    height_     = this->declare_parameter<int>("height", 500);
    resolution_ = this->declare_parameter<double>("resolution", 0.1);

    if (width_ <= 0 || height_ <= 0 || resolution_ <= 0) {
        RCLCPP_FATAL(this->get_logger(), "Invalid map dimensions or resolution!");
        throw std::runtime_error("Invalid map dimensions or resolution");
    }

    // Create the global occupancy grid
    global_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    if (!global_map) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize global_map!");
        throw std::runtime_error("global_map initialization failed");
    }

    // Fill map metadata
    global_map->info.width      = width_;
    global_map->info.height     = height_;
    global_map->info.resolution = resolution_;

    // IMPORTANT: Set the origin so that cell [0,0] corresponds
    // to (-width/2 * resolution, -height/2 * resolution) in the world.
    // That way, the global map is centered on (0,0) at the robot’s start.
    global_map->info.origin.position.x = -(width_ * resolution_) / 2.0;
    global_map->info.origin.position.y = -(height_ * resolution_) / 2.0;
    global_map->info.origin.orientation.w = 1.0; // no rotation

    // Initialize global map data
    global_map->data.assign(width_ * height_, 0);

    pos_ = {0.0, 0.0};
    update_map_state_ = false;
    update_map_       = false;

    // Subscriptions
    grid_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,
        std::bind(&MapMemoryNode::costmapCallBack, this, std::placeholders::_1)
    );

    odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&MapMemoryNode::odomCallBack, this, std::placeholders::_1)
    );

    // Publisher
    map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Timer for periodic map updates
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MapMemoryNode::updateMap, this)
    );

    RCLCPP_INFO(this->get_logger(), "MapMemoryNode initialized successfully.");
}

void MapMemoryNode::costmapCallBack(const nav_msgs::msg::OccupancyGrid &msg)
{
    // Store a copy of the received local (cost) map
    local_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(msg);
    update_map_state_ = true;
}

void MapMemoryNode::odomCallBack(const nav_msgs::msg::Odometry &msg)
{
    // Get the current position
    double x = msg.pose.pose.position.x;
    double y = msg.pose.pose.position.y;
    
    // Convert to Euler angles
    auto orient = msg.pose.pose.orientation;
    tf2::Quaternion quat(orient.x, orient.y, orient.z, orient.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch;
    m.getRPY(roll, pitch, theta_);  // store yaw in theta_

    // Compute how far we have moved since last update
    dist_ = std::sqrt(std::pow(x - pos_.first, 2) + std::pow(y - pos_.second, 2));

    // If we haven’t moved enough, skip
    if (dist_ < THRESHOLD) {
        return;
    }

    // Update our stored position
    pos_ = {x, y};
    update_map_ = true;
}

void MapMemoryNode::updateMap()
{
    // Only fuse if we have a new local map AND have moved
    if (update_map_ && update_map_state_) {
        linearFusion();
        global_map->header.stamp    = this->now();
        global_map->header.frame_id = "sim_world";
        map_pub->publish(*global_map);

        // Reset update flags
        update_map_ = false;
    }
}

// This function fuses the local map into the global map
void MapMemoryNode::linearFusion()
{
    // Dimensions of global and local maps
    int global_h = global_map->info.height;  // Y dimension
    int global_w = global_map->info.width;   // X dimension
    int local_h  = local_map->info.height;
    int local_w  = local_map->info.width;

    double res = local_map->info.resolution; // both local and global can share same or different res

    // In meters, the "center" of local map is half its width & height
    double local_center_x_m = (local_w * res) / 2.0;
    double local_center_y_m = (local_h * res) / 2.0;

    // Similarly, the global map’s center in meters
    double global_center_x_m = (global_w * res) / 2.0;
    double global_center_y_m = (global_h * res) / 2.0;

    // For each cell in the local map
    for (int i = 0; i < local_h; i++) {
        for (int j = 0; j < local_w; j++) {

            int8_t cost = local_map->data[i * local_w + j];
            if (cost < 0) {
                // If it's unknown in the local map, skip
                continue;
            }

            // Convert array coords (i,j) to local (x,y) in meters
            double x_local = (j * res) - local_center_x_m;
            double y_local = (i * res) - local_center_y_m;

            // Rotate local coords by the robot heading, then translate by the robot’s global pos_
            // Then shift by (pos_.first, pos_.second)
            double newX = x_local * cos(theta_) - y_local * sin(theta_) + pos_.first;
            double newY = x_local * sin(theta_) + y_local * cos(theta_) + pos_.second;

            // Convert global (newX, newY) in meters to global grid indices
            // We treat (global_center_x_m, global_center_y_m) as the center in meters
            int arrX = static_cast<int>((newX + global_center_x_m) / res);
            int arrY = static_cast<int>((newY + global_center_y_m) / res);

            // Boundary check in the global map
            if (arrX < 0 || arrX >= global_w || arrY < 0 || arrY >= global_h) {
                continue;
            }

            // Fuse: take the max cost for now
            int index = arrY * global_w + arrX;
            global_map->data[index] = std::max(global_map->data[index], static_cast<int8_t>(cost));
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}