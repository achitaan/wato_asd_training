#include <chrono>
#include <memory>
#include <queue>
#include <utility>
#include <unordered_set>   
#include <cmath>           

#include "costmap_node.hpp"

// If needed for messages
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std;

CostmapNode::CostmapNode() 
: Node("costmap"), 
  costmap_(robot::CostmapCore(this->get_logger()))
{
  grid_data_.size       = 200;   // 100 x 100 grid
  grid_data_.max_cost   = 100;
  grid_data_.radius     = 10;    // inflation radius in cells
  grid_data_.resolution = 0.1;   // each cell is 0.1m

  // Place the "robot's origin" at the center of the array
  // so negative (x,y) from the sensor will become valid indices.
  grid_data_.center.first  = grid_data_.size / 2;
  grid_data_.center.second = grid_data_.size / 2;

  occupancy_grid_ = new int*[grid_data_.size];
  for(int i = 0; i < grid_data_.size; i++){
    occupancy_grid_[i] = new int[grid_data_.size];
    for(int j = 0; j < grid_data_.size; j++){
      occupancy_grid_[i][j] = 0;
    }
  }

  lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10, 
    std::bind(&CostmapNode::lidarCallBack, this, std::placeholders::_1)
  );

  costmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200), 
    std::bind(&CostmapNode::publishGrid, this)
  );
}

void CostmapNode::lidarCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Use the same header in the final OccupancyGrid
  message.header = msg->header;

  // Get the array-center offset (where the robot is in the array)
  int center_x = grid_data_.center.first;
  int center_y = grid_data_.center.second;

  // For each measurement in the LaserScan
  for (size_t i = 0; i < msg->ranges.size(); i++){
    double range = msg->ranges[i];
    double angle = msg->angle_min + i * msg->angle_increment;

    // Optionally skip very large or invalid range readings
    if (!std::isfinite(range)) {
      continue;
    }
    // You may also skip if range is out of map bounds
    if (range > (grid_data_.size * grid_data_.resolution / 2.0)) {
      continue;
    }

    // Compute x,y in the robot frame
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Transform those x,y into array indices by adding center offset
    // and dividing by the resolution
    int grid_x = static_cast<int>( (x / grid_data_.resolution) + center_x );
    int grid_y = static_cast<int>( (y / grid_data_.resolution) + center_y );

    // Boundary check: must be within [0, size)
    if ((grid_x >= 0) && (grid_x < grid_data_.size) &&
        (grid_y >= 0) && (grid_y < grid_data_.size))
    {
      // Mark this cell as an obstacle
      occupancy_grid_[grid_y][grid_x] = 100;

      // Inflate the obstacle with BFS
      this->inflateNeighbours(
        occupancy_grid_,
        grid_x,
        grid_y,
        grid_data_.radius,
        grid_data_.size,
        grid_data_.max_cost
      );
    }
  }
}

void CostmapNode::inflateNeighbours(
  int **occupancy_grid, 
  int x, 
  int y, 
  double inflation_radius, 
  int grid_size,
  double max_cost
){
  // normal BFS
  queue<pair<int, int>> q;
  std::unordered_set<long long> visited; 

  auto encode = [&](int r, int c){
    return (static_cast<long long>(r) << 32) ^ static_cast<long long>(c);
  };

  int dir[4][2] = {
    {1, 0}, {0, 1}, {-1, 0}, {0, -1}
  };

  // Start BFS from the obstacle cell
  q.push({y, x});
  visited.insert(encode(y, x));

  while (!q.empty()){
    auto curr = q.front();
    q.pop();

    int cy = curr.first;
    int cx = curr.second;

    for (auto &d : dir){
      int newY = cy + d[0];
      int newX = cx + d[1];

      if (newX < 0 || newX >= grid_size || newY < 0 || newY >= grid_size)
        continue;

      double dist_squared = std::pow(static_cast<double>(newX - x), 2.0)
                          + std::pow(static_cast<double>(newY - y), 2.0);

      if (dist_squared < (inflation_radius * inflation_radius)){
        if (visited.find(encode(newY, newX)) == visited.end()){
          visited.insert(encode(newY, newX));

          double cost = max_cost * (1.0 - (std::sqrt(dist_squared) / inflation_radius));
          int int_cost = std::max(occupancy_grid[newY][newX], static_cast<int>(cost));

          occupancy_grid[newY][newX] = int_cost;
          q.push({newY, newX});
        }
      }
    }
  }
}

int* CostmapNode::flattenArray(int **arr, int grid_size){
  int *new_occupancy = new int[grid_size * grid_size];

  for (int i = 0; i < grid_size; i++){
    for (int j = 0; j < grid_size; j++){
      new_occupancy[i * grid_size + j] = arr[i][j];
      // Optionally reset each cell after flattening
      arr[i][j] = 0;
    }
  }
  return new_occupancy;
}

void CostmapNode::publishGrid(){
  message.info.width      = grid_data_.size;
  message.info.height     = grid_data_.size;
  message.info.resolution = grid_data_.resolution;

  // Place the origin so that cell [0,0] is at
  //  (-size/2 * resolution, -size/2 * resolution) in world coordinates.
  message.info.origin.position.x = -(grid_data_.size / 2.0) * grid_data_.resolution;
  message.info.origin.position.y = -(grid_data_.size / 2.0) * grid_data_.resolution;
  message.info.origin.position.z = 0.0;
  message.info.origin.orientation.w = 1.0;  // no rotation

  // Initialize data with -1 (unknown)
  message.data.assign(grid_data_.size * grid_data_.size, -1);

  // Flatten the 2D array to 1D
  int *flat = CostmapNode::flattenArray(occupancy_grid_, grid_data_.size);

  // OccupancyGrid data is a std::vector<int8_t>, so we must clamp [0..100]
  // and cast each cell into int8_t.
  message.data.resize(grid_data_.size * grid_data_.size);
  for(int i = 0; i < grid_data_.size * grid_data_.size; i++){
    int val = flat[i];
    if (val > 100) val = 100;
    if (val < 0)   val = 0;
    message.data[i] = static_cast<int8_t>(val);
  }
  delete [] flat; // Clean up

  // Publish
  costmap_pub->publish(message);
}

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}