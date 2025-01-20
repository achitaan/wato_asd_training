#include <algorithm>
#include <unordered_map>
#include "planner_node.hpp"

PlannerNode::PlannerNode() : rclcpp::Node("planner") {

    // Create subscribers
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallBack, this, std::placeholders::_1));
    goal_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallBack, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallBack, this, std::placeholders::_1));

    // Create publisher
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Create timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallBack, this));
}

void PlannerNode::mapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    grid_ = msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        pathPlan();
    }
}

void PlannerNode::goalCallBack(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    dest_ = msg;
    dest_received_ = true;
    RCLCPP_INFO(this->get_logger(), "trying to execute goalCallBack");
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    pathPlan();
    RCLCPP_INFO(this->get_logger(), "finished goal callback");
}

void PlannerNode::odomCallBack(const nav_msgs::msg::Odometry::SharedPtr msg) {
    pose_ = std::make_shared<geometry_msgs::msg::Pose>(msg->pose.pose);
}

void PlannerNode::timerCallBack() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal Reached!!");
            state_ = State::WAITING_FOR_GOAL;
            nav_msgs::msg::Path empty_path;
            path_pub->publish(empty_path);
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            pathPlan();
        }
    }
}

bool PlannerNode::goalReached() {
    RCLCPP_INFO(this->get_logger(), "goarReached() has started :)");
    double dx;
    double dy;
    if (dest_) {
        dx = dest_->point.x - pose_->position.x;
        dy = dest_->point.y - pose_->position.y;
        RCLCPP_INFO(this->get_logger(), "dog %lf,  %lf", dx, dy);
    } else {
        return false;
    }
    return std::sqrt(dx * dx + dy * dy) < 1; // Threshold for reaching goal
}

void PlannerNode::pathPlan() {
  RCLCPP_INFO(this->get_logger(), "started running path plan");
  if (!dest_received_ || !grid_ || grid_->data.empty()) {
    RCLCPP_INFO(this->get_logger(), "Cannot plan path: Missing map or goal");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = grid_->header.frame_id;

  std::vector<CellIndex> path_nodes;
  if (Astar(path_nodes)) {
    path.poses.clear();

    for (const auto &node : path_nodes) {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = node.x;
      p.pose.position.y = node.y;
      p.pose.orientation.w = 1.0;

      path.poses.push_back(p);
    }
  } 

  else {
    RCLCPP_INFO(this->get_logger(), "Path planning failed.");
    return;
  }


  path_pub->publish(path);
}

bool PlannerNode::Astar(std::vector<CellIndex> &path) {
    RCLCPP_INFO(this->get_logger(), "Started A* Algorithm");

    // Ensure pose and destination are valid
    if (!pose_ || !dest_ || !grid_ || grid_->data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Pose, destination, or map grid is invalid.");
        return false;
    }

    CellIndex start(static_cast<int>(pose_->position.x), static_cast<int>(pose_->position.y));
    CellIndex goal(static_cast<int>(dest_->point.x), static_cast<int>(dest_->point.y));

    if (start == goal) {
        RCLCPP_INFO(this->get_logger(), "Start is the goal. No path planning needed.");
        path.push_back(start);
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "Start: {%d, %d}, Goal: {%d, %d}", start.x, start.y, goal.x, goal.y);

    // Priority queue for A* with custom comparator
    std::priority_queue<Node, std::vector<Node>, CompareF> open_set;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    // Initialize start node
    g_score[start] = 0.0;
    open_set.emplace(start, 0.0, manhattanDist(start, goal));

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        RCLCPP_INFO(this->get_logger(), "Looking at: {%d, %d}", current.index.x, current.index.y);

        if (current.index == goal) {
            RCLCPP_INFO(this->get_logger(), "Goal reached. Constructing path.");
            
            CellIndex current_node = goal;
            while (current_node != start) {
                path.push_back(current_node);
                current_node = came_from[current_node];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return true;
        }

        for (const auto &dir : dir_) {
            CellIndex neighbor(current.index.x + dir[0], current.index.y + dir[1]);

            // Check grid boundaries
            if (neighbor.x < -30 || neighbor.y < -30 ||
                neighbor.x >= static_cast<int>(grid_->info.width) ||
                neighbor.y >= static_cast<int>(grid_->info.height)) {
                RCLCPP_INFO(this->get_logger(), "Neighbor out of bounds: {%d, %d}", neighbor.x, neighbor.y);
                continue;
            }

            int cost = getCost(neighbor);

            // Skip high-cost cells
            if (cost > 85) {
                RCLCPP_INFO(this->get_logger(), "Neighbor {%d, %d} has high cost: %d", neighbor.x, neighbor.y, cost);
                continue;
            }

            double tentative_g_score = g_score[current.index] + 1.0 + cost/5;
        
            if ((g_score.find(neighbor) == g_score.end()) || (tentative_g_score < g_score[neighbor])) {
                g_score[neighbor] = tentative_g_score;
                came_from[neighbor] = current.index;
                double f_score = tentative_g_score + manhattanDist(neighbor, goal);
                open_set.emplace(neighbor, tentative_g_score, f_score);
                RCLCPP_INFO(this->get_logger(), "Added neighbor {%d, %d} to open set with f_score: %f", neighbor.x, neighbor.y, f_score);
            }
        }
    }

    RCLCPP_WARN(this->get_logger(), "A* failed to find a path.");
    return false;
}



std::vector<PlannerNode::CellIndex> PlannerNode::neigh(const CellIndex &cidx) {
    std::vector<CellIndex> res;
    for (const auto &d : dir_) {
        res.emplace_back(cidx.x + d[0], cidx.y + d[1]);
    }
    return res;
}

double PlannerNode::manhattanDist(const CellIndex &pose, const CellIndex &dest) {
    return std::abs(dest.x - pose.x) + std::abs(dest.y - pose.y);
}

bool PlannerNode::convertToMap(double x, double y, int &arrX, int &arrY) {
    int global_h = grid_->info.height;
    int global_w = grid_->info.width;
    double res = grid_->info.resolution;

    int origin_x = global_w * res / 2;
    int origin_y = global_h * res / 2;

    int newX = static_cast<int>((x + origin_x) / res);
    int newY = static_cast<int>((y + origin_y) / res);

    if (newX < 0 || newX >= global_w || newY < 0 || newY >= global_h) {
        return false;
    }

    arrX = newX;
    arrY = newY;
    return true;
}

int PlannerNode::getCost(const CellIndex &id) {
    int arrX, arrY;
    if (!convertToMap(id.x, id.y, arrX, arrY)) {
        return 150;
    }

    int idx = arrY * grid_->info.width + arrX;
    int val = grid_->data[idx];
    return val < 0 ? 150 : val;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
