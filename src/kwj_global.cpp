#include <kwj_global_planner/kwj_global.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

// Register the A* planner as a plugin
PLUGINLIB_EXPORT_CLASS(my_global_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace my_global_planner {

    AStarPlanner::AStarPlanner() : costmap_(nullptr), initialized_(false) {}

    AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros): costmap_(nullptr), initialized_(false) {
        initialize(name, costmap_ros);
    }

   void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {

        ros::NodeHandle n;

        if (!initialized_) {
            costmap_ = costmap_ros->getCostmap();
            origin_x_ = costmap_->getOriginX();
            origin_y_ = costmap_->getOriginY();
            resolution_ = costmap_->getResolution();
            width_ = costmap_->getSizeInCellsX();
            height_ = costmap_->getSizeInCellsY();
            initialized_ = true;
        } else {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

   bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                          const geometry_msgs::PoseStamped& goal, 
                          std::vector<geometry_msgs::PoseStamped>& plan) {

        ros::NodeHandle n;

        if (!initialized_) {
            ROS_ERROR("AStarPlanner has not been initialized, please call initialize() before use.");
            return false;
        }

        plan.clear();

        unsigned int start_x, start_y, goal_x, goal_y;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
            !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
            ROS_WARN("The start or goal is out of the map bounds.");
            return false;
        }

        std::vector<Node> path = aStarSearch(start_x, start_y, goal_x, goal_y);

        if (path.empty()) {
            ROS_WARN("Failed to find a valid plan.");
            return false;
        }

        for (size_t i =0; i < path.size(); ++i) {
            double world_x, world_y;
            mapToWorld(path[i].x, path[i].y, world_x, world_y);
            geometry_msgs::PoseStamped pose = goal;
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }

        return true;
    }

    std::vector<Node> AStarPlanner::aStarSearch(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y) {

        goal_x_ = goal_x;
        goal_y_ = goal_y;
        std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_list;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));

        Node* start_node = new Node(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y));
        open_list.push(start_node);

        while (!open_list.empty()) {
            Node* current = open_list.top();
            open_list.pop();

            if (current->x == goal_x && current->y == goal_y) {
                return reconstructPath(current);
            }

            closed_list[current->x][current->y] = true;

            for (const auto& neighbor : getNeighbors(current)) {
                if (closed_list[neighbor->x][neighbor->y]) continue;

                double tentative_g_cost = current->g_cost + distance(current->x, current->y, neighbor->x, neighbor->y);

                if (tentative_g_cost < neighbor->g_cost) {
                    neighbor->g_cost = tentative_g_cost;
                    neighbor->parent = current;
                    open_list.push(neighbor);
                }
            }
        }

        return std::vector<Node>(); // Return empty path if no path found
    }

    std::vector<Node*> AStarPlanner::getNeighbors(Node* node) {
      std::vector<Node*> neighbors;
     for (int dx = -1; dx <= 1; ++dx) {
         for (int dy = -1; dy <= 1; ++dy) {
             if (dx == 0 && dy == 0) continue;

            unsigned int nx = node->x + dx;
            unsigned int ny = node->y + dy;

            if (nx < width_ && ny < height_ && costmap_->getCost(nx, ny) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                double h = heuristic(nx, ny, goal_x_, goal_y_);
                Node* neighbor = new Node(nx, ny, std::numeric_limits<double>::max(), h, node);
                neighbors.push_back(neighbor);
            }
        }
    }
    return neighbors;
   }


    std::vector<Node> AStarPlanner::reconstructPath(Node* goal_node) {
      std::vector<Node> path;
      Node* current = goal_node;
      while (current != nullptr) {
         path.push_back(*current);
         Node* to_delete = current;
         current = current->parent;
         delete to_delete; 
      }
      std::reverse(path.begin(), path.end());
      return path;
    }


    double AStarPlanner::heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const {
        return std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
    }

    double AStarPlanner::distance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const {
        return std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
    }

    void AStarPlanner::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
        wx = origin_x_ + mx * resolution_;
        wy = origin_y_ + my * resolution_;
    }
};


