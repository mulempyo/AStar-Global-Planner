#include <astar_planner/astar.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <mutex>

// Register the A* planner as a plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner {

    AStarPlanner::AStarPlanner() : costmap_(nullptr), initialized_(false) {}

    AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_(nullptr), initialized_(false) {
        initialize(name, costmap_ros);
    }

    AStarPlanner::~AStarPlanner() {
   
       if(neighbor)
         delete neighbor;
       if(start_node)
         delete start_node;
    }

    void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {

        if (!initialized_) {
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("astar_plan",1);
            costmap_ = costmap_ros->getCostmap();
            origin_x_ = costmap_->getOriginX();
            origin_y_ = costmap_->getOriginY();
            resolution_ = costmap_->getResolution();
            width_ = costmap_->getSizeInCellsX();
            height_ = costmap_->getSizeInCellsY();
            global_frame_ = costmap_ros->getGlobalFrameID();
            initialized_ = true;
        } else {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal, 
                                std::vector<geometry_msgs::PoseStamped>& plan) {
        boost::mutex::scoped_lock lock(mutex_); 

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

        std::vector<Node*> path = aStarSearch(start_x, start_y, goal_x, goal_y);

        if (path.empty()) {
            ROS_WARN("Failed to find a valid plan.");
            return false;
        }

        for (Node* node : path) {
            double world_x, world_y;
            mapToWorld(node->x, node->y, world_x, world_y);
            geometry_msgs::PoseStamped pose = goal;
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
            plan.push_back(pose);
        }
        publishPlan(plan);
        return true;
    }
   
    void AStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){

         if (!initialized_) {
            ROS_ERROR("AStarPlanner has not been initialized, please call initialize() before use.");
            return;
        }
 
         nav_msgs::Path gui_path;
         gui_path.poses.resize(path.size());
        
         if(path.empty()) {
        //still set a valid frame so visualization won't hit transform issues
     	   gui_path.header.frame_id = global_frame_;
           gui_path.header.stamp = ros::Time::now();
          } else { 
           gui_path.header.frame_id = path[0].header.frame_id;
           gui_path.header.stamp = path[0].header.stamp;
          }

         // Extract the plan in world co-ordinates, we assume the path is all in the same frame
         for(unsigned int i=0; i < path.size(); i++){
           gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

    std::vector<Node*> AStarPlanner::aStarSearch(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y) {
        goal_x_ = goal_x;
        goal_y_ = goal_y;
 
        std::priority_queue<Node*, std::vector<Node*>, CompareNodes> open_list;
        std::unordered_set<Node, Node::HashFunction> closed_set;

        Node* start_node = new Node(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y), nullptr);
        open_list.push(start_node);

        while (!open_list.empty()) {
            Node* current = open_list.top();
            open_list.pop();

            if (*current == Node(goal_x, goal_y, 0.0, 0.0)) {
                return reconstructPath(current);
            }

            closed_set.insert(*current);

            for (Node* neighbor : getNeighbors(current)) {
                if (closed_set.find(*neighbor) != closed_set.end()) {
                    continue;
                }

                double tentative_g_cost = current->g_cost + distance(current->x, current->y, neighbor->x, neighbor->y);

                if (!neighbor->parent || tentative_g_cost < neighbor->g_cost) {
                    neighbor->g_cost = tentative_g_cost;
                    neighbor->parent = current;
                    open_list.push(neighbor);
                }
            }
        }

        return std::vector<Node*>(); // Return empty path if no path found
    }

    std::vector<Node*> AStarPlanner::getNeighbors(Node* node) {
        std::vector<Node*> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = static_cast<int>(node->x) + dx;
                int ny = static_cast<int>(node->y) + dy;

                if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width_) && ny < static_cast<int>(height_) &&
                    costmap_->getCost(nx, ny) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    double h = heuristic(nx, ny, goal_x_, goal_y_);
                    Node* neighbor = new Node(nx, ny, std::numeric_limits<double>::infinity(), h, node);
                    neighbors.push_back(neighbor);
                }
            }
        }
        return neighbors;
    }

    std::vector<Node*> AStarPlanner::reconstructPath(Node* goal_node) {
        std::vector<Node*> path;
        Node* current = goal_node;
        while (current != nullptr) {
            path.push_back(current);
            current = current->parent;
        }
        std::reverse(path.begin(), path.end());

        return path;
    }

    double AStarPlanner::potentialFieldCost(unsigned int x, unsigned int y) const {
        double cost = 0.0;
        for (int dx = -2; dx <= 2; ++dx) {
            for (int dy = -2; dy <= 2; ++dy) {
                int nx = static_cast<int>(x) + dx;
                int ny = static_cast<int>(y) + dy;
                if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width_) && ny < static_cast<int>(height_)) {
                    double distance_to_obstacle = std::hypot(dx, dy);
                    if (distance_to_obstacle > 0.0) {
                        double obstacle_cost = costmap_->getCost(nx, ny);
                        if (obstacle_cost == costmap_2d::LETHAL_OBSTACLE) {
                            cost += 1.0 / distance_to_obstacle;  
                        }
                    }
                }
            }
        }
        return cost;
    }

    double AStarPlanner::heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const {
        double euclidean_distance = std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
        double potential_cost = potentialFieldCost(x1, y1);
        return euclidean_distance + potential_cost;
    }

    double AStarPlanner::distance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const {
        return std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
    }

    void AStarPlanner::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
        wx = origin_x_ + mx * resolution_;
        wy = origin_y_ + my * resolution_;
    }
};

