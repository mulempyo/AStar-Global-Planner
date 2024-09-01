#ifndef ASTAR_GLOBAL_H
#define ASTAR_GLOBAL_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <unordered_map>

namespace astar_planner {

    struct Node {
        unsigned int x, y;
        double g_cost, h_cost;
        Node* parent;

        Node(unsigned int x, unsigned int y, double g, double h, Node* parent = nullptr)
            : x(x), y(y), g_cost(g), h_cost(h), parent(parent) {}

        double f_cost() const { return g_cost + h_cost; }

        bool operator==(const Node& other) const {
            return x == other.x && y == other.y;
        }

        struct HashFunction {
            std::size_t operator()(const Node& node) const {
                return std::hash<unsigned int>()(node.x) ^ std::hash<unsigned int>()(node.y);
            }
        };
    };

    struct CompareNodes {
        bool operator()(const Node* a, const Node* b) const {
            return a->f_cost() > b->f_cost();
        }
    };

    class AStarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        AStarPlanner();
        AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        ~AStarPlanner();

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
        bool makePlan(const geometry_msgs::PoseStamped& start, 
                      const geometry_msgs::PoseStamped& goal, 
                      std::vector<geometry_msgs::PoseStamped>& plan) override;
        ros::Publisher plan_pub_;
        std::string global_frame_;

    private:
        costmap_2d::Costmap2D* costmap_;
        bool initialized_;
        double origin_x_, origin_y_, resolution_;
        unsigned int width_, height_;
        unsigned int goal_x_, goal_y_;

        boost::mutex mutex_;
        std::vector<Node*> allocated_nodes_;
        Node* start_node = nullptr;
        Node* neighbor = nullptr;
     
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        std::vector<Node*> aStarSearch(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y);
        std::vector<Node*> getNeighbors(Node* node);
        std::vector<Node*> reconstructPath(Node* goal_node);
        double potentialFieldCost(unsigned int x, unsigned int y) const;
        double heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const;
        double distance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const;
        void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
    };
};

#endif // KWJ_GLOBAL_H

