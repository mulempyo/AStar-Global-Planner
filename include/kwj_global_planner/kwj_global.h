#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>

   struct Node {
        unsigned int x, y;
        double g_cost, h_cost;
        Node* parent;

        Node(unsigned int x_, unsigned int y_, double g_, double h_, Node* p = nullptr)
            : x(x_), y(y_), g_cost(g_), h_cost(h_), parent(p) {}

        double f_cost() const { return g_cost + h_cost; }
    };

   struct CompareNodes {
        bool operator()(const Node* lhs, const Node* rhs) const {
            return lhs->f_cost() > rhs->f_cost();
        }
    };


namespace my_global_planner {

class AStarPlanner : public nav_core::BaseGlobalPlanner {
public:

    AStarPlanner(); 
    AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(const geometry_msgs::PoseStamped& start, 
                          const geometry_msgs::PoseStamped& goal, 
                          std::vector<geometry_msgs::PoseStamped>& plan); 
    costmap_2d::Costmap2D* costmap_;
    double origin_x_, origin_y_, resolution_;
    unsigned int width_, height_;
    bool initialized_;
    unsigned int goal_x_, goal_y_;

private:
    
    std::vector<Node> aStarSearch(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y);
    std::vector<Node*> getNeighbors(Node* node);
    std::vector<Node> reconstructPath(Node* goal_node);

    double heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const;
    double distance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const;
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;

};

}  // namespace my_global_planner

#endif
