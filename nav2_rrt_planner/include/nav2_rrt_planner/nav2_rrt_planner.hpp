#pragma once

#include <string>
#include <memory>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <random>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "std_msgs/msg/string.hpp"

#include "utility/point.hpp"

namespace nav2_rrt_planner
{
class RRT : public nav2_core::GlobalPlanner
{
public:
    RRT() = default;
    ~RRT() = default;

    /**
     * @brief Called when planner server enters on_configure state
     * @param parent Shared pointer to parent node
     * @param name Planner name
     * @param tf tf buffer pointer
     * @param costmap_ros Shared pointer to costmap
     */
    void configure( const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros ) override;

    /**
     * @brief Called when planner server goes to on_cleanup state
     */
    void cleanup() override;

    /**
     * @brief Called when planner server enters on_activate state
     */
    void activate() override;

    /**
     * @brief Called when planner server enters on_deactivate state
     * 
     */
    void deactivate() override;

    /**
     * @brief Called when planner server demands a global plan 
     *      for specified start and goal pose
     * @param start Start pose
     * @param goal Goal pose
     * @return nav_msgs::msg::Path Global plan
     */
    nav_msgs::msg::Path createPlan( const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal ) override;

    /**
    * @brief Generate a random position on the map
    * @return Point2D 
    */
    Point2D randomPosition();

    /**
    * @brief Get the vertex (already generated) nearest to the random node
    * @param random_node 
    * @param Xnearest_dist track the distance between nearest vertex and random node
    * @return Point2D 
    */
    Point2D findNearest(const Point2D& random_node, double& Xnearest_dist);

    /**
    * @brief Get a new node that is on the line (connecting the nearest vertex to random node) 
            and within the max_dist_ to random node
    * @param p1 nearest vertex (already in the generated list)
    * @param p2 random node
    * @param Xnearest_dist track the distance between nearest vertex and random node
    * @return Point2D 
    */
    Point2D findNew(const Point2D& p1, const Point2D& p2, const double& Xnearest_dist);

    /**
    * @brief Check if the new random node is at obstacle on map
    * @param random_node a random generated nose
    * @return true 
    * @return false 
    */
    bool isInObstacle(const Point2D& random_node);

    /**
    * @brief Check if there is any obstacle between the nearest vertex and random node
    * @param p1 nearest node
    * @param p2 random node (aka new node)
    * @return true 
    * @return false 
    */
    bool isCollisionFree(const Point2D& p1, const Point2D& p2);

    /**
    * @brief Generate a node on the line connecting nearest vertex and new random node
    * @param p1 new generated node along the line
    * @param p2 new random node
    * @param dist distance between the new generated along the line and the random node
    * @return Point2D 
    */
    Point2D generateAlongLine(const Point2D& p1, const Point2D& p2, double& dist);

    /**
    * @brief Get the distance between 2 points on map
    * @param p1 a 2D point
    * @param p2 a 2D point
    * @return double 
    */
    double getDistance(const Point2D& p1, const Point2D& p2);

    /**
    * @brief Get the index of a point in the vector list (which corresponds to graph G)
    * @param key 
    * @return int
    */
    int getIndex(const Point2D& key);

    /**
    * @brief Generate a path from the graph
    * @param current the latest generated random node that falls in the goal
    * @param path
    */
    void trace_back_path(const Point2D& end, nav_msgs::msg::Path& path) noexcept;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D* costmap_;
    std::string global_frame_, name_;
    double interpolation_resolution_;
    Point2D start_, goal_;

    std::vector<std::vector<int>> G;
    std::vector<int> E;
    std::vector<Point2D> V;

    double max_dist_;
    int lim_;

    // Random number generator
    std::mt19937 rng;
};
} // namespace nav2_rrt_planner