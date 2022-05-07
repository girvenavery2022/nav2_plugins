#pragma once

#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <random>
#include <cmath>
#include <iostream> // for std::cout

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_straightline_planner
{   
    // from isc_nav/include/utility/point.hpp
    struct Point2D
    {
        double x, y;
        Point2D( const double& ix, const double& iy )
            : x( ix )
            , y( iy )
        {
        }
        Point2D() {}
    };

    // from isc_nav/include/utility/point.hpp
    inline bool operator==( const Point2D& lhs, const Point2D& rhs )
    {
        return ( lhs.x == rhs.x && lhs.y == rhs.y );
    }
    
    class RRT 
    {
    public:
        /**
        * @brief Construct a new RRT object and seed the number generator
        * @param grid occupancy grid
        */
        explicit RRT(nav2_costmap_2d::Costmap2D* costmap) : start_{}, goal_{}, map_(costmap) {
            std::cout << "INSIDE RRT CONSTRUCTOR\n";
            std::random_device rd;
            std::mt19937 rng(rd());
            
            this->rng = rng;
        }

        /**
         * @brief Get path to start rrt
         * @param start_pose a 2D point
         * @param goal_pose a 2D point
         * @return const nav_msgs::msg::Path 
         */
        const nav_msgs::msg::Path get_path(const geometry_msgs::msg::Pose& start_pose, 
            const geometry_msgs::msg::Pose& goal_pose,
            double resolution)
        {
            nav_msgs::msg::Path path;
            start_ = Point2D(start_pose.position.x, start_pose.position.y);
            goal_ = Point2D(goal_pose.position.x, goal_pose.position.y);
            resolution_ = resolution;

            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "INSIDE GET_PATH");
            std::cout << "INSIDE GET_PATH\n";

            Point2D random{}, Xnew{}, Xnearest{};
            double Xnearest_dist = 0;

            // Graph with vertices and edges
            V.push_back(start_);
            E.push_back(0);
            G.push_back(E);

            for (int i = 0; i < lim_; i++)
            {   
                random = randomPosition();

                if (isInObstacle(random))
                {
                    continue; // Skip this loop
                }

                Xnearest = findNearest(random, Xnearest_dist);

                if (getDistance(Xnearest, random) <= max_dist_)
                {
                    // If the random node is within the max_dist_ set it as the new node
                    Xnew = Xnearest;
                }
                else
                {   
                    Xnew = findNew(Xnearest, random, Xnearest_dist);
                }
                
                if (!isCollisionFree(Xnearest, Xnew))
                {
                    continue; // Skip this loop
                }

                // Get the index of new random node and the nearest vertex
                int attach_to = getIndex(Xnew);
                int attach_from = getIndex(Xnearest);

                if (attach_from == -1) {
                    std::cout << "Xnearest does not exist";
                    continue; // Skip this loop
                }

                if (attach_to == -1)
                {
                    int add_new = V.size();
                    V.push_back(Xnew);
                    E.push_back(0);

                    for (int i = 0; i < add_new; i++)
                    {
                        // Fill in the new column
                        G.at(i).push_back(0);
                    }

                    // Fill in new row
                    G.push_back(E);
                    // Add edge
                    G.at(attach_from).at(add_new)++;
                }
                else 
                {
                    // Add edge
                    G.at(attach_from).at(attach_to)++;
                }

                // If vertex at goal return graph
                if ((Xnew.x - goal_.x < 0.1) && (Xnew.y - goal_.y < 0.1))
                {
                    trace_back_path(Xnew, path);
                }
            }

            return path;
        }

        /**
         * @brief Generate a random position on the map
         * @return Point2D 
         */
        Point2D randomPosition()
        {   
            std::uniform_real_distribution<double> dist_x(0.0, map_->getSizeInCellsX());
            std::uniform_real_distribution<double> dist_y(0.0, map_->getSizeInCellsY());

            std::cout << "INSIDE RANDOM_POSITION";

            double x = dist_x(rng);
            double y = dist_y(rng);

            Point2D random_node(x, y);
            return random_node;
        }

        /**
         * @brief Get the vertex (already generated) nearest to the random node
         * @param random_node 
         * @param Xnearest_dist track the distance between nearest vertex and random node
         * @return Point2D 
         */
        Point2D findNearest(const Point2D& random_node, double& Xnearest_dist)
        {
            double dist{};
            Point2D Xnearest{};

            if (!V.empty()) 
            {
                // Assign values to be compared
                Xnearest_dist = getDistance(V.at(0), random_node);
                Xnearest = V.at(0);
            }
            else {
                std::cout << "Empty list";
                return Xnearest;
            }

            for (auto vertex : V)
            {
                dist = getDistance(vertex, random_node);
                if (dist < Xnearest_dist)
                {   
                    // Update distance between nearest vertex and random node
                    Xnearest_dist = dist;
                    Xnearest = vertex;
                }
            }

            return Xnearest;
        }

        /**
         * @brief Get a new node that is on the line (connecting the nearest vertex to random node) 
                    and within the max_dist_ to random node
         * @param p1 nearest vertex (already in the generated list)
         * @param p2 random node
         * @param Xnearest_dist track the distance between nearest vertex and random node
         * @return Point2D 
         */
        Point2D findNew(const Point2D& p1, const Point2D& p2, const double& Xnearest_dist)
        {
            Point2D Xnew{};
            double n = max_dist_;
            double m = Xnearest_dist - n;

            Xnew.x = (m * p2.x + n * p1.x)/(m + n);
            Xnew.y = (m * p2.y + n * p1.y)/(m + n);

            return Xnew;
        }

        /**
         * @brief Check if the new random node is at obstacle on map
         * @param random_node a random generated nose
         * @return true 
         * @return false 
         */
        bool isInObstacle(const Point2D& random_node)
        {
            return map_->getCost(random_node.x, random_node.y) == nav2_costmap_2d::LETHAL_OBSTACLE;
        }

        /**
         * @brief Check if there is any obstacle between the nearest vertex and random node
         * @param p1 nearest node
         * @param p2 random node (aka new node)
         * @return true 
         * @return false 
         */
        bool isCollisionFree(const Point2D& p1, const Point2D& p2)
        {
            double dist = getDistance(p1, p2);
            int num_of_pt = static_cast<int>(dist / resolution_);
            Point2D pt = p1;

            for(int i = 0; i < num_of_pt; i++)
            {
                pt = generateAlongLine(pt, p2, dist);
                if (isInObstacle(pt))
                {
                    return false;
                }
            }
            
            return true;
        }

        /**
         * @brief Generate a node on the line connecting nearest vertex and new random node
         * @param p1 new generated node along the line
         * @param p2 new random node
         * @param dist distance between the new generated along the line and the random node
         * @return Point2D 
         */
        Point2D generateAlongLine(const Point2D& p1, const Point2D& p2, double& dist)
        {
            Point2D Xnew{};
            double n = resolution_;
            double m = dist - n;

            Xnew.x = (m * p2.x + n * p1.x)/(m + n);
            Xnew.y = (m * p2.y + n * p1.y)/(m + n);

            dist -= n;
            return Xnew;
        }

        /**
         * @brief Get the distance between 2 points on map
         * @param p1 a 2D point
         * @param p2 a 2D point
         * @return double 
         */
        double getDistance(const Point2D& p1, const Point2D& p2)
        {
            double dist = std::hypot(p1.x + p2.x, p1.y + p2.y);
            return dist;
        }

        /**
         * @brief Get the index of a point in the vector list (which corresponds to graph G)
         * @param key 
         * @return int
         */
        int getIndex(const Point2D& key)
        {
            auto it = find(V.begin(), V.end(), key);
        
            // If element was found
            if (it != V.end())
            {
                // Calculate the index for key
                int index = it - V.begin();
                return index;
            }
            else {
                // If the element is not present in the vector
                return -1;
            }
        }

        /**
         * @brief Generate a path from the graph
         * @param current the latest generated random node that falls in the goal
         * @param path
         */
        void trace_back_path(const Point2D& end, nav_msgs::msg::Path& path) noexcept
        {
            int size = V.size();
            Point2D current = end;
            int curr_index = getIndex(current);

            while (current.x != start_.x && current.y != start_.y)
            {
                geometry_msgs::msg::PoseStamped pose{};
                pose.pose.position.x = current.x;
                pose.pose.position.y = current.y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                path.poses.emplace_back(pose);
                
                for (int i = 0; i < size; i++)
                {
                    if (G.at(i).at(curr_index))
                    {
                        current = V.at(i);
                        curr_index = i;
                        break;
                    }
                }
            }

            std::reverse(path.poses.begin(), path.poses.end());
        }

    private:
        Point2D start_;
        Point2D goal_;
        //CostMap map_;
        nav2_costmap_2d::Costmap2D* map_;

        double resolution_;

        std::vector<std::vector<int>> G;
        std::vector<int> E;
        std::vector<Point2D> V;

        // Constants
        double max_dist_ = 50; // Shld be shorter of longer??
        int lim_ = 1000; // Number of iterations

        // Random number generator
        std::mt19937 rng;
    };

} // namespace nav2_straightline_planner