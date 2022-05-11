#include <cmath>
#include <random>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "nav2_rrt_planner/nav2_rrt_planner.hpp"

namespace nav2_rrt_planner
{
void RRT::configure( const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Random number generator
  std::random_device rd;
  std::mt19937 rng(rd());
  this->rng = rng;

  node_->get_parameter("max_dist", max_dist_);
  node_->get_parameter("lim", lim_);

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void RRT::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str()
  );
}

void RRT::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str()
  );
}

void RRT::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str()
  );
}

nav_msgs::msg::Path RRT::createPlan( const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  start_ = Point2D(start.pose.position.x, start.pose.position.y);
  goal_ = Point2D(goal.pose.position.x, goal.pose.position.y);

  Point2D random{}, Xnew{}, Xnearest{};
  double Xnearest_dist = 0;

  // Graph with vertices and edges
  V.push_back(start_);
  E.push_back(0);
  G.push_back(E);

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

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
        trace_back_path(Xnew, global_path);
    }
  }

  return global_path;
}

Point2D RRT::randomPosition()
{   
  std::uniform_real_distribution<double> dist_x(0.0, costmap_->getSizeInCellsX());
  std::uniform_real_distribution<double> dist_y(0.0, costmap_->getSizeInCellsY());

  double x = dist_x(rng);
  double y = dist_y(rng);

  Point2D random_node(x, y);
  return random_node;
}

Point2D RRT::findNearest(const Point2D& random_node, double& Xnearest_dist)
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

Point2D RRT::findNew(const Point2D& p1, const Point2D& p2, const double& Xnearest_dist)
{
  Point2D Xnew{};
  double n = max_dist_;
  double m = Xnearest_dist - n;

  Xnew.x = (m * p2.x + n * p1.x)/(m + n);
  Xnew.y = (m * p2.y + n * p1.y)/(m + n);

  return Xnew;
}

bool RRT::isInObstacle(const Point2D& random_node)
{
  return costmap_->getCost(random_node.x, random_node.y) == nav2_costmap_2d::LETHAL_OBSTACLE;
}

bool RRT::isCollisionFree(const Point2D& p1, const Point2D& p2)
{
  double dist = getDistance(p1, p2);
  int num_of_pt = static_cast<int>(dist / interpolation_resolution_);
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

Point2D RRT::generateAlongLine(const Point2D& p1, const Point2D& p2, double& dist)
{
    Point2D Xnew{};
    double n = interpolation_resolution_;
    double m = dist - n;

    Xnew.x = (m * p2.x + n * p1.x)/(m + n);
    Xnew.y = (m * p2.y + n * p1.y)/(m + n);

    dist -= n;
    return Xnew;
}

double RRT::getDistance(const Point2D& p1, const Point2D& p2)
{
  double dist = std::hypot(p1.x + p2.x, p1.y + p2.y);
  return dist;
}

int RRT::getIndex(const Point2D& key)
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

void RRT::trace_back_path(const Point2D& end, nav_msgs::msg::Path& path) noexcept
{
  int size = V.size();
  Point2D current = end;
  int curr_index = getIndex(current);

  while (current != start_)
  {
      geometry_msgs::msg::PoseStamped pose{};
      pose.pose.position.x = current.x;
      pose.pose.position.y = current.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
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
} // namespace nav2_rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_planner::RRT, nav2_core::GlobalPlanner)