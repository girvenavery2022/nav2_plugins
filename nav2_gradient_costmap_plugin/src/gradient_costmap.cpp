#include "nav2_gradient_costmap_plugin/gradient_costmap.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_gradient_costmap_plugin
{
GradientLayer::GradientLayer() 
: last_min_x(-std::numeric_limits<float>::max()),
  last_min_y(-std::numeric_limits<float>::max()),
  last_max_x(std::numeric_limits<float>::max()),
  last_max_y(std::numeric_limits<float>::max())
{
}

void GradientLayer::onInitialize()
{
  auto node = node_.lock();
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  need_recalculation = false;
  current_ = true;
}

void GradientLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
  double *min_y, double *max_x, double *max_y)
{
  if (need_recalculation) {
    // recalculate the gradient 
    last_min_x = *min_x;
    last_min_y = *min_y;
    last_max_x = *max_x;
    last_max_y = *max_y;

    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation = false;
  }
  else
  {
    double tmp_min_x = last_min_x;
    double tmp_min_y = last_min_y;
    double tmp_max_x = last_max_x;
    double tmp_max_y = last_max_y;

    last_min_x = *min_x;
    last_min_y = *min_y;
    last_max_x = *max_x;
    last_max_y = *max_y;

    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}

void GradientLayer::onFootprintChanged()
{
  // robot footprint changed, we need to recalculate the gradient 
  need_recalculation = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
    "nav2_costmap_2d"), "GradientLayer::onFootprintChanged(): num footprint points: %lu", 
    layered_costmap_->getFootprint().size());

}

void GradientLayer::updateCosts(
  nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  return; // remove after implementing function
}

} // namespace nav2 gradient costmap plugin
