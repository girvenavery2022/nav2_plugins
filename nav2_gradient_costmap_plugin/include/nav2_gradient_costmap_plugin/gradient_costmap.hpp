#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_gradient_costmap_plugin
{
class GradientLayer: public nav2_costmap_2d::Layer
{

public:
  GradientLayer();
  
  /**
   * @brief This method is called at the end of plugin initialization.
   * It contains ROS parameter(s) declaration and initialization
   * of need_recalculation variable
   */
  virtual void onInitialize();

  /**
   * @brief The method is called to ask the plugin: which area of costmap it needs to update.
   * Inside this method window bounds are re-calculated if need_recalculation is true
   * and updated independently on its value.
   * @param robot_x 
   * @param robot_y 
   * @param robot_yaw 
   * @param min_x 
   * @param min_y 
   * @param max_x 
   * @param max_y 
   */
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double *min_x,
    double *min_y, 
    double *max_x,
    double *max_y
  );

  /**
   * @brief The method is called when costmap recalculation is required.
   * It updates the costmap within its window bounds.
   * Inside this method the costmap gradient is generated and is writing directly
   * to the resulting costmap master_grid without any merging with previous layers.
   * @param master_grid 
   * @param min_i 
   * @param min_j 
   * @param max_i 
   * @param max_j 
   */
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D &master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j
  );

  virtual void reset()
  {
    return;
  }

  /**
   * @brief The method is called when footprint was changed.
   * Here it just resets need_recalculation variable.
   */
  virtual void onFootprintChanged();

  virtual bool isClearable() { return false;}

private:
  
  double last_min_x;
  double last_min_y;
  double last_max_x;
  double last_max_y;

  // indicates that the entire gradient should be recalculated next time
  bool need_recalculation;

  // size of the gradient in cells
  int GRADIENT_SIZE = 20;

  // step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;

};
} // namespace gradient costmap plugin