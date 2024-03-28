#include "obstacle_finder/obstacle_finder.h"

namespace obstacle_finder {
  ObstacleFinder::ObstacleFinder(costmap_2d::Costmap2DROS* costmap)
    : costmap_(costmap), robot_odom_x_(0), robot_odom_y_(0) { }

  ObstacleFinder::ObstacleFinder(costmap_2d::Costmap2DROS* costmap, double robot_odom_x, double robot_odom_y)
    : costmap_(costmap), robot_odom_x_(robot_odom_x), robot_odom_y_(robot_odom_y) {}

  Obstacle ObstacleFinder::nearestObstacle() {
    return nearestObstacle(costmap_, robot_odom_x_, robot_odom_y_);
  }

  Obstacle ObstacleFinder::nearestObstacle(double robot_odom_x, double robot_odom_y) {
    robot_odom_x_ = robot_odom_x;
    robot_odom_y_ = robot_odom_y;
    return nearestObstacle(costmap_, robot_odom_x, robot_odom_y);
  }

  Obstacle ObstacleFinder::nearestObstacle(costmap_2d::Costmap2DROS* new_costmap, double robot_odom_x,
    double robot_odom_y) {
    costmap_ = new_costmap;
    robot_odom_x_ = robot_odom_x;
    robot_odom_y_ = robot_odom_y;
    unsigned int min_x = INT_MAX;
    unsigned int min_y = INT_MAX;
    double minimum_distance_squared = DBL_MAX;
    unsigned int cell_x_idx, cell_y_idx;

    // lock costmap so content doesn't change while adding cell costs
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getCostmap()->getMutex()));

    costmap_2d::Costmap2D* costmap = costmap_->getCostmap();

    unsigned int robot_map_x, robot_map_y;
    costmap->worldToMap(robot_odom_x, robot_odom_y, robot_map_x, robot_map_y);

    for (cell_x_idx = 0; cell_x_idx < costmap->getSizeInCellsX(); cell_x_idx++) {
      for (cell_y_idx = 0; cell_y_idx < costmap->getSizeInCellsY(); cell_y_idx++) {
        double cost_idx = costmap->getCost(cell_x_idx, cell_y_idx);

        // if we found an obstacle, check and set if it's the new closest
        if (cost_idx >= costmap_2d::LETHAL_OBSTACLE) {
          int dx = cell_x_idx - robot_map_x;
          int dy = cell_y_idx - robot_map_y;

          double dist_idx_squared = dx * dx + dy * dy;
          if (dist_idx_squared < minimum_distance_squared) {
            minimum_distance_squared = dist_idx_squared;
            min_x = cell_x_idx;
            min_y = cell_y_idx;
          }
        }
      }
    }

    Obstacle co;
    costmap->mapToWorld(min_x, min_y, co.x, co.y);
    co.dist = sqrt(minimum_distance_squared) * costmap->getResolution();
    return co;
  }

  Obstacle::Obstacle(double x, double y, double dist) : x(x), y(y), dist(dist) {}

  Obstacle::Obstacle() : x(0), y(0), dist(0) {}
}
