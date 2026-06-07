#ifndef FRONT_END_IDE_STUB_PLAN_ENV_SDF_MAP_H
#define FRONT_END_IDE_STUB_PLAN_ENV_SDF_MAP_H

#include <limits>

#include <Eigen/Dense>
#include <ros/ros.h>

class SDFmap
{
public:
  double grid_interval_ = 0.1;
  double inv_grid_interval_ = 10.0;

  double global_x_upper_ = 10.0;
  double global_y_upper_ = 10.0;
  double global_x_lower_ = -10.0;
  double global_y_lower_ = -10.0;

  int GLX_SIZE_ = 200;
  int GLY_SIZE_ = 200;
  int GLXY_SIZE_ = 40000;
  Eigen::Vector2i EIXY_SIZE_ = Eigen::Vector2i(GLX_SIZE_, GLY_SIZE_);

  SDFmap() = default;
  explicit SDFmap(const ros::NodeHandle &) {}

  Eigen::Vector2d gridIndex2coordd(const Eigen::Vector2i &index) const
  {
    return Eigen::Vector2d(index.x() * grid_interval_, index.y() * grid_interval_);
  }

  Eigen::Vector2d gridIndex2coordd(const int &x, const int &y) const
  {
    return Eigen::Vector2d(x * grid_interval_, y * grid_interval_);
  }

  Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d &pt) const
  {
    return Eigen::Vector2i(static_cast<int>(pt.x() / grid_interval_),
                           static_cast<int>(pt.y() / grid_interval_));
  }

  int Index2Vectornum(const int &x, const int &y) const
  {
    return y * GLX_SIZE_ + x;
  }

  int Index2Vectornum(const Eigen::Vector2i &index) const
  {
    return Index2Vectornum(index.x(), index.y());
  }

  bool isOccupied(const int &, const int &) const { return false; }
  bool isOccupied(const Eigen::Vector2i &) const { return false; }
  bool isUnOccupied(const int &, const int &) const { return true; }
  bool isUnOccupied(const Eigen::Vector2i &) const { return true; }
  bool isOccWithSafeDis(const Eigen::Vector2i &, const double &) const { return false; }
  bool isOccWithSafeDis(const int &, const int &, const double &) const { return false; }

  double getDistanceReal(const Eigen::Vector2d &) const
  {
    return std::numeric_limits<double>::max();
  }
};

#endif
