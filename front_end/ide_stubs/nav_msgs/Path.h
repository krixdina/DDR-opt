#ifndef FRONT_END_IDE_STUB_NAV_MSGS_PATH_H
#define FRONT_END_IDE_STUB_NAV_MSGS_PATH_H

#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace nav_msgs
{
struct Path
{
  struct Header
  {
    ros::Time stamp;
    std::string frame_id;
  } header;

  std::vector<geometry_msgs::PoseStamped> poses;
};
}  // namespace nav_msgs

#endif
