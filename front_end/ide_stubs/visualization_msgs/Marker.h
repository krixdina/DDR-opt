#ifndef FRONT_END_IDE_STUB_VISUALIZATION_MSGS_MARKER_H
#define FRONT_END_IDE_STUB_VISUALIZATION_MSGS_MARKER_H

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace visualization_msgs
{
struct Marker
{
  enum
  {
    ADD = 0,
    TEXT_VIEW_FACING = 9
  };

  struct Header
  {
    ros::Time stamp;
    std::string frame_id;
  } header;

  struct Color
  {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 0.0f;
  };

  std::string ns;
  int id = 0;
  int type = 0;
  int action = 0;
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 scale;
  Color color;
  std::string text;
};
}  // namespace visualization_msgs

#endif
