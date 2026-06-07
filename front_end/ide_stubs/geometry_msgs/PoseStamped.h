#ifndef FRONT_END_IDE_STUB_GEOMETRY_MSGS_POSESTAMPED_H
#define FRONT_END_IDE_STUB_GEOMETRY_MSGS_POSESTAMPED_H

namespace geometry_msgs
{
struct Point
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

struct Pose
{
  Point position;
  Quaternion orientation;
};

struct PoseStamped
{
  Pose pose;
};

struct Vector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};
}  // namespace geometry_msgs

#endif
