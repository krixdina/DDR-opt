#ifndef FRONT_END_IDE_STUB_ROS_H
#define FRONT_END_IDE_STUB_ROS_H

#include <string>

namespace ros
{
struct Duration
{
  double sec;
  explicit Duration(double s = 0.0) : sec(s) {}
  double toSec() const { return sec; }
};

struct Time
{
  double sec;
  explicit Time(double s = 0.0) : sec(s) {}
  static Time now() { return Time(); }
  double toSec() const { return sec; }
};

inline Duration operator-(const Time &lhs, const Time &rhs)
{
  return Duration(lhs.sec - rhs.sec);
}

class Publisher
{
public:
  Publisher() = default;

  template <typename T>
  void publish(const T &) const {}
};

class NodeHandle
{
public:
  NodeHandle() = default;
  explicit NodeHandle(const std::string &) {}

  template <typename T>
  Publisher advertise(const std::string &, int) const
  {
    return Publisher();
  }

  template <typename T>
  bool getParam(const std::string &, T &value) const
  {
    value = T();
    return true;
  }

  template <typename T>
  void param(const std::string &, T &value, const T &default_value) const
  {
    value = default_value;
  }
};

namespace this_node
{
inline std::string getName()
{
  return "/front_end_ide";
}
}  // namespace this_node

}  // namespace ros

#define ROS_INFO(...)
#define ROS_ERROR(...)
#define ROS_WARN(...)
#define ROS_INFO_STREAM(...)
#define ROS_WARN_STREAM(...)
#define ROS_ERROR_STREAM(...)

#endif
