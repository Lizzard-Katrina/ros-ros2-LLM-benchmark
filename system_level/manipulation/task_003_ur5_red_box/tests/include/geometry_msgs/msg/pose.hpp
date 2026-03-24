#pragma once

namespace geometry_msgs
{
namespace msg
{

struct Point
{
  double x{0}, y{0}, z{0};
};

struct Quaternion
{
  double x{0}, y{0}, z{0}, w{1};
};

struct Pose
{
  Point position;
  Quaternion orientation;
};

}  // namespace msg
}  // namespace geometry_msgs
