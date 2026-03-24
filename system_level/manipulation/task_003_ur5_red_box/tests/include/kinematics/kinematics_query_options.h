#pragma once

namespace kinematics
{

struct KinematicsQueryOptions
{
  int discretization_method;
  KinematicsQueryOptions() : discretization_method(0) {}
};

}  // namespace kinematics
