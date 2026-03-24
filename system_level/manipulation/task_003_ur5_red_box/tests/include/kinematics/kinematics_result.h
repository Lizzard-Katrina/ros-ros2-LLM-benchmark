#pragma once

namespace kinematics
{

struct KinematicsResult
{
  int error_code;
  KinematicsResult() : error_code(0) {}
};

}  // namespace kinematics
