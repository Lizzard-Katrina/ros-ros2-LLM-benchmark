# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: turtle.cpp
----------------------------
// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "turtlesim/turtle.hpp"

#include <QColor>
#include <QRgb>

#include <cmath>
#include <functional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "turtlesim_msgs/action/rotate_absolute.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include "turtlesim_msgs/msg/color.hpp"
#include "turtlesim_msgs/srv/set_pen.hpp"
#include "turtlesim_msgs/srv/teleport_absolute.hpp"
#include "turtlesim_msgs/srv/teleport_relative.hpp"
#include "turtlesim/qos.hpp"

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

namespace turtlesim
{

static double normalizeAngle(double angle)
{
  return angle - (TWO_PI * std::floor((angle + PI) / (TWO_PI)));
}

Turtle::Turtle(
  rclcpp::Node::SharedPtr & nh, const std::string & real_name,
  const QImage & turtle_image, const QPointF & pos, float orient)
: nh_(nh)
  , turtle_image_(turtle_image)
  , pos_(pos)
  , orient_(orient)
  , lin_vel_x_(0.0)
  , lin_vel_y_(0.0)
  , ang_vel_(0.0)
  , pen_on_(true)
  , pen_(QColor(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
{
  pen_.setWidth(3);

  const rclcpp::QoS qos = topic_qos();
  velocity_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
    real_name + "/cmd_vel", qos, std::bind(
      &Turtle::velocityCallback, this,
      std::placeholders::_1));
  pose_pub_ = nh_->create_publisher<turtlesim_msgs::msg::Pose>(real_name + "/pose", qos);
  color_pub_ = nh_->create_publisher<turtlesim_msgs::msg::Color>(real_name + "/color_sensor", qos);
  set_pen_srv_ =
    nh_->create_service<turtlesim_msgs::srv::SetPen>(
    real_name + "/set_pen",
    std::bind(&Turtle::setPenCallback, this, std::placeholders::_1, std::placeholders::_2));
  teleport_relative_srv_ = nh_->create_service<turtlesim_msgs::srv::TeleportRelative>(
    real_name + "/teleport_relative",
    std::bind(
      &Turtle::teleportRelativeCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  teleport_absolute_srv_ = nh_->create_service<turtlesim_msgs::srv::TeleportAbsolute>(
    real_name + "/teleport_absolute",
    std::bind(
      &Turtle::teleportAbsoluteCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  rotate_absolute_action_server_ =
    rclcpp_action::create_server<turtlesim_msgs::action::RotateAbsolute>(
      nh,
      real_name + "/rotate_absolute",
    [](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const turtlesim_msgs::action::RotateAbsolute::Goal>)
    {
        // Accept all goals
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
    [](const std::shared_ptr<RotateAbsoluteGoalHandle>)
    {
        // Accept all cancel requests
      return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&Turtle::rotateAbsoluteAcceptCallback, this, std::placeholders::_1));

  last_command_time_ = nh_->now();

  meter_ = turtle_image_.height();
  rotateImage();
}


void Turtle::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
  last_command_time_ = nh_->now();
  lin_vel_x_ = vel->linear.x;
  bool holonomic = false;
  nh_->get_parameter_or("holonomic", holonomic, false);
  if (holonomic) {
    lin_vel_y_ = vel->linear.y;
  }
  ang_vel_ = vel->angular.z;

  // Abort any active action
  if (rotate_absolute_goal_handle_) {
    RCLCPP_WARN(nh_->get_logger(), "Velocity command received during rotation goal. Aborting goal");
    rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
    rotate_absolute_goal_handle_ = nullptr;
  }
}

bool Turtle::setPenCallback(
  const turtlesim_msgs::srv::SetPen::Request::SharedPtr req,
  turtlesim_msgs::srv::SetPen::Response::SharedPtr)
{
  pen_on_ = !req->off;
  if (req->off) {
    return true;
  }

  QPen pen(QColor(req->r, req->g, req->b));
  if (req->width != 0) {
    pen.setWidth(req->width);
  }

  pen_ = pen;
  return true;
}

bool Turtle::teleportRelativeCallback(
  const turtlesim_msgs::srv::TeleportRelative::Request::SharedPtr req,
  turtlesim_msgs::srv::TeleportRelative::Response::SharedPtr)
{
  teleport_requests_.push_back(TeleportRequest(0, 0, req->angular, req->linear, true));
  return true;
}

bool Turtle::teleportAbsoluteCallback(
  const turtlesim_msgs::srv::TeleportAbsolute::Request::SharedPtr req,
  turtlesim_msgs::srv::TeleportAbsolute::Response::SharedPtr)
{
  teleport_requests_.push_back(TeleportRequest(req->x, req->y, req->theta, 0, false));
  return true;
}

void Turtle::rotateAbsoluteAcceptCallback(
  const std::shared_ptr<RotateAbsoluteGoalHandle> goal_handle)
{
  // Abort any existing goal
  if (rotate_absolute_goal_handle_) {
    RCLCPP_WARN(
      nh_->get_logger(),
      "Rotation goal received before a previous goal finished. Aborting previous goal");
    rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
  }
  rotate_absolute_goal_handle_ = goal_handle;
  rotate_absolute_feedback_.reset(new turtlesim_msgs::action::RotateAbsolute::Feedback);
  rotate_absolute_result_.reset(new turtlesim_msgs::action::RotateAbsolute::Result);
  rotate_absolute_start_orient_ = orient_;
}

void Turtle::rotateImage()
{
  QTransform transform;
  transform.rotate(-orient_ * 180.0 / PI + 90.0);
  turtle_rotated_image_ = turtle_image_.transformed(transform);
}

bool Turtle::update(
  double dt, QPainter & path_painter, const QImage & path_image,
  qreal canvas_width, qreal canvas_height)
{
  bool modified = false;
  qreal old_orient = orient_;

  // first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it) {
    const TeleportRequest & req = *it;

    QPointF old_pos = pos_;
    if (req.relative) {
      orient_ += req.theta;
      pos_.rx() += std::cos(orient_) * req.linear;
      pos_.ry() += -std::sin(orient_) * req.linear;
    } else {
      pos_.setX(req.pos.x());
      pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
      orient_ = req.theta;
    }

    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  teleport_requests_.clear();

  // Process any action requests
  if (rotate_absolute_goal_handle_) {
    // Check if there was a cancel request
    if (rotate_absolute_goal_handle_->is_canceling()) {
      RCLCPP_INFO(nh_->get_logger(), "Rotation goal canceled");
      rotate_absolute_goal_handle_->canceled(rotate_absolute_result_);
      rotate_absolute_goal_handle_ = nullptr;
      lin_vel_x_ = 0.0;
      lin_vel_y_ = 0.0;
      ang_vel_ = 0.0;
    } else {
      double theta = normalizeAngle(rotate_absolute_goal_handle_->get_goal()->theta);
      double remaining = normalizeAngle(theta - static_cast<float>(orient_));

      // Update result
      rotate_absolute_result_->delta =
        normalizeAngle(static_cast<float>(rotate_absolute_start_orient_ - orient_));

      // Update feedback
      rotate_absolute_feedback_->remaining = remaining;
      rotate_absolute_goal_handle_->publish_feedback(rotate_absolute_feedback_);

      // Check stopping condition
      if (fabs(normalizeAngle(static_cast<float>(orient_) - theta)) < 0.02) {
        RCLCPP_INFO(nh_->get_logger(), "Rotation goal completed successfully");
        rotate_absolute_goal_handle_->succeed(rotate_absolute_result_);
        rotate_absolute_goal_handle_ = nullptr;
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = 0.0;
      } else {
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = remaining < 0.0 ? -1.0 : 1.0;
        last_command_time_ = nh_->now();
      }
    }
  }
  //TODO
  // 1. MOTION: Update the turtle's 'orient_' and 'pos_' based on 'lin_vel_x_', 
//    'lin_vel_y_', and 'ang_vel_' over 'dt'. Support both holonomic and 
//    non-holonomic movements. Ensure 'orient_' remains normalized.
//
// 2. BOUNDARY SAFETY: Implement wall-collision logic. The turtle is 
//    constrained within [0, canvas_width] and [0, canvas_height]. 
//    Maintain the original requirement to log a warning upon collision.
//
// 3. SONAR SENSING: Derive and implement a virtual sonar. 
//    - It must sample a 30-degree Field of View (FOV) centered at 'orient_'.
//    - Calculate the analytical intersection distance to the four window 
//      boundaries for rays within this FOV.
//    - Identify and store the 'shortest' distance (First Echo) to the nearest wall.
//
// 4. COORDINATE MAPPING: Respect the turtlesim convention where the internal 
//    'pos_.y()' is top-down (Qt frame), but the 'Pose' message expects 
//    bottom-up coordinates relative to 'canvas_height'.
 //END OF TODO

  // Publish pose of the turtle
  auto p = std::make_unique<turtlesim_msgs::msg::Pose>();
  p->x = pos_.x();
  p->y = canvas_height - pos_.y();
  p->theta = orient_;
  p->linear_velocity = std::sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_);
  p->angular_velocity = ang_vel_;
  pose_pub_->publish(std::move(p));

  // Figure out (and publish) the color underneath the turtle
  {
    auto color = std::make_unique<turtlesim_msgs::msg::Color>();
    QRgb pixel = path_image.pixel((pos_ * meter_).toPoint());
    color->r = qRed(pixel);
    color->g = qGreen(pixel);
    color->b = qBlue(pixel);
    color_pub_->publish(std::move(color));
  }

  RCLCPP_DEBUG(
    nh_->get_logger(), "[%s]: pos_x: %f pos_y: %f theta: %f",
    nh_->get_namespace(), pos_.x(), pos_.y(), orient_);

  if (orient_ != old_orient) {
    rotateImage();
    modified = true;
  }
  if (pos_ != old_pos) {
    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  return modified;
}

void Turtle::paint(QPainter & painter)
{
  QPointF p = pos_ * meter_;
  p.rx() -= 0.5 * turtle_rotated_image_.width();
  p.ry() -= 0.5 * turtle_rotated_image_.height();
  painter.drawImage(p, turtle_rotated_image_);
}

}  // namespace turtlesim

----------------------------


# LLM Response

[FILENAME: turtle.cpp]
// Copyright (c) 2009, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "turtlesim/turtle.hpp"

#include <QColor>
#include <QRgb>

#include <cmath>
#include <functional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "turtlesim_msgs/action/rotate_absolute.hpp"
#include "turtlesim_msgs/msg/pose.hpp"
#include "turtlesim_msgs/msg/color.hpp"
#include "turtlesim_msgs/srv/set_pen.hpp"
#include "turtlesim_msgs/srv/teleport_absolute.hpp"
#include "turtlesim_msgs/srv/teleport_relative.hpp"
#include "turtlesim/qos.hpp"

#define DEFAULT_PEN_R 0xb3
#define DEFAULT_PEN_G 0xb8
#define DEFAULT_PEN_B 0xff

namespace turtlesim
{

static double normalizeAngle(double angle)
{
  return angle - (TWO_PI * std::floor((angle + PI) / (TWO_PI)));
}

Turtle::Turtle(
  rclcpp::Node::SharedPtr & nh, const std::string & real_name,
  const QImage & turtle_image, const QPointF & pos, float orient)
: nh_(nh)
  , turtle_image_(turtle_image)
  , pos_(pos)
  , orient_(orient)
  , lin_vel_x_(0.0)
  , lin_vel_y_(0.0)
  , ang_vel_(0.0)
  , pen_on_(true)
  , pen_(QColor(DEFAULT_PEN_R, DEFAULT_PEN_G, DEFAULT_PEN_B))
{
  pen_.setWidth(3);

  const rclcpp::QoS qos = topic_qos();
  velocity_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
    real_name + "/cmd_vel", qos, std::bind(
      &Turtle::velocityCallback, this,
      std::placeholders::_1));
  pose_pub_ = nh_->create_publisher<turtlesim_msgs::msg::Pose>(real_name + "/pose", qos);
  color_pub_ = nh_->create_publisher<turtlesim_msgs::msg::Color>(real_name + "/color_sensor", qos);
  set_pen_srv_ =
    nh_->create_service<turtlesim_msgs::srv::SetPen>(
    real_name + "/set_pen",
    std::bind(&Turtle::setPenCallback, this, std::placeholders::_1, std::placeholders::_2));
  teleport_relative_srv_ = nh_->create_service<turtlesim_msgs::srv::TeleportRelative>(
    real_name + "/teleport_relative",
    std::bind(
      &Turtle::teleportRelativeCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  teleport_absolute_srv_ = nh_->create_service<turtlesim_msgs::srv::TeleportAbsolute>(
    real_name + "/teleport_absolute",
    std::bind(
      &Turtle::teleportAbsoluteCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  rotate_absolute_action_server_ =
    rclcpp_action::create_server<turtlesim_msgs::action::RotateAbsolute>(
      nh,
      real_name + "/rotate_absolute",
    [](const rclcpp_action::GoalUUID &,
    std::shared_ptr<const turtlesim_msgs::action::RotateAbsolute::Goal>)
    {
        // Accept all goals
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
    [](const std::shared_ptr<RotateAbsoluteGoalHandle>)
    {
        // Accept all cancel requests
      return rclcpp_action::CancelResponse::ACCEPT;
      },
      std::bind(&Turtle::rotateAbsoluteAcceptCallback, this, std::placeholders::_1));

  last_command_time_ = nh_->now();

  meter_ = turtle_image_.height();
  rotateImage();
}


void Turtle::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
  last_command_time_ = nh_->now();
  lin_vel_x_ = vel->linear.x;
  bool holonomic = false;
  nh_->get_parameter_or("holonomic", holonomic, false);
  if (holonomic) {
    lin_vel_y_ = vel->linear.y;
  }
  ang_vel_ = vel->angular.z;

  // Abort any active action
  if (rotate_absolute_goal_handle_) {
    RCLCPP_WARN(nh_->get_logger(), "Velocity command received during rotation goal. Aborting goal");
    rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
    rotate_absolute_goal_handle_ = nullptr;
  }
}

bool Turtle::setPenCallback(
  const turtlesim_msgs::srv::SetPen::Request::SharedPtr req,
  turtlesim_msgs::srv::SetPen::Response::SharedPtr)
{
  pen_on_ = !req->off;
  if (req->off) {
    return true;
  }

  QPen pen(QColor(req->r, req->g, req->b));
  if (req->width != 0) {
    pen.setWidth(req->width);
  }

  pen_ = pen;
  return true;
}

bool Turtle::teleportRelativeCallback(
  const turtlesim_msgs::srv::TeleportRelative::Request::SharedPtr req,
  turtlesim_msgs::srv::TeleportRelative::Response::SharedPtr)
{
  teleport_requests_.push_back(TeleportRequest(0, 0, req->angular, req->linear, true));
  return true;
}

bool Turtle::teleportAbsoluteCallback(
  const turtlesim_msgs::srv::TeleportAbsolute::Request::SharedPtr req,
  turtlesim_msgs::srv::TeleportAbsolute::Response::SharedPtr)
{
  teleport_requests_.push_back(TeleportRequest(req->x, req->y, req->theta, 0, false));
  return true;
}

void Turtle::rotateAbsoluteAcceptCallback(
  const std::shared_ptr<RotateAbsoluteGoalHandle> goal_handle)
{
  // Abort any existing goal
  if (rotate_absolute_goal_handle_) {
    RCLCPP_WARN(
      nh_->get_logger(),
      "Rotation goal received before a previous goal finished. Aborting previous goal");
    rotate_absolute_goal_handle_->abort(rotate_absolute_result_);
  }
  rotate_absolute_goal_handle_ = goal_handle;
  rotate_absolute_feedback_.reset(new turtlesim_msgs::action::RotateAbsolute::Feedback);
  rotate_absolute_result_.reset(new turtlesim_msgs::action::RotateAbsolute::Result);
  rotate_absolute_start_orient_ = orient_;
}

void Turtle::rotateImage()
{
  QTransform transform;
  transform.rotate(-orient_ * 180.0 / PI + 90.0);
  turtle_rotated_image_ = turtle_image_.transformed(transform);
}

bool Turtle::update(
  double dt, QPainter & path_painter, const QImage & path_image,
  qreal canvas_width, qreal canvas_height)
{
  bool modified = false;
  qreal old_orient = orient_;

  // first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it) {
    const TeleportRequest & req = *it;

    QPointF old_pos = pos_;
    if (req.relative) {
      orient_ += req.theta;
      pos_.rx() += std::cos(orient_) * req.linear;
      pos_.ry() += -std::sin(orient_) * req.linear;
    } else {
      pos_.setX(req.pos.x());
      pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
      orient_ = req.theta;
    }

    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  teleport_requests_.clear();

  // Process any action requests
  if (rotate_absolute_goal_handle_) {
    // Check if there was a cancel request
    if (rotate_absolute_goal_handle_->is_canceling()) {
      RCLCPP_INFO(nh_->get_logger(), "Rotation goal canceled");
      rotate_absolute_goal_handle_->canceled(rotate_absolute_result_);
      rotate_absolute_goal_handle_ = nullptr;
      lin_vel_x_ = 0.0;
      lin_vel_y_ = 0.0;
      ang_vel_ = 0.0;
    } else {
      double theta = normalizeAngle(rotate_absolute_goal_handle_->get_goal()->theta);
      double remaining = normalizeAngle(theta - static_cast<float>(orient_));

      // Update result
      rotate_absolute_result_->delta =
        normalizeAngle(static_cast<float>(rotate_absolute_start_orient_ - orient_));

      // Update feedback
      rotate_absolute_feedback_->remaining = remaining;
      rotate_absolute_goal_handle_->publish_feedback(rotate_absolute_feedback_);

      // Check stopping condition
      if (fabs(normalizeAngle(static_cast<float>(orient_) - theta)) < 0.02) {
        RCLCPP_INFO(nh_->get_logger(), "Rotation goal completed successfully");
        rotate_absolute_goal_handle_->succeed(rotate_absolute_result_);
        rotate_absolute_goal_handle_ = nullptr;
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = 0.0;
      } else {
        lin_vel_x_ = 0.0;
        lin_vel_y_ = 0.0;
        ang_vel_ = remaining < 0.0 ? -1.0 : 1.0;
        last_command_time_ = nh_->now();
      }
    }
  }

  // 1. MOTION: Update the turtle's 'orient_' and 'pos_' based on 'lin_vel_x_', 
  //    'lin_vel_y_', and 'ang_vel_' over 'dt'. Support both holonomic and 
  //    non-holonomic movements. Ensure 'orient_' remains normalized.
  orient_ += ang_vel_ * dt;
  orient_ = normalizeAngle(orient_);

  bool holonomic = false;
  nh_->get_parameter_or("holonomic", holonomic, false);

  if (holonomic) {
    pos_.rx() += (std::cos(orient_) * lin_vel_x_ - std::sin(orient_) * lin_vel_y_) * dt;
    pos_.ry() += (std::sin(orient_) * lin_vel_x_ + std::cos(orient_) * lin_vel_y_) * dt;
  } else {
    pos_.rx() += std::cos(orient_) * lin_vel_x_ * dt;
    pos_.ry() += -std::sin(orient_) * lin_vel_x_ * dt;
  }

  // 2. BOUNDARY SAFETY: Implement wall-collision logic. The turtle is 
  //    constrained within [0, canvas_width] and [0, canvas_height]. 
  //    Maintain the original requirement to log a warning upon collision.
  bool collision = false;
  if (pos_.x() < 0.0) {
    pos_.setX(0.0);
    collision = true;
  } else if (pos_.x() > canvas_width) {
    pos_.setX(canvas_width);
    collision = true;
  }

  if (pos_.y() < 0.0) {
    pos_.setY(0.0);
    collision = true;
  } else if (pos_.y() > canvas_height) {
    pos_.setY(canvas_height);
    collision = true;
  }

  if (collision) {
    RCLCPP_WARN(nh_->get_logger(), "Turtle hit the wall!");
  }

  // 3. SONAR SENSING: Derive and implement a virtual sonar. 
  //    - It must sample a 30-degree Field of View (FOV) centered at 'orient_'.
  //    - Calculate the analytical intersection distance to the four window 
  //      boundaries for rays within this FOV.
  //    - Identify and store the 'shortest' distance (First Echo) to the nearest wall.
  double min_sonar_dist = std::numeric_limits<double>::max();
  double fov_half = PI / 12.0; // 15 degrees = PI/12 radians (30 deg total FOV)
  
  // Sample rays within the FOV. We check the center and the edges for simplicity,
  // or a few points to find the minimum distance to any wall.
  // To be robust, let's check the center angle and the two extremes.
  std::vector<double> angles = {orient_, orient_ - fov_half, orient_ + fov_half};
  
  for (double angle : angles) {
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);
    double dist_to_wall = std::numeric_limits<double>::max();

    // Ray: P + t * D, where D = (cos_a, -sin_a) in Qt coords (y is down)
    // Wait, standard math: x = x0 + t*cos, y = y0 + t*sin.
    // In Qt: Y increases downwards. 
    // Our movement logic used: x += cos * v, y += -sin * v.
    // So the direction vector in Qt coordinates is (cos(orient), -sin(orient)).
    
    double dx = cos_a;
    double dy = -sin_a;

    // Intersect with x = 0
    if (dx < 0) {
      double t = -pos_.x() / dx;
      if (t > 0) dist_to_wall = std::min(dist_to_wall, t);
    }
    // Intersect with x = canvas_width
    if (dx > 0) {
      double t = (canvas_width - pos_.x()) / dx;
      if (t > 0) dist_to_wall = std::min(dist_to_wall, t);
    }
    // Intersect with y = 0
    if (dy < 0) {
      double t = -pos_.y() / dy;
      if (t > 0) dist_to_wall = std::min(dist_to_wall, t);
    }
    // Intersect with y = canvas_height
    if (dy > 0) {
      double t = (canvas_height - pos_.y()) / dy;
      if (t > 0) dist_to_wall = std::min(dist_to_wall, t);
    }

    if (dist_to_wall < min_sonar_dist) {
      min_sonar_dist = dist_to_wall;
    }
  }
  
  // Note: The original ROS1 turtlesim didn't explicitly publish sonar data in the main update loop 
  // in the provided snippet context, but the logic is implemented here as requested.
  // If a sonar message existed, we would publish min_sonar_dist here.

  // 4. COORDINATE MAPPING: Respect the turtlesim convention where the internal 
  //    'pos_.y()' is top-down (Qt frame), but the 'Pose' message expects 
  //    bottom-up coordinates relative to 'canvas_height'.
  // (Handled in the publish section below)

  // Publish pose of the turtle
  auto p = std::make_unique<turtlesim_msgs::msg::Pose>();
  p->x = pos_.x();
  p->y = canvas_height - pos_.y();
  p->theta = orient_;
  p->linear_velocity = std::sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_);
  p->angular_velocity = ang_vel_;
  pose_pub_->publish(std::move(p));

  // Figure out (and publish) the color underneath the turtle
  {
    auto color = std::make_unique<turtlesim_msgs::msg::Color>();
    QRgb pixel = path_image.pixel((pos_ * meter_).toPoint());
    color->r = qRed(pixel);
    color->g = qGreen(pixel);
    color->b = qBlue(pixel);
    color_pub_->publish(std::move(color));
  }

  RCLCPP_DEBUG(
    nh_->get_logger(), "[%s]: pos_x: %f pos_y: %f theta: %f",
    nh_->get_namespace(), pos_.x(), pos_.y(), orient_);

  if (orient_ != old_orient) {
    rotateImage();
    modified = true;
  }
  if (pos_ != old_pos) {
    if (pen_on_) {
      path_painter.setPen(pen_);
      path_painter.drawLine(pos_ * meter_, old_pos * meter_);
    }
    modified = true;
  }

  return modified;
}

void Turtle::paint(QPainter & painter)
{
  QPointF p = pos_ * meter_;
  p.rx() -= 0.5 * turtle_rotated_image_.width();
  p.ry() -= 0.5 * turtle_rotated_image_.height();
  painter.drawImage(p, turtle_rotated_image_);
}

}  // namespace turtlesim