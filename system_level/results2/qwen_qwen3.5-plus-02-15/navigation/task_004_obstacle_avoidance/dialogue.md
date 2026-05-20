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

FILE_PATH: draw_square.cpp
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

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim_msgs/msg/pose.hpp>

#include "turtlesim/qos.hpp"

#define PI 3.141592f

class DrawSquare final : public rclcpp::Node
{
public:
  explicit DrawSquare(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("draw_square", options)
  {
    const rclcpp::QoS qos = turtlesim::topic_qos();
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos);

    pose_sub_ =
      this->create_subscription<turtlesim_msgs::msg::Pose>(
      "turtle1/pose", qos, std::bind(&DrawSquare::poseCallback, this, std::placeholders::_1));

    reset_client_ = this->create_client<std_srvs::srv::Empty>("reset");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(16), [this]() {timerCallback();});

    auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_result_ = reset_client_->async_send_request(empty).future;
  }

private:
// TODO [Hole 2]: Implement a Navigation State Machine to draw a 2x2 meter square.
  // You must define the movement states (e.g., FORWARD, TURN) and implement the 
  // transition logic within a timer callback. 
  // Requirements:
  // 1. Use 'current_pose_' to determine if the turtle reached 'goal_pose_'.
  // 2. Linear tolerance < 0.1, Angular tolerance < 0.01.
  // 3. After each 2-meter side, rotate the turtle exactly PI/2.
  // 4. Ensure theta is wrapped correctly and velocity is published to 'turtle1/cmd_vel'.
//END OF TODO

  turtlesim_msgs::msg::Pose current_pose_;
  turtlesim_msgs::msg::Pose goal_pose_;
  bool first_goal_set_ = false;
  bool first_pose_set_ = false;
  State state_ = FORWARD;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedFuture reset_result_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto nh = std::make_shared<DrawSquare>();

  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}

----------------------------

FILE_PATH: turtle_frame.cpp
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

#include "turtlesim/turtle_frame.hpp"

#include <QPointF>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <string>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "turtlesim_msgs/srv/kill.hpp"
#include "turtlesim_msgs/srv/spawn.hpp"

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace turtlesim
{

TurtleFrame::TurtleFrame(rclcpp::Node::SharedPtr & node_handle, QWidget * parent, Qt::WindowFlags f)
: QFrame(parent, f)
  , path_image_(500, 500, QImage::Format_ARGB32)
  , path_painter_(&path_image_)
  , frame_count_(0)
  , id_counter_(0)
{
  setFixedSize(500, 500);
  setWindowTitle("TurtleSim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_ = node_handle;
  executor_.add_node(nh_);

// TODO [Hole 1]: Implement the ROS 2 Parameter system and Event-driven GUI updates.
// 1. Declare 'background_r', 'background_g', and 'background_b' with IntegerRange [0, 255].
// 2. Declare a boolean parameter 'holonomic' to toggle movement styles.
// 3. Set up a subscription to "/parameter_events" (SensorData QoS).
// 4. Implement parameterEventCallback to call update() only for this node's parameter changes.
// Style Note: Use rclcpp::Node handles and ensure the logic integrates with the existing QFrame structure.
//END OF TODO

void TurtleFrame::parameterEventCallback(
  const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
  // only consider events from this node
  if (event->node == nh_->get_fully_qualified_name()) {
    // since parameter events for this event aren't expected frequently just always call update()
    update();
  }
}

bool TurtleFrame::hasTurtle(const std::string & name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string & name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

std::string TurtleFrame::spawnTurtle(
  const std::string & name, float x, float y, float angle,
  size_t index)
{
  std::string real_name = name;
  if (real_name.empty()) {
    do{
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  } else {
    if (hasTurtle(real_name)) {
      return "";
    }
  }

  TurtlePtr t = std::make_shared<Turtle>(
    nh_, real_name, turtle_images_[static_cast<int>(index)], QPointF(
      x,
      height_in_meters_ - y), angle);
  turtles_[real_name] = t;
  update();

  RCLCPP_INFO(
    nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]",
    real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  // make all pixels fully transparent
  path_image_.fill(qRgba(255, 255, 255, 0));
  update();
}

void TurtleFrame::onUpdate()
{
  if (!rclcpp::ok()) {
    close();
    return;
  }

  executor_.spin_some();

  updateTurtles();
}

void TurtleFrame::paintEvent(QPaintEvent * event)
{
  (void)event;  // NO LINT
  QPainter painter(this);

  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;
  nh_->get_parameter("background_r", r);
  nh_->get_parameter("background_g", g);
  nh_->get_parameter("background_b", b);
  QRgb background_color = qRgb(r, g, b);
  painter.fillRect(0, 0, width(), height(), background_color);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it) {
    it->second->paint(painter);
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.nanoseconds() == 0) {
    last_turtle_update_ = nh_->now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it) {
    modified |= it->second->update(
      0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_,
      height_in_meters_);
  }
  if (modified) {
    update();
  }

  ++frame_count_;
}


bool TurtleFrame::clearCallback(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}  // namespace turtlesim

----------------------------


# LLM Response

[FILENAME: draw_square.cpp]
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

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim_msgs/msg/pose.hpp>

#include "turtlesim/qos.hpp"

#define PI 3.141592f

enum State {
  FORWARD,
  TURN
};

class DrawSquare final : public rclcpp::Node
{
public:
  explicit DrawSquare(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("draw_square", options)
  {
    const rclcpp::QoS qos = turtlesim::topic_qos();
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos);

    pose_sub_ =
      this->create_subscription<turtlesim_msgs::msg::Pose>(
      "turtle1/pose", qos, std::bind(&DrawSquare::poseCallback, this, std::placeholders::_1));

    reset_client_ = this->create_client<std_srvs::srv::Empty>("reset");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(16), [this]() {timerCallback();});

    auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_result_ = reset_client_->async_send_request(empty).future;
  }

private:
  void poseCallback(const turtlesim_msgs::msg::Pose::ConstSharedPtr msg)
  {
    current_pose_ = *msg;
    first_pose_set_ = true;
  }

  void timerCallback()
  {
    if (!first_pose_set_) {
      return;
    }

    if (!reset_result_.valid()) {
      return;
    }

    if (!first_goal_set_) {
      goal_pose_.x = current_pose_.x + 2.0;
      goal_pose_.y = current_pose_.y;
      goal_pose_.theta = current_pose_.theta;
      first_goal_set_ = true;
      state_ = FORWARD;
    }

    geometry_msgs::msg::Twist twist;
    double linear_tol = 0.1;
    double angular_tol = 0.01;

    if (state_ == FORWARD) {
      double dx = goal_pose_.x - current_pose_.x;
      double dy = goal_pose_.y - current_pose_.y;
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist < linear_tol) {
        state_ = TURN;
        goal_pose_.theta = current_pose_.theta + PI / 2.0;
      } else {
        double desired_theta = std::atan2(dy, dx);
        double dtheta = desired_theta - current_pose_.theta;
        
        // Normalize angle
        while (dtheta > PI) dtheta -= 2 * PI;
        while (dtheta < -PI) dtheta += 2 * PI;

        if (std::abs(dtheta) > angular_tol) {
          twist.angular.z = 2.0 * dtheta;
        } else {
          twist.linear.x = 2.0 * dist;
        }
      }
    } else if (state_ == TURN) {
      double dtheta = goal_pose_.theta - current_pose_.theta;
      
      // Normalize angle
      while (dtheta > PI) dtheta -= 2 * PI;
      while (dtheta < -PI) dtheta += 2 * PI;

      if (std::abs(dtheta) < angular_tol) {
        state_ = FORWARD;
        double side_x = std::cos(current_pose_.theta) * 2.0;
        double side_y = std::sin(current_pose_.theta) * 2.0;
        goal_pose_.x = current_pose_.x + side_x;
        goal_pose_.y = current_pose_.y + side_y;
      } else {
        twist.angular.z = 2.0 * dtheta;
      }
    }

    twist_pub_->publish(twist);
  }

  turtlesim_msgs::msg::Pose current_pose_;
  turtlesim_msgs::msg::Pose goal_pose_;
  bool first_goal_set_ = false;
  bool first_pose_set_ = false;
  State state_ = FORWARD;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedFuture reset_result_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto nh = std::make_shared<DrawSquare>();

  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}

[FILENAME: turtle_frame.cpp]
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

#include "turtlesim/turtle_frame.hpp"

#include <QPointF>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <string>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "turtlesim_msgs/srv/kill.hpp"
#include "turtlesim_msgs/srv/spawn.hpp"

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace turtlesim
{

TurtleFrame::TurtleFrame(rclcpp::Node::SharedPtr & node_handle, QWidget * parent, Qt::WindowFlags f)
: QFrame(parent, f)
  , path_image_(500, 500, QImage::Format_ARGB32)
  , path_painter_(&path_image_)
  , frame_count_(0)
  , id_counter_(0)
{
  setFixedSize(500, 500);
  setWindowTitle("TurtleSim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_ = node_handle;
  executor_.add_node(nh_);

  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 0;
  range.to_value = 255;
  range.step = 1;

  rcl_interfaces::msg::ParameterDescriptor desc_r;
  desc_r.name = "background_r";
  desc_r.integer_range.push_back(range);
  desc_r.description = "Red channel of the background color";
  nh_->declare_parameter("background_r", rclcpp::ParameterValue(DEFAULT_BG_R), desc_r);

  rcl_interfaces::msg::ParameterDescriptor desc_g;
  desc_g.name = "background_g";
  desc_g.integer_range.push_back(range);
  desc_g.description = "Green channel of the background color";
  nh_->declare_parameter("background_g", rclcpp::ParameterValue(DEFAULT_BG_G), desc_g);

  rcl_interfaces::msg::ParameterDescriptor desc_b;
  desc_b.name = "background_b";
  desc_b.integer_range.push_back(range);
  desc_b.description = "Blue channel of the background color";
  nh_->declare_parameter("background_b", rclcpp::ParameterValue(DEFAULT_BG_B), desc_b);

  rcl_interfaces::msg::ParameterDescriptor desc_holo;
  desc_holo.name = "holonomic";
  desc_holo.description = "Enable holonomic movement";
  nh_->declare_parameter("holonomic", rclcpp::ParameterValue(false), desc_holo);

  const rclcpp::QoS qos = rclcpp::SensorDataQoS();
  param_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events",
    qos,
    std::bind(&TurtleFrame::parameterEventCallback, this, std::placeholders::_1));

void TurtleFrame::parameterEventCallback(
  const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
  // only consider events from this node
  if (event->node == nh_->get_fully_qualified_name()) {
    // since parameter events for this event aren't expected frequently just always call update()
    update();
  }
}

bool TurtleFrame::hasTurtle(const std::string & name)
{
  return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string & name, float x, float y, float angle)
{
  return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

std::string TurtleFrame::spawnTurtle(
  const std::string & name, float x, float y, float angle,
  size_t index)
{
  std::string real_name = name;
  if (real_name.empty()) {
    do{
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  } else {
    if (hasTurtle(real_name)) {
      return "";
    }
  }

  TurtlePtr t = std::make_shared<Turtle>(
    nh_, real_name, turtle_images_[static_cast<int>(index)], QPointF(
      x,
      height_in_meters_ - y), angle);
  turtles_[real_name] = t;
  update();

  RCLCPP_INFO(
    nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]",
    real_name.c_str(), x, y, angle);

  return real_name;
}

void TurtleFrame::clear()
{
  // make all pixels fully transparent
  path_image_.fill(qRgba(255, 255, 255, 0));
  update();
}

void TurtleFrame::onUpdate()
{
  if (!rclcpp::ok()) {
    close();
    return;
  }

  executor_.spin_some();

  updateTurtles();
}

void TurtleFrame::paintEvent(QPaintEvent * event)
{
  (void)event;  // NO LINT
  QPainter painter(this);

  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;
  nh_->get_parameter("background_r", r);
  nh_->get_parameter("background_g", g);
  nh_->get_parameter("background_b", b);
  QRgb background_color = qRgb(r, g, b);
  painter.fillRect(0, 0, width(), height(), background_color);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it) {
    it->second->paint(painter);
  }
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.nanoseconds() == 0) {
    last_turtle_update_ = nh_->now();
    return;
  }

  bool modified = false;
  M_Turtle::iterator it = turtles_.begin();
  M_Turtle::iterator end = turtles_.end();
  for (; it != end; ++it) {
    modified |= it->second->update(
      0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_,
      height_in_meters_);
  }
  if (modified) {
    update();
  }

  ++frame_count_;
}


bool TurtleFrame::clearCallback(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Clearing turtlesim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Resetting turtlesim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}  // namespace turtlesim