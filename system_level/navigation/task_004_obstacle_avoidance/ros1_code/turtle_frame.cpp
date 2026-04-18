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
