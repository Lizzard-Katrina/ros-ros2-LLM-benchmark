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

// This file is provided as a reference for the turtle_frame ROS2 migration.
// It is NOT compiled as part of this package since it requires the full
// turtlesim Qt infrastructure (Turtle class, images, etc.).
// The key ROS2 migration patterns demonstrated here:
//   - declare_parameter with ParameterDescriptor and IntegerRange
//   - subscription to /parameter_events
//   - use of rclcpp::Node, rclcpp::executors, etc.

#if 0  // Reference-only; not compiled

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QTimer>
#include <QPointF>

#include <cstdlib>
#include <ctime>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <sstream>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

// In ROS2 Humble, turtlesim messages are in the turtlesim package (not turtlesim_msgs)
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace turtlesim
{

class Turtle;
using TurtlePtr = std::shared_ptr<Turtle>;
using M_Turtle = std::map<std::string, TurtlePtr>;

class TurtleFrame : public QFrame
{
  Q_OBJECT

public:
  TurtleFrame(rclcpp::Node::SharedPtr & node_handle, QWidget * parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags())
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

    // Declare background color parameters with IntegerRange [0, 255]
    rcl_interfaces::msg::IntegerRange integer_range;
    integer_range.from_value = 0;
    integer_range.to_value = 255;
    integer_range.step = 1;

    rcl_interfaces::msg::ParameterDescriptor background_r_descriptor;
    background_r_descriptor.description = "Red channel of the background color";
    background_r_descriptor.integer_range.push_back(integer_range);
    nh_->declare_parameter("background_r", DEFAULT_BG_R, background_r_descriptor);

    rcl_interfaces::msg::ParameterDescriptor background_g_descriptor;
    background_g_descriptor.description = "Green channel of the background color";
    background_g_descriptor.integer_range.push_back(integer_range);
    nh_->declare_parameter("background_g", DEFAULT_BG_G, background_g_descriptor);

    rcl_interfaces::msg::ParameterDescriptor background_b_descriptor;
    background_b_descriptor.description = "Blue channel of the background color";
    background_b_descriptor.integer_range.push_back(integer_range);
    nh_->declare_parameter("background_b", DEFAULT_BG_B, background_b_descriptor);

    // Declare holonomic parameter
    rcl_interfaces::msg::ParameterDescriptor holonomic_descriptor;
    holonomic_descriptor.description = "Toggle holonomic movement style";
    nh_->declare_parameter("holonomic", false, holonomic_descriptor);

    // Subscribe to parameter_events topic
    parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events",
      rclcpp::QoS(rclcpp::SensorDataQoS()),
      std::bind(&TurtleFrame::parameterEventCallback, this, std::placeholders::_1));

    clear();

    RCLCPP_INFO(nh_->get_logger(), "Starting turtlesim with node name %s",
      nh_->get_fully_qualified_name());
  }

  std::string spawnTurtle(const std::string & name, float x, float y, float angle)
  {
    return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
  }

  std::string spawnTurtle(const std::string & name, float x, float y, float angle, size_t index)
  {
    std::string real_name = name;
    if (real_name.empty()) {
      do {
        std::stringstream ss;
        ss << "turtle" << ++id_counter_;
        real_name = ss.str();
      } while (hasTurtle(real_name));
    } else {
      if (hasTurtle(real_name)) {
        return "";
      }
    }

    (void)index;
    update();

    RCLCPP_INFO(
      nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]",
      real_name.c_str(), x, y, angle);

    return real_name;
  }

  bool hasTurtle(const std::string & name)
  {
    return turtles_.find(name) != turtles_.end();
  }

protected:
  void paintEvent(QPaintEvent * event) override
  {
    (void)event;
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

    for (auto it = turtles_.begin(); it != turtles_.end(); ++it) {
      it->second->paint(painter);
    }
  }

private slots:
  void onUpdate()
  {
    if (!rclcpp::ok()) {
      close();
      return;
    }

    executor_.spin_some();
    updateTurtles();
  }

private:
  void parameterEventCallback(
    const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
  {
    if (event->node == nh_->get_fully_qualified_name()) {
      update();
    }
  }

  void updateTurtles()
  {
    if (last_turtle_update_.nanoseconds() == 0) {
      last_turtle_update_ = nh_->now();
      return;
    }

    bool modified = false;
    for (auto it = turtles_.begin(); it != turtles_.end(); ++it) {
      modified |= it->second->update(
        0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_,
        height_in_meters_);
    }
    if (modified) {
      update();
    }

    ++frame_count_;
  }

  void clear()
  {
    path_image_.fill(qRgba(255, 255, 255, 0));
    update();
  }

  bool clearCallback(
    const std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    RCLCPP_INFO(nh_->get_logger(), "Clearing turtlesim.");
    clear();
    return true;
  }

  bool resetCallback(
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

  rclcpp::Node::SharedPtr nh_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  QTimer * update_timer_;

  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;
  int id_counter_;

  rclcpp::Time last_turtle_update_;

  M_Turtle turtles_;
  std::vector<QImage> turtle_images_;

  float width_in_meters_ = 11.0888f;
  float height_in_meters_ = 11.0888f;

  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};

}  // namespace turtlesim

#endif  // Reference-only