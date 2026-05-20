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

  /* 
* TODO: 
 * 1. Assign the 'node_handle' to 'nh_' and add it to the 'executor_'.
 * 2. Declare 'background_r', 'background_g', and 'background_b' parameters.
 * Requirement: Use 'rcl_interfaces::msg::IntegerRange' (0-255) for each.
 * 3. Initialize 'clear', 'reset', 'spawn', and 'kill' services using 'nh_->create_service<T>'.
* * CRITICAL Style Constraints for Validation:
   * - Use 'std::bind' for all service callback attachments.
   * - Use the full namespace for placeholders: 'std::placeholders::_1' and 'std::placeholders::_2'.
   * - Ensure callback functions (e.g., &TurtleFrame::clearCallback) are correctly scoped.
   * - Do not use 'using namespace std::placeholders;' inside this constructor. * - Use 'std::bind' with placeholders (_1, _2) for service callbacks.
 *END OF TODO 
*/

  // spawn all available turtle types
  if (false) {
    for (int index = 0; index < turtles.size(); ++index) {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(
        name.toStdString(), 1.0f + 1.5f * (index % 7), 1.0f + 1.5f * (index / 7),
        static_cast<float>(PI) / 2.0f, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(
  const turtlesim_msgs::srv::Spawn::Request::SharedPtr req,
  turtlesim_msgs::srv::Spawn::Response::SharedPtr res)
{
  std::string name = spawnTurtle(req->name, req->x, req->y, req->theta);
  if (name.empty()) {
    RCLCPP_ERROR(nh_->get_logger(), "A turtle named [%s] already exists", req->name.c_str());
    return false;
  }

  res->name = name;

  return true;
}

bool TurtleFrame::killCallback(
  const turtlesim_msgs::srv::Kill::Request::SharedPtr req,
  turtlesim_msgs::srv::Kill::Response::SharedPtr)
{
  M_Turtle::iterator it = turtles_.find(req->name);
  if (it == turtles_.end()) {
    RCLCPP_ERROR(
      nh_->get_logger(), "Tried to kill turtle [%s], which does not exist", req->name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

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
/* * TODO: Runtime Loop Migration
 * 1. Check if ROS context is still valid.
 * 2. Trigger a non-blocking spin for the 'executor_'.
 * 3. Update turtles and handle window closing if ROS is shut down.
 *END OF TODO 
*/
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

FILE_PATH: turtle_frame.hpp
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

#ifndef TURTLESIM__TURTLE_FRAME_HPP_
#define TURTLESIM__TURTLE_FRAME_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "turtle.hpp"  // NO LINT
#endif

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim_msgs/srv/spawn.hpp>
#include <turtlesim_msgs/srv/kill.hpp>
#endif

namespace turtlesim
{
class TurtleFrame : public QFrame
{
  Q_OBJECT

public:
  TurtleFrame(
    rclcpp::Node::SharedPtr & node_handle, QWidget * parent = 0,
    Qt::WindowFlags f = Qt::WindowFlags());
  ~TurtleFrame();

  std::string spawnTurtle(const std::string & name, float x, float y, float angle);
  std::string spawnTurtle(const std::string & name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent * event);

private slots:
  void onUpdate();

private:
  
/* * TODO: System-level Migration (Header)
 * 1. Declare ROS 2 node pointer 'nh_' and a SingleThreadedExecutor 'executor_'.
 * 2. Declare ROS 2 Service members for 'clear', 'reset', 'spawn', and 'kill'.
 * 3. Declare a ParameterEvent subscription and its corresponding callback.
 * Style Constraints: 
 * - Use 'rclcpp::Service<T>::SharedPtr' for services.
 * - Callback signatures must use '(Request::SharedPtr, Response::SharedPtr)'.
 * - Keep original variable names (e.g., clear_srv_, reset_srv_) for linkage.
 *END OF TODO 
*/
  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_;
  uint32_t id_counter_;

  QVector<QImage> turtle_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};
}  // namespace turtlesim
#endif  // TURTLESIM__TURTLE_FRAME_HPP_

----------------------------


# LLM Response

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
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(nh_);

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

  clear_srv_ = nh_->create_service<std_srvs::srv::Empty>(
    "clear", std::bind(&TurtleFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  reset_srv_ = nh_->create_service<std_srvs::srv::Empty>(
    "reset", std::bind(&TurtleFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  spawn_srv_ = nh_->create_service<turtlesim_msgs::srv::Spawn>(
    "spawn", std::bind(&TurtleFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  kill_srv_ = nh_->create_service<turtlesim_msgs::srv::Kill>(
    "kill", std::bind(&TurtleFrame::killCallback, this, std::placeholders::_1, std::placeholders::_2));

  param_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "parameter_events", 1, std::bind(&TurtleFrame::parameterEventCallback, this, std::placeholders::_1));

  // spawn all available turtle types
  if (false) {
    for (int index = 0; index < turtles.size(); ++index) {
      QString name = turtles[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnTurtle(
        name.toStdString(), 1.0f + 1.5f * (index % 7), 1.0f + 1.5f * (index / 7),
        static_cast<float>(PI) / 2.0f, index);
    }
  }
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

bool TurtleFrame::spawnCallback(
  const turtlesim_msgs::srv::Spawn::Request::SharedPtr req,
  turtlesim_msgs::srv::Spawn::Response::SharedPtr res)
{
  std::string name = spawnTurtle(req->name, req->x, req->y, req->theta);
  if (name.empty()) {
    RCLCPP_ERROR(nh_->get_logger(), "A turtle named [%s] already exists", req->name.c_str());
    return false;
  }

  res->name = name;

  return true;
}

bool TurtleFrame::killCallback(
  const turtlesim_msgs::srv::Kill::Request::SharedPtr req,
  turtlesim_msgs::srv::Kill::Response::SharedPtr)
{
  M_Turtle::iterator it = turtles_.find(req->name);
  if (it == turtles_.end()) {
    RCLCPP_ERROR(
      nh_->get_logger(), "Tried to kill turtle [%s], which does not exist", req->name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();

  return true;
}

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

  executor_->spin_some();

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

[FILENAME: turtle_frame.hpp]
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

#ifndef TURTLESIM__TURTLE_FRAME_HPP_
#define TURTLESIM__TURTLE_FRAME_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "turtle.hpp"  // NO LINT
#endif

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <map>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim_msgs/srv/spawn.hpp>
#include <turtlesim_msgs/srv/kill.hpp>
#endif

namespace turtlesim
{
class TurtleFrame : public QFrame
{
  Q_OBJECT

public:
  TurtleFrame(
    rclcpp::Node::SharedPtr & node_handle, QWidget * parent = 0,
    Qt::WindowFlags f = Qt::WindowFlags());
  ~TurtleFrame();

  std::string spawnTurtle(const std::string & name, float x, float y, float angle);
  std::string spawnTurtle(const std::string & name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent * event);

private slots:
  void onUpdate();

private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Executor::SharedPtr executor_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<turtlesim_msgs::srv::Spawn>::SharedPtr spawn_srv_;
  rclcpp::Service<turtlesim_msgs::srv::Kill>::SharedPtr kill_srv_;

  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;

  bool clearCallback(
    const std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);
  bool resetCallback(
    const std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);
  bool spawnCallback(
    const turtlesim_msgs::srv::Spawn::Request::SharedPtr req,
    turtlesim_msgs::srv::Spawn::Response::SharedPtr res);
  bool killCallback(
    const turtlesim_msgs::srv::Kill::Request::SharedPtr req,
    turtlesim_msgs::srv::Kill::Response::SharedPtr res);
  void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event);

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_;
  uint32_t id_counter_;

  QVector<QImage> turtle_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};
}  // namespace turtlesim
#endif  // TURTLESIM__TURTLE_FRAME_HPP_