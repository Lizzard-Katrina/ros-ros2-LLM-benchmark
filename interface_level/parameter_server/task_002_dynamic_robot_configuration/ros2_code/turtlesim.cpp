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
// INTERRUPTION) HOWEVER CAUSED ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "turtlesim/turtle_frame.hpp"

class TurtleApp : public QApplication
{
public:
  rclcpp::Node::SharedPtr nh_;

  explicit TurtleApp(int & argc, char ** argv)
  : QApplication(argc, argv)
  {
    rclcpp::init(argc, argv);
    nh_ = rclcpp::Node::make_shared("turtlesim");

    // Synchronize the default workspace color configuration from the parameter server.
    // Retrieve the RGB values as a single 'std::vector<int64_t>' named 'background_color_rgb'
    std::vector<int64_t> background_color_rgb;
    if (nh_->get_parameter("background_color_rgb", background_color_rgb)) {
      // Use the retrieved parameter values as needed, e.g., set workspace color
      // (Assuming some member or global variable to set color, not shown here)
    } else {
      // If parameter not set, optionally set a default or handle error
      background_color_rgb = {255, 255, 255};  // default white background
      nh_->set_parameter(rclcpp::Parameter("background_color_rgb", background_color_rgb));
    }
  }
};

int main(int argc, char ** argv)
{
  TurtleApp app(argc, argv);
  return app.exec();
}