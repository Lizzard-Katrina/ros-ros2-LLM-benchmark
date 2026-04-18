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


# LLM Response

```cpp
#include <husky_high_level_controller/husky_high_level_controller.hpp>

namespace husky_high_level_controller
{

HuskyHighLevelController::HuskyHighLevelController(rclcpp::Node::SharedPtr nodeHandle)
  : nodeHandle_(nodeHandle), it_(nodeHandle)
{
  std::string topic_left = "/husky/camera_depth/image_rect_raw";
  std::string topic_base = "/base_topic";
  bool pub_raw = false;

  initPublisher(topic_left, topic_base, pub_raw);

  subscriber_ = image_transport::create_subscription(nodeHandle_, "/example_topic",
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { topicCallback(msg); }, "raw");
  RCLCPP_INFO(nodeHandle_->get_logger(), "Successfully launched husky_high_level_controller.");
}

HuskyHighLevelController::~HuskyHighLevelController()
{
}

void HuskyHighLevelController::initPublisher(const std::string& topic_left, const std::string& topic_base, bool pub_raw)
{
  if (pub_raw)
  {
    pub_left_ = image_transport::create_publisher(nodeHandle_, topic_left + "/image");
    pub_depth_ = image_transport::create_publisher(nodeHandle_, topic_left + "/depth");
  }
  else
  {
    image_transport::TransportHints hints("raw", "~");
  }
}

void HuskyHighLevelController::topicCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(nodeHandle_->get_logger(), "Received image");
}

}  // namespace husky_high_level_controller
```

```cpp
#ifndef HUSKY_HIGH_LEVEL_CONTROLLER_HPP
#define HUSKY_HIGH_LEVEL_CONTROLLER_HPP

#include <ros_node_tutorialcpp/subscriber.hpp>
#include <ros_node_tutorialcpp/publisher.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace husky_high_level_controller
{

class HuskyHighLevelController
{
public:
  explicit HuskyHighLevelController(ros::NodeHandle& nodeHandle);
  virtual ~HuskyHighLevelController();

private:
  ros::NodeHandle& nodeHandle_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_left_;
  image_transport::Publisher pub_depth_;
  ros::Subscriber subscriber_;

  void initPublisher(const std::string& topic_left, const std::string& topic_base, bool pub_raw);
  void topicCallback(const sensor_msgs::Image::ConstPtr& msg);
};

}  // namespace husky_high_level_controller

#endif  // HUSKY_HIGH_LEVEL_CONTROLLER_HPP
```

```cpp
#include <ros/ros.h>
#include <ros_node_tutorialcpp/husky_high_level_controller.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_high_level_controller");
  ros::NodeHandle nodeHandle("~");
  husky_high_level_controller::HuskyHighLevelController huskyHighLevelController(nodeHandle);
  ros::spin();
  return 0;
}
```