/*
 * Minimal stub node for task_007 runtime testing.
 * Publishes a geometry_msgs/PoseStamped on /move_base/current_goal periodically,
 * so the runtime test can verify the node is alive.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MoveBaseStub : public rclcpp::Node
{
public:
  MoveBaseStub() : Node("move_base_node")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("move_base/current_goal", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = 1.0;
        msg.pose.position.y = 2.0;
        msg.pose.orientation.w = 1.0;
        pub_->publish(msg);
      });
  }
private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveBaseStub>());
  rclcpp::shutdown();
  return 0;
}