#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "task_001_basic_param/param.hpp"
#include "task_001_basic_param/srv/get_cached_param.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("param_cache_node");
  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  std::string remote_node = "/param_provider";
  if (argc > 1) {
    remote_node = argv[1];
  }

  ros::param::init(node, remote_node, callback_group);

  auto service = node->create_service<task_001_basic_param::srv::GetCachedParam>(
    "get_cached_param",
    [](const std::shared_ptr<task_001_basic_param::srv::GetCachedParam::Request> request,
       std::shared_ptr<task_001_basic_param::srv::GetCachedParam::Response> response) {
      XmlRpc::XmlRpcValue value;
      if (request->use_cache) {
        response->success = ros::param::getCached(request->key, value);
      } else {
        response->success = ros::param::get(request->key, value);
      }
      response->value = response->success ? value.toString() : "";
    },
    rmw_qos_profile_services_default,
    callback_group);

  (void)service;

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}