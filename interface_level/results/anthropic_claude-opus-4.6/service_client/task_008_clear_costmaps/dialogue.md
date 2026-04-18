# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>

#include <costmap_2d/testing_helper.h>
#include <tf2_ros/transform_listener.h>

tf2_ros::Buffer* transformer;
tf2_ros::TransformListener* tfl;

using costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(std::string name, 
                       double distance, 
                       bool obstacles, 
                       bool static_map,
                       costmap_2d::Costmap2DROS* global_costmap, 
                       costmap_2d::Costmap2DROS* local_costmap){
// TODO: Replace the direct behavior invocation with a ROS2 request/response call.
// The implementation should issue a service request to trigger the clearing behavior,
// wait for completion, and handle success/failure appropriately.
//END OF TODO

}

void testCountLethal(std::string name, double distance, bool obstacles, bool static_map, int global_lethal, int local_lethal=0)
{
    costmap_2d::Costmap2DROS global(name + "/global", *transformer);
    costmap_2d::Costmap2DROS  local(name + "/local" , *transformer);
    boost::shared_ptr<costmap_2d::ObstacleLayer> olayer;
        
    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = global.getLayeredCostmap()->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
        boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
        if(plugin->getName().find("obstacles")!=std::string::npos){
            olayer = boost::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
            addObservation(&(*olayer), 5.0, 0.0, MAX_Z/2, 0, 0, MAX_Z/2);
            addObservation(&(*olayer), 0.0, 5.0, MAX_Z/2, 0, 0, MAX_Z/2);
        }
    }
    
    global.updateMap();
    local.updateMap();
    olayer->clearStaticObservations(true, true);

    testClearBehavior("clear", distance, obstacles, static_map, &global, &local);
    
    global.updateMap();
    local.updateMap();
    
    printMap(*global.getCostmap());
    ASSERT_EQ(countValues(*global.getCostmap(), LETHAL_OBSTACLE), global_lethal);
    ASSERT_EQ(countValues( *local.getCostmap(), LETHAL_OBSTACLE),  local_lethal);
    
}

TEST(ClearTester, basicTest){
  testCountLethal("base", 3.0, true, false, 20);
}

TEST(ClearTester, bigRadiusTest){
  testCountLethal("base", 20.0, true, false, 22);
}

TEST(ClearTester, clearNoLayersTest){
  testCountLethal("base", 20.0, false, false, 22);
}

TEST(ClearTester, clearBothTest){
  testCountLethal("base", 3.0, true, true, 0);
}

TEST(ClearTester, clearBothTest2){
  testCountLethal("base", 12.0, true, true, 6);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "clear_tests");
  testing::InitGoogleTest(&argc, argv);
  transformer = new tf2_ros::Buffer(ros::Duration(10));
  tfl = new tf2_ros::TransformListener(*transformer);
  return RUN_ALL_TESTS();
}

----------------------------


# LLM Response



```cpp
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/testing_helper.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

std::shared_ptr<tf2_ros::Buffer> transformer;
std::shared_ptr<tf2_ros::TransformListener> tfl;
std::shared_ptr<rclcpp::Node> test_node;

using nav2_costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(std::string name,
                       double distance,
                       bool obstacles,
                       bool static_map,
                       nav2_costmap_2d::Costmap2DROS* global_costmap,
                       nav2_costmap_2d::Costmap2DROS* local_costmap)
{
  // Create a client to call the clear costmap service
  auto client = test_node->create_client<nav2_msgs::srv::ClearEntireCostmap>(
    name + "/clear_entirely");

  // Wait for the service to be available
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(test_node->get_logger(),
      "Service not available, performing direct clear instead");

    // Fallback: directly reset the costmap layers based on parameters
    auto* global_layered = global_costmap->getLayeredCostmap();
    auto* local_layered = local_costmap->getLayeredCostmap();

    auto global_plugins = global_layered->getPlugins();
    for (auto& plugin : *global_plugins) {
      std::string plugin_name = plugin->getName();
      bool is_obstacle = plugin_name.find("obstacles") != std::string::npos;
      bool is_static = plugin_name.find("static") != std::string::npos;

      if (obstacles && is_obstacle) {
        auto obstacle_layer = std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);
        if (obstacle_layer) {
          obstacle_layer->reset();
        }
      }
      if (static_map && is_static) {
        plugin->reset();
      }
    }

    auto local_plugins = local_layered->getPlugins();
    for (auto& plugin : *local_plugins) {
      std::string plugin_name = plugin->getName();
      bool is_obstacle = plugin_name.find("obstacles") != std::string::npos;
      bool is_static = plugin_name.find("static") != std::string::npos;

      if (obstacles && is_obstacle) {
        auto obstacle_layer = std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);
        if (obstacle_layer) {
          obstacle_layer->reset();
        }
      }
      if (static_map && is_static) {
        plugin->reset();
      }
    }
    return;
  }

  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(test_node, future, std::chrono::seconds(5)) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(test_node->get_logger(), "Clear costmap service call succeeded");
  } else {
    RCLCPP_ERROR(test_node->get_logger(), "Clear costmap service call failed");
  }
}

void testCountLethal(std::string name, double distance, bool obstacles, bool static_map, int global_lethal, int local_lethal = 0)
{
  auto global = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name + "/global", std::string(""), std::string(""));
  auto local = std::make_shared<nav2_costmap_2d::Costmap2DROS>(name + "/local", std::string(""), std::string(""));

  std::shared_ptr<nav2_costmap_2d::ObstacleLayer> olayer;

  auto plugins = global->getLayeredCostmap()->getPlugins();
  for (auto& plugin : *plugins) {
    if (plugin->getName().find("obstacles") != std::string::npos) {
      olayer = std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(plugin);
      addObservation(&(*olayer), 5.0, 0.0, MAX_Z / 2, 0, 0, MAX_Z / 2);
      addObservation(&(*olayer), 0.0, 5.0, MAX_Z / 2, 0, 0, MAX_Z / 2);
    }
  }

  global->updateMap();
  local->updateMap();
  olayer->clearStaticObservations(true, true);

  testClearBehavior("clear", distance, obstacles, static_map, global.get(), local.get());

  global->updateMap();
  local->updateMap();

  printMap(*global->getCostmap());
  ASSERT_EQ(countValues(*global->getCostmap(), LETHAL_OBSTACLE), global_lethal);
  ASSERT_EQ(countValues(*local->getCostmap(), LETHAL_OBSTACLE), local_lethal);
}

TEST(ClearTester, basicTest) {
  testCountLethal("base", 3.0, true, false, 20);
}

TEST(ClearTester, bigRadiusTest) {
  testCountLethal("base", 20.0, true, false, 22);
}

TEST(ClearTester, clearNoLayersTest) {
  testCountLethal("base", 20.0, false, false, 22);
}

TEST(ClearTester, clearBothTest) {
  testCountLethal("base", 3.0, true, true, 0);
}

TEST(ClearTester, clearBothTest2) {
  testCountLethal("base", 12.0, true, true, 6);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  test_node = std::make_shared<rclcpp::Node>("clear_tests");
  testing::InitGoogleTest(&argc, argv);
  transformer = std::make_shared<tf2_ros::Buffer>(test_node->get_clock(), tf2::durationFromSec(10.0));
  tfl = std::make_shared<tf2_ros::TransformListener>(*transformer);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
```