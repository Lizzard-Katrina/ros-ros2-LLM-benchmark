#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <nav2_costmap_2d/testing_helper.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace costmap_2d = nav2_costmap_2d;

tf2_ros::Buffer * transformer;
tf2_ros::TransformListener * tfl;
rclcpp::Node::SharedPtr test_node;

using costmap_2d::LETHAL_OBSTACLE;

void testClearBehavior(
  std::string name,
  double distance,
  bool obstacles,
  bool static_map,
  costmap_2d::Costmap2DROS * global_costmap,
  costmap_2d::Costmap2DROS * local_costmap)
{
  (void)distance;
  (void)obstacles;
  (void)static_map;
  (void)global_costmap;
  (void)local_costmap;

  auto client = test_node->create_client<std_srvs::srv::Trigger>(name);

  ASSERT_TRUE(
    client->wait_for_service(std::chrono::seconds(5)))
    << "Service not available: " << name;

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  auto rc = rclcpp::spin_until_future_complete(test_node, future, std::chrono::seconds(10));
  ASSERT_EQ(rc, rclcpp::FutureReturnCode::SUCCESS) << "Service call timed out: " << name;

  auto response = future.get();
  ASSERT_TRUE(response->success) << "Clear behavior failed: " << response->message;
}

void testCountLethal(
  std::string name, double distance, bool obstacles, bool static_map, int global_lethal,
  int local_lethal = 0)
{
  costmap_2d::Costmap2DROS global(name + "/global", *transformer);
  costmap_2d::Costmap2DROS local(name + "/local", *transformer);
  std::shared_ptr<costmap_2d::ObstacleLayer> olayer;

  std::vector<std::shared_ptr<costmap_2d::Layer>> * plugins = global.getLayeredCostmap()->getPlugins();
  for (auto pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    std::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    if (plugin->getName().find("obstacles") != std::string::npos) {
      olayer = std::static_pointer_cast<costmap_2d::ObstacleLayer>(plugin);
      addObservation(&(*olayer), 5.0, 0.0, MAX_Z / 2, 0, 0, MAX_Z / 2);
      addObservation(&(*olayer), 0.0, 5.0, MAX_Z / 2, 0, 0, MAX_Z / 2);
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
  ASSERT_EQ(countValues(*local.getCostmap(), LETHAL_OBSTACLE), local_lethal);
}

TEST(ClearTester, basicTest)
{
  testCountLethal("base", 3.0, true, false, 20);
}

TEST(ClearTester, bigRadiusTest)
{
  testCountLethal("base", 20.0, true, false, 22);
}

TEST(ClearTester, clearNoLayersTest)
{
  testCountLethal("base", 20.0, false, false, 22);
}

TEST(ClearTester, clearBothTest)
{
  testCountLethal("base", 3.0, true, true, 0);
}

TEST(ClearTester, clearBothTest2)
{
  testCountLethal("base", 12.0, true, true, 6);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  test_node = rclcpp::Node::make_shared("clear_tests");
  transformer = new tf2_ros::Buffer(test_node->get_clock());
  tfl = new tf2_ros::TransformListener(*transformer, test_node, false);

  int result = RUN_ALL_TESTS();

  delete tfl;
  delete transformer;
  rclcpp::shutdown();
  return result;
}