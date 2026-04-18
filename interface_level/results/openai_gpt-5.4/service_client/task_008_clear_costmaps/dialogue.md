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

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <std_srvs/srv/empty.hpp>

#include <costmap_2d/testing_helper.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

tf2_ros::Buffer* transformer;
tf2_ros::TransformListener* tfl;
rclcpp::Node::SharedPtr g_node;

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
  clear_costmap_recovery::ClearCostmapRecovery clear_behavior;
  clear_behavior.initialize(
    name,
    transformer,
    global_costmap,
    local_costmap);

  auto clear_client = g_node->create_client<std_srvs::srv::Empty>(name + "/run_behavior");
  auto clear_service = g_node->create_service<std_srvs::srv::Empty>(
    name + "/run_behavior",
    [&](const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
      clear_behavior.runBehavior();
    });

  std::vector<rclcpp::Parameter> params;
  params.emplace_back(name + ".reset_distance", distance);
  params.emplace_back(name + ".force_updating", true);
  params.emplace_back(name + ".affected_maps", obstacles && static_map ? std::string("both") :
                                              obstacles ? std::string("obstacles") :
                                              static_map ? std::string("static") :
                                                           std::string("none"));
  g_node->set_parameters(params);

  auto start = std::chrono::steady_clock::now();
  while (!clear_client->wait_for_service(std::chrono::milliseconds(100))) {
    rclcpp::spin_some(g_node);
    ASSERT_LT(
      std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start).count(), 5);
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future = clear_client->async_send_request(request);

  auto result = rclcpp::spin_until_future_complete(g_node, future, std::chrono::seconds(5));
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_NE(future.get(), nullptr);
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
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  g_node = std::make_shared<rclcpp::Node>("clear_tests");
  transformer = new tf2_ros::Buffer(g_node->get_clock());
  tfl = new tf2_ros::TransformListener(*transformer);
  int result = RUN_ALL_TESTS();
  delete tfl;
  delete transformer;
  rclcpp::shutdown();
  return result;
}